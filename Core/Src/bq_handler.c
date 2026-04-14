/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bq_handler.c
  * @brief          : Cell voltage monitoring task implementation
  ******************************************************************************
  * @attention
  *
  * This module handles reading cell voltages from the BQ76952 battery monitor
  * IC via I2C bus. It reads voltage data from cells 1-9 on the I2C1 bus (BMS1).
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bq_handler.h"
#include "can_manager.h"
#include "error_manager.h"
#include <string.h>

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern uint32_t CAN_VOLTAGE_0_ID;
extern uint32_t CAN_VOLTAGE_1_ID;
extern uint32_t CAN_VOLTAGE_2_ID;
extern uint32_t CAN_VOLTAGE_3_ID;
extern uint32_t CAN_VOLTAGE_4_ID;
extern uint32_t CAN_VOLTAGE_5_ID;
extern uint32_t CAN_CHIP_STATUS_ID;

/* Private variables ---------------------------------------------------------*/
static BQ_Data_t voltage_data_bms1 = {0};
static BQ_Data_BMS2_t voltage_data_bms2 = {0};
static osMutexId_t voltage_mutex = NULL;
static osMutexId_t voltage_mutex_bms2 = NULL;

// Store last I2C errors for diagnostics
static uint32_t g_last_i2c1_error = 0;
static uint32_t g_last_i2c3_error = 0;

// I2C error counters for recovery tracking
static uint32_t g_i2c1_error_count = 0;
static uint32_t g_i2c3_error_count = 0;
static uint32_t g_i2c1_recovery_count = 0;
static uint32_t g_i2c3_recovery_count = 0;

// Flag to enable/disable fault reporting (1 = enabled, 0 = disabled)
static uint8_t g_fault_reporting_enabled = BQ_FAULT_REPORTING_DEFAULT;

// BMS chip internal temperatures (0.1°C units, updated every 5 seconds)
static int16_t g_bms1_internal_temp = 0;  // BMS1 internal die temperature
static int16_t g_bms2_internal_temp = 0;  // BMS2 internal die temperature

// BMS chip status data (updated each read cycle, sent in combined CAN message)
static uint16_t g_bms1_stack_voltage = 0;  // BMS1 stack voltage (10mV/LSB from register)
static uint16_t g_bms1_alarm_status = 0;   // BMS1 alarm status bitmask
static uint16_t g_bms2_stack_voltage = 0;  // BMS2 stack voltage (10mV/LSB from register)
static uint16_t g_bms2_alarm_status = 0;   // BMS2 alarm status bitmask

// Runtime voltage thresholds (can be set via CAN, reset to defaults on reboot)
static uint16_t g_cell_voltage_min_mv = CELL_VOLTAGE_MIN_MV;  // Min cell voltage threshold
static uint16_t g_cell_voltage_max_mv = CELL_VOLTAGE_MAX_MV;  // Max cell voltage threshold

// BQ76952 power mode tracking (protected by power_mode_mutex)
static BQ_PowerMode_t g_bq_power_mode = BQ_DEFAULT_POWER_MODE;
static osMutexId_t power_mode_mutex = NULL;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef BQ76952_ReadRegister16(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                                 uint16_t reg_addr, uint16_t *value);
static HAL_StatusTypeDef BQ76952_SendSubcommand(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                                 uint16_t subcmd);
static HAL_StatusTypeDef I2C_RecoverBus(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef BQ_UpdateChipData(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
                                            uint16_t *stack_voltage_out, uint16_t *alarm_status_out);
static HAL_StatusTypeDef BQ_SendCombinedChipStatus(void);

/**
  * @brief  Recover I2C bus from stuck state
  * @param  hi2c: I2C handle to recover
  * @retval HAL_StatusTypeDef: HAL_OK if recovered, HAL_ERROR if recovery failed
  * @note   This function attempts multiple recovery strategies:
  *         1. Clear error flags
  *         2. DeInit/Init the peripheral
  *         3. Toggle SCL to release stuck SDA (if GPIO available)
  */
static HAL_StatusTypeDef I2C_RecoverBus(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status = HAL_OK;
    GPIO_TypeDef *scl_port = NULL;
    uint16_t scl_pin = 0;
    GPIO_TypeDef *sda_port = NULL;
    uint16_t sda_pin = 0;
    
    // Determine which I2C and its pins (from .ioc file)
    if (hi2c->Instance == I2C1) {
        // I2C1: SCL=PA9, SDA=PA10
        scl_port = GPIOA;
        scl_pin = GPIO_PIN_9;
        sda_port = GPIOA;
        sda_pin = GPIO_PIN_10;
        g_i2c1_recovery_count++;
    } else if (hi2c->Instance == I2C3) {
        // I2C3: SCL=PA7, SDA=PB4
        scl_port = GPIOA;
        scl_pin = GPIO_PIN_7;
        sda_port = GPIOB;
        sda_pin = GPIO_PIN_4;
        g_i2c3_recovery_count++;
    }
    
    // Step 1: DeInit the I2C peripheral
    HAL_I2C_DeInit(hi2c);
    
    // Step 2: Configure SCL as GPIO output to clock out stuck slave
    if (scl_port != NULL && sda_port != NULL) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        
        // Save original state of SDA
        HAL_GPIO_DeInit(scl_port, scl_pin);
        HAL_GPIO_DeInit(sda_port, sda_pin);
        
        // Configure SCL as output
        GPIO_InitStruct.Pin = scl_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(scl_port, &GPIO_InitStruct);
        
        // Configure SDA as input to monitor
        GPIO_InitStruct.Pin = sda_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(sda_port, &GPIO_InitStruct);
        
        // Toggle SCL up to 16 times to release any stuck slave
        for (int i = 0; i < 16; i++) {
            HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
            for (volatile int d = 0; d < 100; d++);  // Short delay
            HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
            for (volatile int d = 0; d < 100; d++);  // Short delay
            
            // Check if SDA is released (high)
            if (HAL_GPIO_ReadPin(sda_port, sda_pin) == GPIO_PIN_SET) {
                break;  // SDA released, slave is unstuck
            }
        }
        
        // Generate STOP condition: SDA low -> high while SCL high
        GPIO_InitStruct.Pin = sda_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(sda_port, &GPIO_InitStruct);
        
        HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);  // SDA low
        for (volatile int d = 0; d < 100; d++);
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);    // SCL high
        for (volatile int d = 0; d < 100; d++);
        HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);    // SDA high (STOP)
        for (volatile int d = 0; d < 100; d++);
        
        // De-init GPIO so HAL_I2C_Init can reconfigure them
        HAL_GPIO_DeInit(scl_port, scl_pin);
        HAL_GPIO_DeInit(sda_port, sda_pin);
    }
    
    // Step 3: Re-initialize the I2C peripheral
    status = HAL_I2C_Init(hi2c);
    
    // Small delay to let the bus settle
    osDelay(5);
    
    // Clear any remaining error flags
    if (status == HAL_OK) {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF | I2C_FLAG_OVR);
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        hi2c->State = HAL_I2C_STATE_READY;
    }
    
    return status;
}

/**
  * @brief  Enable or disable fault reporting from BQ handler
  * @param  enabled: 1 to enable fault reporting, 0 to disable
  */
void BQ_SetFaultReportingEnabled(uint8_t enabled)
{
    g_fault_reporting_enabled = enabled ? 1 : 0;
}

/**
  * @brief  Get current fault reporting state
  * @retval uint8_t: 1 if enabled, 0 if disabled
  */
uint8_t BQ_GetFaultReportingEnabled(void)
{
    return g_fault_reporting_enabled;
}

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Cell voltage monitoring task (RTOS task function)
  * @param  argument: Not used
  * @retval None
  */
void BQ_MonitorTask(void *argument)
{
    HAL_StatusTypeDef status;
    uint32_t last_read_tick = 0;
    uint32_t last_can_tick = 0;
    uint32_t last_internal_temp_tick = 0;
    uint32_t current_tick;
    
    // Create mutex for thread-safe data access
    const osMutexAttr_t mutex_attr = {
        .name = "VoltageMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = NULL,
        .cb_size = 0U
    };
    
    voltage_mutex = osMutexNew(&mutex_attr);
    
    // Initialize power mode (only once, from BMS1 monitor task)
    BQ_InitPowerMode();
    
    // Wait for system initialization
    osDelay(500);
    
    // Initialize timestamps
    last_read_tick = osKernelGetTickCount();
    last_can_tick = osKernelGetTickCount();
    last_internal_temp_tick = osKernelGetTickCount();
    
    /* Infinite loop */
    for(;;)
    {
        current_tick = osKernelGetTickCount();
        
        // Select intervals based on current power mode
        BQ_PowerMode_t current_mode = BQ_GetPowerMode();
        uint32_t read_interval = (current_mode == BQ_MODE_SLEEP) ? BQ_SLEEP_READ_INTERVAL_MS : BQ_NORMAL_READ_INTERVAL_MS;
        uint32_t can_interval  = (current_mode == BQ_MODE_SLEEP) ? BQ_SLEEP_CAN_INTERVAL_MS  : BQ_NORMAL_CAN_INTERVAL_MS;
        
        // Read cell voltages at mode-dependent interval
        if ((current_tick - last_read_tick) >= read_interval) {
            
            // In SLEEP mode: wake chip before reading
            if (current_mode == BQ_MODE_SLEEP) {
                if (I2C1Handle != NULL) {
                    osMutexAcquire(I2C1Handle, I2C_TIMEOUT_MS);
                }
                BQ76952_SendSubcommand(&hi2c1, BQ76952_I2C_ADDR_BMS1, SLEEP_DISABLE);
                if (I2C1Handle != NULL) {
                    osMutexRelease(I2C1Handle);
                }
                osDelay(2);
            }
            
            // Read BMS1 (cells 1-9 on I2C1)
            status = BQ_ReadBMS1(&voltage_data_bms1);
            
            if (status != HAL_OK) {
                // Set I2C error flags (only if fault reporting is enabled)
                if (g_fault_reporting_enabled) {
                    ErrorMgr_SetError(ERROR_I2C_BMS1);
                    //ErrorMgr_SetError(ERROR_I2C_FAULT);  // General I2C fault flag
                }
                
                // Clear all voltage readings to 0V during I2C fault
                if (voltage_mutex != NULL) {
                    osMutexAcquire(voltage_mutex, osWaitForever);
                }
                
                for (uint8_t i = 0; i < BMS1_NUM_CELLS; i++) {
                    voltage_data_bms1.cell_voltage_mv[i] = 0;
                }
                voltage_data_bms1.valid = 0;
                voltage_data_bms1.last_update_tick = osKernelGetTickCount();
                
                if (voltage_mutex != NULL) {
                    osMutexRelease(voltage_mutex);
                }
            } else {
                // Clear I2C error flag on successful read (only if fault reporting is enabled)
                if (g_fault_reporting_enabled) {
                    ErrorMgr_ClearError(ERROR_I2C_BMS1);
                    
                    // Only clear general I2C fault if BMS2 is also OK
                    // Check if BMS2 error flag is not set
                    if (!(ErrorMgr_GetErrors() & ERROR_I2C_BMS2)) {
                        ErrorMgr_ClearError(ERROR_I2C_FAULT);
                    }
                    
                    // Check voltage limits and set error/warning flags
                    BQ_CheckLimits(&voltage_data_bms1);
                }
            }
            
            // In SLEEP mode: put chip back to sleep after reading
            if (current_mode == BQ_MODE_SLEEP) {
                if (I2C1Handle != NULL) {
                    osMutexAcquire(I2C1Handle, I2C_TIMEOUT_MS);
                }
                BQ76952_SendSubcommand(&hi2c1, BQ76952_I2C_ADDR_BMS1, SLEEP_ENABLE);
                if (I2C1Handle != NULL) {
                    osMutexRelease(I2C1Handle);
                }
            }
            
            last_read_tick = current_tick;
        }
        
        // Read BMS1 internal temperature every 5 seconds
        if ((current_tick - last_internal_temp_tick) >= 5000) {
            int16_t int_temp = 0;
            if (I2C1Handle != NULL) {
                osMutexAcquire(I2C1Handle, osWaitForever);
            }
            status = BQ76952_ReadRegister16(&hi2c1, BQ76952_I2C_ADDR_BMS1, IntTemperature, (uint16_t*)&int_temp);
            if (I2C1Handle != NULL) {
                osMutexRelease(I2C1Handle);
            }
            if (status == HAL_OK) {
                // Convert from 0.1K (Kelvin) to 0.1°C (Celsius)
                // 0°C = 273.15K, so subtract 2732 (273.2K in 0.1K units)
                g_bms1_internal_temp = int_temp - 2732;
            }
            last_internal_temp_tick = current_tick;
        }
        
        // Send CAN messages at mode-dependent interval
        if ((current_tick - last_can_tick) >= can_interval) {
            // Send voltage data via CAN (send even if invalid for debugging)
            // When I2C fails, the voltages will be 0 or old data
            BQ_SendCANMessage(&voltage_data_bms1);
            
            // Update BMS1 chip data (stack voltage, alarm status)
            BQ_UpdateChipData(&hi2c1, BQ76952_I2C_ADDR_BMS1, &g_bms1_stack_voltage, &g_bms1_alarm_status);
            
            // Send combined chip status for both BMS1 and BMS2
            BQ_SendCombinedChipStatus();
            
            last_can_tick = current_tick;
        }
        
        // Task delay (10ms)
        osDelay(10);
    }
}

/**
  * @brief  Read cell voltages from BQ76952 chip on I2C1 (BMS1)
  * @param  data: Pointer to structure to store voltage data
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_ReadBMS1(BQ_Data_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    if (data == NULL) {
        return HAL_ERROR;
    }
    
    // Acquire I2C1 mutex
    if (I2C1Handle != NULL) {
        if (osMutexAcquire(I2C1Handle, I2C_TIMEOUT_MS) != osOK) {
            return HAL_ERROR;
        }
    }
    
    // Check I2C1 state and recover if needed
    if (hi2c1.State != HAL_I2C_STATE_READY) {
        // I2C is stuck - attempt recovery
        I2C_RecoverBus(&hi2c1);
    }
    
    // Check I2C1 error state for diagnostics
    g_last_i2c1_error = HAL_I2C_GetError(&hi2c1);
    
    // If I2C1 has errors, try to clear them or recover
    if (g_last_i2c1_error != HAL_I2C_ERROR_NONE) {
        g_i2c1_error_count++;
        // Try to clear error flags first
        __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF | I2C_FLAG_OVR);
        hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
        hi2c1.State = HAL_I2C_STATE_READY;
        
        // If still not ready, do full recovery
        if (hi2c1.State != HAL_I2C_STATE_READY) {
            I2C_RecoverBus(&hi2c1);
        }
    }
    
    // Read cells 1-9
    for (uint8_t i = 0; i < BMS1_NUM_CELLS; i++) {
        uint16_t voltage_mv = 0;
        
        status = BQ_ReadCell(&hi2c1, BQ76952_I2C_ADDR_BMS1, i + 1, &voltage_mv);
        
        if (status != HAL_OK) {
            // First read failed - attempt recovery and retry once
            g_i2c1_error_count++;
            I2C_RecoverBus(&hi2c1);
            
            // Retry the read
            status = BQ_ReadCell(&hi2c1, BQ76952_I2C_ADDR_BMS1, i + 1, &voltage_mv);
        }
        
        if (status == HAL_OK) {
            data->cell_voltage_mv[i] = voltage_mv;
        } else {
            // Communication error even after retry - mark data as invalid
            data->valid = 0;
            
            // Release I2C1 mutex
            if (I2C1Handle != NULL) {
                osMutexRelease(I2C1Handle);
            }
            
            return HAL_ERROR;
        }
    }
    
    // Release I2C1 mutex
    if (I2C1Handle != NULL) {
        osMutexRelease(I2C1Handle);
    }
    
    // Update timestamp and mark data as valid
    if (voltage_mutex != NULL) {
        osMutexAcquire(voltage_mutex, osWaitForever);
    }
    
    data->last_update_tick = osKernelGetTickCount();
    data->valid = 1;
    
    if (voltage_mutex != NULL) {
        osMutexRelease(voltage_mutex);
    }
    
    return HAL_OK;
}

/**
  * @brief  Read single cell voltage from BQ76952
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address
  * @param  cell_num: Cell number (1-16)
  * @param  voltage_mv: Pointer to store voltage in millivolts
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Cell voltage registers are 16-bit, LSB first
  *         Voltage = raw_value * 1 mV (direct millivolt reading)
  */
HAL_StatusTypeDef BQ_ReadCell(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                       uint8_t cell_num, uint16_t *voltage_mv)
{
    HAL_StatusTypeDef status;
    uint16_t reg_addr;
    uint16_t raw_value;
    
    // Validate inputs
    if (hi2c == NULL || voltage_mv == NULL || cell_num < 1 || cell_num > 16) {
        return HAL_ERROR;
    }
    
    // Map cell number to register address (from bq76952.h)
    // Cells are numbered 1-16, registers start at Cell1Voltage (0x14)
    switch (cell_num) {
        case 1:  reg_addr = Cell1Voltage;  break;
        case 2:  reg_addr = Cell2Voltage;  break;
        case 3:  reg_addr = Cell3Voltage;  break;
        case 4:  reg_addr = Cell4Voltage;  break;
        case 5:  reg_addr = Cell5Voltage;  break;
        case 6:  reg_addr = Cell6Voltage;  break;
        case 7:  reg_addr = Cell7Voltage;  break;
        case 8:  reg_addr = Cell8Voltage;  break;
        case 9:  reg_addr = Cell9Voltage;  break;
        case 10: reg_addr = Cell10Voltage; break;
        case 11: reg_addr = Cell11Voltage; break;
        case 12: reg_addr = Cell12Voltage; break;
        case 13: reg_addr = Cell13Voltage; break;
        case 14: reg_addr = Cell14Voltage; break;
        case 15: reg_addr = Cell15Voltage; break;
        case 16: reg_addr = Cell16Voltage; break;
        default: return HAL_ERROR;
    }
    
    // Read 16-bit register (Direct Command - single byte address)
    status = BQ76952_ReadRegister16(hi2c, device_addr, reg_addr, &raw_value);
    
    if (status == HAL_OK) {
        // BQ76952 voltage registers are already in millivolts (1 mV resolution)
        *voltage_mv = raw_value;
    }
    
    return status;
}

/**
  * @brief  Read 16-bit register from BQ76952 (Direct Command)
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  reg_addr: Register address (8-bit for Direct Commands)
  * @param  value: Pointer to store 16-bit value
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   BQ76952 Direct Commands use single-byte addressing
  *         Data is returned LSB first (little-endian)
  */
static HAL_StatusTypeDef BQ76952_ReadRegister16(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                                 uint16_t reg_addr, uint16_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t reg_byte = (uint8_t)(reg_addr & 0xFF);  // Direct commands are 8-bit addresses
    uint8_t data[2];
    
    // Write register address
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), &reg_byte, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    // Read 2 bytes (LSB first)
    status = HAL_I2C_Master_Receive(hi2c, (device_addr << 1), data, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    // Combine bytes (LSB first)
    *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    
    return HAL_OK;
}

/**
  * @brief  Send cell voltage data via CAN bus
  * @param  data: Pointer to voltage data structure
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Sends 3 CAN messages with 3 voltages each (6 bytes per message):
  *         - Cell_Voltage_0: Cells 1-3 (6 bytes)
  *         - Cell_Voltage_1: Cells 4-6 (6 bytes)
  *         - Cell_Voltage_2: Cells 7-9 (6 bytes)
  */
HAL_StatusTypeDef BQ_SendCANMessage(BQ_Data_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t can_data[8];

    if (data == NULL) {
        return HAL_ERROR;
    }

    // Calculate average, min, and max
    uint32_t volt_avg = 0;
    uint16_t volt_min = 0, volt_max = 0;
    uint8_t volt_min_id, volt_max_id;
    for (int i = 0; i < BMS1_NUM_CELLS; i++) {
        uint16_t curr_voltage = data->cell_voltage_mv[i];
        volt_avg += curr_voltage;
        if (volt_min == 0 || curr_voltage < volt_min) {
            volt_min = curr_voltage;
            volt_min_id = i + 1;
        }
        if (volt_max == 0 || curr_voltage > volt_max){
            volt_max = curr_voltage;
            volt_max_id = i + 1;
        }
    }
    volt_avg = volt_avg / BMS1_NUM_CELLS;

    // BMS1 Summary Message (avg, min, max)
    can_data[0] = (uint8_t)(volt_avg & 0xFF);
    can_data[1] = (uint8_t)((volt_avg >> 8) & 0xFF);
    can_data[2] = (uint8_t)(volt_min & 0xFF);
    can_data[3] = (uint8_t)((volt_min >> 8) & 0xFF);
    can_data[4] = (uint8_t)(volt_max & 0xFF);
    can_data[5] = (uint8_t)((volt_max >> 8) & 0xFF);
    can_data[6] = volt_min_id;  // Padding
    can_data[7] = volt_max_id;  // Padding
    
    status = CAN_SendMessage(CAN_VOLTAGE_BMS_1_SUMMARY_ID, can_data, 8, CAN_PRIORITY_NORMAL);
    if (status != HAL_OK) {
        return status;
    }

    // Message 0: Cells 1-3 (6 bytes)
    can_data[0] = (uint8_t)(data->cell_voltage_mv[0] & 0xFF);
    can_data[1] = (uint8_t)((data->cell_voltage_mv[0] >> 8) & 0xFF);
    can_data[2] = (uint8_t)(data->cell_voltage_mv[1] & 0xFF);
    can_data[3] = (uint8_t)((data->cell_voltage_mv[1] >> 8) & 0xFF);
    can_data[4] = (uint8_t)(data->cell_voltage_mv[2] & 0xFF);
    can_data[5] = (uint8_t)((data->cell_voltage_mv[2] >> 8) & 0xFF);
    can_data[6] = 0x00;  // Padding
    can_data[7] = 0x00;  // Padding
    
    status = CAN_SendMessage(CAN_VOLTAGE_0_ID, can_data, 6, CAN_PRIORITY_NORMAL);
    if (status != HAL_OK) {
        return status;
    }
    
    // Message 1: Cells 4-6 (6 bytes)
    can_data[0] = (uint8_t)(data->cell_voltage_mv[3] & 0xFF);
    can_data[1] = (uint8_t)((data->cell_voltage_mv[3] >> 8) & 0xFF);
    can_data[2] = (uint8_t)(data->cell_voltage_mv[4] & 0xFF);
    can_data[3] = (uint8_t)((data->cell_voltage_mv[4] >> 8) & 0xFF);
    can_data[4] = (uint8_t)(data->cell_voltage_mv[5] & 0xFF);
    can_data[5] = (uint8_t)((data->cell_voltage_mv[5] >> 8) & 0xFF);
    can_data[6] = 0x00;  // Padding
    can_data[7] = 0x00;  // Padding
    
    status = CAN_SendMessage(CAN_VOLTAGE_1_ID, can_data, 6, CAN_PRIORITY_NORMAL);
    if (status != HAL_OK) {
        return status;
    }
    
    // Message 2: Cells 7-9 (6 bytes)
    can_data[0] = (uint8_t)(data->cell_voltage_mv[6] & 0xFF);
    can_data[1] = (uint8_t)((data->cell_voltage_mv[6] >> 8) & 0xFF);
    can_data[2] = (uint8_t)(data->cell_voltage_mv[7] & 0xFF);
    can_data[3] = (uint8_t)((data->cell_voltage_mv[7] >> 8) & 0xFF);
    can_data[4] = (uint8_t)(data->cell_voltage_mv[8] & 0xFF);
    can_data[5] = (uint8_t)((data->cell_voltage_mv[8] >> 8) & 0xFF);
    can_data[6] = 0x00;  // Padding
    can_data[7] = 0x00;  // Padding
    
    status = CAN_SendMessage(CAN_VOLTAGE_2_ID, can_data, 6, CAN_PRIORITY_NORMAL);
    
    return status;
}

/**
  * @brief  Get current cell voltage data (thread-safe)
  * @param  data: Pointer to structure to copy data into
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_GetData(BQ_Data_t *data)
{
    if (data == NULL) {
        return HAL_ERROR;
    }
    
    // Thread-safe copy
    if (voltage_mutex != NULL) {
        osMutexAcquire(voltage_mutex, osWaitForever);
    }
    
    memcpy(data, &voltage_data_bms1, sizeof(BQ_Data_t));
    
    if (voltage_mutex != NULL) {
        osMutexRelease(voltage_mutex);
    }
    
    return HAL_OK;
}

/**
  * @brief  Check cell voltages against limits and set error/warning flags
  * @param  data: Pointer to voltage data structure
  * @retval None
  * @note   Sets ERROR_OVER_VOLTAGE or ERROR_UNDER_VOLTAGE if out of range
  *         Sets WARNING_HIGH_VOLTAGE or WARNING_LOW_VOLTAGE if approaching limits
  */
void BQ_CheckLimits(BQ_Data_t *data)
{
    if (data == NULL || !data->valid) {
        return;
    }
    
    // Skip all error/warning reporting if fault reporting is disabled
    if (!g_fault_reporting_enabled) {
        return;
    }
    
    bool over_voltage_detected = false;
    bool under_voltage_detected = false;
    bool high_voltage_warning = false;
    bool low_voltage_warning = false;
    
    // Check all cells in the data structure
    for (uint8_t i = 0; i < BMS1_NUM_CELLS; i++) {
        uint16_t voltage_mv = data->cell_voltage_mv[i];
        
        // Skip zero readings (likely invalid/disconnected cells)
        if (voltage_mv == 0) {
            continue;
        }
        
        // Check for over-voltage error (above max limit)
        if (voltage_mv > g_cell_voltage_max_mv) {
            over_voltage_detected = true;
        }
        // Check for under-voltage error (below min limit)
        else if (voltage_mv < g_cell_voltage_min_mv) {
            under_voltage_detected = true;
        }
        // Check for high voltage warning (approaching max)
        else if (voltage_mv > CELL_VOLTAGE_WARNING_HIGH_MV) {
            high_voltage_warning = true;
        }
        // Check for low voltage warning (approaching min)
        else if (voltage_mv < CELL_VOLTAGE_WARNING_LOW_MV) {
            low_voltage_warning = true;
        }
    }
    
    // Set or clear error flags
    if (over_voltage_detected) {
        ErrorMgr_SetError(ERROR_OVER_VOLTAGE);
    } else {
        ErrorMgr_ClearError(ERROR_OVER_VOLTAGE);
    }
    
    if (under_voltage_detected) {
        ErrorMgr_SetError(ERROR_UNDER_VOLTAGE);
    } else {
        ErrorMgr_ClearError(ERROR_UNDER_VOLTAGE);
    }
    
    // Set or clear warning flags
    if (high_voltage_warning) {
        ErrorMgr_SetWarning(WARNING_HIGH_VOLTAGE);
    } else {
        ErrorMgr_ClearWarning(WARNING_HIGH_VOLTAGE);
    }
    
    if (low_voltage_warning) {
        ErrorMgr_SetWarning(WARNING_LOW_VOLTAGE);
    } else {
        ErrorMgr_ClearWarning(WARNING_LOW_VOLTAGE);
    }
}

/* ============================================================================ */
/* BMS2 Functions (I2C3 - Cells 10-18)                                         */
/* ============================================================================ */

/**
  * @brief  Cell voltage monitoring task for BMS2 (RTOS task function)
  * @param  argument: Not used
  * @retval None
  */
void BQ_MonitorTask_BMS2(void *argument)
{
    HAL_StatusTypeDef status;
    uint32_t last_read_tick = 0;
    uint32_t last_can_tick = 0;
    uint32_t last_internal_temp_tick = 0;
    uint32_t current_tick;
    
    // Create mutex for thread-safe data access
    const osMutexAttr_t mutex_attr = {
        .name = "VoltageMutexBMS2",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = NULL,
        .cb_size = 0U
    };
    
    voltage_mutex_bms2 = osMutexNew(&mutex_attr);
    
    // Wait for system initialization - stagger 125ms after BMS1 to avoid lockstep execution
    osDelay(625);
    
    // Diagnostic: Check if I2C3 bus is operational by scanning for BMS2 device
    if (I2C3Handle != NULL) {
        osMutexAcquire(I2C3Handle, osWaitForever);
    }
    
    // Try to detect BMS2 device at expected address
    status = HAL_I2C_IsDeviceReady(&hi2c3, (BQ76952_I2C_ADDR_BMS2 << 1), 3, 100);
    if (status != HAL_OK) {
        // Device not found - set error and continue trying (only if fault reporting is enabled)
        if (g_fault_reporting_enabled) {
            ErrorMgr_SetError(ERROR_I2C_BMS2);
        }
    }
    
    if (I2C3Handle != NULL) {
        osMutexRelease(I2C3Handle);
    }
    
    // Initialize timestamps
    last_read_tick = osKernelGetTickCount();
    last_can_tick = osKernelGetTickCount();
    last_internal_temp_tick = osKernelGetTickCount();
    
    /* Infinite loop */
    for(;;)
    {
        current_tick = osKernelGetTickCount();
        
        // Select intervals based on current power mode
        BQ_PowerMode_t current_mode = BQ_GetPowerMode();
        uint32_t read_interval = (current_mode == BQ_MODE_SLEEP) ? BQ_SLEEP_READ_INTERVAL_MS : BQ_NORMAL_READ_INTERVAL_MS;
        uint32_t can_interval  = (current_mode == BQ_MODE_SLEEP) ? BQ_SLEEP_CAN_INTERVAL_MS  : BQ_NORMAL_CAN_INTERVAL_MS;
        
        // Read cell voltages at mode-dependent interval
        if ((current_tick - last_read_tick) >= read_interval) {
            
            // In SLEEP mode: wake chip before reading
            if (current_mode == BQ_MODE_SLEEP) {
                if (I2C3Handle != NULL) {
                    osMutexAcquire(I2C3Handle, I2C_TIMEOUT_MS);
                }
                BQ76952_SendSubcommand(&hi2c3, BQ76952_I2C_ADDR_BMS2, SLEEP_DISABLE);
                if (I2C3Handle != NULL) {
                    osMutexRelease(I2C3Handle);
                }
                osDelay(2);
            }
            
            // Read BMS2 (cells 10-18 on I2C3)
            status = BQ_ReadBMS2(&voltage_data_bms2);
            
            if (status != HAL_OK) {
                // Set I2C error flags (only if fault reporting is enabled)
                if (g_fault_reporting_enabled) {
                    ErrorMgr_SetError(ERROR_I2C_BMS2);
                    //ErrorMgr_SetError(ERROR_I2C_FAULT);  // General I2C fault flag
                }
                
                // Clear all voltage readings to 0V during I2C fault
                if (voltage_mutex_bms2 != NULL) {
                    osMutexAcquire(voltage_mutex_bms2, osWaitForever);
                }
                
                for (uint8_t i = 0; i < BMS2_NUM_CELLS; i++) {
                    voltage_data_bms2.cell_voltage_mv[i] = 0;
                }
                voltage_data_bms2.valid = 0;
                voltage_data_bms2.last_update_tick = osKernelGetTickCount();
                
                if (voltage_mutex_bms2 != NULL) {
                    osMutexRelease(voltage_mutex_bms2);
                }
            } else {
                // Clear I2C error flag on successful read (only if fault reporting is enabled)
                if (g_fault_reporting_enabled) {
                    ErrorMgr_ClearError(ERROR_I2C_BMS2);
                    
                    // Only clear general I2C fault if BMS1 is also OK
                    // Check if BMS1 error flag is not set
                    if (!(ErrorMgr_GetErrors() & ERROR_I2C_BMS1)) {
                        ErrorMgr_ClearError(ERROR_I2C_FAULT);
                    }
                    
                    // Check voltage limits and set error/warning flags
                    BQ_CheckLimits_BMS2(&voltage_data_bms2);
                }
            }
            
            // In SLEEP mode: put chip back to sleep after reading
            if (current_mode == BQ_MODE_SLEEP) {
                if (I2C3Handle != NULL) {
                    osMutexAcquire(I2C3Handle, I2C_TIMEOUT_MS);
                }
                BQ76952_SendSubcommand(&hi2c3, BQ76952_I2C_ADDR_BMS2, SLEEP_ENABLE);
                if (I2C3Handle != NULL) {
                    osMutexRelease(I2C3Handle);
                }
            }
            
            last_read_tick = current_tick;
        }
        
        // Read BMS2 internal temperature every 5 seconds
        if ((current_tick - last_internal_temp_tick) >= 5000) {
            int16_t int_temp = 0;
            if (I2C3Handle != NULL) {
                osMutexAcquire(I2C3Handle, osWaitForever);
            }
            status = BQ76952_ReadRegister16(&hi2c3, BQ76952_I2C_ADDR_BMS2, IntTemperature, (uint16_t*)&int_temp);
            if (I2C3Handle != NULL) {
                osMutexRelease(I2C3Handle);
            }
            if (status == HAL_OK) {
                // Convert from 0.1K (Kelvin) to 0.1°C (Celsius)
                // 0°C = 273.15K, so subtract 2732 (273.2K in 0.1K units)
                g_bms2_internal_temp = int_temp - 2732;
            }
            last_internal_temp_tick = current_tick;
        }
        
        // Send CAN messages at mode-dependent interval
        if ((current_tick - last_can_tick) >= can_interval) {
            // Send voltage data via CAN (send even if invalid for debugging)
            // When I2C fails, the voltages will be 0 or old data
            BQ_SendCANMessage_BMS2(&voltage_data_bms2);
            
            // Update BMS2 chip data (stack voltage, alarm status)
            // Combined status message is sent by BMS1 task
            BQ_UpdateChipData(&hi2c3, BQ76952_I2C_ADDR_BMS2, &g_bms2_stack_voltage, &g_bms2_alarm_status);
            
            last_can_tick = current_tick;
        }
        
        // Task delay (10ms)
        osDelay(10);
    }
}

/**
  * @brief  Read cell voltages from BQ76952 chip on I2C3 (BMS2)
  * @param  data: Pointer to structure to store voltage data
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_ReadBMS2(BQ_Data_BMS2_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    if (data == NULL) {
        return HAL_ERROR;
    }
    
    // Acquire I2C3 mutex
    if (I2C3Handle != NULL) {
        if (osMutexAcquire(I2C3Handle, I2C_TIMEOUT_MS) != osOK) {
            return HAL_ERROR;
        }
    }
    
    // Check I2C3 state and recover if needed
    if (hi2c3.State != HAL_I2C_STATE_READY) {
        // I2C is stuck - attempt full recovery with GPIO clock toggle
        I2C_RecoverBus(&hi2c3);
    }
    
    // Check I2C3 error state for diagnostics
    g_last_i2c3_error = HAL_I2C_GetError(&hi2c3);
    
    // If I2C3 has errors, try to clear them or recover
    if (g_last_i2c3_error != HAL_I2C_ERROR_NONE) {
        g_i2c3_error_count++;
        // Try to clear error flags first
        __HAL_I2C_CLEAR_FLAG(&hi2c3, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF | I2C_FLAG_OVR);
        hi2c3.ErrorCode = HAL_I2C_ERROR_NONE;
        hi2c3.State = HAL_I2C_STATE_READY;
        
        // If still not ready, do full recovery
        if (hi2c3.State != HAL_I2C_STATE_READY) {
            I2C_RecoverBus(&hi2c3);
        }
    }
    
    // Read cells 10-18 (map to BQ76952 cells 1-9 on second chip)
    for (uint8_t i = 0; i < BMS2_NUM_CELLS; i++) {
        uint16_t voltage_mv = 0;
        
        // Read from BQ76952 cells 1-9 (physical cells 10-18)
        status = BQ_ReadCell(&hi2c3, BQ76952_I2C_ADDR_BMS2, i + 1, &voltage_mv);
        
        if (status != HAL_OK) {
            // First read failed - attempt recovery and retry once
            g_i2c3_error_count++;
            I2C_RecoverBus(&hi2c3);
            
            // Retry the read
            status = BQ_ReadCell(&hi2c3, BQ76952_I2C_ADDR_BMS2, i + 1, &voltage_mv);
        }
        
        if (status == HAL_OK) {
            data->cell_voltage_mv[i] = voltage_mv;
        } else {
            // Communication error even after retry - mark data as invalid
            data->valid = 0;
            
            // Release I2C3 mutex
            if (I2C3Handle != NULL) {
                osMutexRelease(I2C3Handle);
            }
            
            return HAL_ERROR;
        }
    }
    
    // Release I2C3 mutex
    if (I2C3Handle != NULL) {
        osMutexRelease(I2C3Handle);
    }
    
    // Update timestamp and mark data as valid
    if (voltage_mutex_bms2 != NULL) {
        osMutexAcquire(voltage_mutex_bms2, osWaitForever);
    }
    
    data->last_update_tick = osKernelGetTickCount();
    data->valid = 1;
    
    if (voltage_mutex_bms2 != NULL) {
        osMutexRelease(voltage_mutex_bms2);
    }
    
    return HAL_OK;
}

/**
  * @brief  Send BMS2 cell voltage data via CAN bus
  * @param  data: Pointer to BMS2 voltage data structure
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Sends 3 CAN messages with 3 voltages each (6 bytes per message):
  *         - Cell_Voltage_3: Cells 10-12 (6 bytes)
  *         - Cell_Voltage_4: Cells 13-15 (6 bytes)
  *         - Cell_Voltage_5: Cells 16-18 (6 bytes)
  */
HAL_StatusTypeDef BQ_SendCANMessage_BMS2(BQ_Data_BMS2_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t can_data[8];
    
    if (data == NULL) {
        return HAL_ERROR;
    }
    
    // Calculate average, min, and max
    uint32_t volt_avg = 0;
    uint16_t volt_min = 0, volt_max = 0;
    uint8_t volt_min_id, volt_max_id;
    for (int i = 0; i < BMS2_NUM_CELLS; i++) {
        uint16_t curr_voltage = data->cell_voltage_mv[i];
        volt_avg += curr_voltage;
        if (volt_min == 0 || curr_voltage < volt_min) {
            volt_min = curr_voltage;
            volt_min_id = BMS1_NUM_CELLS + i + 1;
        }
        if (volt_max == 0 || curr_voltage > volt_max){
            volt_max = curr_voltage;
            volt_max_id = BMS1_NUM_CELLS + i + 1;
        }
    }
    volt_avg = volt_avg / BMS2_NUM_CELLS;

    // BMS1 Summary Message (avg, min, max)
    can_data[0] = (uint8_t)(volt_avg & 0xFF);
    can_data[1] = (uint8_t)((volt_avg >> 8) & 0xFF);
    can_data[2] = (uint8_t)(volt_min & 0xFF);
    can_data[3] = (uint8_t)((volt_min >> 8) & 0xFF);
    can_data[4] = (uint8_t)(volt_max & 0xFF);
    can_data[5] = (uint8_t)((volt_max >> 8) & 0xFF);
    can_data[6] = volt_min_id;
    can_data[7] = volt_max_id;
    
    status = CAN_SendMessage(CAN_VOLTAGE_BMS_2_SUMMARY_ID, can_data, 8, CAN_PRIORITY_NORMAL);
    if (status != HAL_OK) {
        return status;
    }

    // Message 3: Cells 10-12 (6 bytes)
    can_data[0] = (uint8_t)(data->cell_voltage_mv[0] & 0xFF);
    can_data[1] = (uint8_t)((data->cell_voltage_mv[0] >> 8) & 0xFF);
    can_data[2] = (uint8_t)(data->cell_voltage_mv[1] & 0xFF);
    can_data[3] = (uint8_t)((data->cell_voltage_mv[1] >> 8) & 0xFF);
    can_data[4] = (uint8_t)(data->cell_voltage_mv[2] & 0xFF);
    can_data[5] = (uint8_t)((data->cell_voltage_mv[2] >> 8) & 0xFF);
    can_data[6] = 0x00;  // Padding
    can_data[7] = 0x00;  // Padding
    
    status = CAN_SendMessage(CAN_VOLTAGE_3_ID, can_data, 6, CAN_PRIORITY_NORMAL);
    if (status != HAL_OK) {
        return status;
    }
    
    // Message 4: Cells 13-15 (6 bytes)
    can_data[0] = (uint8_t)(data->cell_voltage_mv[3] & 0xFF);
    can_data[1] = (uint8_t)((data->cell_voltage_mv[3] >> 8) & 0xFF);
    can_data[2] = (uint8_t)(data->cell_voltage_mv[4] & 0xFF);
    can_data[3] = (uint8_t)((data->cell_voltage_mv[4] >> 8) & 0xFF);
    can_data[4] = (uint8_t)(data->cell_voltage_mv[5] & 0xFF);
    can_data[5] = (uint8_t)((data->cell_voltage_mv[5] >> 8) & 0xFF);
    can_data[6] = 0x00;  // Padding
    can_data[7] = 0x00;  // Padding
    
    status = CAN_SendMessage(CAN_VOLTAGE_4_ID, can_data, 6, CAN_PRIORITY_NORMAL);
    if (status != HAL_OK) {
        return status;
    }
    
    // Message 5: Cells 16-18 (6 bytes)
    can_data[0] = (uint8_t)(data->cell_voltage_mv[6] & 0xFF);
    can_data[1] = (uint8_t)((data->cell_voltage_mv[6] >> 8) & 0xFF);
    can_data[2] = (uint8_t)(data->cell_voltage_mv[7] & 0xFF);
    can_data[3] = (uint8_t)((data->cell_voltage_mv[7] >> 8) & 0xFF);
    can_data[4] = (uint8_t)(data->cell_voltage_mv[8] & 0xFF);
    can_data[5] = (uint8_t)((data->cell_voltage_mv[8] >> 8) & 0xFF);
    can_data[6] = 0x00;  // Padding
    can_data[7] = 0x00;  // Padding
    
    status = CAN_SendMessage(CAN_VOLTAGE_5_ID, can_data, 6, CAN_PRIORITY_NORMAL);
    
    return status;
}

/**
  * @brief  Update chip status data (stack voltage, alarm status) from a BQ76952 chip
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address
  * @param  stack_voltage_out: Pointer to store stack voltage (10mV/LSB)
  * @param  alarm_status_out: Pointer to store alarm status bitmask
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
static HAL_StatusTypeDef BQ_UpdateChipData(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
                                            uint16_t *stack_voltage_out, uint16_t *alarm_status_out)
{
    HAL_StatusTypeDef status;
    uint16_t stack_voltage = 0;
    uint16_t alarm_status = 0;
    
    // Determine which mutex to use based on I2C handle
    osMutexId_t i2c_mutex = NULL;
    if (hi2c == &hi2c1) {
        i2c_mutex = I2C1Handle;
    } else if (hi2c == &hi2c3) {
        i2c_mutex = I2C3Handle;
    }
    
    // Acquire I2C mutex
    if (i2c_mutex != NULL) {
        if (osMutexAcquire(i2c_mutex, I2C_TIMEOUT_MS) != osOK) {
            return HAL_ERROR;
        }
    }
    
    // Read Stack Voltage (0x34)
    status = BQ76952_ReadRegister16(hi2c, device_addr, StackVoltage, &stack_voltage);
    if (status != HAL_OK) {
        if (i2c_mutex != NULL) {
            osMutexRelease(i2c_mutex);
        }
        return status;
    }
    
    // Read Alarm Status (0x62)
    status = BQ76952_ReadRegister16(hi2c, device_addr, AlarmStatus, &alarm_status);
    if (status != HAL_OK) {
        if (i2c_mutex != NULL) {
            osMutexRelease(i2c_mutex);
        }
        return status;
    }
    
    // Release I2C mutex
    if (i2c_mutex != NULL) {
        osMutexRelease(i2c_mutex);
    }
    
    // Store results in provided output pointers
    *stack_voltage_out = stack_voltage;
    *alarm_status_out = alarm_status;
    
    return HAL_OK;
}

/**
  * @brief  Send combined chip status for both BMS1 and BMS2 via a single CAN message
  * @note   Layout: [BMS1_StackV(1B) | BMS2_StackV(1B) | BMS1_ICTemp(1B) | BMS2_ICTemp(1B) |
  *                  BMS1_Alarm(2B) | BMS2_Alarm(2B)]
  *         Stack voltage: raw / 20 → 200mV per bit
  *         IC temp: (0.1°C value / 10) + 40 → 1°C per bit, offset -40
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
static HAL_StatusTypeDef BQ_SendCombinedChipStatus(void)
{
    uint8_t can_data[8];
    
    // Byte 0: BMS1 stack voltage (200mV per bit)
    // Register value is in 10mV units, divide by 20 to get 200mV units
    can_data[0] = (uint8_t)(g_bms1_stack_voltage / 20);
    
    // Byte 1: BMS2 stack voltage (200mV per bit)
    can_data[1] = (uint8_t)(g_bms2_stack_voltage / 20);
    
    // Byte 2: BMS1 IC temperature (1°C per bit, offset -40)
    // g_bms1_internal_temp is in 0.1°C units, divide by 10 and add 40
    can_data[2] = (uint8_t)((g_bms1_internal_temp / 10) + 40);
    
    // Byte 3: BMS2 IC temperature (1°C per bit, offset -40)
    can_data[3] = (uint8_t)((g_bms2_internal_temp / 10) + 40);
    
    // Bytes 4-5: BMS1 alarm status (16-bit LE)
    can_data[4] = (uint8_t)(g_bms1_alarm_status & 0xFF);
    can_data[5] = (uint8_t)((g_bms1_alarm_status >> 8) & 0xFF);
    
    // Bytes 6-7: BMS2 alarm status (16-bit LE)
    can_data[6] = (uint8_t)(g_bms2_alarm_status & 0xFF);
    can_data[7] = (uint8_t)((g_bms2_alarm_status >> 8) & 0xFF);
    
    return CAN_SendMessage(CAN_CHIP_STATUS_ID, can_data, 8, CAN_PRIORITY_NORMAL);
}

/**
  * @brief  Get current BMS2 cell voltage data (thread-safe)
  * @param  data: Pointer to structure to copy BMS2 data into
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_GetData_BMS2(BQ_Data_BMS2_t *data)
{
    if (data == NULL) {
        return HAL_ERROR;
    }
    
    // Thread-safe copy
    if (voltage_mutex_bms2 != NULL) {
        osMutexAcquire(voltage_mutex_bms2, osWaitForever);
    }
    
    memcpy(data, &voltage_data_bms2, sizeof(BQ_Data_BMS2_t));
    
    if (voltage_mutex_bms2 != NULL) {
        osMutexRelease(voltage_mutex_bms2);
    }
    
    return HAL_OK;
}

/**
  * @brief  Check BMS2 cell voltages against limits and set error/warning flags
  * @param  data: Pointer to BMS2 voltage data structure
  * @retval None
  * @note   Sets ERROR_OVER_VOLTAGE or ERROR_UNDER_VOLTAGE if out of range
  *         Sets WARNING_HIGH_VOLTAGE or WARNING_LOW_VOLTAGE if approaching limits
  */
void BQ_CheckLimits_BMS2(BQ_Data_BMS2_t *data)
{
    if (data == NULL || !data->valid) {
        return;
    }
    
    // Skip all error/warning reporting if fault reporting is disabled
    if (!g_fault_reporting_enabled) {
        return;
    }
    
    bool over_voltage_detected = false;
    bool under_voltage_detected = false;
    bool high_voltage_warning = false;
    bool low_voltage_warning = false;
    
    // Check all cells in the data structure
    for (uint8_t i = 0; i < BMS2_NUM_CELLS; i++) {
        uint16_t voltage_mv = data->cell_voltage_mv[i];
        
        // Skip zero readings (likely invalid/disconnected cells)
        if (voltage_mv == 0) {
            continue;
        }
        
        // Check for over-voltage error (above max limit)
        if (voltage_mv > g_cell_voltage_max_mv) {
            over_voltage_detected = true;
        }
        // Check for under-voltage error (below min limit)
        else if (voltage_mv < g_cell_voltage_min_mv) {
            under_voltage_detected = true;
        }
        // Check for high voltage warning (approaching max)
        else if (voltage_mv > CELL_VOLTAGE_WARNING_HIGH_MV) {
            high_voltage_warning = true;
        }
        // Check for low voltage warning (approaching min)
        else if (voltage_mv < CELL_VOLTAGE_WARNING_LOW_MV) {
            low_voltage_warning = true;
        }
    }
    
    // Set or clear error flags
    if (over_voltage_detected) {
        ErrorMgr_SetError(ERROR_OVER_VOLTAGE);
    } else {
        ErrorMgr_ClearError(ERROR_OVER_VOLTAGE);
    }
    
    if (under_voltage_detected) {
        ErrorMgr_SetError(ERROR_UNDER_VOLTAGE);
    } else {
        ErrorMgr_ClearError(ERROR_UNDER_VOLTAGE);
    }
    
    // Set or clear warning flags
    if (high_voltage_warning) {
        ErrorMgr_SetWarning(WARNING_HIGH_VOLTAGE);
    } else {
        ErrorMgr_ClearWarning(WARNING_HIGH_VOLTAGE);
    }
    
    if (low_voltage_warning) {
        ErrorMgr_SetWarning(WARNING_LOW_VOLTAGE);
    } else {
        ErrorMgr_ClearWarning(WARNING_LOW_VOLTAGE);
    }
}

/**
  * @brief  Reset BQ76952 chips by pulsing PB6 (BMS_RESET) high for 500ms
  * @retval HAL_StatusTypeDef: HAL_OK on success
  * @note   This function pulses the hardware reset pin connected to both BQ76952 chips.
  *         The reset pin is active-high and should be held high for at least 500ms.
  *         Both BMS1 and BMS2 chips share the same reset line.
  */
HAL_StatusTypeDef BQ_ResetChips(void)
{
    // Set BMS_RESET pin high to trigger reset
    HAL_GPIO_WritePin(BMS_RESET_GPIO_Port, BMS_RESET_Pin, GPIO_PIN_SET);
    
    // Hold reset high for 500ms (as specified)
    osDelay(500);
    
    // Release reset by setting pin low
    HAL_GPIO_WritePin(BMS_RESET_GPIO_Port, BMS_RESET_Pin, GPIO_PIN_RESET);
    
    // Brief delay to allow chips to complete reset sequence
    osDelay(100);
    
    return HAL_OK;
}

/**
  * @brief  Wake both BQ76952 chips and disable sleep mode
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure
  * @note   Sends SLEEP_DISABLE subcommand to BMS1 and BMS2.
  *         This function is intended for startup use before RTOS scheduling begins.
  */
HAL_StatusTypeDef BQ_WakeChips(void)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[3];
    volatile uint32_t delay_count;

    tx_buf[0] = 0x3E;
    tx_buf[1] = (uint8_t)(SLEEP_DISABLE & 0xFF);
    tx_buf[2] = (uint8_t)((SLEEP_DISABLE >> 8) & 0xFF);

    status = HAL_I2C_Master_Transmit(&hi2c1, (BQ76952_I2C_ADDR_BMS1 << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    for (delay_count = 0; delay_count < 20000U; delay_count++) {
        __NOP();
    }

    status = HAL_I2C_Master_Transmit(&hi2c3, (BQ76952_I2C_ADDR_BMS2 << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    for (delay_count = 0; delay_count < 20000U; delay_count++) {
        __NOP();
    }

    return HAL_OK;
}

/**
  * @brief  Wake both BQ76952 chips and disable sleep mode (RTOS-safe)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure
  * @note   Uses I2C mutexes and osDelay, intended for runtime use after scheduler start.
  */
HAL_StatusTypeDef BQ_WakeChipsRTOS(void)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[3];

    tx_buf[0] = 0x3E;
    tx_buf[1] = (uint8_t)(SLEEP_DISABLE & 0xFF);
    tx_buf[2] = (uint8_t)((SLEEP_DISABLE >> 8) & 0xFF);

    if (I2C1Handle != NULL) {
        if (osMutexAcquire(I2C1Handle, I2C_TIMEOUT_MS) != osOK) {
            return HAL_ERROR;
        }
    }

    status = HAL_I2C_Master_Transmit(&hi2c1, (BQ76952_I2C_ADDR_BMS1 << 1), tx_buf, 3, I2C_TIMEOUT_MS);

    if (I2C1Handle != NULL) {
        osMutexRelease(I2C1Handle);
    }

    if (status != HAL_OK) {
        return status;
    }

    osDelay(2);

    if (I2C3Handle != NULL) {
        if (osMutexAcquire(I2C3Handle, I2C_TIMEOUT_MS) != osOK) {
            return HAL_ERROR;
        }
    }

    status = HAL_I2C_Master_Transmit(&hi2c3, (BQ76952_I2C_ADDR_BMS2 << 1), tx_buf, 3, I2C_TIMEOUT_MS);

    if (I2C3Handle != NULL) {
        osMutexRelease(I2C3Handle);
    }

    return status;
}

/**
  * @brief  Set BQ76952 power mode for both chips (NORMAL or SLEEP)
  * @param  mode: Desired power mode (BQ_MODE_NORMAL or BQ_MODE_SLEEP)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR if either chip fails
  * @note   Sends SLEEP_ENABLE or SLEEP_DISABLE to both BMS1 and BMS2.
  *         If the second chip fails, the first chip is rolled back.
  */
HAL_StatusTypeDef BQ_SetPowerMode(BQ_PowerMode_t mode)
{
    HAL_StatusTypeDef status;
    uint16_t subcmd = (mode == BQ_MODE_SLEEP) ? SLEEP_ENABLE : SLEEP_DISABLE;
    uint16_t rollback_subcmd = (mode == BQ_MODE_SLEEP) ? SLEEP_DISABLE : SLEEP_ENABLE;

    // Send to BMS1
    if (I2C1Handle != NULL) {
        if (osMutexAcquire(I2C1Handle, I2C_TIMEOUT_MS) != osOK) {
            return HAL_ERROR;
        }
    }
    status = BQ76952_SendSubcommand(&hi2c1, BQ76952_I2C_ADDR_BMS1, subcmd);
    if (I2C1Handle != NULL) {
        osMutexRelease(I2C1Handle);
    }
    if (status != HAL_OK) {
        return status;
    }

    osDelay(2);

    // Send to BMS2
    if (I2C3Handle != NULL) {
        if (osMutexAcquire(I2C3Handle, I2C_TIMEOUT_MS) != osOK) {
            // Roll back BMS1
            if (I2C1Handle != NULL) {
                osMutexAcquire(I2C1Handle, I2C_TIMEOUT_MS);
            }
            BQ76952_SendSubcommand(&hi2c1, BQ76952_I2C_ADDR_BMS1, rollback_subcmd);
            if (I2C1Handle != NULL) {
                osMutexRelease(I2C1Handle);
            }
            return HAL_ERROR;
        }
    }
    status = BQ76952_SendSubcommand(&hi2c3, BQ76952_I2C_ADDR_BMS2, subcmd);
    if (I2C3Handle != NULL) {
        osMutexRelease(I2C3Handle);
    }

    if (status != HAL_OK) {
        // Roll back BMS1
        if (I2C1Handle != NULL) {
            osMutexAcquire(I2C1Handle, I2C_TIMEOUT_MS);
        }
        BQ76952_SendSubcommand(&hi2c1, BQ76952_I2C_ADDR_BMS1, rollback_subcmd);
        if (I2C1Handle != NULL) {
            osMutexRelease(I2C1Handle);
        }
        return status;
    }

    // Update cached mode
    if (power_mode_mutex != NULL) {
        osMutexAcquire(power_mode_mutex, osWaitForever);
    }
    g_bq_power_mode = mode;
    if (power_mode_mutex != NULL) {
        osMutexRelease(power_mode_mutex);
    }

    return HAL_OK;
}

/**
  * @brief  Get current BQ76952 power mode (cached value)
  * @retval BQ_PowerMode_t: Current power mode
  */
BQ_PowerMode_t BQ_GetPowerMode(void)
{
    BQ_PowerMode_t mode;
    if (power_mode_mutex != NULL) {
        osMutexAcquire(power_mode_mutex, osWaitForever);
    }
    mode = g_bq_power_mode;
    if (power_mode_mutex != NULL) {
        osMutexRelease(power_mode_mutex);
    }
    return mode;
}

/**
  * @brief  Read actual power mode from a BQ76952 chip via Battery Status register
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  mode: Pointer to store detected power mode
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Reads Battery Status (0x12) bit 15 to determine if chip is in SLEEP.
  *         Caller must hold the appropriate I2C mutex.
  */
HAL_StatusTypeDef BQ_ReadChipPowerMode(I2C_HandleTypeDef *hi2c, uint8_t device_addr, BQ_PowerMode_t *mode)
{
    HAL_StatusTypeDef status;
    uint16_t batt_status = 0;

    if (mode == NULL) {
        return HAL_ERROR;
    }

    status = BQ76952_ReadRegister16(hi2c, device_addr, BatteryStatus, &batt_status);
    if (status != HAL_OK) {
        return status;
    }

    *mode = (batt_status & BQ_BATT_STATUS_SLEEP_BIT) ? BQ_MODE_SLEEP : BQ_MODE_NORMAL;
    return HAL_OK;
}

/**
  * @brief  Initialize BQ76952 power mode at startup
  * @retval HAL_StatusTypeDef: HAL_OK on success
  * @note   Creates power mode mutex, reads actual mode from both chips,
  *         and forces BQ_DEFAULT_POWER_MODE if needed.
  */
HAL_StatusTypeDef BQ_InitPowerMode(void)
{
    BQ_PowerMode_t bms1_mode, bms2_mode;
    HAL_StatusTypeDef status;

    // Create power mode mutex
    const osMutexAttr_t pm_mutex_attr = {
        .name = "PowerModeMutex",
        .attr_bits = osMutexPrioInherit,
        .cb_mem = NULL,
        .cb_size = 0U
    };
    power_mode_mutex = osMutexNew(&pm_mutex_attr);

    // Read actual mode from BMS1
    if (I2C1Handle != NULL) {
        osMutexAcquire(I2C1Handle, I2C_TIMEOUT_MS);
    }
    status = BQ_ReadChipPowerMode(&hi2c1, BQ76952_I2C_ADDR_BMS1, &bms1_mode);
    if (I2C1Handle != NULL) {
        osMutexRelease(I2C1Handle);
    }

    // Read actual mode from BMS2
    if (I2C3Handle != NULL) {
        osMutexAcquire(I2C3Handle, I2C_TIMEOUT_MS);
    }
    if (status == HAL_OK) {
        status = BQ_ReadChipPowerMode(&hi2c3, BQ76952_I2C_ADDR_BMS2, &bms2_mode);
    }
    if (I2C3Handle != NULL) {
        osMutexRelease(I2C3Handle);
    }

    // Force default mode regardless (ensures both chips are synchronized)
    BQ_SetPowerMode(BQ_DEFAULT_POWER_MODE);

    return HAL_OK;
}

/**
  * @brief  Get last I2C1 error code for diagnostics
  * @retval uint32_t: HAL I2C error code
  */
uint32_t BQ_GetLastI2C1Error(void)
{
    return g_last_i2c1_error;
}

/**
  * @brief  Get last I2C3 error code for diagnostics
  * @retval uint32_t: HAL I2C error code
  */
uint32_t BQ_GetLastI2C3Error(void)
{
    return g_last_i2c3_error;
}

/**
  * @brief  Get I2C error and recovery statistics
  * @param  i2c1_errors: Pointer to store I2C1 error count (can be NULL)
  * @param  i2c1_recoveries: Pointer to store I2C1 recovery count (can be NULL)
  * @param  i2c3_errors: Pointer to store I2C3 error count (can be NULL)
  * @param  i2c3_recoveries: Pointer to store I2C3 recovery count (can be NULL)
  */
void BQ_GetI2CStats(uint32_t *i2c1_errors, uint32_t *i2c1_recoveries,
                    uint32_t *i2c3_errors, uint32_t *i2c3_recoveries)
{
    if (i2c1_errors) *i2c1_errors = g_i2c1_error_count;
    if (i2c1_recoveries) *i2c1_recoveries = g_i2c1_recovery_count;
    if (i2c3_errors) *i2c3_errors = g_i2c3_error_count;
    if (i2c3_recoveries) *i2c3_recoveries = g_i2c3_recovery_count;
}

/**
  * @brief  Get BMS1 internal die temperature
  * @retval int16_t: Temperature in 0.1°C units (e.g., 250 = 25.0°C)
  * @note   Updated every 5 seconds by BQ_MonitorTask
  */
int16_t BQ_GetBMS1InternalTemp(void)
{
    return g_bms1_internal_temp;
}

/**
  * @brief  Get BMS2 internal die temperature
  * @retval int16_t: Temperature in 0.1°C units (e.g., 250 = 25.0°C)
  * @note   Updated every 5 seconds by BQ_MonitorTask_BMS2
  */
int16_t BQ_GetBMS2InternalTemp(void)
{
    return g_bms2_internal_temp;
}

/**
  * @brief  Set minimum cell voltage threshold for under-voltage detection
  * @param  min_voltage_mv: Minimum voltage in millivolts
  * @retval None
  * @note   This is a temporary setting that resets to CELL_VOLTAGE_MIN_MV (2500mV) on device reset.
  */
void BQ_SetMinVoltage(uint16_t min_voltage_mv)
{
    g_cell_voltage_min_mv = min_voltage_mv;
}

/**
  * @brief  Get current minimum cell voltage threshold
  * @retval Minimum voltage threshold in millivolts
  */
uint16_t BQ_GetMinVoltage(void)
{
    return g_cell_voltage_min_mv;
}

/**
  * @brief  Set maximum cell voltage threshold for over-voltage detection
  * @param  max_voltage_mv: Maximum voltage in millivolts
  * @retval None
  * @note   This is a temporary setting that resets to CELL_VOLTAGE_MAX_MV (4200mV) on device reset.
  */
void BQ_SetMaxVoltage(uint16_t max_voltage_mv)
{
    g_cell_voltage_max_mv = max_voltage_mv;
}

/**
  * @brief  Get current maximum cell voltage threshold
  * @retval Maximum voltage threshold in millivolts
  */
uint16_t BQ_GetMaxVoltage(void)
{
    return g_cell_voltage_max_mv;
}

/* Cell Balancing Functions --------------------------------------------------*/

/**
  * @brief  Write a subcommand with data to BQ76952 (RAM-type subcommand)
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  subcmd: Subcommand address (16-bit)
  * @param  data: Data to write (16-bit)
  * @retval HAL_StatusTypeDef
  * @note   For RAM-type subcommands like CB_ACTIVE_CELLS:
  *         1. Write subcommand to 0x3E/0x3F
  *         2. Write data to 0x40/0x41
  *         3. Write checksum to 0x60 and length to 0x61
  */
static HAL_StatusTypeDef BQ76952_WriteSubcommand(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                                  uint16_t subcmd, uint16_t data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[4];
    uint8_t checksum;
    
    // Step 1: Write subcommand address to 0x3E/0x3F
    tx_buf[0] = 0x3E;                           // Register address for subcommand
    tx_buf[1] = (uint8_t)(subcmd & 0xFF);       // Subcommand LSB
    tx_buf[2] = (uint8_t)((subcmd >> 8) & 0xFF); // Subcommand MSB
    
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    // Step 2: Write data to 0x40/0x41
    tx_buf[0] = 0x40;                           // Data buffer register
    tx_buf[1] = (uint8_t)(data & 0xFF);         // Data LSB
    tx_buf[2] = (uint8_t)((data >> 8) & 0xFF);  // Data MSB
    
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    // Step 3: Compute checksum = ~(sum of all data bytes) & 0xFF
    // Data includes: subcmd_lo, subcmd_hi, data_lo, data_hi
    checksum = (uint8_t)(subcmd & 0xFF);
    checksum += (uint8_t)((subcmd >> 8) & 0xFF);
    checksum += (uint8_t)(data & 0xFF);
    checksum += (uint8_t)((data >> 8) & 0xFF);
    checksum = ~checksum;
    
    // Step 4: Write checksum and length to 0x60/0x61
    // Length = 4 bytes (subcmd_lo, subcmd_hi, data_lo, data_hi) + 4 = 0x06
    tx_buf[0] = 0x60;
    tx_buf[1] = checksum;
    tx_buf[2] = 0x06;  // Length: 2 (subcmd) + 2 (data) + 2 (checksum+length) = 6
    
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    
    return status;
}

/**
  * @brief  Read subcommand result from BQ76952
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  subcmd: Subcommand address (16-bit)
  * @param  data: Pointer to store result (16-bit)
  * @retval HAL_StatusTypeDef
  * @note   Uses indirect addressing: write subcmd to 0x3E/0x3F, wait, read from 0x40
  */
static HAL_StatusTypeDef BQ76952_ReadSubcommand(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                                 uint16_t subcmd, uint16_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[3];
    uint8_t rx_buf[2];
    uint8_t reg_addr;
    
    // Write subcommand address
    tx_buf[0] = 0x3E;                           // Register address for subcommand
    tx_buf[1] = (uint8_t)(subcmd & 0xFF);       // Subcommand LSB
    tx_buf[2] = (uint8_t)((subcmd >> 8) & 0xFF); // Subcommand MSB
    
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    // Small delay for BQ76952 to process (1ms typical)
    osDelay(2);
    
    // Read result from transfer buffer at 0x40
    reg_addr = 0x40;
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), &reg_addr, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    status = HAL_I2C_Master_Receive(hi2c, (device_addr << 1), rx_buf, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    // Combine bytes (LSB first)
    *data = (uint16_t)rx_buf[0] | ((uint16_t)rx_buf[1] << 8);
    
    return HAL_OK;
}

/**
  * @brief  Send a command-only subcommand to BQ76952
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  subcmd: Command subcommand address (16-bit)
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef BQ76952_SendSubcommand(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                                 uint16_t subcmd)
{
    uint8_t tx_buf[3];
    
    tx_buf[0] = 0x3E;                           // Register address for subcommand
    tx_buf[1] = (uint8_t)(subcmd & 0xFF);       // Subcommand LSB
    tx_buf[2] = (uint8_t)((subcmd >> 8) & 0xFF); // Subcommand MSB
    
    return HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), tx_buf, 3, I2C_TIMEOUT_MS);
}

/**
  * @brief  Write two bytes to BQ76952 data memory
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  addr: Data memory address (16-bit)
  * @param  data: Data word to write (16-bit, little-endian)
  * @retval HAL_StatusTypeDef
  * @note   Uses indirect addressing: write addr to 0x3E/0x3F, data to 0x40/0x41, checksum+length to 0x60/0x61
  */
static HAL_StatusTypeDef BQ76952_WriteDataMemoryWord(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                                      uint16_t addr, uint16_t data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[4];
    uint8_t checksum;
    
    // Step 1: Write data memory address to 0x3E/0x3F
    tx_buf[0] = 0x3E;                           // Register address for subcommand
    tx_buf[1] = (uint8_t)(addr & 0xFF);         // Address LSB
    tx_buf[2] = (uint8_t)((addr >> 8) & 0xFF);  // Address MSB
    
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    // Step 2: Write data word to 0x40/0x41
    tx_buf[0] = 0x40;
    tx_buf[1] = (uint8_t)(data & 0xFF);         // Data LSB
    tx_buf[2] = (uint8_t)((data >> 8) & 0xFF);  // Data MSB
    
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    // Step 3: Compute checksum = ~(sum of addr_lo + addr_hi + data_lo + data_hi) & 0xFF
    checksum = (uint8_t)(addr & 0xFF);
    checksum += (uint8_t)((addr >> 8) & 0xFF);
    checksum += (uint8_t)(data & 0xFF);
    checksum += (uint8_t)((data >> 8) & 0xFF);
    checksum = ~checksum;
    
    // Step 4: Write checksum and length to 0x60/0x61
    // Length = 2 (addr) + 2 (data) + 2 (checksum+length) = 6
    tx_buf[0] = 0x60;
    tx_buf[1] = checksum;
    tx_buf[2] = 0x06;
    
    status = HAL_I2C_Master_Transmit(hi2c, (device_addr << 1), tx_buf, 3, I2C_TIMEOUT_MS);
    
    return status;
}

/**
  * @brief  Configure CB_LOOP_SLOW for maximum balancing current
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Sets CB_LOOP_SLOW to 3 (eighth-speed mode) in Power Config register
  *         This provides maximum average balancing current by slowing the ADC
  *         scan loop while balancing is active.
  *         Power Config register (0x9234):
  *           Bits 5-4: CB_LOOP_SLOW (0=full, 1=half, 2=quarter, 3=eighth)
  *         Default value is 0x2982, we set to 0x29B2 (bits 5-4 = 11)
  *         Requires entering CONFIG_UPDATE mode.
  */
HAL_StatusTypeDef BQ_ConfigureBalancingSpeed(I2C_HandleTypeDef *hi2c, uint8_t device_addr)
{
    HAL_StatusTypeDef status;
    osMutexId_t mutex = (hi2c == &hi2c1) ? I2C1Handle : I2C3Handle;
    
    if (osMutexAcquire(mutex, osWaitForever) != osOK) {
        return HAL_ERROR;
    }
    
    // Step 1: Enter CONFIG_UPDATE mode
    status = BQ76952_SendSubcommand(hi2c, device_addr, SET_CFGUPDATE);
    if (status != HAL_OK) {
        osMutexRelease(mutex);
        return status;
    }
    
    // Wait for CONFIG_UPDATE mode to be active (10ms recommended)
    osDelay(10);
    
    // Step 2: Write Power Config (0x9234) with CB_LOOP_SLOW = 3 (bits 5-4)
    // Power Config is a 16-bit register (H2 type)
    // Default value: 0x2982
    // We set CB_LOOP_SLOW bits [5:4] = 11 (3 = eighth speed)
    // New value: 0x2982 | (3 << 4) = 0x2982 | 0x0030 = 0x29B2
    uint16_t power_config = 0x29B2;  // Default with CB_LOOP_SLOW = 3
    
    status = BQ76952_WriteDataMemoryWord(hi2c, device_addr, PowerConfig, power_config);
    if (status != HAL_OK) {
        // Try to exit CONFIG_UPDATE mode anyway
        BQ76952_SendSubcommand(hi2c, device_addr, EXIT_CFGUPDATE);
        osDelay(10);
        osMutexRelease(mutex);
        return status;
    }
    
    // Step 3: Exit CONFIG_UPDATE mode
    status = BQ76952_SendSubcommand(hi2c, device_addr, EXIT_CFGUPDATE);
    
    // Wait for exit to complete
    osDelay(10);
    
    osMutexRelease(mutex);
    return status;
}

/**
  * @brief  Set active balancing cells on a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  cell_mask: 16-bit mask of cells to balance (bit 0 = cell 1, etc.)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Uses CB_ACTIVE_CELLS (0x0083) subcommand
  *         This resets the internal balance timer on each call
  */
HAL_StatusTypeDef BQ_SetBalanceCells(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t cell_mask)
{
    HAL_StatusTypeDef status;
    osMutexId_t mutex = (hi2c == &hi2c1) ? I2C1Handle : I2C3Handle;
    
    if (osMutexAcquire(mutex, osWaitForever) != osOK) {
        return HAL_ERROR;
    }
    
    // Write cell mask to CB_ACTIVE_CELLS (0x0083)
    status = BQ76952_WriteSubcommand(hi2c, device_addr, CB_ACTIVE_CELLS, cell_mask);
    
    osMutexRelease(mutex);
    return status;
}

/**
  * @brief  Read currently active balancing cells from a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  cell_mask: Pointer to store 16-bit mask of balancing cells
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_GetBalanceCells(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t *cell_mask)
{
    HAL_StatusTypeDef status;
    osMutexId_t mutex = (hi2c == &hi2c1) ? I2C1Handle : I2C3Handle;
    
    if (cell_mask == NULL) {
        return HAL_ERROR;
    }
    
    if (osMutexAcquire(mutex, osWaitForever) != osOK) {
        return HAL_ERROR;
    }
    
    // Read from CB_ACTIVE_CELLS (0x0083)
    status = BQ76952_ReadSubcommand(hi2c, device_addr, CB_ACTIVE_CELLS, cell_mask);
    
    osMutexRelease(mutex);
    return status;
}

/**
  * @brief  Check if cell balancing is active on a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  active: Pointer to store result (true = balancing active)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Checks bit 2 [CB] of Alarm Raw Status (0x64)
  */
HAL_StatusTypeDef BQ_IsBalancingActive(I2C_HandleTypeDef *hi2c, uint8_t device_addr, bool *active)
{
    HAL_StatusTypeDef status;
    uint16_t alarm_raw;
    osMutexId_t mutex = (hi2c == &hi2c1) ? I2C1Handle : I2C3Handle;
    
    if (active == NULL) {
        return HAL_ERROR;
    }
    
    if (osMutexAcquire(mutex, osWaitForever) != osOK) {
        return HAL_ERROR;
    }
    
    // Read Alarm Raw Status (0x64) - direct command
    status = BQ76952_ReadRegister16(hi2c, device_addr, AlarmRawStatus, &alarm_raw);
    
    osMutexRelease(mutex);
    
    if (status == HAL_OK) {
        // Bit 2 is CB (Cell Balancing) active flag
        *active = (alarm_raw & 0x0004) ? true : false;
    }
    
    return status;
}

/**
  * @brief  Stop all cell balancing on a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_StopBalancing(I2C_HandleTypeDef *hi2c, uint8_t device_addr)
{
    // Set cell mask to 0 to stop all balancing
    return BQ_SetBalanceCells(hi2c, device_addr, 0x0000);
}

/**
  * @brief  Read CBSTATUS registers from a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  cbstatus1: Pointer to store CBSTATUS1 (cells 1-8 actual balance state)
  * @param  cbstatus2: Pointer to store CBSTATUS2 (cells 9-16 actual balance state)
  * @param  cbstatus3: Pointer to store CBSTATUS3 (balance summary flags)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   CBSTATUS shows which cells are ACTUALLY balancing, not just commanded
  */
HAL_StatusTypeDef BQ_GetCBStatus(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                  uint8_t *cbstatus1, uint8_t *cbstatus2, uint8_t *cbstatus3)
{
    HAL_StatusTypeDef status;
    uint16_t result;
    osMutexId_t mutex = (hi2c == &hi2c1) ? I2C1Handle : I2C3Handle;
    
    if (osMutexAcquire(mutex, osWaitForever) != osOK) {
        return HAL_ERROR;
    }
    
    // Read CBSTATUS1 (0x0085) - which cells 1-8 are balancing
    if (cbstatus1 != NULL) {
        status = BQ76952_ReadSubcommand(hi2c, device_addr, CBSTATUS1, &result);
        if (status != HAL_OK) {
            osMutexRelease(mutex);
            return status;
        }
        *cbstatus1 = (uint8_t)(result & 0xFF);
    }
    
    // Read CBSTATUS2 (0x0086) - which cells 9-16 are balancing
    if (cbstatus2 != NULL) {
        status = BQ76952_ReadSubcommand(hi2c, device_addr, CBSTATUS2, &result);
        if (status != HAL_OK) {
            osMutexRelease(mutex);
            return status;
        }
        *cbstatus2 = (uint8_t)(result & 0xFF);
    }
    
    // Read CBSTATUS3 (0x0087) - balance summary/status flags
    if (cbstatus3 != NULL) {
        status = BQ76952_ReadSubcommand(hi2c, device_addr, CBSTATUS3, &result);
        if (status != HAL_OK) {
            osMutexRelease(mutex);
            return status;
        }
        *cbstatus3 = (uint8_t)(result & 0xFF);
    }
    
    osMutexRelease(mutex);
    return HAL_OK;
}

/**
  * @brief  Read AlarmRawStatus register from a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  alarm_status: Pointer to store AlarmRawStatus (16-bit)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_GetAlarmRawStatus(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t *alarm_status)
{
    HAL_StatusTypeDef status;
    osMutexId_t mutex = (hi2c == &hi2c1) ? I2C1Handle : I2C3Handle;
    
    if (alarm_status == NULL) {
        return HAL_ERROR;
    }
    
    if (osMutexAcquire(mutex, osWaitForever) != osOK) {
        return HAL_ERROR;
    }
    
    // Read Alarm Raw Status (0x64) - direct register read
    status = BQ76952_ReadRegister16(hi2c, device_addr, AlarmRawStatus, alarm_status);
    
    osMutexRelease(mutex);
    return status;
}
