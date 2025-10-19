/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : cell_voltage_handler.c
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
#include "cell_voltage_handler.h"
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

/* Private variables ---------------------------------------------------------*/
static CellVoltage_Data_t voltage_data_bms1 = {0};
static CellVoltage_Data_BMS2_t voltage_data_bms2 = {0};
static osMutexId_t voltage_mutex = NULL;
static osMutexId_t voltage_mutex_bms2 = NULL;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef BQ76952_ReadRegister16(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                                 uint16_t reg_addr, uint16_t *value);

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Cell voltage monitoring task (RTOS task function)
  * @param  argument: Not used
  * @retval None
  */
void CellVoltage_MonitorTask(void *argument)
{
    HAL_StatusTypeDef status;
    uint32_t last_read_tick = 0;
    uint32_t last_can_tick = 0;
    uint32_t current_tick;
    
    // Create mutex for thread-safe data access
    const osMutexAttr_t mutex_attr = {
        .name = "VoltageMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = NULL,
        .cb_size = 0U
    };
    
    voltage_mutex = osMutexNew(&mutex_attr);
    
    // Wait for system initialization
    osDelay(500);
    
    // Initialize timestamps
    last_read_tick = osKernelGetTickCount();
    last_can_tick = osKernelGetTickCount();
    
    /* Infinite loop */
    for(;;)
    {
        current_tick = osKernelGetTickCount();
        
        // Read cell voltages at specified interval
        if ((current_tick - last_read_tick) >= VOLTAGE_READ_INTERVAL_MS) {
            // Read BMS1 (cells 1-9 on I2C1)
            status = CellVoltage_ReadBMS1(&voltage_data_bms1);
            
            if (status != HAL_OK) {
                // Set I2C error flag
                ErrorMgr_SetError(ERROR_I2C_BMS1);
                
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
                // Clear I2C error flag on successful read
                ErrorMgr_ClearError(ERROR_I2C_BMS1);
                
                // Check voltage limits and set error/warning flags
                CellVoltage_CheckLimits(&voltage_data_bms1);
            }
            
            last_read_tick = current_tick;
        }
        
        // Send CAN messages at specified interval
        if ((current_tick - last_can_tick) >= VOLTAGE_CAN_INTERVAL_MS) {
            // Send voltage data via CAN (send even if invalid for debugging)
            // When I2C fails, the voltages will be 0 or old data
            CellVoltage_SendCANMessage(&voltage_data_bms1);
            
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
HAL_StatusTypeDef CellVoltage_ReadBMS1(CellVoltage_Data_t *data)
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
    
    // Read cells 1-9
    for (uint8_t i = 0; i < BMS1_NUM_CELLS; i++) {
        uint16_t voltage_mv = 0;
        
        status = CellVoltage_ReadCell(&hi2c1, BQ76952_I2C_ADDR_BMS1, i + 1, &voltage_mv);
        
        if (status == HAL_OK) {
            data->cell_voltage_mv[i] = voltage_mv;
        } else {
            // Communication error - mark data as invalid
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
HAL_StatusTypeDef CellVoltage_ReadCell(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
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
HAL_StatusTypeDef CellVoltage_SendCANMessage(CellVoltage_Data_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t can_data[8];
    
    if (data == NULL) {
        return HAL_ERROR;
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
HAL_StatusTypeDef CellVoltage_GetData(CellVoltage_Data_t *data)
{
    if (data == NULL) {
        return HAL_ERROR;
    }
    
    // Thread-safe copy
    if (voltage_mutex != NULL) {
        osMutexAcquire(voltage_mutex, osWaitForever);
    }
    
    memcpy(data, &voltage_data_bms1, sizeof(CellVoltage_Data_t));
    
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
void CellVoltage_CheckLimits(CellVoltage_Data_t *data)
{
    if (data == NULL || !data->valid) {
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
        if (voltage_mv > CELL_VOLTAGE_MAX_MV) {
            over_voltage_detected = true;
        }
        // Check for under-voltage error (below min limit)
        else if (voltage_mv < CELL_VOLTAGE_MIN_MV) {
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
void CellVoltage_MonitorTask_BMS2(void *argument)
{
    HAL_StatusTypeDef status;
    uint32_t last_read_tick = 0;
    uint32_t last_can_tick = 0;
    uint32_t current_tick;
    
    // Create mutex for thread-safe data access
    const osMutexAttr_t mutex_attr = {
        .name = "VoltageMutexBMS2",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = NULL,
        .cb_size = 0U
    };
    
    voltage_mutex_bms2 = osMutexNew(&mutex_attr);
    
    // Wait for system initialization
    osDelay(500);
    
    // Initialize timestamps
    last_read_tick = osKernelGetTickCount();
    last_can_tick = osKernelGetTickCount();
    
    /* Infinite loop */
    for(;;)
    {
        current_tick = osKernelGetTickCount();
        
        // Read cell voltages at specified interval
        if ((current_tick - last_read_tick) >= VOLTAGE_READ_INTERVAL_MS) {
            // Read BMS2 (cells 10-18 on I2C3)
            status = CellVoltage_ReadBMS2(&voltage_data_bms2);
            
            if (status != HAL_OK) {
                // Set I2C error flag
                ErrorMgr_SetError(ERROR_I2C_BMS2);
                
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
                // Clear I2C error flag on successful read
                ErrorMgr_ClearError(ERROR_I2C_BMS2);
                
                // Check voltage limits and set error/warning flags
                CellVoltage_CheckLimits_BMS2(&voltage_data_bms2);
            }
            
            last_read_tick = current_tick;
        }
        
        // Send CAN messages at specified interval
        if ((current_tick - last_can_tick) >= VOLTAGE_CAN_INTERVAL_MS) {
            // Send voltage data via CAN (send even if invalid for debugging)
            // When I2C fails, the voltages will be 0 or old data
            CellVoltage_SendCANMessage_BMS2(&voltage_data_bms2);
            
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
HAL_StatusTypeDef CellVoltage_ReadBMS2(CellVoltage_Data_BMS2_t *data)
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
    
    // Read cells 10-18 (map to BQ76952 cells 1-9 on second chip)
    for (uint8_t i = 0; i < BMS2_NUM_CELLS; i++) {
        uint16_t voltage_mv = 0;
        
        // Read from BQ76952 cells 1-9 (physical cells 10-18)
        status = CellVoltage_ReadCell(&hi2c3, BQ76952_I2C_ADDR_BMS2, i + 1, &voltage_mv);
        
        if (status == HAL_OK) {
            data->cell_voltage_mv[i] = voltage_mv;
        } else {
            // Communication error - mark data as invalid
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
HAL_StatusTypeDef CellVoltage_SendCANMessage_BMS2(CellVoltage_Data_BMS2_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t can_data[8];
    
    if (data == NULL) {
        return HAL_ERROR;
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
  * @brief  Get current BMS2 cell voltage data (thread-safe)
  * @param  data: Pointer to structure to copy BMS2 data into
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef CellVoltage_GetData_BMS2(CellVoltage_Data_BMS2_t *data)
{
    if (data == NULL) {
        return HAL_ERROR;
    }
    
    // Thread-safe copy
    if (voltage_mutex_bms2 != NULL) {
        osMutexAcquire(voltage_mutex_bms2, osWaitForever);
    }
    
    memcpy(data, &voltage_data_bms2, sizeof(CellVoltage_Data_BMS2_t));
    
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
void CellVoltage_CheckLimits_BMS2(CellVoltage_Data_BMS2_t *data)
{
    if (data == NULL || !data->valid) {
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
        if (voltage_mv > CELL_VOLTAGE_MAX_MV) {
            over_voltage_detected = true;
        }
        // Check for under-voltage error (below min limit)
        else if (voltage_mv < CELL_VOLTAGE_MIN_MV) {
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
