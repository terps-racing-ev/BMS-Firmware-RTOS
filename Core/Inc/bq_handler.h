/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bq_handler.h
  * @brief          : BQ76952 monitoring task header
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

#ifndef __BQ_HANDLER_H
#define __BQ_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "bq76952.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define BQ76952_I2C_ADDR_BMS1       0x08    /**< BQ76952 I2C address for BMS1 (7-bit, default address) */
#define BQ76952_I2C_ADDR_BMS2       0x08    /**< BQ76952 I2C address for BMS2 (alternative address) */

#define BMS1_NUM_CELLS              9       /**< Number of cells monitored by BMS1 (cells 1-9) */
#define BMS2_NUM_CELLS              9       /**< Number of cells monitored by BMS2 (cells 10-18) */

#define VOLTAGE_READ_INTERVAL_MS    250     /**< Cell voltage reading interval (legacy, use mode-specific defines) */
#define VOLTAGE_CAN_INTERVAL_MS     500     /**< CAN transmission interval (legacy, use mode-specific defines) */

#define I2C_TIMEOUT_MS              100     /**< I2C communication timeout */

#define BQ_FAULT_REPORTING_DEFAULT  1       /**< Default fault reporting state (1=enabled, 0=disabled) */

/* BQ76952 Power Mode Configuration ------------------------------------------*/
#define BQ_DEFAULT_POWER_MODE       BQ_MODE_SLEEP  /**< Default power mode at startup */
#define BQ_NORMAL_READ_INTERVAL_MS  500     /**< Cell voltage read interval in NORMAL mode (ms) */
#define BQ_NORMAL_CAN_INTERVAL_MS   500     /**< CAN reporting interval in NORMAL mode (ms) */
#define BQ_SLEEP_READ_INTERVAL_MS   2000    /**< Cell voltage read interval in SLEEP mode (ms) */
#define BQ_SLEEP_CAN_INTERVAL_MS    2000    /**< CAN reporting interval in SLEEP mode (ms) */

/* Voltage Limits (in millivolts) */
#define CELL_VOLTAGE_MIN_MV         2500    /**< Minimum safe cell voltage (2.5V) */
#define CELL_VOLTAGE_MAX_MV         4200    /**< Maximum safe cell voltage (4.2V) */
#define CELL_VOLTAGE_WARNING_LOW_MV 3000    /**< Low voltage warning threshold (2.8V) */
#define CELL_VOLTAGE_WARNING_HIGH_MV 4200   /**< High voltage warning threshold (4.2V) */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  BQ76952 power mode enumeration
  */
typedef enum {
    BQ_MODE_NORMAL = 0,  /**< NORMAL mode: full measurement rate, higher power */
    BQ_MODE_SLEEP  = 1   /**< SLEEP mode: reduced rate with wake-read-sleep, lower power */
} BQ_PowerMode_t;

/**
  * @brief  BQ76952 voltage data structure
  */
typedef struct {
    uint16_t cell_voltage_mv[BMS1_NUM_CELLS];  /**< Cell voltages in millivolts (cells 1-9) */
    uint32_t last_update_tick;                  /**< Timestamp of last successful read */
    uint8_t valid;                              /**< Data validity flag (1 = valid, 0 = invalid) */
} BQ_Data_t;

/**
  * @brief  BQ76952 voltage data structure for BMS2
  */
typedef struct {
    uint16_t cell_voltage_mv[BMS2_NUM_CELLS];  /**< Cell voltages in millivolts (cells 10-18) */
    uint32_t last_update_tick;                  /**< Timestamp of last successful read */
    uint8_t valid;                              /**< Data validity flag (1 = valid, 0 = invalid) */
} BQ_Data_BMS2_t;

/* Exported variables --------------------------------------------------------*/
extern osMutexId_t I2C1Handle;
extern osMutexId_t I2C3Handle;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  BQ76952 monitoring task (RTOS task function)
  * @param  argument: Not used
  * @retval None
  * @note   Reads cells 1-9 from BMS1 on I2C1
  */
void BQ_MonitorTask(void *argument);

/**
  * @brief  BQ76952 monitoring task for BMS2 (RTOS task function)
  * @param  argument: Not used
  * @retval None
  * @note   Reads cells 10-18 from BMS2 on I2C3
  */
void BQ_MonitorTask_BMS2(void *argument);

/**
  * @brief  Read cell voltages from BQ76952 chip on I2C1 (BMS1)
  * @param  data: Pointer to structure to store voltage data
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_ReadBMS1(BQ_Data_t *data);

/**
  * @brief  Get last I2C1 error code for diagnostics
  * @retval uint32_t: HAL I2C error code
  */
uint32_t BQ_GetLastI2C1Error(void);

/**
  * @brief  Get last I2C3 error code for diagnostics
  * @retval uint32_t: HAL I2C error code
  */
uint32_t BQ_GetLastI2C3Error(void);

/**
  * @brief  Get I2C error and recovery statistics
  * @param  i2c1_errors: Pointer to store I2C1 error count (can be NULL)
  * @param  i2c1_recoveries: Pointer to store I2C1 recovery count (can be NULL)
  * @param  i2c3_errors: Pointer to store I2C3 error count (can be NULL)
  * @param  i2c3_recoveries: Pointer to store I2C3 recovery count (can be NULL)
  */
void BQ_GetI2CStats(uint32_t *i2c1_errors, uint32_t *i2c1_recoveries,
                    uint32_t *i2c3_errors, uint32_t *i2c3_recoveries);

/**
  * @brief  Read cell voltages from BQ76952 chip on I2C3 (BMS2)
  * @param  data: Pointer to structure to store voltage data
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_ReadBMS2(BQ_Data_BMS2_t *data);

/**
  * @brief  Read single cell voltage from BQ76952
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address
  * @param  cell_num: Cell number (1-16)
  * @param  voltage_mv: Pointer to store voltage in millivolts
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_ReadCell(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                              uint8_t cell_num, uint16_t *voltage_mv);

/**
  * @brief  Send cell voltage data via CAN bus
  * @param  data: Pointer to voltage data structure
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Sends BMS1 data (cells 1-9) in 3 CAN messages
  */
HAL_StatusTypeDef BQ_SendCANMessage(BQ_Data_t *data);

/**
  * @brief  Send BMS2 cell voltage data via CAN bus
  * @param  data: Pointer to BMS2 voltage data structure
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Sends BMS2 data (cells 10-18) in 3 CAN messages
  */
HAL_StatusTypeDef BQ_SendCANMessage_BMS2(BQ_Data_BMS2_t *data);

/**
  * @brief  Get current cell voltage data (thread-safe)
  * @param  data: Pointer to structure to copy data into
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_GetData(BQ_Data_t *data);

/**
  * @brief  Get current BMS2 cell voltage data (thread-safe)
  * @param  data: Pointer to structure to copy BMS2 data into
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_GetData_BMS2(BQ_Data_BMS2_t *data);

/**
  * @brief  Check cell voltages against limits and set error/warning flags
  * @param  data: Pointer to voltage data structure
  * @retval None
  * @note   Sets ERROR_OVER_VOLTAGE or ERROR_UNDER_VOLTAGE if out of range
  *         Sets WARNING_HIGH_VOLTAGE or WARNING_LOW_VOLTAGE if approaching limits
  */
void BQ_CheckLimits(BQ_Data_t *data);

/**
  * @brief  Check BMS2 cell voltages against limits and set error/warning flags
  * @param  data: Pointer to BMS2 voltage data structure
  * @retval None
  * @note   Sets ERROR_OVER_VOLTAGE or ERROR_UNDER_VOLTAGE if out of range
  *         Sets WARNING_HIGH_VOLTAGE or WARNING_LOW_VOLTAGE if approaching limits
  */
void BQ_CheckLimits_BMS2(BQ_Data_BMS2_t *data);

/**
  * @brief  Reset BQ76952 chips by pulsing PB6 (BMS_RESET) high for 500ms
  * @retval HAL_StatusTypeDef: HAL_OK on success
  * @note   This function pulses the hardware reset pin connected to both BQ76952 chips.
  *         The reset pin is active-high and should be held high for at least 500ms.
  */
HAL_StatusTypeDef BQ_ResetChips(void);

/**
  * @brief  Wake both BQ76952 chips and disable sleep mode
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure
  * @note   Sends SLEEP_DISABLE subcommand to BMS1 and BMS2 over I2C.
  *         Intended to be called during startup before RTOS tasks begin.
  */
HAL_StatusTypeDef BQ_WakeChips(void);

/**
  * @brief  Wake both BQ76952 chips and disable sleep mode (RTOS-safe)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure
  * @note   Uses I2C mutexes and osDelay, intended for runtime use after scheduler start.
  */
HAL_StatusTypeDef BQ_WakeChipsRTOS(void);

/**
  * @brief  Set BQ76952 power mode for both chips (NORMAL or SLEEP)
  * @param  mode: Desired power mode (BQ_MODE_NORMAL or BQ_MODE_SLEEP)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR if either chip fails
  * @note   Sends SLEEP_ENABLE or SLEEP_DISABLE to both BMS1 and BMS2.
  *         If the second chip fails, the first chip is rolled back.
  *         Both chips are always kept in the same mode.
  */
HAL_StatusTypeDef BQ_SetPowerMode(BQ_PowerMode_t mode);

/**
  * @brief  Get current BQ76952 power mode (cached value)
  * @retval BQ_PowerMode_t: Current power mode
  */
BQ_PowerMode_t BQ_GetPowerMode(void);

/**
  * @brief  Read actual power mode from a BQ76952 chip via Battery Status register
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  mode: Pointer to store detected power mode
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Reads Battery Status (0x12) bit 15 to determine if chip is in SLEEP.
  *         Caller must hold the appropriate I2C mutex.
  */
HAL_StatusTypeDef BQ_ReadChipPowerMode(I2C_HandleTypeDef *hi2c, uint8_t device_addr, BQ_PowerMode_t *mode);

/**
  * @brief  Initialize BQ76952 power mode at startup
  * @retval HAL_StatusTypeDef: HAL_OK on success
  * @note   Reads actual mode from both chips and forces BQ_DEFAULT_POWER_MODE.
  *         Must be called after RTOS scheduler starts (uses mutexes and osDelay).
  */
HAL_StatusTypeDef BQ_InitPowerMode(void);

/**
  * @brief  Enable or disable fault reporting from BQ handler
  * @param  enabled: 1 to enable fault reporting, 0 to disable
  * @retval None
  * @note   When disabled, no errors or warnings will be set by the BQ handler.
  *         Useful for testing/debugging without triggering system faults.
  */
void BQ_SetFaultReportingEnabled(uint8_t enabled);

/**
  * @brief  Get current fault reporting state
  * @retval uint8_t: 1 if enabled, 0 if disabled
  */
uint8_t BQ_GetFaultReportingEnabled(void);

/**
  * @brief  Send BMS chip status via CAN (stack voltage, alarm status, TS2 temperature)
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address
  * @param  can_id: CAN message ID to use
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_SendChipStatus(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint32_t can_id);

/**
  * @brief  Get BMS1 internal die temperature
  * @retval int16_t: Temperature in 0.1°C units (e.g., 250 = 25.0°C)
  * @note   Updated every 5 seconds by BQ_MonitorTask
  */
int16_t BQ_GetBMS1InternalTemp(void);

/**
  * @brief  Get BMS2 internal die temperature
  * @retval int16_t: Temperature in 0.1°C units (e.g., 250 = 25.0°C)
  * @note   Updated every 5 seconds by BQ_MonitorTask_BMS2
  */
int16_t BQ_GetBMS2InternalTemp(void);

/**
  * @brief  Set minimum cell voltage threshold for under-voltage detection
  * @param  min_voltage_mv: Minimum voltage in millivolts
  * @retval None
  * @note   This is a temporary setting that resets to default (2500mV) on device reset.
  */
void BQ_SetMinVoltage(uint16_t min_voltage_mv);

/**
  * @brief  Get current minimum cell voltage threshold
  * @retval Minimum voltage threshold in millivolts
  */
uint16_t BQ_GetMinVoltage(void);

/**
  * @brief  Set maximum cell voltage threshold for over-voltage detection
  * @param  max_voltage_mv: Maximum voltage in millivolts
  * @retval None
  * @note   This is a temporary setting that resets to default (4200mV) on device reset.
  */
void BQ_SetMaxVoltage(uint16_t max_voltage_mv);

/**
  * @brief  Get current maximum cell voltage threshold
  * @retval Maximum voltage threshold in millivolts
  */
uint16_t BQ_GetMaxVoltage(void);

/* Cell Balancing Functions --------------------------------------------------*/

/**
  * @brief  Configure CB_LOOP_SLOW for maximum balancing current
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Sets CB_LOOP_SLOW to 3 (eighth-speed mode) in Power Config register
  *         This slows ADC scan during balancing for higher average current.
  *         Should be called once before starting balancing.
  */
HAL_StatusTypeDef BQ_ConfigureBalancingSpeed(I2C_HandleTypeDef *hi2c, uint8_t device_addr);

/**
  * @brief  Set active balancing cells on a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  cell_mask: 16-bit mask of cells to balance (bit 0 = cell 1, etc.)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Uses CB_ACTIVE_CELLS (0x0083) subcommand
  *         This resets the internal balance timer on each call
  */
HAL_StatusTypeDef BQ_SetBalanceCells(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t cell_mask);

/**
  * @brief  Read currently active balancing cells from a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  cell_mask: Pointer to store 16-bit mask of balancing cells
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_GetBalanceCells(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t *cell_mask);

/**
  * @brief  Check if cell balancing is active on a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  active: Pointer to store result (true = balancing active)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Checks bit 2 [CB] of Alarm Raw Status (0x64)
  */
HAL_StatusTypeDef BQ_IsBalancingActive(I2C_HandleTypeDef *hi2c, uint8_t device_addr, bool *active);

/**
  * @brief  Stop all cell balancing on a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_StopBalancing(I2C_HandleTypeDef *hi2c, uint8_t device_addr);

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
                                  uint8_t *cbstatus1, uint8_t *cbstatus2, uint8_t *cbstatus3);

/**
  * @brief  Read AlarmRawStatus register from a BQ76952 chip
  * @param  hi2c: I2C handle (hi2c1 for BMS1, hi2c3 for BMS2)
  * @param  device_addr: BQ76952 I2C device address (7-bit)
  * @param  alarm_status: Pointer to store AlarmRawStatus (16-bit)
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_GetAlarmRawStatus(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t *alarm_status);

#ifdef __cplusplus
}
#endif

#endif /* __BQ_HANDLER_H */
