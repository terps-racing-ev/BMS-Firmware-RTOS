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

/* Defines -------------------------------------------------------------------*/
#define BQ76952_I2C_ADDR_BMS1       0x08    /**< BQ76952 I2C address for BMS1 (7-bit, default address) */
#define BQ76952_I2C_ADDR_BMS2       0x08    /**< BQ76952 I2C address for BMS2 (alternative address) */

#define BMS1_NUM_CELLS              9       /**< Number of cells monitored by BMS1 (cells 1-9) */
#define BMS2_NUM_CELLS              9       /**< Number of cells monitored by BMS2 (cells 10-18) */

#define VOLTAGE_READ_INTERVAL_MS    500     /**< Cell voltage reading interval (500ms = 2Hz) */
#define VOLTAGE_CAN_INTERVAL_MS     500     /**< CAN transmission interval for voltage data */

#define I2C_TIMEOUT_MS              100     /**< I2C communication timeout */

/* Voltage Limits (in millivolts) */
#define CELL_VOLTAGE_MIN_MV         2500    /**< Minimum safe cell voltage (2.5V) */
#define CELL_VOLTAGE_MAX_MV         4200    /**< Maximum safe cell voltage (4.2V) */
#define CELL_VOLTAGE_WARNING_LOW_MV 2800    /**< Low voltage warning threshold (2.8V) */
#define CELL_VOLTAGE_WARNING_HIGH_MV 4100   /**< High voltage warning threshold (4.1V) */

/* Exported types ------------------------------------------------------------*/

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
  * @brief  Get last I2C3 error code for diagnostics
  * @retval uint32_t: HAL I2C error code
  */
uint32_t BQ_GetLastI2C3Error(void);

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
  * @brief  Send BMS chip status via CAN (stack voltage, alarm status, TS2 temperature)
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address
  * @param  can_id: CAN message ID to use
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef BQ_SendChipStatus(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint32_t can_id);

#ifdef __cplusplus
}
#endif

#endif /* __BQ_HANDLER_H */
