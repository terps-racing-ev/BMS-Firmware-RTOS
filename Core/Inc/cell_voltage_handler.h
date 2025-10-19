/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : cell_voltage_handler.h
  * @brief          : Cell voltage monitoring task header
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

#ifndef __CELL_VOLTAGE_HANDLER_H
#define __CELL_VOLTAGE_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "bq76952.h"

/* Defines -------------------------------------------------------------------*/
#define BQ76952_I2C_ADDR_BMS1       0x08    /**< BQ76952 I2C address for BMS1 (7-bit, default address) */
#define BQ76952_I2C_ADDR_BMS2       0x09    /**< BQ76952 I2C address for BMS2 (alternative address) */

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
  * @brief  Cell voltage data structure
  */
typedef struct {
    uint16_t cell_voltage_mv[BMS1_NUM_CELLS];  /**< Cell voltages in millivolts (cells 1-9) */
    uint32_t last_update_tick;                  /**< Timestamp of last successful read */
    uint8_t valid;                              /**< Data validity flag (1 = valid, 0 = invalid) */
} CellVoltage_Data_t;

/**
  * @brief  Cell voltage data structure for BMS2
  */
typedef struct {
    uint16_t cell_voltage_mv[BMS2_NUM_CELLS];  /**< Cell voltages in millivolts (cells 10-18) */
    uint32_t last_update_tick;                  /**< Timestamp of last successful read */
    uint8_t valid;                              /**< Data validity flag (1 = valid, 0 = invalid) */
} CellVoltage_Data_BMS2_t;

/* Exported variables --------------------------------------------------------*/
extern osMutexId_t I2C1Handle;
extern osMutexId_t I2C3Handle;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Cell voltage monitoring task (RTOS task function)
  * @param  argument: Not used
  * @retval None
  * @note   Reads cells 1-9 from BMS1 on I2C1
  */
void CellVoltage_MonitorTask(void *argument);

/**
  * @brief  Cell voltage monitoring task for BMS2 (RTOS task function)
  * @param  argument: Not used
  * @retval None
  * @note   Reads cells 10-18 from BMS2 on I2C3
  */
void CellVoltage_MonitorTask_BMS2(void *argument);

/**
  * @brief  Read cell voltages from BQ76952 chip on I2C1 (BMS1)
  * @param  data: Pointer to structure to store voltage data
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef CellVoltage_ReadBMS1(CellVoltage_Data_t *data);

/**
  * @brief  Read cell voltages from BQ76952 chip on I2C3 (BMS2)
  * @param  data: Pointer to structure to store voltage data
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef CellVoltage_ReadBMS2(CellVoltage_Data_BMS2_t *data);

/**
  * @brief  Read single cell voltage from BQ76952
  * @param  hi2c: I2C handle
  * @param  device_addr: BQ76952 I2C device address
  * @param  cell_num: Cell number (1-16)
  * @param  voltage_mv: Pointer to store voltage in millivolts
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef CellVoltage_ReadCell(I2C_HandleTypeDef *hi2c, uint8_t device_addr, 
                                       uint8_t cell_num, uint16_t *voltage_mv);

/**
  * @brief  Send cell voltage data via CAN bus
  * @param  data: Pointer to voltage data structure
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Sends BMS1 data (cells 1-9) in 3 CAN messages
  */
HAL_StatusTypeDef CellVoltage_SendCANMessage(CellVoltage_Data_t *data);

/**
  * @brief  Send BMS2 cell voltage data via CAN bus
  * @param  data: Pointer to BMS2 voltage data structure
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  * @note   Sends BMS2 data (cells 10-18) in 3 CAN messages
  */
HAL_StatusTypeDef CellVoltage_SendCANMessage_BMS2(CellVoltage_Data_BMS2_t *data);

/**
  * @brief  Get current cell voltage data (thread-safe)
  * @param  data: Pointer to structure to copy data into
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef CellVoltage_GetData(CellVoltage_Data_t *data);

/**
  * @brief  Get current BMS2 cell voltage data (thread-safe)
  * @param  data: Pointer to structure to copy BMS2 data into
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on failure
  */
HAL_StatusTypeDef CellVoltage_GetData_BMS2(CellVoltage_Data_BMS2_t *data);

/**
  * @brief  Check cell voltages against limits and set error/warning flags
  * @param  data: Pointer to voltage data structure
  * @retval None
  * @note   Sets ERROR_OVER_VOLTAGE or ERROR_UNDER_VOLTAGE if out of range
  *         Sets WARNING_HIGH_VOLTAGE or WARNING_LOW_VOLTAGE if approaching limits
  */
void CellVoltage_CheckLimits(CellVoltage_Data_t *data);

/**
  * @brief  Check BMS2 cell voltages against limits and set error/warning flags
  * @param  data: Pointer to BMS2 voltage data structure
  * @retval None
  * @note   Sets ERROR_OVER_VOLTAGE or ERROR_UNDER_VOLTAGE if out of range
  *         Sets WARNING_HIGH_VOLTAGE or WARNING_LOW_VOLTAGE if approaching limits
  */
void CellVoltage_CheckLimits_BMS2(CellVoltage_Data_BMS2_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __CELL_VOLTAGE_HANDLER_H */
