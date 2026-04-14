/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : config_manager.h
  * @brief          : BMS Configuration Manager
  ******************************************************************************
  * @attention
  *
  * This file manages configuration parameters for the BMS, including
  * module ID for CAN addressing.
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

#ifndef __CONFIG_MANAGER_H
#define __CONFIG_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
#define CONFIG_MODULE_ID_DEFAULT 0      // Default module ID if flash is uninitialized
#define CONFIG_MODULE_ID_MAX 15         // Maximum module ID (0xF)

/* Flash Storage Addresses ---------------------------------------------------*/
/* Module number stored at first word of reserved storage region (16KB at end of flash) */
/* Storage Region: 0x0803C000 - 0x0803FFFF (16KB reserved for config/data) */
#define CONFIG_FLASH_MODULE_ADDR    0x0803C000  // First word of storage region

/* Flash Magic Value (to detect if flash has been initialized) */
#define CONFIG_FLASH_MAGIC          0xBEEF0000  // Upper 16 bits
#define CONFIG_FLASH_MAGIC_MASK     0xFFFF0000
#define CONFIG_FLASH_MODULE_MASK    0x0000000F  // Lower 4 bits for module ID

/* CAN Command Definitions ---------------------------------------------------*/
#define CONFIG_CMD_SET_MODULE_ID 0x01   // Command to set module ID
#define CONFIG_CMD_SET_MAX_TEMP  0x02   // Command to set max thermistor temperature (°C)
#define CONFIG_CMD_SET_MIN_TEMP  0x03   // Command to set min thermistor temperature (°C, signed)
#define CONFIG_CMD_SET_MIN_VOLTAGE 0x04 // Command to set min cell voltage (value * 100 = mV, e.g. 25 = 2500mV)
#define CONFIG_CMD_SET_MAX_VOLTAGE 0x05 // Command to set max cell voltage (value * 100 = mV, e.g. 42 = 4200mV)
#define CONFIG_CMD_GET_VALUE     0x06   // Command to get current config value
#define CONFIG_CMD_SET_BQ_MODE  0x07   // Command to set BQ76952 power mode (0=NORMAL, 1=SLEEP)

/* Parameter Selectors for GET_VALUE command ---------------------------------*/
#define CONFIG_PARAM_MODULE_ID   0x01   // Get current module ID
#define CONFIG_PARAM_MAX_TEMP    0x02   // Get max temperature threshold
#define CONFIG_PARAM_MIN_TEMP    0x03   // Get min temperature threshold
#define CONFIG_PARAM_MIN_VOLTAGE 0x04   // Get min voltage threshold
#define CONFIG_PARAM_MAX_VOLTAGE 0x05   // Get max voltage threshold
#define CONFIG_PARAM_BQ_MODE    0x06   // Get BQ76952 power mode (0=NORMAL, 1=SLEEP)
#define CONFIG_PARAM_BQ_NORMAL_READ_INT  0x07  // Get BQ normal mode read interval (ms)
#define CONFIG_PARAM_BQ_NORMAL_CAN_INT   0x08  // Get BQ normal mode CAN interval (ms)
#define CONFIG_PARAM_BQ_SLEEP_READ_INT   0x09  // Get BQ sleep mode read interval (ms)
#define CONFIG_PARAM_BQ_SLEEP_CAN_INT    0x0A  // Get BQ sleep mode CAN interval (ms)
#define CONFIG_PARAM_I2C_TIMEOUT         0x0B  // Get I2C timeout (ms)
#define CONFIG_PARAM_BAL_CMD_TIMEOUT     0x0C  // Get balance command timeout (ms)
#define CONFIG_PARAM_BAL_REEVALUATE      0x0D  // Get balance reevaluate interval (ms)
#define CONFIG_PARAM_BAL_REFRESH         0x0E  // Get balance refresh interval (ms)
#define CONFIG_PARAM_BAL_OCV_SETTLE      0x0F  // Get balance OCV settle time (ms)
#define CONFIG_PARAM_BAL_STATUS_INT      0x10  // Get balance status interval (ms)
#define CONFIG_PARAM_CAN_HEARTBEAT_INT   0x11  // Get CAN heartbeat interval (ms)
#define CONFIG_PARAM_TEMP_SUMMARY_INT    0x12  // Get temperature summary interval (ms)
#define CONFIG_PARAM_CAN_TX_TIMEOUT      0x13  // Get CAN TX timeout (ms)

/* CAN Response Status Codes -------------------------------------------------*/
#define CONFIG_STATUS_SUCCESS 0x00      // Command successful
#define CONFIG_STATUS_FAIL 0x01         // Command failed (invalid parameter)
#define CONFIG_STATUS_RESET_REQUIRED 0x02 // Command successful, reset required

/* Function Prototypes -------------------------------------------------------*/

/**
  * @brief  Initialize configuration manager
  * @note   Reads module ID from flash and initializes CAN IDs
  * @retval None
  */
void Config_Init(void);

/**
  * @brief  Set module ID (0-15) - writes to flash, requires reset
  * @param  module_id: Module ID value
  * @retval 0 on success, -1 on error
  * @note   After successful write, device must be reset for changes to take effect
  */
int8_t Config_SetModuleID(uint8_t module_id);

/**
  * @brief  Get current module ID
  * @retval Current module ID (0-15)
  */
uint8_t Config_GetModuleID(void);

/**
  * @brief  Read module ID from flash storage
  * @retval Module ID from flash, or CONFIG_MODULE_ID_DEFAULT if uninitialized
  */
uint8_t Config_ReadModuleIDFromFlash(void);

/**
  * @brief  Write module ID to flash storage
  * @param  module_id: Module ID value to write
  * @retval 0 on success, -1 on error
  */
int8_t Config_WriteModuleIDToFlash(uint8_t module_id);

/**
  * @brief  Initialize all CAN IDs with module offset
  * @note   Called during Config_Init() after module ID is read
  * @retval None
  */
void Config_InitCANIDs(void);

/**
  * @brief  Process configuration CAN command
  * @param  data: CAN message data (8 bytes)
  * @param  length: Data length
  * @retval None
  */
void Config_ProcessCANCommand(uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_MANAGER_H */
