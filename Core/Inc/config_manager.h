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
