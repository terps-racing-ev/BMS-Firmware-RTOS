/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : config_manager.c
  * @brief          : BMS Configuration Manager Implementation
  ******************************************************************************
  * @attention
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
#include "config_manager.h"
#include "can_ids.h"
#include <string.h>

/* External function prototypes ----------------------------------------------*/
extern HAL_StatusTypeDef CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t length, uint8_t priority);
extern HAL_StatusTypeDef CAN_ReconfigureFilters(void);

/* Private variables ---------------------------------------------------------*/
static uint8_t module_id = CONFIG_MODULE_ID_DEFAULT;
static osMutexId_t config_mutex = NULL;

/* CAN ID variables (initialized with module offset) */
uint32_t CAN_TEMP_ID = 0;
uint32_t CAN_TEMP_RAW_ID = 0;
uint32_t CAN_VOLTAGE_0_ID = 0;
uint32_t CAN_VOLTAGE_1_ID = 0;
uint32_t CAN_VOLTAGE_2_ID = 0;
uint32_t CAN_VOLTAGE_3_ID = 0;
uint32_t CAN_VOLTAGE_4_ID = 0;
uint32_t CAN_VOLTAGE_5_ID = 0;
uint32_t CAN_BMS_HEARTBEAT_ID = 0;
uint32_t CAN_BMS_STATS_ID = 0;
uint32_t CAN_CONFIG_ACK_ID = 0;
uint32_t CAN_RESET_CMD_ID = 0;
uint32_t CAN_DEBUG_RESPONSE_ID = 0;

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Initialize configuration manager
  * @note   Reads module ID from flash and initializes CAN IDs
  * @retval None
  */
void Config_Init(void)
{
    // Create mutex for thread-safe access
    const osMutexAttr_t mutex_attr = {
        .name = "ConfigMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = NULL,
        .cb_size = 0U
    };
    
    config_mutex = osMutexNew(&mutex_attr);
    
    // Read module ID from flash storage
    module_id = Config_ReadModuleIDFromFlash();
    
    // Initialize CAN IDs with module offset
    Config_InitCANIDs();
}

/**
  * @brief  Set module ID (0-15) - writes to flash, requires reset
  * @param  module_id_new: Module ID value
  * @retval 0 on success, -1 on error
  * @note   After successful write, device must be reset for changes to take effect
  */
int8_t Config_SetModuleID(uint8_t module_id_new)
{
    // Validate module ID range
    if (module_id_new > CONFIG_MODULE_ID_MAX) {
        return -1;
    }
    
    // Write to flash
    int8_t result = Config_WriteModuleIDToFlash(module_id_new);
    
    return result;
}

/**
  * @brief  Get current module ID
  * @retval Current module ID (0-15)
  */
uint8_t Config_GetModuleID(void)
{
    uint8_t id;
    
    // Thread-safe read
    if (config_mutex != NULL) {
        osMutexAcquire(config_mutex, osWaitForever);
    }
    
    id = module_id;
    
    if (config_mutex != NULL) {
        osMutexRelease(config_mutex);
    }
    
    return id;
}

/**
  * @brief  Read module ID from flash storage
  * @retval Module ID from flash, or CONFIG_MODULE_ID_DEFAULT if uninitialized
  */
uint8_t Config_ReadModuleIDFromFlash(void)
{
    // Read 32-bit word from flash
    uint32_t flash_value = *(__IO uint32_t*)CONFIG_FLASH_MODULE_ADDR;
    
    // Check if magic value is present (upper 16 bits)
    if ((flash_value & CONFIG_FLASH_MAGIC_MASK) == CONFIG_FLASH_MAGIC) {
        // Extract module ID from lower 4 bits
        uint8_t stored_module_id = (uint8_t)(flash_value & CONFIG_FLASH_MODULE_MASK);
        
        // Validate range
        if (stored_module_id <= CONFIG_MODULE_ID_MAX) {
            return stored_module_id;
        }
    }
    
    // Flash uninitialized or invalid, return default
    return CONFIG_MODULE_ID_DEFAULT;
}

/**
  * @brief  Write module ID to flash storage
  * @param  module_id: Module ID value to write
  * @retval 0 on success, -1 on error
  */
int8_t Config_WriteModuleIDToFlash(uint8_t module_id)
{
    HAL_StatusTypeDef status;
    
    // Validate module ID range
    if (module_id > CONFIG_MODULE_ID_MAX) {
        return -1;
    }
    
    // Prepare flash value: magic + module ID
    uint32_t flash_value = CONFIG_FLASH_MAGIC | (uint32_t)module_id;
    
    // Unlock flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return -1;
    }
    
    // Erase the page containing our address
    // STM32L432 has 2KB pages, need to erase page before writing
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    
    // Calculate page number (page size = 2048 bytes = 0x800)
    uint32_t page_address = CONFIG_FLASH_MODULE_ADDR;
    uint32_t page_number = (page_address - FLASH_BASE) / FLASH_PAGE_SIZE;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = page_number;
    erase_init.NbPages = 1;
    
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return -1;
    }
    
    // Program the double-word (64-bit) - STM32L4 requires double-word programming
    // We'll write our 32-bit value twice to make 64 bits
    uint64_t data = ((uint64_t)flash_value << 32) | (uint64_t)flash_value;
    
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CONFIG_FLASH_MODULE_ADDR, data);
    
    // Lock flash
    HAL_FLASH_Lock();
    
    return (status == HAL_OK) ? 0 : -1;
}

/**
  * @brief  Initialize all CAN IDs with module offset
  * @note   Called during Config_Init() after module ID is read
  * @retval None
  */
void Config_InitCANIDs(void)
{
    uint8_t mod_id = Config_GetModuleID();
    
    // Initialize all CAN IDs with module offset
    CAN_TEMP_ID = CAN_ID(CAN_TEMP_BASE, mod_id);
    CAN_TEMP_RAW_ID = CAN_ID(CAN_TEMP_RAW_BASE, mod_id);
    CAN_VOLTAGE_0_ID = CAN_ID(CAN_VOLTAGE_0_BASE, mod_id);
    CAN_VOLTAGE_1_ID = CAN_ID(CAN_VOLTAGE_1_BASE, mod_id);
    CAN_VOLTAGE_2_ID = CAN_ID(CAN_VOLTAGE_2_BASE, mod_id);
    CAN_VOLTAGE_3_ID = CAN_ID(CAN_VOLTAGE_3_BASE, mod_id);
    CAN_VOLTAGE_4_ID = CAN_ID(CAN_VOLTAGE_4_BASE, mod_id);
    CAN_VOLTAGE_5_ID = CAN_ID(CAN_VOLTAGE_5_BASE, mod_id);
    CAN_BMS_HEARTBEAT_ID = CAN_ID(CAN_BMS_HEARTBEAT_BASE, mod_id);
    CAN_BMS_STATS_ID = CAN_ID(CAN_BMS_STATS_BASE, mod_id);
    CAN_CONFIG_ACK_ID = CAN_ID(CAN_CONFIG_ACK_BASE, mod_id);
    CAN_RESET_CMD_ID = CAN_ID(CAN_RESET_CMD_BASE, mod_id);
    CAN_DEBUG_RESPONSE_ID = CAN_ID(CAN_DEBUG_RESPONSE_BASE, mod_id);
}

/**
  * @brief  Process configuration CAN command
  * @param  data: CAN message data (8 bytes)
  * @param  length: Data length
  * @retval None
  * @note   Sends acknowledgement message, then resets device if successful
  */
void Config_ProcessCANCommand(uint8_t *data, uint8_t length)
{
    if (data == NULL || length < 2) {
        return;
    }
    
    uint8_t command = data[0];
    uint8_t value = data[1];
    uint8_t ack_data[8] = {0};
    uint8_t status = CONFIG_STATUS_FAIL;
    
    switch (command) {
        case CONFIG_CMD_SET_MODULE_ID:
        {
            // Get current module ID before changing
            uint8_t old_module_id = Config_GetModuleID();
            
            // Try to write new module ID to flash
            int8_t result = Config_SetModuleID(value);
            
            // Determine status
            if (result == 0) {
                status = CONFIG_STATUS_RESET_REQUIRED; // Success - reset needed
            } else {
                status = CONFIG_STATUS_FAIL;
            }
            
            // Prepare acknowledgement message
            // Byte 0: Command echo
            // Byte 1: Status (0x00 = success, 0x02 = success/reset required, 0x01 = fail)
            // Byte 2: Old module ID (current, pre-reset)
            // Byte 3: New module ID (will be active after reset)
            // Bytes 4-7: Reserved
            ack_data[0] = command;
            ack_data[1] = status;
            ack_data[2] = old_module_id;
            ack_data[3] = value;
            ack_data[4] = 0x00;
            ack_data[5] = 0x00;
            ack_data[6] = 0x00;
            ack_data[7] = 0x00;
            
            // Send acknowledgement using current (old) module ID
            CAN_SendMessage(CAN_CONFIG_ACK_ID, ack_data, 8, 1);
            
            // If successful, wait briefly for ACK to transmit, then reset
            if (result == 0) {
                osDelay(100); // Give time for CAN message to transmit
                NVIC_SystemReset(); // Reset the microcontroller
            }
            break;
        }
            
        default:
            // Unknown command, ignore (no ACK sent)
            break;
    }
}
