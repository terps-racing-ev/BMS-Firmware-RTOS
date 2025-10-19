/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : error_manager.c
  * @brief          : BMS Error Management System Implementation
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
#include "error_manager.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static Error_Manager_t error_mgr = {0};
static osMutexId_t error_mutex = NULL;
static const osMutexAttr_t error_mutex_attributes = {
    .name = "ErrorMutex"
};

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Initialize error manager
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef ErrorMgr_Init(void)
{
    // Initialize error manager structure
    memset(&error_mgr, 0, sizeof(Error_Manager_t));
    
    // Set initial state
    error_mgr.state = BMS_STATE_INIT;
    error_mgr.uptime_seconds = 0;
    error_mgr.fault_count = 0;
    error_mgr.last_heartbeat = osKernelGetTickCount();
    
    // Create mutex for thread-safe access
    error_mutex = osMutexNew(&error_mutex_attributes);
    if (error_mutex == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
  * @brief  Set an error flag
  * @param  error_flag: Error flag bit to set
  * @retval None
  */
void ErrorMgr_SetError(uint32_t error_flag)
{
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        // Check if this is a new error
        if (!(error_mgr.error_flags & error_flag)) {
            error_mgr.fault_count++;
            
            // Auto-transition to error state if critical errors occur
            if (error_flag & (ERROR_OVER_TEMP | ERROR_UNDER_VOLTAGE | 
                              ERROR_OVER_VOLTAGE | ERROR_SHORT_CIRCUIT)) {
                error_mgr.state = BMS_STATE_ERROR;
            }
        }
        
        error_mgr.error_flags |= error_flag;
        osMutexRelease(error_mutex);
    }
}

/**
  * @brief  Clear an error flag
  * @param  error_flag: Error flag bit to clear
  * @retval None
  */
void ErrorMgr_ClearError(uint32_t error_flag)
{
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        // Check if this error was previously set
        if (error_mgr.error_flags & error_flag) {
            // Count how many bits are being cleared
            uint32_t cleared_bits = error_mgr.error_flags & error_flag;
            uint8_t cleared_count = 0;
            for (uint8_t bit = 0; bit < 32; bit++) {
                if (cleared_bits & (1 << bit)) {
                    cleared_count++;
                }
            }
            
            // Decrement fault count by number of errors being cleared
            if (error_mgr.fault_count >= cleared_count) {
                error_mgr.fault_count -= cleared_count;
            } else {
                error_mgr.fault_count = 0;  // Safety check
            }
        }
        
        error_mgr.error_flags &= ~error_flag;
        
        // If no errors remain and in error state, transition to idle
        if (error_mgr.error_flags == 0 && error_mgr.state == BMS_STATE_ERROR) {
            error_mgr.state = BMS_STATE_IDLE;
        }
        
        osMutexRelease(error_mutex);
    }
}

/**
  * @brief  Set a warning flag
  * @param  warning_flag: Warning flag bit to set
  * @retval None
  */
void ErrorMgr_SetWarning(uint32_t warning_flag)
{
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        error_mgr.warning_flags |= warning_flag;
        osMutexRelease(error_mutex);
    }
}

/**
  * @brief  Clear a warning flag
  * @param  warning_flag: Warning flag bit to clear
  * @retval None
  */
void ErrorMgr_ClearWarning(uint32_t warning_flag)
{
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        error_mgr.warning_flags &= ~warning_flag;
        osMutexRelease(error_mutex);
    }
}

/**
  * @brief  Get current error flags
  * @retval uint32_t: Current error flags
  */
uint32_t ErrorMgr_GetErrors(void)
{
    uint32_t errors = 0;
    
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        errors = error_mgr.error_flags;
        osMutexRelease(error_mutex);
    }
    
    return errors;
}

/**
  * @brief  Get current warning flags
  * @retval uint32_t: Current warning flags
  */
uint32_t ErrorMgr_GetWarnings(void)
{
    uint32_t warnings = 0;
    
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        warnings = error_mgr.warning_flags;
        osMutexRelease(error_mutex);
    }
    
    return warnings;
}

/**
  * @brief  Check if any errors are active
  * @retval bool: true if any errors active
  */
bool ErrorMgr_HasErrors(void)
{
    return (ErrorMgr_GetErrors() != 0);
}

/**
  * @brief  Check if any warnings are active
  * @retval bool: true if any warnings active
  */
bool ErrorMgr_HasWarnings(void)
{
    return (ErrorMgr_GetWarnings() != 0);
}

/**
  * @brief  Set BMS state
  * @param  state: New BMS state
  * @retval None
  */
void ErrorMgr_SetState(BMS_State_t state)
{
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        // Don't allow transition out of error state if errors exist
        if (error_mgr.state == BMS_STATE_ERROR && error_mgr.error_flags != 0) {
            // Only allow error state transitions
            if (state != BMS_STATE_ERROR && state != BMS_STATE_SHUTDOWN) {
                osMutexRelease(error_mutex);
                return;
            }
        }
        
        error_mgr.state = state;
        osMutexRelease(error_mutex);
    }
}

/**
  * @brief  Get current BMS state
  * @retval BMS_State_t: Current BMS state
  */
BMS_State_t ErrorMgr_GetState(void)
{
    BMS_State_t state = BMS_STATE_INIT;
    
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        state = error_mgr.state;
        osMutexRelease(error_mutex);
    }
    
    return state;
}

/**
  * @brief  Rotate through active bits in a single byte
  * @param  byte_value: The byte containing error flags
  * @param  rotation_index: Pointer to rotation index for this byte
  * @retval Rotated byte with only one bit set
  */
static uint8_t RotateByteBits(uint8_t byte_value, uint8_t *rotation_index)
{
    if (byte_value == 0) {
        *rotation_index = 0;
        return 0;
    }
    
    // Count active bits in this byte
    uint8_t active_bit_count = 0;
    for (uint8_t bit = 0; bit < 8; bit++) {
        if (byte_value & (1 << bit)) {
            active_bit_count++;
        }
    }
    
    // Find the Nth active bit
    if (active_bit_count > 0) {
        uint8_t current_index = 0;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (byte_value & (1 << bit)) {
                if (current_index == *rotation_index) {
                    // Update rotation for next call
                    *rotation_index = (*rotation_index + 1) % active_bit_count;
                    return (1 << bit);
                }
                current_index++;
            }
        }
    }
    
    return 0;
}

/**
  * @brief  Get complete error manager structure (for heartbeat)
  * @param  mgr: Pointer to structure to fill
  * @retval None
  * @note   Each error flag byte independently rotates through its active bits
  *         to avoid ambiguous combined values in CAN analysis tools.
  */
void ErrorMgr_GetStatus(Error_Manager_t *mgr)
{
    static uint8_t byte0_rotation = 0;
    static uint8_t byte1_rotation = 0;
    static uint8_t byte2_rotation = 0;
    static uint8_t byte3_rotation = 0;
    
    if (mgr == NULL) {
        return;
    }

    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        // Copy base data
        memcpy(mgr, &error_mgr, sizeof(Error_Manager_t));
        
        // Override state to ERROR if any faults exist
        if (error_mgr.fault_count > 0) {
            mgr->state = BMS_STATE_ERROR;
        }
        
        // Rotate each error byte independently
        uint8_t *error_bytes = (uint8_t *)&mgr->error_flags;
        error_bytes[0] = RotateByteBits((uint8_t)(error_mgr.error_flags & 0xFF), &byte0_rotation);
        error_bytes[1] = RotateByteBits((uint8_t)((error_mgr.error_flags >> 8) & 0xFF), &byte1_rotation);
        error_bytes[2] = RotateByteBits((uint8_t)((error_mgr.error_flags >> 16) & 0xFF), &byte2_rotation);
        error_bytes[3] = RotateByteBits((uint8_t)((error_mgr.error_flags >> 24) & 0xFF), &byte3_rotation);
        
        osMutexRelease(error_mutex);
    }
}/**
  * @brief  Update system uptime (call every second)
  * @retval None
  */
void ErrorMgr_UpdateUptime(void)
{
    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        error_mgr.uptime_seconds++;
        osMutexRelease(error_mutex);
    }
}
