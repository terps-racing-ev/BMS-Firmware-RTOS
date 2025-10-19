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
  * @brief  Get complete error manager structure (for heartbeat)
  * @param  mgr: Pointer to structure to fill
  * @retval None
  */
void ErrorMgr_GetStatus(Error_Manager_t *mgr)
{
    if (mgr == NULL) {
        return;
    }

    if (osMutexAcquire(error_mutex, osWaitForever) == osOK) {
        memcpy(mgr, &error_mgr, sizeof(Error_Manager_t));
        
        // Override state to ERROR if any faults exist
        if (error_mgr.fault_count > 0) {
            mgr->state = BMS_STATE_ERROR;
        }
        
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
