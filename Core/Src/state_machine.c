/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : state_machine.c
  * @brief          : BMS State Machine Implementation
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
#include "state_machine.h"

/* Private variables ---------------------------------------------------------*/
static BMS_State_t current_state = BMS_STATE_INIT;
static bool fault_active = false;
static osMutexId_t state_mutex = NULL;
static const osMutexAttr_t state_mutex_attributes = {
    .name = "StateMutex"
};

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Initialize state machine
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef StateMachine_Init(void)
{
    current_state = BMS_STATE_INIT;
    fault_active = false;
    
    // Create mutex for thread-safe access
    state_mutex = osMutexNew(&state_mutex_attributes);
    if (state_mutex == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
  * @brief  Check if state transition is allowed
  * @param  new_state: Desired new state
  * @retval bool: true if transition is allowed
  */
bool StateMachine_CanTransition(BMS_State_t new_state)
{
    bool can_transition = true;
    
    if (osMutexAcquire(state_mutex, osWaitForever) == osOK) {
        // Cannot exit ERROR state while faults are active
        if (current_state == BMS_STATE_ERROR && fault_active) {
            // Only allow staying in ERROR
            if (new_state != BMS_STATE_ERROR) {
                can_transition = false;
            }
        }
        osMutexRelease(state_mutex);
    }
    
    return can_transition;
}

/**
  * @brief  Set BMS state
  * @param  state: New BMS state
  * @retval None
  */
void StateMachine_SetState(BMS_State_t state)
{
    if (osMutexAcquire(state_mutex, osWaitForever) == osOK) {
        // Don't allow transition out of error state if faults are active
        if (current_state == BMS_STATE_ERROR && fault_active) {
            // Only allow staying in error state
            if (state != BMS_STATE_ERROR) {
                osMutexRelease(state_mutex);
                return;
            }
        }
        
        current_state = state;
        osMutexRelease(state_mutex);
    }
}

/**
  * @brief  Get current BMS state
  * @retval BMS_State_t: Current BMS state
  */
BMS_State_t StateMachine_GetState(void)
{
    BMS_State_t state = BMS_STATE_INIT;
    
    if (osMutexAcquire(state_mutex, osWaitForever) == osOK) {
        state = current_state;
        osMutexRelease(state_mutex);
    }
    
    return state;
}

/**
  * @brief  Signal fault entry - called by error manager when critical fault occurs
  * @retval None
  */
void StateMachine_EnterFault(void)
{
    if (osMutexAcquire(state_mutex, osWaitForever) == osOK) {
        fault_active = true;
        current_state = BMS_STATE_ERROR;
        osMutexRelease(state_mutex);
    }
}

/**
  * @brief  Signal fault exit - called by error manager when all faults cleared
  * @retval None
  */
void StateMachine_ExitFault(void)
{
    if (osMutexAcquire(state_mutex, osWaitForever) == osOK) {
        fault_active = false;
        // Transition to IDLE when exiting fault
        if (current_state == BMS_STATE_ERROR) {
            current_state = BMS_STATE_IDLE;
        }
        osMutexRelease(state_mutex);
    }
}

/**
  * @brief  Check if currently in fault state
  * @retval bool: true if in BMS_STATE_ERROR
  */
bool StateMachine_IsInFault(void)
{
    bool in_fault = false;
    
    if (osMutexAcquire(state_mutex, osWaitForever) == osOK) {
        in_fault = (current_state == BMS_STATE_ERROR);
        osMutexRelease(state_mutex);
    }
    
    return in_fault;
}
