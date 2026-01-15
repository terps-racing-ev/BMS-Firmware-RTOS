/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : state_machine.h
  * @brief          : BMS State Machine Management
  ******************************************************************************
  * @attention
  *
  * This module manages the BMS system state machine, handling state transitions
  * and providing a centralized location for state management.
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

#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* BMS State Definitions -----------------------------------------------------*/
typedef enum {
    BMS_STATE_INIT = 0,         /**< Initialization state */
    BMS_STATE_IDLE,             /**< Idle/standby state */
    BMS_STATE_CHARGING,         /**< Charging state */
    BMS_STATE_DISCHARGING,      /**< Discharging state */
    BMS_STATE_BALANCING,        /**< Cell balancing state */
    BMS_STATE_ERROR,            /**< Error/fault state */
    BMS_STATE_RESERVED          /**< Reserved for future use */
} BMS_State_t;

/* Function Prototypes -------------------------------------------------------*/

/**
  * @brief  Initialize state machine
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef StateMachine_Init(void);

/**
  * @brief  Set BMS state
  * @param  state: New BMS state
  * @retval None
  * @note   State transitions are validated. Cannot exit ERROR state while faults exist.
  */
void StateMachine_SetState(BMS_State_t state);

/**
  * @brief  Get current BMS state
  * @retval BMS_State_t: Current BMS state
  */
BMS_State_t StateMachine_GetState(void);

/**
  * @brief  Signal fault entry - called by error manager when critical fault occurs
  * @retval None
  * @note   Forces transition to BMS_STATE_ERROR
  */
void StateMachine_EnterFault(void);

/**
  * @brief  Signal fault exit - called by error manager when all faults cleared
  * @retval None
  * @note   Allows transition from BMS_STATE_ERROR to BMS_STATE_IDLE
  */
void StateMachine_ExitFault(void);

/**
  * @brief  Check if currently in fault state
  * @retval bool: true if in BMS_STATE_ERROR
  */
bool StateMachine_IsInFault(void);

/**
  * @brief  Check if state transition is allowed
  * @param  new_state: Desired new state
  * @retval bool: true if transition is allowed
  */
bool StateMachine_CanTransition(BMS_State_t new_state);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE_H */
