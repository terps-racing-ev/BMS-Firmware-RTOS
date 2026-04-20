/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : balance_manager.h
  * @brief          : Cell Balancing State Manager
  ******************************************************************************
  * @attention
  *
  * This module manages the cell balancing state transitions and balancing
  * operations. Balancing can only be entered from IDLE or CHARGING states
  * when no faults are present. A balance command must be received every
  * 5 seconds to stay in balancing mode.
  *
  * During balancing, the host sends configuration specifying:
  * - Target voltage (mV) - cells above this are balanced
  * - Max cells per BMS chip - limits concurrent balancing
  *
  * The module selects highest voltage cells above target, with spacing
  * to avoid adjacent cells when possible. Re-evaluates every 120 seconds.
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

#ifndef __BALANCE_MANAGER_H
#define __BALANCE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define BALANCE_CMD_TIMEOUT_MS      5000    /**< Balance command timeout (5 seconds) */
#define BALANCE_CFG_TIMEOUT_MS      5000    /**< Balance config timeout (5 seconds) */
#define BALANCE_REEVALUATE_SEC      120U    /**< Re-evaluate cell selection interval (seconds) */
#define BALANCE_REEVALUATE_MS       ((uint32_t)BALANCE_REEVALUATE_SEC * 1000U) /**< Re-evaluate cell selection (120 seconds) */
#define BALANCE_REFRESH_MS          5000    /**< Refresh CB_ACTIVE_CELLS command (5 seconds) */
#define BALANCE_OCV_SETTLE_MS       5000    /**< OCV settling delay before selecting cells (10 seconds) */
#define BALANCE_STATUS_INTERVAL_MS  1000    /**< Send balance status every 1 second */

#define BALANCE_MAX_CELLS_PER_CHIP  9       /**< Maximum cells that can be balanced per chip */
#define BALANCE_DEFAULT_TARGET_MV   3500    /**< Default target voltage if not configured */
#define BALANCE_DEFAULT_MAX_CELLS   4       /**< Default max cells per chip */

/* Balance Command Response Codes */
#define BALANCE_STATUS_SUCCESS      0x00    /**< Balance command accepted */
#define BALANCE_STATUS_FAULT        0x01    /**< Cannot balance - faults present */
#define BALANCE_STATUS_WRONG_STATE  0x02    /**< Cannot balance - wrong state (not IDLE or CHARGING) */
#define BALANCE_STATUS_TIMEOUT      0x03    /**< Balance exited due to timeout */

/* Balance Configuration Structure */
typedef struct {
    uint16_t target_voltage_mv;     /**< Target voltage in mV - cells above this are balanced */
    uint8_t max_cells_per_chip;     /**< Maximum cells to balance per BMS chip */
    bool bms1_enabled;              /**< Whether balancing is enabled for BMS1 */
    bool bms2_enabled;              /**< Whether balancing is enabled for BMS2 */
    uint32_t last_config_tick;      /**< Tick of last config message received */
    bool config_valid;              /**< Whether config has been received */
} Balance_Config_t;

/* Balance Status Structure */
typedef struct {
    uint16_t target_voltage_mv;     /**< Current target voltage */
    uint8_t max_cells_per_chip;     /**< Current max cells per chip */
    uint16_t bms1_active_cells;     /**< Bitmask of cells being balanced on BMS1 (cells 1-9) */
    uint16_t bms2_active_cells;     /**< Bitmask of cells being balanced on BMS2 (cells 10-18) */
    int16_t bms1_ic_temp;           /**< BMS1 IC temperature (0.1°C units) */
    int16_t bms2_ic_temp;           /**< BMS2 IC temperature (0.1°C units) */
    uint8_t bms1_cell_count;        /**< Number of cells currently balancing on BMS1 */
    uint8_t bms2_cell_count;        /**< Number of cells currently balancing on BMS2 */
} Balance_Status_t;

/* Function Prototypes -------------------------------------------------------*/

/**
  * @brief  Initialize balance manager
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BalanceMgr_Init(void);

/**
  * @brief  Process balance enable command received via CAN
  * @param  bms1_enable: 1 to enable balancing on BMS1, 0 to disable
  * @param  bms2_enable: 1 to enable balancing on BMS2, 0 to disable
  * @note   This refreshes the balance timeout and attempts to enter/stay in balance mode
  * @retval uint8_t: Status code (BALANCE_STATUS_*)
  */
uint8_t BalanceMgr_ProcessCommand(uint8_t bms1_enable, uint8_t bms2_enable);

/**
  * @brief  Process balance configuration received via CAN
  * @param  target_voltage_mv: Target cell voltage in mV
  * @param  max_cells_per_chip: Maximum cells to balance per BMS chip (1-9)
  * @retval uint8_t: Status code (BALANCE_STATUS_*)
  */
uint8_t BalanceMgr_ProcessConfig(uint16_t target_voltage_mv, uint8_t max_cells_per_chip);

/**
  * @brief  Check balance timeout and exit if expired
  * @note   Should be called periodically from a task (e.g., every 100ms)
  * @retval None
  */
void BalanceMgr_CheckTimeout(void);

/**
  * @brief  Execute balancing logic - select and balance cells
  * @note   Should be called periodically from the CAN manager task
  *         Re-evaluates cell selection every 120 seconds
  * @retval None
  */
void BalanceMgr_Execute(void);

/**
  * @brief  Send balance status via CAN
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BalanceMgr_SendStatus(void);

/**
  * @brief  Send detailed balance readback from each BMS chip via CAN
  * @note   Reads CB_ACTIVE_CELLS and AlarmRawStatus from each chip
  *         and sends as separate CAN messages for BMS1 and BMS2
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BalanceMgr_SendDetailedStatus(void);

/**
  * @brief  Check if currently in balancing mode
  * @retval bool: true if in balancing mode
  */
bool BalanceMgr_IsBalancing(void);

/**
  * @brief  Get time remaining before balance timeout (in ms)
  * @retval uint32_t: Time remaining in ms, 0 if not balancing
  */
uint32_t BalanceMgr_GetTimeRemaining(void);

/**
  * @brief  Get current balance status
  * @param  status: Pointer to status structure to fill
  * @retval None
  */
void BalanceMgr_GetStatus(Balance_Status_t *status);

/**
  * @brief  Stop all cell balancing immediately
  * @note   Called when exiting balance mode or on error
  * @retval None
  */
void BalanceMgr_StopBalancing(void);

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_MANAGER_H */
