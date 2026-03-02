/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : balance_manager.c
  * @brief          : Cell Balancing State Manager Implementation
  ******************************************************************************
  * @attention
  *
  * This module implements host-controlled cell balancing using the BQ76952's
  * CB_ACTIVE_CELLS subcommand. The host specifies a target voltage and
  * maximum cells per chip, and this module selects the highest voltage cells
  * above target for balancing, with spacing to avoid adjacent cells.
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
#include "balance_manager.h"
#include "state_machine.h"
#include "error_manager.h"
#include "bq_handler.h"
#include "can_manager.h"
#include "can_ids.h"
#include <string.h>

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern uint32_t CAN_BALANCE_STATUS_ID;
extern uint32_t CAN_BMS1_BAL_DETAIL_ID;
extern uint32_t CAN_BMS2_BAL_DETAIL_ID;

/* Private variables ---------------------------------------------------------*/
static uint32_t last_balance_cmd_tick = 0;      /**< Tick of last balance enable command */
static uint32_t last_balance_cfg_tick = 0;      /**< Tick of last balance config message */
static uint32_t last_reevaluate_tick = 0;       /**< Tick of last cell re-evaluation */
static uint32_t last_refresh_tick = 0;          /**< Tick of last CB_ACTIVE_CELLS refresh */
static uint32_t last_status_tick = 0;           /**< Tick of last status message */
static bool balance_timeout_active = false;     /**< Whether balance timeout is active */
static bool config_received = false;            /**< Whether config has been received */

static Balance_Config_t balance_config = {
    .target_voltage_mv = BALANCE_DEFAULT_TARGET_MV,
    .max_cells_per_chip = BALANCE_DEFAULT_MAX_CELLS,
    .last_config_tick = 0,
    .config_valid = false
};

static Balance_Status_t balance_status = {0};

static osMutexId_t balance_mutex = NULL;
static const osMutexAttr_t balance_mutex_attributes = {
    .name = "BalanceMutex"
};

/* Private function prototypes -----------------------------------------------*/
static uint16_t SelectCellsToBalance(uint16_t *voltages, uint8_t num_cells, 
                                      uint16_t target_mv, uint8_t max_cells);
static uint8_t CountBits(uint16_t value);

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Initialize balance manager
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BalanceMgr_Init(void)
{
    last_balance_cmd_tick = 0;
    last_balance_cfg_tick = 0;
    last_reevaluate_tick = 0;
    last_refresh_tick = 0;
    last_status_tick = 0;
    balance_timeout_active = false;
    config_received = false;
    
    // Reset config to defaults
    balance_config.target_voltage_mv = BALANCE_DEFAULT_TARGET_MV;
    balance_config.max_cells_per_chip = BALANCE_DEFAULT_MAX_CELLS;
    balance_config.config_valid = false;
    
    // Clear status
    memset(&balance_status, 0, sizeof(Balance_Status_t));
    
    // Create mutex for thread-safe access
    balance_mutex = osMutexNew(&balance_mutex_attributes);
    if (balance_mutex == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
  * @brief  Process balance enable command received via CAN
  * @note   This refreshes the balance timeout and attempts to enter/stay in balance mode
  * @retval uint8_t: Status code (BALANCE_STATUS_*)
  */
uint8_t BalanceMgr_ProcessCommand(void)
{
    uint8_t status = BALANCE_STATUS_SUCCESS;
    BMS_State_t current_state;
    
    if (osMutexAcquire(balance_mutex, osWaitForever) != osOK) {
        return BALANCE_STATUS_FAULT;
    }
    
    // Check if any faults are present
    if (ErrorMgr_HasErrors()) {
        osMutexRelease(balance_mutex);
        return BALANCE_STATUS_FAULT;
    }
    
    current_state = StateMachine_GetState();
    
    // Check current state - can only enter balancing from IDLE or CHARGING
    if (current_state == BMS_STATE_BALANCING) {
        // Already balancing - just refresh the timeout
        last_balance_cmd_tick = osKernelGetTickCount();
        balance_timeout_active = true;
        status = BALANCE_STATUS_SUCCESS;
    }
    else if (current_state == BMS_STATE_IDLE || current_state == BMS_STATE_CHARGING) {
        // Allowed to enter balancing mode
        
        // Configure BQ76952 for maximum balancing current (CB_LOOP_SLOW = 11)
        // This sets eighth-speed mode for better average balancing current
        BQ_ConfigureBalancingSpeed(&hi2c1, BQ76952_I2C_ADDR_BMS1);
        BQ_ConfigureBalancingSpeed(&hi2c3, BQ76952_I2C_ADDR_BMS2);
        
        StateMachine_SetState(BMS_STATE_BALANCING);
        last_balance_cmd_tick = osKernelGetTickCount();
        last_reevaluate_tick = 0;  // Force immediate evaluation
        last_refresh_tick = 0;     // Force immediate refresh
        balance_timeout_active = true;
        status = BALANCE_STATUS_SUCCESS;
    }
    else {
        // Not in a valid state to enter balancing
        status = BALANCE_STATUS_WRONG_STATE;
    }
    
    osMutexRelease(balance_mutex);
    return status;
}

/**
  * @brief  Process balance configuration received via CAN
  * @param  target_voltage_mv: Target cell voltage in mV
  * @param  max_cells_per_chip: Maximum cells to balance per BMS chip (1-9)
  * @retval uint8_t: Status code (BALANCE_STATUS_*)
  */
uint8_t BalanceMgr_ProcessConfig(uint16_t target_voltage_mv, uint8_t max_cells_per_chip)
{
    if (osMutexAcquire(balance_mutex, osWaitForever) != osOK) {
        return BALANCE_STATUS_FAULT;
    }
    
    // Validate and clamp max_cells_per_chip
    if (max_cells_per_chip == 0) {
        max_cells_per_chip = 1;
    } else if (max_cells_per_chip > BALANCE_MAX_CELLS_PER_CHIP) {
        max_cells_per_chip = BALANCE_MAX_CELLS_PER_CHIP;
    }
    
    // Check if config actually changed - only force re-evaluation if it did
    bool config_changed = (balance_config.target_voltage_mv != target_voltage_mv) ||
                          (balance_config.max_cells_per_chip != max_cells_per_chip) ||
                          (!balance_config.config_valid);
    
    // Store configuration
    balance_config.target_voltage_mv = target_voltage_mv;
    balance_config.max_cells_per_chip = max_cells_per_chip;
    balance_config.last_config_tick = osKernelGetTickCount();
    balance_config.config_valid = true;
    config_received = true;
    
    // Only force re-evaluation if config changed, not on every keep-alive
    if (config_changed) {
        last_reevaluate_tick = 0;
    }
    
    osMutexRelease(balance_mutex);
    return BALANCE_STATUS_SUCCESS;
}

/**
  * @brief  Check balance timeout and exit if expired
  * @note   Should be called periodically from a task (e.g., every 100ms)
  * @retval None
  */
void BalanceMgr_CheckTimeout(void)
{
    if (osMutexAcquire(balance_mutex, osWaitForever) != osOK) {
        return;
    }
    
    uint32_t current_tick = osKernelGetTickCount();
    
    // Only check timeout if we're in balancing mode and timeout is active
    if (balance_timeout_active && StateMachine_GetState() == BMS_STATE_BALANCING) {
        // Check balance enable command timeout (5 seconds)
        uint32_t cmd_elapsed = current_tick - last_balance_cmd_tick;
        
        if (cmd_elapsed >= BALANCE_CMD_TIMEOUT_MS) {
            // Timeout expired - stop balancing and exit to IDLE
            osMutexRelease(balance_mutex);
            BalanceMgr_StopBalancing();
            if (osMutexAcquire(balance_mutex, osWaitForever) != osOK) {
                return;
            }
            StateMachine_SetState(BMS_STATE_IDLE);
            balance_timeout_active = false;
            config_received = false;
            last_balance_cmd_tick = 0;
            last_balance_cfg_tick = 0;
        }
        
        // Check config timeout (5 seconds) - if config was received but stopped
        if (config_received && balance_config.config_valid) {
            uint32_t cfg_elapsed = current_tick - balance_config.last_config_tick;
            if (cfg_elapsed >= BALANCE_CFG_TIMEOUT_MS) {
                // Config timeout - stop active balancing but stay in balance mode
                // Host needs to resend config to continue
                BQ_StopBalancing(&hi2c1, BQ76952_I2C_ADDR_BMS1);
                BQ_StopBalancing(&hi2c3, BQ76952_I2C_ADDR_BMS2);
                balance_config.config_valid = false;
                balance_status.bms1_active_cells = 0;
                balance_status.bms2_active_cells = 0;
                balance_status.bms1_cell_count = 0;
                balance_status.bms2_cell_count = 0;
            }
        }
    }
    else if (!balance_timeout_active && StateMachine_GetState() != BMS_STATE_BALANCING) {
        // Not balancing and no timeout active - reset state
        last_balance_cmd_tick = 0;
        last_balance_cfg_tick = 0;
        config_received = false;
    }
    
    osMutexRelease(balance_mutex);
}

/**
  * @brief  Count number of set bits in a 16-bit value
  */
static uint8_t CountBits(uint16_t value)
{
    uint8_t count = 0;
    while (value) {
        count += value & 1;
        value >>= 1;
    }
    return count;
}

/**
  * @brief  Select cells to balance based on voltage and spacing
  * @param  voltages: Array of cell voltages in mV
  * @param  num_cells: Number of cells in array
  * @param  target_mv: Target voltage - cells above this are candidates
  * @param  max_cells: Maximum number of cells to select
  * @retval uint16_t: Bitmask of cells to balance (bit 0 = cell 1)
  */
static uint16_t SelectCellsToBalance(uint16_t *voltages, uint8_t num_cells, 
                                      uint16_t target_mv, uint8_t max_cells)
{
    uint16_t cell_mask = 0;
    uint8_t selected_count = 0;
    
    // Create array of cell indices and voltages for sorting
    typedef struct {
        uint8_t index;
        uint16_t voltage;
    } CellInfo_t;
    
    CellInfo_t candidates[BALANCE_MAX_CELLS_PER_CHIP];  // Max 9 cells per chip
    uint8_t candidate_count = 0;
    
    // Find all cells above target voltage
    for (uint8_t i = 0; i < num_cells && i < BALANCE_MAX_CELLS_PER_CHIP; i++) {
        if (voltages[i] > target_mv) {
            candidates[candidate_count].index = i;
            candidates[candidate_count].voltage = voltages[i];
            candidate_count++;
        }
    }
    
    // Sort candidates by voltage (descending) - simple bubble sort
    for (uint8_t i = 0; i < candidate_count; i++) {
        for (uint8_t j = i + 1; j < candidate_count; j++) {
            if (candidates[j].voltage > candidates[i].voltage) {
                CellInfo_t temp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = temp;
            }
        }
    }
    
    // Select cells with spacing (try to avoid adjacent)
    // First pass: select non-adjacent cells
    for (uint8_t i = 0; i < candidate_count && selected_count < max_cells; i++) {
        uint8_t cell_idx = candidates[i].index;
        
        // Check if this would create adjacent balancing
        bool has_adjacent = false;
        if (cell_idx > 0 && (cell_mask & (1 << (cell_idx - 1)))) {
            has_adjacent = true;
        }
        if (cell_idx < (BALANCE_MAX_CELLS_PER_CHIP - 1) && (cell_mask & (1 << (cell_idx + 1)))) {
            has_adjacent = true;
        }
        
        // Skip if adjacent (first pass)
        if (!has_adjacent) {
            cell_mask |= (1 << cell_idx);
            selected_count++;
        }
    }
    
    // Second pass: if we haven't selected enough, allow adjacent
    if (selected_count < max_cells) {
        for (uint8_t i = 0; i < candidate_count && selected_count < max_cells; i++) {
            uint8_t cell_idx = candidates[i].index;
            
            // Skip if already selected
            if (cell_mask & (1 << cell_idx)) {
                continue;
            }
            
            cell_mask |= (1 << cell_idx);
            selected_count++;
        }
    }
    
    return cell_mask;
}

/**
  * @brief  Execute balancing logic - select and balance cells
  * @note   Should be called periodically from the CAN manager task
  *         Re-evaluates cell selection every 40 seconds
  *         Refreshes CB_ACTIVE_CELLS command every 5 seconds to prevent BQ76952 timeout
  * @retval None
  */
void BalanceMgr_Execute(void)
{
    if (StateMachine_GetState() != BMS_STATE_BALANCING) {
        return;
    }
    
    if (osMutexAcquire(balance_mutex, osWaitForever) != osOK) {
        return;
    }
    
    uint32_t current_tick = osKernelGetTickCount();
    
    // Check if we have valid config
    if (!balance_config.config_valid) {
        osMutexRelease(balance_mutex);
        return;
    }
    
    // Check if it's time to re-evaluate cell selection (every 40 seconds)
    bool should_evaluate = false;
    if (last_reevaluate_tick == 0) {
        should_evaluate = true;  // First run
    } else {
        uint32_t elapsed = current_tick - last_reevaluate_tick;
        if (elapsed >= BALANCE_REEVALUATE_MS) {
            should_evaluate = true;
        }
    }
    
    // Check if it's time to refresh CB_ACTIVE_CELLS (every 5 seconds)
    // BQ76952 has internal 10-second timeout on CB_ACTIVE_CELLS
    bool should_refresh = false;
    if (last_refresh_tick == 0) {
        should_refresh = true;  // First run
    } else {
        uint32_t elapsed = current_tick - last_refresh_tick;
        if (elapsed >= BALANCE_REFRESH_MS) {
            should_refresh = true;
        }
    }
    
    if (should_evaluate) {
        // Re-evaluate which cells to balance based on current voltages
        BQ_Data_t bms1_data;
        BQ_Data_BMS2_t bms2_data;
        
        if (BQ_GetData(&bms1_data) == HAL_OK && bms1_data.valid) {
            // Select cells to balance for BMS1 (cells 1-9)
            uint16_t bms1_mask = SelectCellsToBalance(
                bms1_data.cell_voltage_mv,
                BMS1_NUM_CELLS,
                balance_config.target_voltage_mv,
                balance_config.max_cells_per_chip
            );
            
            // Issue balance command to BMS1
            if (BQ_SetBalanceCells(&hi2c1, BQ76952_I2C_ADDR_BMS1, bms1_mask) == HAL_OK) {
                balance_status.bms1_active_cells = bms1_mask;
                balance_status.bms1_cell_count = CountBits(bms1_mask);
            }
        }
        
        if (BQ_GetData_BMS2(&bms2_data) == HAL_OK && bms2_data.valid) {
            // Select cells to balance for BMS2 (cells 10-18, stored as 0-8)
            uint16_t bms2_mask = SelectCellsToBalance(
                bms2_data.cell_voltage_mv,
                BMS2_NUM_CELLS,
                balance_config.target_voltage_mv,
                balance_config.max_cells_per_chip
            );
            
            // Issue balance command to BMS2
            if (BQ_SetBalanceCells(&hi2c3, BQ76952_I2C_ADDR_BMS2, bms2_mask) == HAL_OK) {
                balance_status.bms2_active_cells = bms2_mask;
                balance_status.bms2_cell_count = CountBits(bms2_mask);
            }
        }
        
        last_reevaluate_tick = current_tick;
        last_refresh_tick = current_tick;  // Also counts as a refresh
    }
    else if (should_refresh) {
        // Just refresh the CB_ACTIVE_CELLS command with same cell selection
        // This prevents BQ76952's internal 10-second timeout from stopping balancing
        BQ_SetBalanceCells(&hi2c1, BQ76952_I2C_ADDR_BMS1, balance_status.bms1_active_cells);
        BQ_SetBalanceCells(&hi2c3, BQ76952_I2C_ADDR_BMS2, balance_status.bms2_active_cells);
        
        last_refresh_tick = current_tick;
    }
    
    // Update status with current config
    balance_status.target_voltage_mv = balance_config.target_voltage_mv;
    balance_status.max_cells_per_chip = balance_config.max_cells_per_chip;
    balance_status.bms1_ic_temp = BQ_GetBMS1InternalTemp();
    balance_status.bms2_ic_temp = BQ_GetBMS2InternalTemp();
    
    osMutexRelease(balance_mutex);
}

/**
  * @brief  Send balance status via CAN
  * @retval HAL_StatusTypeDef
  * @note   Balance Status format (8 bytes):
  *         Bytes 0-1: Target voltage (mV, little-endian)
  *         Byte 2: Max cells per chip
  *         Byte 3: BMS1 active cell count | BMS2 active cell count (4 bits each)
  *         Bytes 4-5: BMS1 IC temperature (0.1°C, signed, little-endian)
  *         Bytes 6-7: BMS2 IC temperature (0.1°C, signed, little-endian)
  */
HAL_StatusTypeDef BalanceMgr_SendStatus(void)
{
    if (StateMachine_GetState() != BMS_STATE_BALANCING) {
        return HAL_OK;  // Not in balancing mode, don't send
    }
    
    uint8_t data[8];
    
    if (osMutexAcquire(balance_mutex, osWaitForever) != osOK) {
        return HAL_ERROR;
    }
    
    // Pack status message
    data[0] = (uint8_t)(balance_status.target_voltage_mv & 0xFF);
    data[1] = (uint8_t)((balance_status.target_voltage_mv >> 8) & 0xFF);
    data[2] = balance_status.max_cells_per_chip;
    data[3] = (balance_status.bms1_cell_count & 0x0F) | ((balance_status.bms2_cell_count & 0x0F) << 4);
    data[4] = (uint8_t)(balance_status.bms1_ic_temp & 0xFF);
    data[5] = (uint8_t)((balance_status.bms1_ic_temp >> 8) & 0xFF);
    data[6] = (uint8_t)(balance_status.bms2_ic_temp & 0xFF);
    data[7] = (uint8_t)((balance_status.bms2_ic_temp >> 8) & 0xFF);
    
    osMutexRelease(balance_mutex);
    
    return CAN_SendMessage(CAN_BALANCE_STATUS_ID, data, 8, CAN_PRIORITY_NORMAL);
}

/**
  * @brief  Check if currently in balancing mode
  * @retval bool: true if in balancing mode
  */
bool BalanceMgr_IsBalancing(void)
{
    return (StateMachine_GetState() == BMS_STATE_BALANCING);
}

/**
  * @brief  Get time remaining before balance timeout (in ms)
  * @retval uint32_t: Time remaining in ms, 0 if not balancing
  */
uint32_t BalanceMgr_GetTimeRemaining(void)
{
    uint32_t remaining = 0;
    
    if (osMutexAcquire(balance_mutex, osWaitForever) != osOK) {
        return 0;
    }
    
    if (balance_timeout_active && StateMachine_GetState() == BMS_STATE_BALANCING) {
        uint32_t current_tick = osKernelGetTickCount();
        uint32_t elapsed = current_tick - last_balance_cmd_tick;
        
        if (elapsed < BALANCE_CMD_TIMEOUT_MS) {
            remaining = BALANCE_CMD_TIMEOUT_MS - elapsed;
        }
    }
    
    osMutexRelease(balance_mutex);
    return remaining;
}

/**
  * @brief  Get current balance status
  * @param  status: Pointer to status structure to fill
  * @retval None
  */
void BalanceMgr_GetStatus(Balance_Status_t *status)
{
    if (status == NULL) {
        return;
    }
    
    if (osMutexAcquire(balance_mutex, osWaitForever) != osOK) {
        return;
    }
    
    memcpy(status, &balance_status, sizeof(Balance_Status_t));
    
    osMutexRelease(balance_mutex);
}

/**
  * @brief  Stop all cell balancing immediately
  * @note   Called when exiting balance mode or on error
  * @retval None
  */
void BalanceMgr_StopBalancing(void)
{
    // Stop balancing on both chips (no mutex needed, BQ functions have their own)
    BQ_StopBalancing(&hi2c1, BQ76952_I2C_ADDR_BMS1);
    BQ_StopBalancing(&hi2c3, BQ76952_I2C_ADDR_BMS2);
    
    // Clear status
    if (osMutexAcquire(balance_mutex, osWaitForever) == osOK) {
        balance_status.bms1_active_cells = 0;
        balance_status.bms2_active_cells = 0;
        balance_status.bms1_cell_count = 0;
        balance_status.bms2_cell_count = 0;
        balance_config.config_valid = false;
        config_received = false;
        osMutexRelease(balance_mutex);
    }
}

/**
  * @brief  Send detailed balance readback from each BMS chip via CAN
  * @note   Reads CBSTATUS1/2/3 and AlarmRawStatus from each chip
  *         and sends as separate CAN messages for BMS1 and BMS2
  * @retval HAL_StatusTypeDef
  * 
  * BMS1/BMS2 Balance Detail Message Format (8 bytes each):
  *   Byte 0: CBSTATUS1 (cells 1-8 / 10-17 ACTUALLY balancing, bit per cell)
  *   Byte 1: CBSTATUS2 (cell 9 / 18 ACTUALLY balancing, in bit 0)
  *   Byte 2: CBSTATUS3 (balance status flags: OTPW, OT, UT, PAUSE)
  *   Byte 3-4: AlarmRawStatus (16-bit, little-endian)
  *   Byte 5-7: Reserved
  */
HAL_StatusTypeDef BalanceMgr_SendDetailedStatus(void)
{
    if (StateMachine_GetState() != BMS_STATE_BALANCING) {
        return HAL_OK;
    }

    uint8_t data[8];
    uint16_t alarm_raw = 0;
    uint8_t cbstatus1 = 0, cbstatus2 = 0, cbstatus3 = 0;
    HAL_StatusTypeDef status;
    
    // --- BMS1 Detail ---
    // Message format:
    //   Byte 0: CBSTATUS1 (cells 1-8 ACTUALLY balancing, read back from BMS)
    //   Byte 1: CBSTATUS2 (cell 9 actually balancing in bit 0)
    //   Byte 2: CBSTATUS3 (balance status flags: OTPW, OT, UT, PAUSE)
    //   Byte 3-4: AlarmRawStatus (16-bit, little-endian)
    //   Byte 5-7: Reserved
    memset(data, 0, 8);
    
    // Read CBSTATUS from BMS1 - shows which cells are ACTUALLY balancing
    status = BQ_GetCBStatus(&hi2c1, BQ76952_I2C_ADDR_BMS1, &cbstatus1, &cbstatus2, &cbstatus3);
    if (status == HAL_OK) {
        data[0] = cbstatus1;  // Cells 1-8 actually balancing (bit per cell)
        data[1] = cbstatus2;  // Cell 9 in bit 0
        data[2] = cbstatus3;  // Status flags
    }
    
    // Read AlarmRawStatus from BMS1
    status = BQ_GetAlarmRawStatus(&hi2c1, BQ76952_I2C_ADDR_BMS1, &alarm_raw);
    if (status == HAL_OK) {
        data[3] = (uint8_t)(alarm_raw & 0xFF);
        data[4] = (uint8_t)((alarm_raw >> 8) & 0xFF);
    }
    
    // Send BMS1 detail message
    CAN_SendMessage(CAN_BMS1_BAL_DETAIL_ID, data, 8, CAN_PRIORITY_NORMAL);
    
    // --- BMS2 Detail ---
    memset(data, 0, 8);
    alarm_raw = 0;
    cbstatus1 = cbstatus2 = cbstatus3 = 0;
    
    // Read CBSTATUS from BMS2 - shows which cells are ACTUALLY balancing
    status = BQ_GetCBStatus(&hi2c3, BQ76952_I2C_ADDR_BMS2, &cbstatus1, &cbstatus2, &cbstatus3);
    if (status == HAL_OK) {
        data[0] = cbstatus1;  // Cells 10-17 actually balancing (bit per cell)
        data[1] = cbstatus2;  // Cell 18 in bit 0
        data[2] = cbstatus3;  // Status flags
    }
    
    // Read AlarmRawStatus from BMS2
    status = BQ_GetAlarmRawStatus(&hi2c3, BQ76952_I2C_ADDR_BMS2, &alarm_raw);
    if (status == HAL_OK) {
        data[3] = (uint8_t)(alarm_raw & 0xFF);
        data[4] = (uint8_t)((alarm_raw >> 8) & 0xFF);
    }
    
    // Send BMS2 detail message
    CAN_SendMessage(CAN_BMS2_BAL_DETAIL_ID, data, 8, CAN_PRIORITY_NORMAL);
    
    return HAL_OK;
}
