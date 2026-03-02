/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can_manager.c
  * @brief          : CAN bus manager implementation
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
#include "can_manager.h"
#include "main.h"
#include "error_manager.h"
#include "state_machine.h"
#include "config_manager.h"
#include "bq_handler.h"
#include "balance_manager.h"

/* Private defines -----------------------------------------------------------*/
#define CAN_HEALTH_CHECK_INTERVAL_MS  200U
#define CAN_RECOVERY_COOLDOWN_MS      500U

/* Private variables ---------------------------------------------------------*/
osMessageQueueId_t CANTxQueueHandle = NULL;
osMessageQueueId_t CANRxQueueHandle = NULL;

static CAN_Statistics_t can_stats = {0};
static volatile uint8_t g_can_recovery_requested = 0;
static volatile uint32_t g_can_pending_error_flags = 0;
static volatile uint32_t g_can_last_hal_error = HAL_CAN_ERROR_NONE;
static uint32_t g_last_can_recovery_tick = 0;

/* Private function prototypes -----------------------------------------------*/
static void CAN_ProcessTxQueue(void);
static void CAN_ProcessRxMessage(CAN_Message_t *msg);
static HAL_StatusTypeDef CAN_TransmitMessage(CAN_Message_t *msg);
static void CAN_ConfigureFilters(void);
static uint32_t CAN_GetNotificationFlags(void);
static void CAN_RequestRecoveryFromISR(void);
static void CAN_ProcessPendingErrors(void);
static HAL_StatusTypeDef CAN_PerformRecovery(void);

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Initialize CAN manager
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_Manager_Init(void)
{
    // Create TX message queue
    CANTxQueueHandle = osMessageQueueNew(CAN_TX_QUEUE_SIZE, sizeof(CAN_Message_t), NULL);
    if (CANTxQueueHandle == NULL) {
        return HAL_ERROR;
    }
    
    // Create RX message queue
    CANRxQueueHandle = osMessageQueueNew(CAN_RX_QUEUE_SIZE, sizeof(CAN_Message_t), NULL);
    if (CANRxQueueHandle == NULL) {
        return HAL_ERROR;
    }
    
    // NOTE: CAN filter is now configured in MX_CAN1_Init() BEFORE HAL_CAN_Start()
    // This is critical - filters must be configured before starting CAN!
    
    // Activate CAN notifications (CAN must already be started)
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_GetNotificationFlags()) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Reset statistics
    CAN_ResetStatistics();
    
    return HAL_OK;
}

/**
  * @brief  Get full notification mask used by CAN manager
  * @retval Notification bitmask
  */
static uint32_t CAN_GetNotificationFlags(void)
{
    return (CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_RX_FIFO1_MSG_PENDING |
            CAN_IT_ERROR |
            CAN_IT_BUSOFF |
            CAN_IT_ERROR_WARNING |
            CAN_IT_ERROR_PASSIVE |
            CAN_IT_LAST_ERROR_CODE);
}

/**
  * @brief  Request CAN recovery from ISR-safe context
  * @retval None
  */
static void CAN_RequestRecoveryFromISR(void)
{
    g_can_recovery_requested = 1;
}

/**
  * @brief  Apply pending CAN communication error flags in task context
  * @retval None
  */
static void CAN_ProcessPendingErrors(void)
{
    uint32_t pending_flags = g_can_pending_error_flags;

    if (pending_flags == 0U) {
        return;
    }

    g_can_pending_error_flags = 0U;

    if ((pending_flags & ERROR_CAN_RX_OVERFLOW) != 0U) {
        ErrorMgr_SetError(ERROR_CAN_RX_OVERFLOW);
    }

    if ((pending_flags & ERROR_CAN_TX_TIMEOUT) != 0U) {
        ErrorMgr_SetError(ERROR_CAN_TX_TIMEOUT);
    }

    if ((pending_flags & ERROR_CAN_BUS_OFF) != 0U) {
        ErrorMgr_SetError(ERROR_CAN_BUS_OFF);
    }
}

/**
  * @brief  Recover CAN peripheral after severe error state
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef CAN_PerformRecovery(void)
{
    if (HAL_CAN_Stop(&hcan1) != HAL_OK) {
        return HAL_ERROR;
    }

    CAN_ConfigureFilters();

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_GetNotificationFlags()) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_CAN_ResetError(&hcan1);

    g_can_recovery_requested = 0;
    g_can_pending_error_flags &= ~(ERROR_CAN_BUS_OFF | ERROR_CAN_TX_TIMEOUT | ERROR_CAN_RX_OVERFLOW);
    g_can_last_hal_error = HAL_CAN_ERROR_NONE;
    g_last_can_recovery_tick = osKernelGetTickCount();

    can_stats.can_recovery_count++;

    ErrorMgr_ClearError(ERROR_CAN_BUS_OFF);
    ErrorMgr_ClearError(ERROR_CAN_TX_TIMEOUT);
    ErrorMgr_ClearError(ERROR_CAN_RX_OVERFLOW);

    return HAL_OK;
}

/**
  * @brief  Configure CAN filters to accept messages for current module ID
  * @retval None
  * @note   TEMPORARY: Accept ALL extended CAN messages for debugging
  *         TODO: Restore module-specific filtering once RX is working
  */
static void CAN_ConfigureFilters(void)
{
    CAN_FilterTypeDef filterConfig;
    
    // TEMPORARY DEBUG: Accept ALL extended CAN IDs
    // This disables filtering to verify RX path is working
    
    filterConfig.FilterBank = 0;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    
    // Accept all extended IDs: ID=0, Mask=0 means "don't care about any bits"
    filterConfig.FilterIdHigh = 0x0000;
    filterConfig.FilterIdLow = 0x0004;   // Only IDE bit set (extended ID)
    filterConfig.FilterMaskIdHigh = 0x0000;  // Don't care about any ID bits
    filterConfig.FilterMaskIdLow = 0x0004;   // But we DO care about IDE bit (only extended)
    
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation = ENABLE;
    filterConfig.SlaveStartFilterBank = 14;
    
    HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
}

/**
  * @brief  Reconfigure CAN filters for new module ID
  * @note   Call this after changing module ID to update RX filters
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_ReconfigureFilters(void)
{
    // Reconfigure filters with new module ID
    CAN_ConfigureFilters();
    return HAL_OK;
}

/**
  * @brief  Send CAN message (non-blocking, queues message)
  * @param  id: CAN message ID (29-bit extended, max 0x1FFFFFFF)
  * @param  data: Pointer to data buffer (up to 8 bytes)
  * @param  length: Data length (0-8 bytes)
  * @param  priority: Message priority (0 = highest, 3 = lowest)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t length, uint8_t priority)
{
    CAN_Message_t msg;
    
    // Validate inputs (29-bit extended ID max)
    if (length > 8 || id > 0x1FFFFFFF) {
        return HAL_ERROR;
    }
    
    // ID is already configured with module offset at startup
    // No need to apply offset here anymore
    
    // Prepare message
    msg.id = id;
    msg.length = length;
    msg.priority = priority;
    msg.timestamp = osKernelGetTickCount();
    
    // Copy data
    if (data != NULL && length > 0) {
        memcpy(msg.data, data, length);
    }
    
    // Add to queue (non-blocking with timeout in ms)
    if (osMessageQueuePut(CANTxQueueHandle, &msg, priority, CAN_TX_TIMEOUT_MS) != osOK) {
        can_stats.tx_queue_full_count++;
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
  * @brief  Send CAN message with extended ID (legacy compatibility)
  * @note   All IDs are extended in this application, this calls CAN_SendMessage
  * @param  id: Extended CAN message ID (29-bit)
  * @param  data: Pointer to data buffer (up to 8 bytes)
  * @param  length: Data length (0-8 bytes)
  * @param  priority: Message priority (0 = highest, 3 = lowest)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendMessageExt(uint32_t id, uint8_t *data, uint8_t length, uint8_t priority)
{
    // Simply call CAN_SendMessage since all IDs are extended
    return CAN_SendMessage(id, data, length, priority);
}

/**
  * @brief  Transmit single message to CAN hardware
  * @param  msg: Pointer to message structure
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef CAN_TransmitMessage(CAN_Message_t *msg)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    HAL_StatusTypeDef status;
    uint8_t retry_count = 0;
    
    // Configure TX header for extended ID
    TxHeader.ExtId = msg->id;
    TxHeader.StdId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = msg->length;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // Attempt transmission with retries
    while (retry_count < CAN_MAX_RETRIES) {
        status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, msg->data, &TxMailbox);
        
        if (status == HAL_OK) {
            can_stats.tx_success_count++;
            return HAL_OK;
        }
        
        retry_count++;
        
        // Brief delay before retry (1ms)
        if (retry_count < CAN_MAX_RETRIES) {
            osDelay(1);
        }
    }
    
    // All retries failed
    can_stats.tx_error_count++;

    if ((HAL_CAN_GetError(&hcan1) & HAL_CAN_ERROR_TIMEOUT) != 0U) {
        ErrorMgr_SetError(ERROR_CAN_TX_TIMEOUT);
    }

    if ((HAL_CAN_GetError(&hcan1) & HAL_CAN_ERROR_BOF) != 0U) {
        g_can_pending_error_flags |= ERROR_CAN_BUS_OFF;
        CAN_RequestRecoveryFromISR();
    }

    return HAL_ERROR;
}

/**
  * @brief  Process TX queue and transmit messages
  * @retval None
  */
static void CAN_ProcessTxQueue(void)
{
    CAN_Message_t msg;
    
    // Try to send as many messages as possible
    while (osMessageQueueGet(CANTxQueueHandle, &msg, NULL, 0) == osOK) {
        // Transmit the message
        if (CAN_TransmitMessage(&msg) != HAL_OK) {
            // If transmission failed, could re-queue message here if desired
            // For now, we just count it as an error
            break;  // Stop processing queue if hardware is busy
        }
    }
}

/**
  * @brief  Process received CAN message
  * @param  msg: Pointer to received message
  * @retval None
  */
static void CAN_ProcessRxMessage(CAN_Message_t *msg)
{
    // RX messages not for this module are filtered out
    // in the HAL callback function

    can_stats.rx_message_count++;

    // Check for debug info request FIRST (broadcast message)
    if (msg->id == CAN_DEBUG_REQUEST_ID) {
        // Send debug info response
        CAN_SendDebugInfo();
        // Also send I2C diagnostics
        CAN_SendI2CDiagnostics();
        return;
    }

    // Strip module ID to get base message ID (for message type identification)
    uint32_t base_id = msg->id & 0xFFFF0FFF;

    // Check for configuration command message
    if (base_id == (CAN_CONFIG_CMD_BASE & 0xFFFF0FFF)) {
        Config_ProcessCANCommand(msg->data, msg->length);
        return;
    }
    
    // Check for reset command message
    if (base_id == (CAN_RESET_CMD_BASE & 0xFFFF0FFF)) {
        // Reset command - trigger NVIC system reset
        NVIC_SystemReset();
        return;  // Never reached, but good practice
    }
    
    // Check for BMS chip reset command message
    if (base_id == (CAN_BMS_RESET_CMD_BASE & 0xFFFF0FFF)) {
        // BMS chip reset command - signal reset handler task via semaphore
        // This prevents blocking the CAN manager during the 600ms reset sequence
        uint8_t ack_data[8] = {0};
        osStatus_t sem_result = osOK;
        
        // Signal the BMS reset handler task (non-blocking)
        if (BMSResetSemHandle != NULL) {
            sem_result = osSemaphoreRelease(BMSResetSemHandle);
        } else {
            sem_result = osError;
        }
        
        // Prepare acknowledgement message
        // Byte 0: Status (0x00 = success/queued, 0x01 = fail/already pending)
        // Bytes 1-7: Reserved
        ack_data[0] = (sem_result == osOK) ? 0x00 : 0x01;
        ack_data[1] = 0x00;
        ack_data[2] = 0x00;
        ack_data[3] = 0x00;
        ack_data[4] = 0x00;
        ack_data[5] = 0x00;
        ack_data[6] = 0x00;
        ack_data[7] = 0x00;
        
        // Send acknowledgement
        CAN_SendMessage(CAN_BMS_RESET_ACK_ID, ack_data, 8, CAN_PRIORITY_HIGH);
        return;
    }
    
    // Check for balance command message
    if (base_id == (CAN_BALANCE_CMD_BASE & 0xFFFF0FFF)) {
        uint8_t ack_data[8] = {0};
        
        // Process balance command and get status
        uint8_t balance_status = BalanceMgr_ProcessCommand();
        
        // Prepare acknowledgement message
        // Byte 0: Status (BALANCE_STATUS_*)
        // Byte 1-2: Time remaining in ms (little-endian)
        // Bytes 3-7: Reserved
        uint32_t time_remaining = BalanceMgr_GetTimeRemaining();
        ack_data[0] = balance_status;
        ack_data[1] = (uint8_t)(time_remaining & 0xFF);
        ack_data[2] = (uint8_t)((time_remaining >> 8) & 0xFF);
        ack_data[3] = 0x00;
        ack_data[4] = 0x00;
        ack_data[5] = 0x00;
        ack_data[6] = 0x00;
        ack_data[7] = 0x00;
        
        // Send acknowledgement
        CAN_SendMessage(CAN_BALANCE_ACK_ID, ack_data, 8, CAN_PRIORITY_HIGH);
        return;
    }
    
    // Check for balance configuration message
    if (base_id == (CAN_BALANCE_CFG_BASE & 0xFFFF0FFF)) {
        // Parse balance config message
        // Bytes 0-1: Target voltage in mV (little-endian)
        // Byte 2: Max cells per chip (1-9)
        // Bytes 3-7: Reserved
        uint16_t target_voltage = (uint16_t)(msg->data[0]) | ((uint16_t)(msg->data[1]) << 8);
        uint8_t max_cells = msg->data[2];
        
        // Process the configuration
        BalanceMgr_ProcessConfig(target_voltage, max_cells);
        
        // Trigger execution immediately after config update
        BalanceMgr_Execute();
        return;
    }
    
}

/**
  * @brief  Main CAN manager task
  * @param  argument: Not used
  * @retval None
  */
void CAN_ManagerTask(void *argument)
{
    CAN_Message_t rx_msg;
    uint32_t last_heartbeat_tick = 0;
    uint32_t last_stats_tick = 0;
    uint32_t last_uptime_tick = 0;
    uint32_t last_balance_status_tick = 0;
    uint32_t last_can_health_tick = 0;
    uint32_t current_tick = 0;
    
    // Wait 100ms for system to stabilize
    osDelay(100);
    
    // Initialize timing
    last_heartbeat_tick = osKernelGetTickCount();
    last_stats_tick = osKernelGetTickCount();
    last_uptime_tick = osKernelGetTickCount();
    last_balance_status_tick = osKernelGetTickCount();
    last_can_health_tick = osKernelGetTickCount();
    
    /* Infinite loop */
    for(;;)
    {
        current_tick = osKernelGetTickCount();
        
        // Process any received messages from RX queue
        while (osMessageQueueGet(CANRxQueueHandle, &rx_msg, NULL, 0) == osOK) {
            CAN_ProcessRxMessage(&rx_msg);
        }
        
        // Process TX queue and send pending messages
        CAN_ProcessTxQueue();
        
        // Send heartbeat message at regular intervals
        if ((current_tick - last_heartbeat_tick) >= CAN_HEARTBEAT_INTERVAL_MS) {
            CAN_SendHeartbeat();
            last_heartbeat_tick = current_tick;
        }
        
        // Send statistics message at regular intervals (every 1 second)
        if ((current_tick - last_stats_tick) >= 1000) {
            CAN_SendStatistics();
            last_stats_tick = current_tick;
        }
        
        // Update uptime counter every second
        if ((current_tick - last_uptime_tick) >= 1000) {
            ErrorMgr_UpdateUptime();
            last_uptime_tick = current_tick;
        }

        // Periodic CAN health monitor and automated fault recovery
        if ((current_tick - last_can_health_tick) >= CAN_HEALTH_CHECK_INTERVAL_MS) {
            uint32_t can_error = HAL_CAN_GetError(&hcan1);

            CAN_ProcessPendingErrors();

            if (((can_error & HAL_CAN_ERROR_BOF) != 0U) ||
                ((g_can_last_hal_error & HAL_CAN_ERROR_BOF) != 0U)) {
                g_can_pending_error_flags |= ERROR_CAN_BUS_OFF;
                CAN_RequestRecoveryFromISR();
            }

            if ((g_can_recovery_requested != 0U) &&
                ((current_tick - g_last_can_recovery_tick) >= CAN_RECOVERY_COOLDOWN_MS)) {
                (void)CAN_PerformRecovery();
            }

            last_can_health_tick = current_tick;
        }
        
        // Send balance status every 1 second during balancing
        if (BalanceMgr_IsBalancing()) {
            if ((current_tick - last_balance_status_tick) >= BALANCE_STATUS_INTERVAL_MS) {
                BalanceMgr_SendStatus();
                BalanceMgr_SendDetailedStatus();  // Also send detailed readback from each chip
                last_balance_status_tick = current_tick;
            }
        } else {
            // Keep timer aligned while idle to avoid stale interval accumulation
            last_balance_status_tick = current_tick;
        }
        
        // Check balance timeout (runs every 10ms loop iteration)
        BalanceMgr_CheckTimeout();
        
        // Small delay to prevent task from hogging CPU (10ms)
        // The task will wake up on new messages or periodically
        osDelay(10);
    }
}

/**
  * @brief  Get CAN statistics
  * @param  stats: Pointer to statistics structure
  * @retval None
  */
void CAN_GetStatistics(CAN_Statistics_t *stats)
{
    if (stats != NULL) {
        memcpy(stats, &can_stats, sizeof(CAN_Statistics_t));
    }
}

/**
  * @brief  Reset CAN statistics
  * @retval None
  */
void CAN_ResetStatistics(void)
{
    memset(&can_stats, 0, sizeof(CAN_Statistics_t));
}

/**
  * @brief  Get TX queue fill level
  * @retval Number of messages waiting in TX queue
  */
uint32_t CAN_GetTxQueueCount(void)
{
    return osMessageQueueGetCount(CANTxQueueHandle);
}

/**
  * @brief  Get RX queue fill level
  * @retval Number of unprocessed received messages
  */
uint32_t CAN_GetRxQueueCount(void)
{
    return osMessageQueueGetCount(CANRxQueueHandle);
}

/**
  * @brief  Flush TX queue (clear all pending messages)
  * @retval Number of messages flushed
  */
uint32_t CAN_FlushTxQueue(void)
{
    CAN_Message_t dummy_msg;
    uint32_t count = 0;
    
    while (osMessageQueueGet(CANTxQueueHandle, &dummy_msg, NULL, 0) == osOK) {
        count++;
    }
    
    return count;
}

/* CAN RX Interrupt Callbacks ------------------------------------------------*/

/**
  * @brief  CAN RX FIFO 0 message pending callback
  * @param  hcan: pointer to CAN handle
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    CAN_Message_t msg;
    
    // Get message from FIFO
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, msg.data) == HAL_OK) {
        // Store message details (all messages are extended ID)
        msg.id = RxHeader.ExtId;
        msg.length = RxHeader.DLC;
        msg.priority = 0;  // RX messages don't have priority
        msg.timestamp = osKernelGetTickCount();
        
        // Ignore all messages not for this module
        if (CAN_IsMessageForThisModule(msg.id)) {
                // Add to RX queue (from ISR context)
            if (osMessageQueuePut(CANRxQueueHandle, &msg, 0, 0) != osOK) {
                can_stats.rx_queue_full_count++;
            }
        }
    }
}

/**
  * @brief  CAN RX FIFO 1 message pending callback
  * @param  hcan: pointer to CAN handle
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    CAN_Message_t msg;


    // Get message from FIFO
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, msg.data) == HAL_OK) {
        // Store message details (all messages are extended ID)
        msg.id = RxHeader.ExtId;
        msg.length = RxHeader.DLC;
        msg.priority = 0;  // RX messages don't have priority
        msg.timestamp = osKernelGetTickCount();
        
        // Ignore all messages not for this module
        if (CAN_IsMessageForThisModule(msg.id)) {
            // Add to RX queue (from ISR context)
            if (osMessageQueuePut(CANRxQueueHandle, &msg, 0, 0) != osOK) {
                can_stats.rx_queue_full_count++;
            }
        }

    }
}

/**
  * @brief  Check if CAN message is meant for this module through ID
  * @param  hcan: pointer to CAN handle
  * @retval None
  */
bool CAN_IsMessageForThisModule(uint32_t can_id) {
    // Module ID is encoded in bits 12:15 of rx can id
    // TODO: handle messages that might not have this format?
    uint8_t this_module_id = Config_GetModuleID();
    uint8_t rx_module_id = (can_id >> 12) & 0x0F;
    
    return (
        rx_module_id == this_module_id ||   // Specifically for this module
        can_id == CAN_DEBUG_REQUEST_ID      // Debug ID (all modules)
    );
}

/**
  * @brief  CAN error callback
  * @param  hcan: pointer to CAN handle
  * @retval None
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error = HAL_CAN_GetError(hcan);

    if (error == HAL_CAN_ERROR_NONE) {
        return;
    }

    can_stats.can_error_count++;
    g_can_last_hal_error = error;

    if ((error & HAL_CAN_ERROR_BOF) != 0U) {
        can_stats.bus_off_count++;
        g_can_pending_error_flags |= ERROR_CAN_BUS_OFF;
        CAN_RequestRecoveryFromISR();
    }

    if ((error & HAL_CAN_ERROR_EWG) != 0U) {
        can_stats.error_warning_count++;
    }

    if ((error & HAL_CAN_ERROR_EPV) != 0U) {
        can_stats.error_passive_count++;
    }

    if ((error & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1)) != 0U) {
        can_stats.rx_overflow_count++;
        g_can_pending_error_flags |= ERROR_CAN_RX_OVERFLOW;
    }

    if ((error & HAL_CAN_ERROR_TIMEOUT) != 0U) {
        g_can_pending_error_flags |= ERROR_CAN_TX_TIMEOUT;
    }
}

/**
  * @brief  Send BMS heartbeat message
  * @note   Heartbeat format (8 bytes):
  *         Byte 0: BMS State
  *         Byte 1: Error flags byte 0 (Temperature errors)
  *         Byte 2: Error flags byte 1 (Voltage errors)
  *         Byte 3: Error flags byte 2 (Current errors)
  *         Byte 4: Error flags byte 3 (Communication errors)
  *         Byte 5: Warning flags summary (any warnings = 0xFF, none = 0x00)
  *         Byte 6-7: Fault count (16-bit)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendHeartbeat(void)
{
    Error_Manager_t status;
    uint8_t heartbeat_data[8];
    
    // Get current error manager status
    ErrorMgr_GetStatus(&status);
    
    // Pack heartbeat message (state comes from state machine)
    heartbeat_data[0] = (uint8_t)StateMachine_GetState();         // BMS state
    heartbeat_data[1] = (uint8_t)(status.error_flags & 0xFF);     // Error byte 0
    heartbeat_data[2] = (uint8_t)((status.error_flags >> 8) & 0xFF);   // Error byte 1
    heartbeat_data[3] = (uint8_t)((status.error_flags >> 16) & 0xFF);  // Error byte 2
    heartbeat_data[4] = (uint8_t)((status.error_flags >> 24) & 0xFF);  // Error byte 3
    heartbeat_data[5] = (status.warning_flags != 0) ? 0xFF : 0x00;     // Warning summary
    heartbeat_data[6] = (uint8_t)(status.fault_count & 0xFF);          // Fault count low byte
    heartbeat_data[7] = (uint8_t)((status.fault_count >> 8) & 0xFF);   // Fault count high byte
    
    // Send heartbeat with high priority
    return CAN_SendMessage(CAN_BMS_HEARTBEAT_ID, heartbeat_data, 8, CAN_PRIORITY_CRITICAL);
}

/**
  * @brief  Send CAN statistics message for diagnostics
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendStatistics(void)
{
    uint8_t stats_data[8];
    
    // Pack CAN statistics message
    // Bytes 0-1: RX message count (16-bit)
    stats_data[0] = (uint8_t)(can_stats.rx_message_count & 0xFF);
    stats_data[1] = (uint8_t)((can_stats.rx_message_count >> 8) & 0xFF);
    
    // Bytes 2-3: TX success count (16-bit)
    stats_data[2] = (uint8_t)(can_stats.tx_success_count & 0xFF);
    stats_data[3] = (uint8_t)((can_stats.tx_success_count >> 8) & 0xFF);
    
    // Byte 4: TX error count
    stats_data[4] = (uint8_t)(can_stats.tx_error_count & 0xFF);
    
    // Byte 5: RX queue full count
    stats_data[5] = (uint8_t)(can_stats.rx_queue_full_count & 0xFF);
    
    // Byte 6: Bus-off count
    stats_data[6] = (uint8_t)(can_stats.bus_off_count & 0xFF);
    
    // Byte 7: TX queue full count
    stats_data[7] = (uint8_t)(can_stats.tx_queue_full_count & 0xFF);
    
    // Send statistics with normal priority
    return CAN_SendMessage(CAN_BMS_STATS_ID, stats_data, 8, CAN_PRIORITY_NORMAL);
}

/**
  * @brief  Send debug information message
  * @note   Debug info format (8 bytes):
  *         Byte 0: Module ID (0-15)
  *         Byte 1: Free heap MSB (in 256-byte units)
  *         Byte 2: Min free heap MSB (in 256-byte units)
  *         Byte 3: Reserved (CPU usage if implemented)
  *         Bytes 4-7: Uptime in seconds (32-bit)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendDebugInfo(void)
{
    uint8_t debug_data[8];

    // Get current module ID
    extern uint8_t Config_GetModuleID(void);
    uint8_t module_id = Config_GetModuleID();

    // Get heap memory statistics (in bytes)
    size_t free_heap = xPortGetFreeHeapSize();
    size_t min_free_heap = xPortGetMinimumEverFreeHeapSize();

    // Get uptime in seconds (convert from milliseconds)
    uint32_t uptime_sec = osKernelGetTickCount() / 1000;

    // Get bootloader validity flag
    extern uint8_t g_bootloader_app_valid;
    
    // Pack debug information message
    debug_data[0] = module_id;                              // Byte 0: Module ID
    debug_data[1] = (uint8_t)((free_heap >> 8) & 0xFF);     // Byte 1: Free heap MSB (in 256-byte units)
    debug_data[2] = (uint8_t)((min_free_heap >> 8) & 0xFF); // Byte 2: Min free heap MSB (in 256-byte units)
    debug_data[3] = g_bootloader_app_valid;                 // Byte 3: Bootloader validity (1=valid, 0=invalid)
    debug_data[4] = (uint8_t)(uptime_sec & 0xFF);           // Byte 4: Uptime LSB
    debug_data[5] = (uint8_t)((uptime_sec >> 8) & 0xFF);    // Byte 5: Uptime
    debug_data[6] = (uint8_t)((uptime_sec >> 16) & 0xFF);   // Byte 6: Uptime
    debug_data[7] = (uint8_t)((uptime_sec >> 24) & 0xFF);   // Byte 7: Uptime MSB

    // Send debug info with high priority
    return CAN_SendMessage(CAN_DEBUG_RESPONSE_ID, debug_data, 8, CAN_PRIORITY_HIGH);
}

/**
  * @brief  Send I2C diagnostics message
  * @note   I2C Diagnostics format (8 bytes):
  *         Byte 0: I2C1 error code (HAL_I2C_ERROR_*)
  *         Byte 1: I2C3 error code (HAL_I2C_ERROR_*)
  *         Byte 2: I2C1 state (HAL_I2C_STATE_*)
  *         Byte 3: I2C3 state (HAL_I2C_STATE_*)
  *         Bytes 4-7: Reserved for future use
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendI2CDiagnostics(void)
{
    uint8_t diag_data[8] = {0};
    
    // Get I2C error codes for diagnostics
    extern I2C_HandleTypeDef hi2c1;
    extern I2C_HandleTypeDef hi2c3;
    extern uint32_t BQ_GetLastI2C3Error(void);
    
    uint32_t i2c1_error = HAL_I2C_GetError(&hi2c1);
    uint32_t i2c3_error = BQ_GetLastI2C3Error();
    
    HAL_I2C_StateTypeDef i2c1_state = HAL_I2C_GetState(&hi2c1);
    HAL_I2C_StateTypeDef i2c3_state = HAL_I2C_GetState(&hi2c3);
    
    // Pack I2C diagnostics message
    diag_data[0] = (uint8_t)(i2c1_error & 0xFF);        // Byte 0: I2C1 error code
    diag_data[1] = (uint8_t)(i2c3_error & 0xFF);        // Byte 1: I2C3 error code
    diag_data[2] = (uint8_t)(i2c1_state & 0xFF);        // Byte 2: I2C1 state
    diag_data[3] = (uint8_t)(i2c3_state & 0xFF);        // Byte 3: I2C3 state
    diag_data[4] = 0;                                   // Byte 4: Reserved
    diag_data[5] = 0;                                   // Byte 5: Reserved
    diag_data[6] = 0;                                   // Byte 6: Reserved
    diag_data[7] = 0;                                   // Byte 7: Reserved
    
    // Send I2C diagnostics with high priority
    return CAN_SendMessage(CAN_I2C_DIAG_ID, diag_data, 8, CAN_PRIORITY_HIGH);
}