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

/* Private variables ---------------------------------------------------------*/
osMessageQueueId_t CANTxQueueHandle = NULL;
osMessageQueueId_t CANRxQueueHandle = NULL;

static CAN_RxCallback_t rx_callback = NULL;
static CAN_Statistics_t can_stats = {0};

/* Private function prototypes -----------------------------------------------*/
static void CAN_ProcessTxQueue(void);
static void CAN_ProcessRxMessage(CAN_Message_t *msg);
static HAL_StatusTypeDef CAN_TransmitMessage(CAN_Message_t *msg);
static void CAN_ConfigureFilters(void);

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
    
    // Configure CAN filters to accept all messages
    CAN_ConfigureFilters();
    
    // Activate CAN RX FIFO notifications
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | 
                                              CAN_IT_RX_FIFO1_MSG_PENDING |
                                              CAN_IT_ERROR |
                                              CAN_IT_BUSOFF) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Reset statistics
    CAN_ResetStatistics();
    
    return HAL_OK;
}

/**
  * @brief  Configure CAN filters to accept all extended ID messages
  * @retval None
  */
static void CAN_ConfigureFilters(void)
{
    CAN_FilterTypeDef filterConfig;
    
    // Configure filter 0 to accept all extended IDs (29-bit)
    filterConfig.FilterBank = 0;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterIdHigh = 0x0000;
    filterConfig.FilterIdLow = 0x0004;  // Set IDE bit for extended IDs
    filterConfig.FilterMaskIdHigh = 0x0000;
    filterConfig.FilterMaskIdLow = 0x0004;  // Mask only IDE bit
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation = ENABLE;
    filterConfig.SlaveStartFilterBank = 14;
    
    HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
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
    can_stats.rx_message_count++;
    
    // Call registered callback if available
    if (rx_callback != NULL) {
        rx_callback(msg);
    }
    
    // Add your message processing logic here
    // Example: Switch based on message ID
    switch (msg->id) {
        case 0x100:
            // Handle message ID 0x100
            // TODO: Add your message handling logic
            break;
            
        case 0x200:
            // Handle message ID 0x200
            // TODO: Add your message handling logic
            break;
            
        default:
            // Unknown message ID
            break;
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
    
    // Wait 100ms for system to stabilize
    osDelay(100);
    
    /* Infinite loop */
    for(;;)
    {
        // Process any received messages from RX queue
        while (osMessageQueueGet(CANRxQueueHandle, &rx_msg, NULL, 0) == osOK) {
            CAN_ProcessRxMessage(&rx_msg);
        }
        
        // Process TX queue and send pending messages
        CAN_ProcessTxQueue();
        
        // Small delay to prevent task from hogging CPU (10ms)
        // The task will wake up on new messages or periodically
        osDelay(10);
    }
}

/**
  * @brief  Register callback for received CAN messages
  * @param  callback: Function to call when message is received
  * @retval None
  */
void CAN_RegisterRxCallback(CAN_RxCallback_t callback)
{
    rx_callback = callback;
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
        
        // Add to RX queue (from ISR context)
        if (osMessageQueuePut(CANRxQueueHandle, &msg, 0, 0) != osOK) {
            can_stats.rx_queue_full_count++;
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
        
        // Add to RX queue (from ISR context)
        if (osMessageQueuePut(CANRxQueueHandle, &msg, 0, 0) != osOK) {
            can_stats.rx_queue_full_count++;
        }
    }
}

/**
  * @brief  CAN error callback
  * @param  hcan: pointer to CAN handle
  * @retval None
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error = HAL_CAN_GetError(hcan);
    
    // Check for bus-off condition
    if (error & HAL_CAN_ERROR_BOF) {
        can_stats.bus_off_count++;
        
        // Attempt to recover from bus-off
        // Note: May need to stop and restart CAN peripheral
    }
    
    // Handle other errors as needed
}
