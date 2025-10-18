/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can_manager.h
  * @brief          : CAN bus manager header
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

#ifndef __CAN_MANAGER_H
#define __CAN_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can_ids.h"
#include <string.h>

/* Defines -------------------------------------------------------------------*/
#define CAN_TX_QUEUE_SIZE 64        // Number of messages that can be queued
#define CAN_RX_QUEUE_SIZE 32        // Number of received messages to buffer
#define CAN_TX_TIMEOUT_MS 100       // Timeout for adding message to queue
#define CAN_MAX_RETRIES 3           // Maximum transmission retry attempts
#define CAN_HEARTBEAT_INTERVAL_MS 1000  // Heartbeat message interval (1 second)

/* Message Priority Levels */
#define CAN_PRIORITY_CRITICAL 0     // Safety-critical messages (highest)
#define CAN_PRIORITY_HIGH 1         // Important operational messages
#define CAN_PRIORITY_NORMAL 2       // Standard telemetry messages
#define CAN_PRIORITY_LOW 3          // Debug/diagnostic messages (lowest)

/* CAN Message Structure */
typedef struct {
    uint32_t id;                    // CAN message ID (29-bit extended)
    uint8_t data[8];                // Message data (up to 8 bytes)
    uint8_t length;                 // Data length (0-8)
    uint8_t priority;               // Message priority (0 = highest)
    uint32_t timestamp;             // Timestamp when message was queued
} CAN_Message_t;

/* CAN RX Callback Function Type */
typedef void (*CAN_RxCallback_t)(CAN_Message_t *msg);

/* CAN Statistics Structure */
typedef struct {
    uint32_t tx_success_count;      // Successfully transmitted messages
    uint32_t tx_error_count;        // Failed transmissions
    uint32_t tx_queue_full_count;   // Times TX queue was full
    uint32_t rx_message_count;      // Received messages
    uint32_t rx_queue_full_count;   // Times RX queue was full
    uint32_t bus_off_count;         // CAN bus-off events
} CAN_Statistics_t;

/* External Variables --------------------------------------------------------*/
extern osMessageQueueId_t CANTxQueueHandle;
extern osMessageQueueId_t CANRxQueueHandle;
extern CAN_HandleTypeDef hcan1;

/* Function Prototypes -------------------------------------------------------*/

/**
  * @brief  Initialize CAN manager
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_Manager_Init(void);

/**
  * @brief  Main CAN manager task
  * @param  argument: Not used
  * @retval None
  */
void CAN_ManagerTask(void *argument);

/**
  * @brief  Send CAN message (non-blocking, queues message)
  * @param  id: CAN message ID (29-bit extended, max 0x1FFFFFFF)
  * @param  data: Pointer to data buffer (up to 8 bytes)
  * @param  length: Data length (0-8 bytes)
  * @param  priority: Message priority (0 = highest, 3 = lowest)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t length, uint8_t priority);

/**
  * @brief  Send CAN message with extended ID (legacy compatibility)
  * @note   All IDs are extended in this application, this calls CAN_SendMessage
  * @param  id: Extended CAN message ID (29-bit)
  * @param  data: Pointer to data buffer (up to 8 bytes)
  * @param  length: Data length (0-8 bytes)
  * @param  priority: Message priority (0 = highest, 3 = lowest)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendMessageExt(uint32_t id, uint8_t *data, uint8_t length, uint8_t priority);

/**
  * @brief  Register callback for received CAN messages
  * @param  callback: Function to call when message is received
  * @retval None
  */
void CAN_RegisterRxCallback(CAN_RxCallback_t callback);

/**
  * @brief  Get CAN statistics
  * @param  stats: Pointer to statistics structure
  * @retval None
  */
void CAN_GetStatistics(CAN_Statistics_t *stats);

/**
  * @brief  Reset CAN statistics
  * @retval None
  */
void CAN_ResetStatistics(void);

/**
  * @brief  Get TX queue fill level
  * @retval Number of messages waiting in TX queue
  */
uint32_t CAN_GetTxQueueCount(void);

/**
  * @brief  Get RX queue fill level
  * @retval Number of unprocessed received messages
  */
uint32_t CAN_GetRxQueueCount(void);

/**
  * @brief  Flush TX queue (clear all pending messages)
  * @retval Number of messages flushed
  */
uint32_t CAN_FlushTxQueue(void);

/**
  * @brief  Send BMS heartbeat message
  * @note   Called periodically by CAN manager task
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendHeartbeat(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_MANAGER_H */
