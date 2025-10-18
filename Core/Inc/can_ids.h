/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can_ids.h
  * @brief          : CAN message ID definitions
  ******************************************************************************
  * @attention
  *
  * This file contains all CAN message ID definitions used throughout the
  * BMS firmware. All IDs are 29-bit extended format.
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

#ifndef __CAN_IDS_H
#define __CAN_IDS_H

#ifdef __cplusplus
extern "C" {
#endif

/* CAN Message ID Definitions ------------------------------------------------*/
/* All IDs are 29-bit extended format (max value: 0x1FFFFFFF) */

/* Temperature Monitoring Messages */
#define CAN_TEMP_BASE_ID        0x08FF0000  /**< Base ID for temperature messages (extended) */
#define CAN_TEMP_RAW_BASE_ID    0x08FF0100  /**< Base ID for raw ADC diagnostic messages (extended) */

/* Cell Voltage Messages */
/* TODO: Add cell voltage CAN IDs here when defined */
// #define CAN_VOLTAGE_BASE_ID     0x08FF0200  /**< Base ID for voltage messages */

/* BMS Status Messages */
#define CAN_BMS_HEARTBEAT_ID    0x08FF0300  /**< BMS heartbeat/status message */
// #define CAN_BMS_ERROR_ID        0x08FF0301  /**< BMS detailed error status (optional) */

/* Current Monitoring Messages */
/* TODO: Add current monitoring CAN IDs here when defined */
// #define CAN_CURRENT_ID          0x08FF0400  /**< Current measurement */

/* State of Charge (SOC) Messages */
/* TODO: Add SOC CAN IDs here when defined */
// #define CAN_SOC_ID              0x08FF0500  /**< State of charge */

/* Command/Control Messages */
/* TODO: Add command CAN IDs here when defined */
// #define CAN_CMD_BASE_ID         0x08FF0600  /**< Base ID for command messages */

/* Bootloader Messages (for reference - used in bootloader project) */
// #define CAN_BOOTLOADER_ID       0x08000700  /**< Bootloader CAN ID (Extended) */
// #define CAN_HOST_ID             0x08000701  /**< Host/PC CAN ID (Extended) */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_IDS_H */
