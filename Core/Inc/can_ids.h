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
/* 
 * Module ID Offset: Bits 12-15 (0x0000X000) are used for module ID
 * IDs are defined as macros that incorporate the module number at compile/init time
 * Example: CAN_ID_TEMP(5) = 0x08F00000 | (5 << 12) = 0x08F05000
 * 
 * Base pattern: 0x08F0XYZZ where:
 *   - X = Module ID (0-F, inserted via bitwise OR)
 *   - Y = Message category
 *   - ZZ = Specific message within category
 */

/* Module ID bit shift (bits 12-15) */
#define CAN_MODULE_ID_SHIFT     12
#define CAN_MODULE_ID_MASK      0x0000F000

/* Helper macro to create CAN ID with module number */
#define CAN_ID(base, module)    ((base) | ((module) << CAN_MODULE_ID_SHIFT))

/* Temperature Monitoring Messages */
#define CAN_TEMP_BASE           0x08F00000  /**< Base for temperature messages */
#define CAN_TEMP_RAW_BASE       0x08F00100  /**< Base for raw ADC diagnostic messages */

/* Cell Voltage Messages */
#define CAN_VOLTAGE_0_BASE      0x08F00200  /**< Base for voltage message 0 (cells 1-3) */
#define CAN_VOLTAGE_1_BASE      0x08F00201  /**< Base for voltage message 1 (cells 4-6) */
#define CAN_VOLTAGE_2_BASE      0x08F00202  /**< Base for voltage message 2 (cells 7-9) */
#define CAN_VOLTAGE_3_BASE      0x08F00203  /**< Base for voltage message 3 (cells 10-12) */
#define CAN_VOLTAGE_4_BASE      0x08F00204  /**< Base for voltage message 4 (cells 13-15) */
#define CAN_VOLTAGE_5_BASE      0x08F00205  /**< Base for voltage message 5 (cells 16-18) */

/* BQ76952 Chip Status Messages */
#define CAN_BMS1_STATUS_BASE    0x08F00206  /**< Base for BMS1 chip status (stack voltage, alarm, temp) */
#define CAN_BMS2_STATUS_BASE    0x08F00207  /**< Base for BMS2 chip status (stack voltage, alarm, temp) */

/* BMS Status Messages */
#define CAN_BMS_HEARTBEAT_BASE  0x08F00300  /**< Base for BMS heartbeat/status message */
#define CAN_BMS_STATS_BASE      0x08F00301  /**< Base for BMS CAN statistics (RX/TX counters) */
// #define CAN_BMS_ERROR_BASE      0x08F00302  /**< Base for BMS detailed error status (optional) */

/* Current Monitoring Messages */
/* TODO: Add current monitoring CAN IDs here when defined */
// #define CAN_CURRENT_BASE        0x08F00400  /**< Base for current measurement */

/* State of Charge (SOC) Messages */
/* TODO: Add SOC CAN IDs here when defined */
// #define CAN_SOC_BASE            0x08F00500  /**< Base for state of charge */

/* Command/Control Messages */
#define CAN_CONFIG_CMD_BASE     0x08F00F00  /**< Base for configuration command (module ID in bits 15:12, e.g. 0x08F00F00, 0x08F01F00, etc.) */
#define CAN_CONFIG_ACK_BASE     0x08F00F01  /**< Base for configuration command acknowledgement */
#define CAN_RESET_CMD_BASE      0x08F00F02  /**< Base for reset command (module-specific) */
#define CAN_BMS_RESET_CMD_BASE  0x08F00F03  /**< Base for BQ76952 chip reset command (module-specific) */
#define CAN_BMS_RESET_ACK_BASE  0x08F00F04  /**< Base for BQ76952 chip reset acknowledgement */
#define CAN_DEBUG_REQUEST_ID    0x08F00F10  /**< Debug info request (broadcast - no module ID) */
#define CAN_DEBUG_RESPONSE_BASE 0x08F00F11  /**< Base for debug info response */
#define CAN_I2C_DIAG_BASE       0x08F00F12  /**< Base for I2C diagnostics response */
// #define CAN_CMD_BASE            0x08F00600  /**< Base for command messages */

/* Bootloader Messages (for reference - used in bootloader project) */
// #define CAN_BOOTLOADER_ID       0x08000700  /**< Bootloader CAN ID (Extended) */
// #define CAN_HOST_ID             0x08000701  /**< Host/PC CAN ID (Extended) */

/* Actual CAN IDs (computed at runtime with module_id) */
/* These extern variables are set during Config_Init() */
extern uint32_t CAN_TEMP_ID;         /**< Temperature message ID (with module offset) */
extern uint32_t CAN_TEMP_RAW_ID;     /**< Raw ADC diagnostic message ID (with module offset) */
extern uint32_t CAN_VOLTAGE_0_ID;    /**< Voltage message 0 ID (cells 1-3, with module offset) */
extern uint32_t CAN_VOLTAGE_1_ID;    /**< Voltage message 1 ID (cells 4-6, with module offset) */
extern uint32_t CAN_VOLTAGE_2_ID;    /**< Voltage message 2 ID (cells 7-9, with module offset) */
extern uint32_t CAN_VOLTAGE_3_ID;    /**< Voltage message 3 ID (cells 10-12, with module offset) */
extern uint32_t CAN_VOLTAGE_4_ID;    /**< Voltage message 4 ID (cells 13-15, with module offset) */
extern uint32_t CAN_VOLTAGE_5_ID;    /**< Voltage message 5 ID (cells 16-18, with module offset) */
extern uint32_t CAN_BMS1_STATUS_ID;  /**< BMS1 chip status ID (with module offset) */
extern uint32_t CAN_BMS2_STATUS_ID;  /**< BMS2 chip status ID (with module offset) */
extern uint32_t CAN_BMS_HEARTBEAT_ID; /**< BMS heartbeat ID (with module offset) */
extern uint32_t CAN_BMS_STATS_ID;    /**< BMS statistics ID (with module offset) */
extern uint32_t CAN_CONFIG_ACK_ID;   /**< Config ACK ID (with module offset) */
extern uint32_t CAN_RESET_CMD_ID;    /**< Reset command ID (with module offset) */
extern uint32_t CAN_BMS_RESET_CMD_ID; /**< BQ76952 reset command ID (with module offset) */
extern uint32_t CAN_BMS_RESET_ACK_ID; /**< BQ76952 reset ACK ID (with module offset) */
extern uint32_t CAN_DEBUG_RESPONSE_ID; /**< Debug response ID (with module offset) */
extern uint32_t CAN_I2C_DIAG_ID;     /**< I2C diagnostics ID (with module offset) */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_IDS_H */
