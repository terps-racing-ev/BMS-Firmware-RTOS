/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : error_manager.h
  * @brief          : BMS Error Management System
  ******************************************************************************
  * @attention
  *
  * This file manages all error flags and fault conditions for the BMS.
  * Errors can be set by any task and read by the CAN manager for heartbeat.
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

#ifndef __ERROR_MANAGER_H
#define __ERROR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* Error Flag Bit Definitions ------------------------------------------------*/
/* Temperature Errors (Byte 0) */
#define ERROR_OVER_TEMP             (1 << 0)  /**< Any cell over temperature limit */
#define ERROR_UNDER_TEMP            (1 << 1)  /**< Any cell under temperature limit */
#define ERROR_TEMP_SENSOR_FAULT     (1 << 2)  /**< Temperature sensor disconnected/fault */
#define ERROR_TEMP_GRADIENT         (1 << 3)  /**< Large temperature gradient between cells */
#define ERROR_TEMP_RESERVED_4       (1 << 4)  /**< Reserved for future use */
#define ERROR_TEMP_RESERVED_5       (1 << 5)  /**< Reserved for future use */
#define ERROR_TEMP_RESERVED_6       (1 << 6)  /**< Reserved for future use */
#define ERROR_TEMP_RESERVED_7       (1 << 7)  /**< Reserved for future use */

/* Voltage Errors (Byte 1) */
#define ERROR_OVER_VOLTAGE          (1 << 8)  /**< Any cell over voltage limit */
#define ERROR_UNDER_VOLTAGE         (1 << 9)  /**< Any cell under voltage limit */
#define ERROR_VOLTAGE_IMBALANCE     (1 << 10) /**< Large voltage imbalance between cells */
#define ERROR_VOLTAGE_SENSOR_FAULT  (1 << 11) /**< Voltage sensor fault */
#define ERROR_VOLTAGE_RESERVED_4    (1 << 12) /**< Reserved for future use */
#define ERROR_VOLTAGE_RESERVED_5    (1 << 13) /**< Reserved for future use */
#define ERROR_VOLTAGE_RESERVED_6    (1 << 14) /**< Reserved for future use */
#define ERROR_VOLTAGE_RESERVED_7    (1 << 15) /**< Reserved for future use */

/* Current Errors (Byte 2) */
#define ERROR_OVER_CURRENT_CHARGE   (1 << 16) /**< Charge over current */
#define ERROR_OVER_CURRENT_DISCHARGE (1 << 17) /**< Discharge over current */
#define ERROR_CURRENT_SENSOR_FAULT  (1 << 18) /**< Current sensor fault */
#define ERROR_SHORT_CIRCUIT         (1 << 19) /**< Short circuit detected */
#define ERROR_CURRENT_RESERVED_4    (1 << 20) /**< Reserved for future use */
#define ERROR_CURRENT_RESERVED_5    (1 << 21) /**< Reserved for future use */
#define ERROR_CURRENT_RESERVED_6    (1 << 22) /**< Reserved for future use */
#define ERROR_CURRENT_RESERVED_7    (1 << 23) /**< Reserved for future use */

/* Communication Errors (Byte 3) */
#define ERROR_CAN_BUS_OFF           (1 << 24) /**< CAN bus-off state */
#define ERROR_CAN_TX_TIMEOUT        (1 << 25) /**< CAN transmission timeout */
#define ERROR_CAN_RX_OVERFLOW       (1 << 26) /**< CAN receive buffer overflow */
#define ERROR_I2C_TIMEOUT           (1 << 27) /**< I2C communication timeout */
#define ERROR_I2C_FAULT             (1 << 28) /**< I2C communication fault */
#define ERROR_COMM_RESERVED_5       (1 << 29) /**< Reserved for future use */
#define ERROR_COMM_RESERVED_6       (1 << 30) /**< Reserved for future use */
#define ERROR_WATCHDOG              (1 << 31) /**< Watchdog timer triggered */

/* Warning Flag Bit Definitions ----------------------------------------------*/
/* Temperature Warnings (Byte 0) */
#define WARNING_HIGH_TEMP           (1 << 0)  /**< Temperature approaching limit */
#define WARNING_LOW_TEMP            (1 << 1)  /**< Temperature approaching limit */
#define WARNING_TEMP_GRADIENT       (1 << 2)  /**< Moderate temperature gradient */
#define WARNING_TEMP_RESERVED_3     (1 << 3)  /**< Reserved for future use */
#define WARNING_TEMP_RESERVED_4     (1 << 4)  /**< Reserved for future use */
#define WARNING_TEMP_RESERVED_5     (1 << 5)  /**< Reserved for future use */
#define WARNING_TEMP_RESERVED_6     (1 << 6)  /**< Reserved for future use */
#define WARNING_TEMP_RESERVED_7     (1 << 7)  /**< Reserved for future use */

/* Voltage Warnings (Byte 1) */
#define WARNING_HIGH_VOLTAGE        (1 << 8)  /**< Voltage approaching limit */
#define WARNING_LOW_VOLTAGE         (1 << 9)  /**< Voltage approaching limit */
#define WARNING_VOLTAGE_IMBALANCE   (1 << 10) /**< Moderate voltage imbalance */
#define WARNING_VOLTAGE_RESERVED_3  (1 << 11) /**< Reserved for future use */
#define WARNING_VOLTAGE_RESERVED_4  (1 << 12) /**< Reserved for future use */
#define WARNING_VOLTAGE_RESERVED_5  (1 << 13) /**< Reserved for future use */
#define WARNING_VOLTAGE_RESERVED_6  (1 << 14) /**< Reserved for future use */
#define WARNING_VOLTAGE_RESERVED_7  (1 << 15) /**< Reserved for future use */

/* Current Warnings (Byte 2) */
#define WARNING_HIGH_CURRENT_CHARGE (1 << 16) /**< Charge current approaching limit */
#define WARNING_HIGH_CURRENT_DISCHARGE (1 << 17) /**< Discharge current approaching limit */
#define WARNING_CURRENT_RESERVED_2  (1 << 18) /**< Reserved for future use */
#define WARNING_CURRENT_RESERVED_3  (1 << 19) /**< Reserved for future use */
#define WARNING_CURRENT_RESERVED_4  (1 << 20) /**< Reserved for future use */
#define WARNING_CURRENT_RESERVED_5  (1 << 21) /**< Reserved for future use */
#define WARNING_CURRENT_RESERVED_6  (1 << 22) /**< Reserved for future use */
#define WARNING_CURRENT_RESERVED_7  (1 << 23) /**< Reserved for future use */

/* System Warnings (Byte 3) */
#define WARNING_LOW_SOC             (1 << 24) /**< State of charge low */
#define WARNING_HIGH_SOC            (1 << 25) /**< State of charge very high (near full) */
#define WARNING_CAN_TX_QUEUE_FULL   (1 << 26) /**< CAN TX queue near full */
#define WARNING_SYSTEM_RESERVED_3   (1 << 27) /**< Reserved for future use */
#define WARNING_SYSTEM_RESERVED_4   (1 << 28) /**< Reserved for future use */
#define WARNING_SYSTEM_RESERVED_5   (1 << 29) /**< Reserved for future use */
#define WARNING_SYSTEM_RESERVED_6   (1 << 30) /**< Reserved for future use */
#define WARNING_SYSTEM_RESERVED_7   (1 << 31) /**< Reserved for future use */

/* BMS State Definitions -----------------------------------------------------*/
typedef enum {
    BMS_STATE_INIT = 0,         /**< Initialization state */
    BMS_STATE_IDLE,             /**< Idle/standby state */
    BMS_STATE_CHARGING,         /**< Charging state */
    BMS_STATE_DISCHARGING,      /**< Discharging state */
    BMS_STATE_BALANCING,        /**< Cell balancing state */
    BMS_STATE_ERROR,            /**< Error/fault state */
    BMS_STATE_SHUTDOWN,         /**< Shutdown state */
    BMS_STATE_RESERVED          /**< Reserved for future use */
} BMS_State_t;

/* Error Manager Structure ---------------------------------------------------*/
typedef struct {
    uint32_t error_flags;       /**< Active error flags (4 bytes) */
    uint32_t warning_flags;     /**< Active warning flags (4 bytes) */
    BMS_State_t state;          /**< Current BMS state */
    uint8_t fault_count;        /**< Number of faults since boot */
    uint32_t uptime_seconds;    /**< System uptime in seconds */
    uint32_t last_heartbeat;    /**< Timestamp of last heartbeat */
} Error_Manager_t;

/* Function Prototypes -------------------------------------------------------*/

/**
  * @brief  Initialize error manager
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef ErrorMgr_Init(void);

/**
  * @brief  Set an error flag
  * @param  error_flag: Error flag bit to set (use ERROR_* defines)
  * @retval None
  */
void ErrorMgr_SetError(uint32_t error_flag);

/**
  * @brief  Clear an error flag
  * @param  error_flag: Error flag bit to clear (use ERROR_* defines)
  * @retval None
  */
void ErrorMgr_ClearError(uint32_t error_flag);

/**
  * @brief  Set a warning flag
  * @param  warning_flag: Warning flag bit to set (use WARNING_* defines)
  * @retval None
  */
void ErrorMgr_SetWarning(uint32_t warning_flag);

/**
  * @brief  Clear a warning flag
  * @param  warning_flag: Warning flag bit to clear (use WARNING_* defines)
  * @retval None
  */
void ErrorMgr_ClearWarning(uint32_t warning_flag);

/**
  * @brief  Get current error flags
  * @retval uint32_t: Current error flags
  */
uint32_t ErrorMgr_GetErrors(void);

/**
  * @brief  Get current warning flags
  * @retval uint32_t: Current warning flags
  */
uint32_t ErrorMgr_GetWarnings(void);

/**
  * @brief  Check if any errors are active
  * @retval bool: true if any errors active, false otherwise
  */
bool ErrorMgr_HasErrors(void);

/**
  * @brief  Check if any warnings are active
  * @retval bool: true if any warnings active, false otherwise
  */
bool ErrorMgr_HasWarnings(void);

/**
  * @brief  Set BMS state
  * @param  state: New BMS state
  * @retval None
  */
void ErrorMgr_SetState(BMS_State_t state);

/**
  * @brief  Get current BMS state
  * @retval BMS_State_t: Current BMS state
  */
BMS_State_t ErrorMgr_GetState(void);

/**
  * @brief  Get complete error manager structure (for heartbeat)
  * @param  mgr: Pointer to structure to fill
  * @retval None
  */
void ErrorMgr_GetStatus(Error_Manager_t *mgr);

/**
  * @brief  Update system uptime (call every second)
  * @retval None
  */
void ErrorMgr_UpdateUptime(void);

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_MANAGER_H */
