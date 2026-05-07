/**
  ******************************************************************************
  * @file           : watchdog.h
  * @brief          : Independent Watchdog (IWDG) + per-task software supervisor
  ******************************************************************************
  * @attention
  *
  * The hardware IWDG is configured for a ~2000 ms timeout. A FreeRTOS
  * supervisor task refreshes the IWDG only when every monitored task has
  * checked in within its individual deadline. A stalled or starved task
  * therefore forces a deterministic hardware reset.
  *
  * Reset cause is captured at startup; if the previous reset was caused by
  * the IWDG, the @ref ERROR_WATCHDOG flag is set in the error manager so the
  * fault is reported on the next CAN heartbeat.
  *
  ******************************************************************************
  */

#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  Identifiers for tasks supervised by the watchdog.
  * @note   Order is fixed; values are used as array indices.
  */
typedef enum {
    WD_TASK_CAN = 0,    /**< CAN_ManagerTask (10 ms loop) */
    WD_TASK_BMS1,       /**< BQ_MonitorTask (BMS1, 625 ms loop) */
    WD_TASK_BMS2,       /**< BQ_MonitorTask_BMS2 (625 ms loop) */
    WD_TASK_CELLTEMP,   /**< CellTemp_MonitorTask (~125 ms per MUX iteration) */
    WD_TASK_COUNT
} WD_TaskId_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize the IWDG and capture the previous reset cause.
  * @note   Must be called from main() after HAL_Init() and SystemClock_Config(),
  *         and before osKernelStart(). Call after ErrorMgr_Init() so the
  *         ERROR_WATCHDOG flag can be set immediately on an IWDG-induced reset.
  *         Once started, the IWDG cannot be disabled.
  * @retval HAL_OK on success, otherwise an error code from HAL_IWDG_Init().
  */
HAL_StatusTypeDef Watchdog_Init(void);

/**
  * @brief  Create the FreeRTOS supervisor task.
  * @note   Must be called from main() inside the RTOS thread creation block,
  *         after Watchdog_Init() and before osKernelStart().
  * @retval HAL_OK on success, HAL_ERROR if the task could not be created.
  */
HAL_StatusTypeDef Watchdog_StartSupervisor(void);

/**
  * @brief  Mark a supervised task as alive.
  * @param  id: Identifier of the calling task.
  * @note   Should be called once per main loop iteration of the supervised task.
  */
void Watchdog_Heartbeat(WD_TaskId_t id);

/**
  * @brief  Get the cached RCC reset cause snapshot captured at startup.
  * @retval Bitwise OR of RCC_RESET_FLAG_* values from HAL_RCC_GetResetSource().
  */
uint32_t Watchdog_GetResetCause(void);

/**
  * @brief  Convenience query: was the previous reset caused by the IWDG?
  * @retval true if the IWDG reset flag was set at startup, false otherwise.
  */
bool Watchdog_WasIwdgReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H */
