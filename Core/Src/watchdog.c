/**
  ******************************************************************************
  * @file           : watchdog.c
  * @brief          : Independent Watchdog (IWDG) + per-task software supervisor
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "watchdog.h"
#include "cmsis_os.h"
#include "error_manager.h"
#include "stm32l4xx_hal.h"

/* Private defines -----------------------------------------------------------*/
/*
 * IWDG timing
 * -----------
 * Clock source : LSI ~32 kHz
 * Prescaler    : /32        -> 1 kHz counter (1 ms per tick)
 * Reload (RLR) : 2000       -> ~2000 ms hard timeout
 *
 * NOTE: LSI is nominally 32 kHz but may vary +/- ~50% over temperature on
 * the STM32L4. Use generous deadlines below.
 */
#define WD_IWDG_PRESCALER       IWDG_PRESCALER_32
#define WD_IWDG_RELOAD          2000U   /* milliseconds */

/* Supervisor task period (ms). Must be << IWDG timeout. */
#define WD_SUPERVISOR_PERIOD_MS 100U

/* Supervisor task attributes */
#define WD_SUPERVISOR_STACK     (256U * 4U)
#define WD_SUPERVISOR_PRIORITY  osPriorityNormal

/* Per-task deadlines (ms). Must be < WD_IWDG_RELOAD - WD_SUPERVISOR_PERIOD_MS. */
static const uint32_t wd_deadline_ms[WD_TASK_COUNT] = {
    [WD_TASK_CAN]      = 200U,    /* CAN task runs at 10 ms */
    [WD_TASK_BMS1]     = 1500U,   /* BMS1 read interval ~625 ms */
    [WD_TASK_BMS2]     = 1500U,   /* BMS2 read interval ~625 ms */
    [WD_TASK_CELLTEMP] = 1500U,   /* CellTemp MUX iteration ~125 ms; full cycle ~1 s */
};

/* Private variables ---------------------------------------------------------*/
static IWDG_HandleTypeDef hiwdg;
static volatile uint32_t wd_last_heartbeat_tick[WD_TASK_COUNT];
static uint32_t wd_reset_cause;       /* Snapshot of RCC reset flags at boot */
static bool     wd_was_iwdg_reset;    /* Convenience flag */
static bool     wd_initialised = false;

static osThreadId_t wd_supervisor_handle;
static const osThreadAttr_t wd_supervisor_attr = {
    .name       = "Watchdog",
    .stack_size = WD_SUPERVISOR_STACK,
    .priority   = WD_SUPERVISOR_PRIORITY,
};

/* Private function prototypes -----------------------------------------------*/
static void Watchdog_SupervisorTask(void *argument);

/* Public functions ----------------------------------------------------------*/

HAL_StatusTypeDef Watchdog_Init(void)
{
    /* Capture and clear reset-cause flags from the previous reset */
    wd_reset_cause    = HAL_RCC_GetResetSource();
    wd_was_iwdg_reset = (wd_reset_cause & RCC_RESET_FLAG_IWDG) != 0U;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    /* If the previous reset was IWDG-induced, surface it via the error system.
     * Sticky until the next power-on / brown-out reset, which clears all flags
     * naturally (so this branch will not be taken on a fresh power cycle). */
    if (wd_was_iwdg_reset) {
        ErrorMgr_SetError(ERROR_WATCHDOG);
    }

    /* Halt the IWDG counter when the core is halted by the debugger so
     * single-stepping does not trigger an unexpected reset. The DBGMCU
     * registers only have effect when a debugger is connected, so this is
     * harmless on production hardware. */
    SET_BIT(DBGMCU->APB1FZR1, DBGMCU_APB1FZR1_DBG_IWDG_STOP);

    /* Configure and start the IWDG */
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = WD_IWDG_PRESCALER;
    hiwdg.Init.Reload    = WD_IWDG_RELOAD;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;

    HAL_StatusTypeDef status = HAL_IWDG_Init(&hiwdg);
    if (status != HAL_OK) {
        return status;
    }

    /* Seed heartbeat timestamps with the current tick so freshly created
     * tasks are not immediately flagged as missing. */
    uint32_t now = osKernelGetTickCount();
    for (uint32_t i = 0; i < WD_TASK_COUNT; i++) {
        wd_last_heartbeat_tick[i] = now;
    }

    wd_initialised = true;
    return HAL_OK;
}

HAL_StatusTypeDef Watchdog_StartSupervisor(void)
{
    if (!wd_initialised) {
        return HAL_ERROR;
    }
    wd_supervisor_handle = osThreadNew(Watchdog_SupervisorTask, NULL, &wd_supervisor_attr);
    return (wd_supervisor_handle != NULL) ? HAL_OK : HAL_ERROR;
}

void Watchdog_Heartbeat(WD_TaskId_t id)
{
    if ((uint32_t)id >= WD_TASK_COUNT) {
        return;
    }
    /* Single 32-bit store, naturally atomic on Cortex-M4. */
    wd_last_heartbeat_tick[id] = osKernelGetTickCount();
}

uint32_t Watchdog_GetResetCause(void)
{
    return wd_reset_cause;
}

bool Watchdog_WasIwdgReset(void)
{
    return wd_was_iwdg_reset;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Supervisor loop: refresh IWDG only if every monitored task has
  *         checked in within its deadline. Otherwise let the IWDG expire.
  */
static void Watchdog_SupervisorTask(void *argument)
{
    (void)argument;

    /* Re-seed heartbeats so tasks that have not yet started their main loop
     * are not flagged before they get scheduled. */
    uint32_t now = osKernelGetTickCount();
    for (uint32_t i = 0; i < WD_TASK_COUNT; i++) {
        wd_last_heartbeat_tick[i] = now;
    }

    /* Generous warm-up: BQ tasks delay 500-625 ms before their first
     * heartbeat, and the CAN task delays 100 ms. Skip enforcement until
     * the slowest task should have produced its first heartbeat. */
    const uint32_t warmup_ms = 1500U;
    uint32_t warmup_deadline = now + warmup_ms;

    for (;;) {
        now = osKernelGetTickCount();

        bool all_alive = true;
        if ((int32_t)(now - warmup_deadline) >= 0) {
            for (uint32_t i = 0; i < WD_TASK_COUNT; i++) {
                uint32_t age = now - wd_last_heartbeat_tick[i];
                if (age > wd_deadline_ms[i]) {
                    all_alive = false;
                    break;
                }
            }
        }

        if (all_alive) {
            HAL_IWDG_Refresh(&hiwdg);
        }

        osDelay(WD_SUPERVISOR_PERIOD_MS);
    }
}
