#include <string.h>

#include "balance_manager.h"
#include "bq_handler.h"
#include "can_manager.h"
#include "error_manager.h"
#include "state_machine.h"
#include "test_support.h"

void TestSupport_ResetRtosStubs(void);

I2C_HandleTypeDef hi2c1 = {0};
I2C_HandleTypeDef hi2c3 = {0};
ADC_HandleTypeDef hadc1 = {0};
CAN_HandleTypeDef hcan1 = {0};
osSemaphoreId_t BMSResetSemHandle = NULL;

static GPIO_TypeDef g_gpioa = {0};
static GPIO_TypeDef g_gpiob = {0};
GPIO_TypeDef *GPIOA = &g_gpioa;
GPIO_TypeDef *GPIOB = &g_gpiob;

uint32_t CAN_BALANCE_STATUS_ID = 0x18FF5001U;
uint32_t CAN_BMS1_BAL_DETAIL_ID = 0x18FF5002U;
uint32_t CAN_BMS2_BAL_DETAIL_ID = 0x18FF5003U;
uint32_t CAN_TEMP_ID = 0x18FF5100U;
uint32_t CAN_TEMP_SUMMARY_ID = 0x18FF5101U;

void Error_Handler(void)
{
}

static BMS_State_t g_state = BMS_STATE_INIT;
static bool g_has_errors = false;
static uint32_t g_error_flags = 0U;

static BQ_Data_t g_bms1_data = {0};
static BQ_Data_BMS2_t g_bms2_data = {0};
static HAL_StatusTypeDef g_bq_data_status_bms1 = HAL_OK;
static HAL_StatusTypeDef g_bq_data_status_bms2 = HAL_OK;

static int16_t g_bms1_temp = 0;
static int16_t g_bms2_temp = 0;
static uint16_t g_alarm_bms1 = 0;
static uint16_t g_alarm_bms2 = 0;
static uint16_t g_readback_bms1 = 0;
static uint16_t g_readback_bms2 = 0;

static uint32_t g_configure_speed_calls = 0;
static uint32_t g_stop_balancing_calls = 0;
static uint32_t g_set_balance_calls_bms1 = 0;
static uint32_t g_set_balance_calls_bms2 = 0;
static uint16_t g_last_set_balance_mask_bms1 = 0;
static uint16_t g_last_set_balance_mask_bms2 = 0;
static uint16_t g_adc_default_value = 2048U;
static uint16_t g_adc_script[1024] = {0};
static uint32_t g_adc_script_count = 0U;
static uint32_t g_adc_script_index = 0U;

typedef struct
{
    TestSupport_CanMessage_t msgs[64];
    uint32_t count;
} CanLog_t;

static CanLog_t g_can_log = {0};

void TestSupport_ResetAll(void)
{
    TestSupport_ResetRtosStubs();
    g_state = BMS_STATE_INIT;
    g_has_errors = false;
    g_error_flags = 0U;

    memset(&g_bms1_data, 0, sizeof(g_bms1_data));
    memset(&g_bms2_data, 0, sizeof(g_bms2_data));
    g_bq_data_status_bms1 = HAL_OK;
    g_bq_data_status_bms2 = HAL_OK;

    g_bms1_temp = 0;
    g_bms2_temp = 0;
    g_alarm_bms1 = 0;
    g_alarm_bms2 = 0;
    g_readback_bms1 = 0;
    g_readback_bms2 = 0;

    g_configure_speed_calls = 0;
    g_stop_balancing_calls = 0;
    g_set_balance_calls_bms1 = 0;
    g_set_balance_calls_bms2 = 0;
    g_last_set_balance_mask_bms1 = 0;
    g_last_set_balance_mask_bms2 = 0;

    g_adc_default_value = 2048U;
    g_adc_script_count = 0U;
    g_adc_script_index = 0U;
    memset(g_adc_script, 0, sizeof(g_adc_script));

    memset(&g_can_log, 0, sizeof(g_can_log));
}

void TestSupport_SetState(BMS_State_t state)
{
    g_state = state;
}

void TestSupport_SetHasErrors(bool has_errors)
{
    g_has_errors = has_errors;
    if (has_errors && g_error_flags == 0U)
    {
        g_error_flags = 1U;
    }
    if (!has_errors)
    {
        g_error_flags = 0U;
    }
}

void TestSupport_SetAdcDefault(uint16_t adc_value)
{
    g_adc_default_value = adc_value;
}

void TestSupport_SetAdcScript(const uint16_t *samples, uint32_t count)
{
    g_adc_script_count = count;
    g_adc_script_index = 0U;
    if ((samples != NULL) && (count > 0U))
    {
        if (count > (uint32_t)(sizeof(g_adc_script) / sizeof(g_adc_script[0])))
        {
            count = (uint32_t)(sizeof(g_adc_script) / sizeof(g_adc_script[0]));
            g_adc_script_count = count;
        }
        memcpy(g_adc_script, samples, count * sizeof(uint16_t));
    }
}

void TestSupport_SetBqData(const uint16_t bms1_cells[9], uint8_t bms1_valid,
                           const uint16_t bms2_cells[9], uint8_t bms2_valid)
{
    memcpy(g_bms1_data.cell_voltage_mv, bms1_cells, sizeof(g_bms1_data.cell_voltage_mv));
    memcpy(g_bms2_data.cell_voltage_mv, bms2_cells, sizeof(g_bms2_data.cell_voltage_mv));
    g_bms1_data.valid = bms1_valid;
    g_bms2_data.valid = bms2_valid;
}

void TestSupport_SetBqDataStatus(HAL_StatusTypeDef bms1_status, HAL_StatusTypeDef bms2_status)
{
    g_bq_data_status_bms1 = bms1_status;
    g_bq_data_status_bms2 = bms2_status;
}

void TestSupport_SetTemps(int16_t bms1_temp_decic, int16_t bms2_temp_decic)
{
    g_bms1_temp = bms1_temp_decic;
    g_bms2_temp = bms2_temp_decic;
}

void TestSupport_SetAlarmRaw(uint16_t bms1_alarm, uint16_t bms2_alarm)
{
    g_alarm_bms1 = bms1_alarm;
    g_alarm_bms2 = bms2_alarm;
}

void TestSupport_SetBalanceReadback(uint16_t bms1_mask, uint16_t bms2_mask)
{
    g_readback_bms1 = bms1_mask;
    g_readback_bms2 = bms2_mask;
}

uint32_t TestSupport_GetConfigureSpeedCallCount(void)
{
    return g_configure_speed_calls;
}

uint32_t TestSupport_GetStopBalancingCallCount(void)
{
    return g_stop_balancing_calls;
}

uint32_t TestSupport_GetSetBalanceCallCountBms1(void)
{
    return g_set_balance_calls_bms1;
}

uint32_t TestSupport_GetSetBalanceCallCountBms2(void)
{
    return g_set_balance_calls_bms2;
}

uint16_t TestSupport_GetLastSetBalanceMaskBms1(void)
{
    return g_last_set_balance_mask_bms1;
}

uint16_t TestSupport_GetLastSetBalanceMaskBms2(void)
{
    return g_last_set_balance_mask_bms2;
}

uint32_t TestSupport_GetCanSendCount(void)
{
    return g_can_log.count;
}

uint32_t TestSupport_GetErrorFlags(void)
{
    return g_error_flags;
}

const TestSupport_CanMessage_t *TestSupport_GetCanMessage(uint32_t index)
{
    if (index >= g_can_log.count)
    {
        return NULL;
    }
    return &g_can_log.msgs[index];
}

HAL_StatusTypeDef StateMachine_Init(void)
{
    g_state = BMS_STATE_INIT;
    return HAL_OK;
}

void StateMachine_SetState(BMS_State_t state)
{
    g_state = state;
}

BMS_State_t StateMachine_GetState(void)
{
    return g_state;
}

void StateMachine_EnterFault(void)
{
    g_state = BMS_STATE_ERROR;
}

void StateMachine_ExitFault(void)
{
    g_state = BMS_STATE_IDLE;
}

bool StateMachine_IsInFault(void)
{
    return (g_state == BMS_STATE_ERROR);
}

bool StateMachine_CanTransition(BMS_State_t new_state)
{
    (void)new_state;
    return true;
}

HAL_StatusTypeDef ErrorMgr_Init(void)
{
    g_has_errors = false;
    return HAL_OK;
}

void ErrorMgr_ClearError(uint32_t error_flag)
{
    g_error_flags &= ~error_flag;
    g_has_errors = (g_error_flags != 0U);
}
void ErrorMgr_SetWarning(uint32_t warning_flag) { (void)warning_flag; }
void ErrorMgr_ClearWarning(uint32_t warning_flag) { (void)warning_flag; }
uint32_t ErrorMgr_GetErrors(void) { return g_error_flags; }
uint32_t ErrorMgr_GetWarnings(void) { return 0U; }
bool ErrorMgr_HasErrors(void) { return g_has_errors; }
bool ErrorMgr_HasWarnings(void) { return false; }
void ErrorMgr_GetStatus(Error_Manager_t *mgr) { (void)mgr; }
void ErrorMgr_UpdateUptime(void) {}

void ErrorMgr_SetError(uint32_t error_flag)
{
    g_error_flags |= error_flag;
    g_has_errors = (g_error_flags != 0U);
}

HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *config)
{
    (void)hadc;
    (void)config;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout)
{
    (void)hadc;
    (void)timeout;
    return HAL_OK;
}

uint16_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    if (g_adc_script_index < g_adc_script_count)
    {
        return g_adc_script[g_adc_script_index++];
    }
    return g_adc_default_value;
}

HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    (void)GPIOx;
    (void)GPIO_Pin;
    (void)PinState;
}

HAL_StatusTypeDef BQ_ConfigureBalancingSpeed(I2C_HandleTypeDef *hi2c, uint8_t device_addr)
{
    (void)hi2c;
    (void)device_addr;
    g_configure_speed_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef BQ_GetData(BQ_Data_t *data)
{
    if (data != NULL)
    {
        *data = g_bms1_data;
    }
    return g_bq_data_status_bms1;
}

HAL_StatusTypeDef BQ_GetData_BMS2(BQ_Data_BMS2_t *data)
{
    if (data != NULL)
    {
        *data = g_bms2_data;
    }
    return g_bq_data_status_bms2;
}

HAL_StatusTypeDef BQ_SetBalanceCells(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t cell_mask)
{
    (void)device_addr;
    if (hi2c == &hi2c1)
    {
        g_set_balance_calls_bms1++;
        g_last_set_balance_mask_bms1 = cell_mask;
    }
    else if (hi2c == &hi2c3)
    {
        g_set_balance_calls_bms2++;
        g_last_set_balance_mask_bms2 = cell_mask;
    }
    return HAL_OK;
}

HAL_StatusTypeDef BQ_StopBalancing(I2C_HandleTypeDef *hi2c, uint8_t device_addr)
{
    (void)hi2c;
    (void)device_addr;
    g_stop_balancing_calls++;
    return HAL_OK;
}

int16_t BQ_GetBMS1InternalTemp(void)
{
    return g_bms1_temp;
}

int16_t BQ_GetBMS2InternalTemp(void)
{
    return g_bms2_temp;
}

HAL_StatusTypeDef BQ_GetAlarmRawStatus(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t *alarm_raw)
{
    (void)device_addr;
    if (alarm_raw == NULL)
    {
        return HAL_ERROR;
    }
    *alarm_raw = (hi2c == &hi2c1) ? g_alarm_bms1 : g_alarm_bms2;
    return HAL_OK;
}

HAL_StatusTypeDef BQ_GetBalanceCells(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint16_t *cell_mask)
{
    (void)device_addr;
    if (cell_mask == NULL)
    {
        return HAL_ERROR;
    }
    *cell_mask = (hi2c == &hi2c1) ? g_readback_bms1 : g_readback_bms2;
    return HAL_OK;
}

HAL_StatusTypeDef CAN_Manager_Init(void)
{
    return HAL_OK;
}

void CAN_ManagerTask(void *argument)
{
    (void)argument;
}

HAL_StatusTypeDef CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t length, uint8_t priority)
{
    if (g_can_log.count < (uint32_t)(sizeof(g_can_log.msgs) / sizeof(g_can_log.msgs[0])))
    {
        TestSupport_CanMessage_t *slot = &g_can_log.msgs[g_can_log.count++];
        slot->id = id;
        slot->length = length;
        slot->priority = priority;
        memset(slot->data, 0, sizeof(slot->data));
        if ((data != NULL) && (length <= sizeof(slot->data)))
        {
            memcpy(slot->data, data, length);
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef CAN_SendMessageExt(uint32_t id, uint8_t *data, uint8_t length, uint8_t priority)
{
    return CAN_SendMessage(id, data, length, priority);
}

HAL_StatusTypeDef CAN_ReconfigureFilters(void) { return HAL_OK; }
void CAN_GetStatistics(CAN_Statistics_t *stats) { (void)stats; }
void CAN_ResetStatistics(void) {}
uint32_t CAN_GetTxQueueCount(void) { return 0U; }
uint32_t CAN_GetRxQueueCount(void) { return 0U; }
uint32_t CAN_FlushTxQueue(void) { return 0U; }
bool CAN_IsMessageForThisModule(uint32_t can_id) { (void)can_id; return true; }
HAL_StatusTypeDef CAN_SendHeartbeat(void) { return HAL_OK; }
HAL_StatusTypeDef CAN_SendStatistics(void) { return HAL_OK; }
HAL_StatusTypeDef CAN_SendDebugInfo(void) { return HAL_OK; }
HAL_StatusTypeDef CAN_SendI2CDiagnostics(void) { return HAL_OK; }
