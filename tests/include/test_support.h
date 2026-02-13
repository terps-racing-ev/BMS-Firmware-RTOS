#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    uint8_t priority;
} TestSupport_CanMessage_t;

void TestSupport_ResetAll(void);

void TestSupport_SetTick(uint32_t tick);
void TestSupport_AdvanceTick(uint32_t delta_ms);

void TestSupport_SetState(BMS_State_t state);
void TestSupport_SetHasErrors(bool has_errors);
void TestSupport_SetMutexCreateShouldFail(bool should_fail);
void TestSupport_SetAdcDefault(uint16_t adc_value);
void TestSupport_SetAdcScript(const uint16_t *samples, uint32_t count);

void TestSupport_SetBqData(const uint16_t bms1_cells[9], uint8_t bms1_valid,
                           const uint16_t bms2_cells[9], uint8_t bms2_valid);
void TestSupport_SetBqDataStatus(HAL_StatusTypeDef bms1_status, HAL_StatusTypeDef bms2_status);
void TestSupport_SetTemps(int16_t bms1_temp_decic, int16_t bms2_temp_decic);
void TestSupport_SetAlarmRaw(uint16_t bms1_alarm, uint16_t bms2_alarm);
void TestSupport_SetBalanceReadback(uint16_t bms1_mask, uint16_t bms2_mask);

uint32_t TestSupport_GetConfigureSpeedCallCount(void);
uint32_t TestSupport_GetStopBalancingCallCount(void);
uint32_t TestSupport_GetSetBalanceCallCountBms1(void);
uint32_t TestSupport_GetSetBalanceCallCountBms2(void);
uint16_t TestSupport_GetLastSetBalanceMaskBms1(void);
uint16_t TestSupport_GetLastSetBalanceMaskBms2(void);

uint32_t TestSupport_GetCanSendCount(void);
const TestSupport_CanMessage_t *TestSupport_GetCanMessage(uint32_t index);
uint32_t TestSupport_GetErrorFlags(void);

#ifdef __cplusplus
}
#endif

#endif
