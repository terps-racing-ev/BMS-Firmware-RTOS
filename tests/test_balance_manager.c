#include "unity.h"

#include "balance_manager.h"
#include "test_support.h"

void setUp(void)
{
    TestSupport_ResetAll();
    TEST_ASSERT_EQUAL(HAL_OK, BalanceMgr_Init());
}

void tearDown(void)
{
}

static void enter_balancing_mode(void)
{
    TestSupport_SetState(BMS_STATE_IDLE);
    TEST_ASSERT_EQUAL(BALANCE_STATUS_SUCCESS, BalanceMgr_ProcessCommand());
    TEST_ASSERT_EQUAL(BMS_STATE_BALANCING, StateMachine_GetState());
}

void test_BalanceMgr_Init_returns_HAL_ERROR_when_mutex_creation_fails(void)
{
    TestSupport_ResetAll();
    TestSupport_SetMutexCreateShouldFail(true);

    TEST_ASSERT_EQUAL(HAL_ERROR, BalanceMgr_Init());
}

void test_BalanceMgr_ProcessCommand_rejects_when_errors_present(void)
{
    TestSupport_SetState(BMS_STATE_IDLE);
    TestSupport_SetHasErrors(true);

    TEST_ASSERT_EQUAL(BALANCE_STATUS_FAULT, BalanceMgr_ProcessCommand());
    TEST_ASSERT_EQUAL(BMS_STATE_IDLE, StateMachine_GetState());
}

void test_BalanceMgr_ProcessCommand_enters_balancing_and_configures_speed(void)
{
    TestSupport_SetState(BMS_STATE_IDLE);

    TEST_ASSERT_EQUAL(BALANCE_STATUS_SUCCESS, BalanceMgr_ProcessCommand());
    TEST_ASSERT_EQUAL(BMS_STATE_BALANCING, StateMachine_GetState());
    TEST_ASSERT_EQUAL_UINT32(2U, TestSupport_GetConfigureSpeedCallCount());
    TEST_ASSERT_EQUAL_UINT32(BALANCE_CMD_TIMEOUT_MS, BalanceMgr_GetTimeRemaining());
}

void test_BalanceMgr_Execute_selects_non_adjacent_highest_cells(void)
{
    const uint16_t bms1_cells[9] = {3600, 3590, 3580, 3570, 3560, 3550, 3400, 3390, 3380};
    const uint16_t bms2_cells[9] = {3400, 3400, 3400, 3400, 3400, 3400, 3400, 3400, 3400};

    enter_balancing_mode();
    TEST_ASSERT_EQUAL(BALANCE_STATUS_SUCCESS, BalanceMgr_ProcessConfig(3500, 3));

    TestSupport_SetBqData(bms1_cells, 1U, bms2_cells, 1U);
    BalanceMgr_Execute();

    TEST_ASSERT_EQUAL_HEX16(0x0015, TestSupport_GetLastSetBalanceMaskBms1());
    TEST_ASSERT_EQUAL_HEX16(0x0000, TestSupport_GetLastSetBalanceMaskBms2());
    TEST_ASSERT_EQUAL_UINT32(1U, TestSupport_GetSetBalanceCallCountBms1());
    TEST_ASSERT_EQUAL_UINT32(1U, TestSupport_GetSetBalanceCallCountBms2());
}

void test_BalanceMgr_CheckTimeout_exits_balancing_after_command_timeout(void)
{
    enter_balancing_mode();

    TestSupport_SetTick(BALANCE_CMD_TIMEOUT_MS + 1U);
    BalanceMgr_CheckTimeout();

    TEST_ASSERT_EQUAL(BMS_STATE_IDLE, StateMachine_GetState());
    TEST_ASSERT_EQUAL_UINT32(2U, TestSupport_GetStopBalancingCallCount());
    TEST_ASSERT_EQUAL_UINT32(0U, BalanceMgr_GetTimeRemaining());
}

void test_BalanceMgr_SendStatus_packs_expected_payload(void)
{
    const uint16_t bms1_cells[9] = {3600, 3590, 3580, 3570, 3560, 3550, 3400, 3390, 3380};
    const uint16_t bms2_cells[9] = {3400, 3400, 3400, 3400, 3400, 3400, 3400, 3400, 3400};

    enter_balancing_mode();
    TEST_ASSERT_EQUAL(BALANCE_STATUS_SUCCESS, BalanceMgr_ProcessConfig(3500, 3));
    TestSupport_SetBqData(bms1_cells, 1U, bms2_cells, 1U);
    TestSupport_SetTemps(251, -34);

    BalanceMgr_Execute();
    TEST_ASSERT_EQUAL(HAL_OK, BalanceMgr_SendStatus());

    TEST_ASSERT_EQUAL_UINT32(1U, TestSupport_GetCanSendCount());
    const TestSupport_CanMessage_t *msg = TestSupport_GetCanMessage(0U);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_UINT32(0x18FF5001U, msg->id);
    TEST_ASSERT_EQUAL_UINT8(8U, msg->length);

    TEST_ASSERT_EQUAL_HEX8(0xAC, msg->data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0D, msg->data[1]);
    TEST_ASSERT_EQUAL_UINT8(3U, msg->data[2]);
    TEST_ASSERT_EQUAL_UINT8(0x03U, msg->data[3]);
    TEST_ASSERT_EQUAL_HEX8(0xFB, msg->data[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, msg->data[5]);
    TEST_ASSERT_EQUAL_HEX8(0xDE, msg->data[6]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, msg->data[7]);
}

void test_BalanceMgr_SendDetailedStatus_sends_both_bms_messages(void)
{
    enter_balancing_mode();
    TestSupport_SetBalanceReadback(0x0123, 0x01A5);
    TestSupport_SetAlarmRaw(0xA10B, 0xB20C);

    TEST_ASSERT_EQUAL(HAL_OK, BalanceMgr_SendDetailedStatus());

    TEST_ASSERT_EQUAL_UINT32(2U, TestSupport_GetCanSendCount());

    const TestSupport_CanMessage_t *bms1 = TestSupport_GetCanMessage(0U);
    const TestSupport_CanMessage_t *bms2 = TestSupport_GetCanMessage(1U);

    TEST_ASSERT_NOT_NULL(bms1);
    TEST_ASSERT_NOT_NULL(bms2);

    TEST_ASSERT_EQUAL_UINT32(0x18FF5002U, bms1->id);
    TEST_ASSERT_EQUAL_HEX8(0x23, bms1->data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, bms1->data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0B, bms1->data[3]);
    TEST_ASSERT_EQUAL_HEX8(0xA1, bms1->data[4]);

    TEST_ASSERT_EQUAL_UINT32(0x18FF5003U, bms2->id);
    TEST_ASSERT_EQUAL_HEX8(0xA5, bms2->data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, bms2->data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0C, bms2->data[3]);
    TEST_ASSERT_EQUAL_HEX8(0xB2, bms2->data[4]);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_BalanceMgr_Init_returns_HAL_ERROR_when_mutex_creation_fails);
    RUN_TEST(test_BalanceMgr_ProcessCommand_rejects_when_errors_present);
    RUN_TEST(test_BalanceMgr_ProcessCommand_enters_balancing_and_configures_speed);
    RUN_TEST(test_BalanceMgr_Execute_selects_non_adjacent_highest_cells);
    RUN_TEST(test_BalanceMgr_CheckTimeout_exits_balancing_after_command_timeout);
    RUN_TEST(test_BalanceMgr_SendStatus_packs_expected_payload);
    RUN_TEST(test_BalanceMgr_SendDetailedStatus_sends_both_bms_messages);

    return UNITY_END();
}
