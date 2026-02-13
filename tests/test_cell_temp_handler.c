#include "unity.h"

#include "cell_temp_handler.h"

static uint16_t find_adc_for_over_temp(void)
{
    for (uint16_t adc = 10U; adc <= 4095U; adc++)
    {
        if (CellTemp_CalculateTemperature(adc) > (float)CellTemp_GetMaxTemp())
        {
            return adc;
        }
    }
    return 0U;
}

static uint16_t find_adc_for_under_temp(void)
{
    for (uint16_t adc = 10U; adc <= 4095U; adc++)
    {
        if (CellTemp_CalculateTemperature(adc) < (float)CellTemp_GetMinTemp())
        {
            return adc;
        }
    }
    return 0U;
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_Thermistor_NoConnection_returns_invalid_marker(void)
{
    TEST_ASSERT_EQUAL_FLOAT(-127.0f, CellTemp_CalculateTemperature(0U));
    TEST_ASSERT_EQUAL_FLOAT(-127.0f, CellTemp_CalculateTemperature(9U));
}

void test_Thermistor_OverTemp_condition_is_detectable_against_default_threshold(void)
{
    CellTemp_SetMaxTemp((int8_t)TEMP_MAX_CELSIUS);

    uint16_t adc = find_adc_for_over_temp();
    TEST_ASSERT_NOT_EQUAL_UINT16(0U, adc);

    float temp_c = CellTemp_CalculateTemperature(adc);
    TEST_ASSERT_GREATER_THAN_FLOAT((float)CellTemp_GetMaxTemp(), temp_c);
}

void test_Thermistor_UnderTemp_condition_is_detectable_against_default_threshold(void)
{
    CellTemp_SetMinTemp((int8_t)TEMP_MIN_CELSIUS);

    uint16_t adc = find_adc_for_under_temp();
    TEST_ASSERT_NOT_EQUAL_UINT16(0U, adc);

    float temp_c = CellTemp_CalculateTemperature(adc);
    TEST_ASSERT_LESS_THAN_FLOAT((float)CellTemp_GetMinTemp(), temp_c);
}

void test_Thermistor_OverUnder_behavior_tracks_runtime_threshold_updates(void)
{
    CellTemp_SetMaxTemp(35);
    CellTemp_SetMinTemp(5);

    uint16_t adc_over = find_adc_for_over_temp();
    uint16_t adc_under = find_adc_for_under_temp();

    TEST_ASSERT_NOT_EQUAL_UINT16(0U, adc_over);
    TEST_ASSERT_NOT_EQUAL_UINT16(0U, adc_under);

    TEST_ASSERT_GREATER_THAN_FLOAT((float)CellTemp_GetMaxTemp(), CellTemp_CalculateTemperature(adc_over));
    TEST_ASSERT_LESS_THAN_FLOAT((float)CellTemp_GetMinTemp(), CellTemp_CalculateTemperature(adc_under));
}

void test_Thermistor_MaxTemp_setter_clamps_negative_values_to_zero(void)
{
    CellTemp_SetMaxTemp(-10);
    TEST_ASSERT_EQUAL_INT8(0, CellTemp_GetMaxTemp());
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_Thermistor_NoConnection_returns_invalid_marker);
    RUN_TEST(test_Thermistor_OverTemp_condition_is_detectable_against_default_threshold);
    RUN_TEST(test_Thermistor_UnderTemp_condition_is_detectable_against_default_threshold);
    RUN_TEST(test_Thermistor_OverUnder_behavior_tracks_runtime_threshold_updates);
    RUN_TEST(test_Thermistor_MaxTemp_setter_clamps_negative_values_to_zero);

    return UNITY_END();
}
