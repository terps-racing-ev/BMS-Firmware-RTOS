/**
    * @file           : ds18b20_handler.c
    * @brief          : DS18B20 1-Wire digital thermometer handler implementation
    */

/* Includes ------------------------------------------------------------------*/
#include "ds18b20_handler.h"
#include "can_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>

#if DS18B20_FEATURE_ENABLED

/* Private defines -----------------------------------------------------------*/
#define DS18B20_DQ_GPIO_PORT           GPIOB
#define DS18B20_DQ_PIN                 GPIO_PIN_7

#define DS18B20_CMD_SKIP_ROM           0xCCU
#define DS18B20_CMD_CONVERT_T          0x44U
#define DS18B20_CMD_READ_SCRATCHPAD    0xBEU
#define DS18B20_SCRATCHPAD_SIZE        9U
#define DS18B20_SCRATCHPAD_CRC_INDEX   8U

#define OW_RESET_LOW_US                480U
#define OW_PRESENCE_SAMPLE_US          70U
#define OW_RESET_RECOVERY_US           410U
#define OW_WRITE_1_LOW_US              6U
#define OW_WRITE_1_RELEASE_US          64U
#define OW_WRITE_0_LOW_US              60U
#define OW_WRITE_0_RELEASE_US          10U
#define OW_READ_LOW_US                 3U
#define OW_READ_SAMPLE_US              12U
#define OW_READ_RECOVERY_US            55U

/* Private variables ---------------------------------------------------------*/
static osMutexId_t ds18b20_mutex = NULL;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef DS18B20_InitGPIO(void);
static HAL_StatusTypeDef DS18B20_InitDWT(void);
static DS18B20_Status_t DS18B20_ReadTemperatureAttempt(int16_t *temperature_c);
static int16_t DS18B20_RawToIntegerCelsius(int16_t raw_temperature);
static void DS18B20_SendCANMessage(const int16_t *thermistor_temps_c);
static void OW_DelayUs(uint32_t delay_us);
static void OW_DriveLow(void);
static void OW_Release(void);
static bool OW_ReadPin(void);
static bool OW_Reset(void);
static void OW_WriteBit(bool bit_value);
static bool OW_ReadBit(void);
static void OW_WriteByte(uint8_t byte_value);
static uint8_t OW_ReadByte(void);
static uint8_t OW_ComputeCRC8(const uint8_t *data, uint8_t length);

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Initialize DS18B20 GPIO, DWT timing, and bus mutex
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef DS18B20_Init(void)
{
    const osMutexAttr_t mutex_attr = {
        .name = "DS18B20Mutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = NULL,
        .cb_size = 0U
    };

    if (DS18B20_InitGPIO() != HAL_OK) {
        return HAL_ERROR;
    }

    if (DS18B20_InitDWT() != HAL_OK) {
        return HAL_ERROR;
    }

    ds18b20_mutex = osMutexNew(&mutex_attr);
    if (ds18b20_mutex == NULL) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Read DS18B20 temperature with integer degree Celsius resolution
  * @param  temperature_c: Pointer to store signed integer Celsius reading
  * @retval DS18B20_Status_t
  */
DS18B20_Status_t DS18B20_ReadTemperatureC(int16_t *temperature_c)
{
    DS18B20_Status_t status = DS18B20_STATUS_NO_PRESENCE;

    if (temperature_c == NULL) {
        return DS18B20_STATUS_INVALID_PARAM;
    }

    *temperature_c = DS18B20_INVALID_TEMP_C;

    if (ds18b20_mutex == NULL) {
        return DS18B20_STATUS_NOT_INITIALIZED;
    }

    osMutexAcquire(ds18b20_mutex, osWaitForever);

    for (uint8_t attempt = 0U; attempt < DS18B20_MAX_READ_RETRIES; attempt++) {
        status = DS18B20_ReadTemperatureAttempt(temperature_c);
        if (status == DS18B20_STATUS_OK) {
            break;
        }

        osDelay(DS18B20_RETRY_DELAY_MS);
    }

    osMutexRelease(ds18b20_mutex);

    return status;
}

/**
  * @brief  Main DS18B20 monitoring task
  * @param  argument: Not used
  * @retval None
  */
void DS18B20_MonitorTask(void *argument)
{
    int16_t temperature_c = DS18B20_INVALID_TEMP_C;
    DS18B20_Status_t status = DS18B20_STATUS_NOT_INITIALIZED;
    int16_t thermistor_temps_c[DS18B20_CAN_THERMISTOR_SLOTS] = {
        DS18B20_INVALID_TEMP_C,
        DS18B20_INVALID_TEMP_C,
        DS18B20_INVALID_TEMP_C,
        DS18B20_INVALID_TEMP_C,
        DS18B20_INVALID_TEMP_C,
        DS18B20_INVALID_TEMP_C
    };

    (void)argument;

    osDelay(DS18B20_STARTUP_DELAY_MS);

    for (;;) {
        status = DS18B20_ReadTemperatureC(&temperature_c);

        for (uint8_t slot = 0U; slot < DS18B20_CAN_THERMISTOR_SLOTS; slot++) {
            thermistor_temps_c[slot] = DS18B20_INVALID_TEMP_C;
        }

        if (status == DS18B20_STATUS_OK) {
            thermistor_temps_c[0] = temperature_c;
        }

        DS18B20_SendCANMessage(thermistor_temps_c);
        osDelay(DS18B20_READ_INTERVAL_MS);
    }
}

/* Static Function Implementations ------------------------------------------*/

/**
  * @brief  Configure PB7 as open-drain 1-Wire bus with internal pull-up
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef DS18B20_InitGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = DS18B20_DQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(DS18B20_DQ_GPIO_PORT, &GPIO_InitStruct);

    OW_Release();

    return HAL_OK;
}

/**
  * @brief  Enable Cortex-M DWT cycle counter for microsecond timing
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef DS18B20_InitDWT(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    return ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? HAL_OK : HAL_ERROR;
}

/**
  * @brief  Perform one complete DS18B20 conversion and scratchpad read
  * @param  temperature_c: Pointer to store signed integer Celsius reading
  * @retval DS18B20_Status_t
  */
static DS18B20_Status_t DS18B20_ReadTemperatureAttempt(int16_t *temperature_c)
{
    uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE] = {0};
    int16_t raw_temperature;

    if (!OW_Reset()) {
        return DS18B20_STATUS_NO_PRESENCE;
    }

    OW_WriteByte(DS18B20_CMD_SKIP_ROM);
    OW_WriteByte(DS18B20_CMD_CONVERT_T);

    osDelay(DS18B20_CONVERT_TIME_MS);

    if (!OW_Reset()) {
        return DS18B20_STATUS_NO_PRESENCE;
    }

    OW_WriteByte(DS18B20_CMD_SKIP_ROM);
    OW_WriteByte(DS18B20_CMD_READ_SCRATCHPAD);

    for (uint8_t index = 0U; index < DS18B20_SCRATCHPAD_SIZE; index++) {
        scratchpad[index] = OW_ReadByte();
    }

    if (OW_ComputeCRC8(scratchpad, DS18B20_SCRATCHPAD_CRC_INDEX) != scratchpad[DS18B20_SCRATCHPAD_CRC_INDEX]) {
        return DS18B20_STATUS_CRC_ERROR;
    }

    raw_temperature = (int16_t)(((uint16_t)scratchpad[1] << 8U) | scratchpad[0]);
    *temperature_c = DS18B20_RawToIntegerCelsius(raw_temperature);

    return DS18B20_STATUS_OK;
}

/**
  * @brief  Round raw 1/16 degC DS18B20 value to nearest integer degC
  * @param  raw_temperature: Signed DS18B20 raw temperature
  * @retval Rounded integer temperature in degrees Celsius
  */
static int16_t DS18B20_RawToIntegerCelsius(int16_t raw_temperature)
{
    int16_t integer_c = raw_temperature / 16;
    int16_t fraction = raw_temperature % 16;

    if (fraction >= 8) {
        integer_c++;
    } else if (fraction <= -8) {
        integer_c--;
    }

    return integer_c;
}

/**
 * @brief  Send up to six E-meter thermistor temperatures over CAN
 * @param  thermistor_temps_c: Array of integer Celsius values for six slots
  * @retval None
  */
static void DS18B20_SendCANMessage(const int16_t *thermistor_temps_c)
{
    uint8_t tx_data[8] = {0};
    int8_t temp_c;

    if (thermistor_temps_c == NULL) {
        return;
    }

    for (uint8_t slot = 0U; slot < DS18B20_CAN_THERMISTOR_SLOTS; slot++) {
        if ((thermistor_temps_c[slot] < -55) ||
            (thermistor_temps_c[slot] > 125) ||
            (thermistor_temps_c[slot] == DS18B20_INVALID_TEMP_C)) {
            temp_c = DS18B20_CAN_INVALID_TEMP_C;
        } else {
            temp_c = (int8_t)thermistor_temps_c[slot];
        }

        tx_data[slot] = (uint8_t)temp_c;
    }

    tx_data[6] = 0U;
    tx_data[7] = 0U;

    (void)CAN_SendMessage(CAN_DS18B20_TEMP_ID, tx_data, 8U, CAN_PRIORITY_NORMAL);
}

/**
  * @brief  Busy-wait for a microsecond interval using DWT cycle counter
  * @param  delay_us: Delay in microseconds
  * @retval None
  */
static void OW_DelayUs(uint32_t delay_us)
{
    uint32_t start_cycle = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * delay_us;

    while ((DWT->CYCCNT - start_cycle) < cycles) {
    }
}

/**
  * @brief  Actively drive the 1-Wire bus low
  * @retval None
  */
static void OW_DriveLow(void)
{
    DS18B20_DQ_GPIO_PORT->BSRR = ((uint32_t)DS18B20_DQ_PIN << 16U);
}

/**
  * @brief  Release the 1-Wire bus high through internal pull-up
  * @retval None
  */
static void OW_Release(void)
{
    DS18B20_DQ_GPIO_PORT->BSRR = DS18B20_DQ_PIN;
}

/**
  * @brief  Read current 1-Wire bus level
  * @retval true if high, false if low
  */
static bool OW_ReadPin(void)
{
    return ((DS18B20_DQ_GPIO_PORT->IDR & DS18B20_DQ_PIN) != 0U);
}

/**
  * @brief  Issue 1-Wire reset and detect DS18B20 presence pulse
  * @retval true if presence pulse detected, false otherwise
  */
static bool OW_Reset(void)
{
    bool presence_detected;

    taskENTER_CRITICAL();
    OW_DriveLow();
    OW_DelayUs(OW_RESET_LOW_US);
    OW_Release();
    OW_DelayUs(OW_PRESENCE_SAMPLE_US);
    presence_detected = !OW_ReadPin();
    OW_DelayUs(OW_RESET_RECOVERY_US);
    taskEXIT_CRITICAL();

    return presence_detected;
}

/**
  * @brief  Write one 1-Wire bit
  * @param  bit_value: Bit value to write
  * @retval None
  */
static void OW_WriteBit(bool bit_value)
{
    taskENTER_CRITICAL();
    OW_DriveLow();
    if (bit_value) {
        OW_DelayUs(OW_WRITE_1_LOW_US);
        OW_Release();
        OW_DelayUs(OW_WRITE_1_RELEASE_US);
    } else {
        OW_DelayUs(OW_WRITE_0_LOW_US);
        OW_Release();
        OW_DelayUs(OW_WRITE_0_RELEASE_US);
    }
    taskEXIT_CRITICAL();
}

/**
  * @brief  Read one 1-Wire bit
  * @retval Bit value read from bus
  */
static bool OW_ReadBit(void)
{
    bool bit_value;

    taskENTER_CRITICAL();
    OW_DriveLow();
    OW_DelayUs(OW_READ_LOW_US);
    OW_Release();
    OW_DelayUs(OW_READ_SAMPLE_US);
    bit_value = OW_ReadPin();
    OW_DelayUs(OW_READ_RECOVERY_US);
    taskEXIT_CRITICAL();

    return bit_value;
}

/**
  * @brief  Write one 1-Wire byte, least significant bit first
  * @param  byte_value: Byte value to write
  * @retval None
  */
static void OW_WriteByte(uint8_t byte_value)
{
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        OW_WriteBit(((byte_value >> bit) & 0x01U) != 0U);
    }
}

/**
  * @brief  Read one 1-Wire byte, least significant bit first
  * @retval Byte read from bus
  */
static uint8_t OW_ReadByte(void)
{
    uint8_t byte_value = 0U;

    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if (OW_ReadBit()) {
            byte_value |= (uint8_t)(1U << bit);
        }
    }

    return byte_value;
}

/**
  * @brief  Compute Maxim/Dallas 1-Wire CRC-8
  * @param  data: Input bytes
  * @param  length: Number of bytes to include
  * @retval CRC-8 value
  */
static uint8_t OW_ComputeCRC8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    for (uint8_t index = 0U; index < length; index++) {
        uint8_t input_byte = data[index];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            uint8_t mix = (crc ^ input_byte) & 0x01U;
            crc >>= 1U;
            if (mix != 0U) {
                crc ^= 0x8CU;
            }
            input_byte >>= 1U;
        }
    }

    return crc;
}

#else

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  No-op DS18B20 init when feature is disabled
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef DS18B20_Init(void)
{
    return HAL_OK;
}

/**
  * @brief  Disabled DS18B20 temperature read stub
  * @param  temperature_c: Pointer to store invalid marker
  * @retval DS18B20_Status_t
  */
DS18B20_Status_t DS18B20_ReadTemperatureC(int16_t *temperature_c)
{
    if (temperature_c != NULL) {
        *temperature_c = DS18B20_INVALID_TEMP_C;
    }

    return DS18B20_STATUS_DISABLED;
}

/**
  * @brief  Disabled DS18B20 monitor task stub
  * @param  argument: Not used
  * @retval None
  */
void DS18B20_MonitorTask(void *argument)
{
    (void)argument;

    for (;;) {
        osDelay(1000U);
    }
}

#endif /* DS18B20_FEATURE_ENABLED */