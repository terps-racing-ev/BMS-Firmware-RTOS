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

#define DS18B20_CMD_SEARCH_ROM         0xF0U
#define DS18B20_CMD_MATCH_ROM          0x55U
#define DS18B20_CMD_SKIP_ROM           0xCCU
#define DS18B20_CMD_CONVERT_T          0x44U
#define DS18B20_CMD_READ_SCRATCHPAD    0xBEU
#define DS18B20_SCRATCHPAD_SIZE        9U
#define DS18B20_SCRATCHPAD_CRC_INDEX   8U
#define DS18B20_FAMILY_CODE            0x28U
#define DS18B20_DEBUG_INVALID_RAW      DS18B20_INVALID_TEMP_C

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

/* Private types -------------------------------------------------------------*/
typedef struct {
    uint8_t discovery_attempts;
    uint8_t search_passes;
    uint8_t rom_crc_errors;
    uint8_t family_mismatch_errors;
    uint8_t present_mask;
    uint8_t successful_read_mask;
    uint8_t no_presence_mask;
    uint8_t crc_error_mask;
    uint8_t last_overall_status;
} DS18B20_DebugState_t;

/* Private variables ---------------------------------------------------------*/
static osMutexId_t ds18b20_mutex = NULL;
static DS18B20_Sensor_t ds18b20_sensors[DS18B20_MAX_SENSORS] = {0};
static uint8_t ds18b20_sensor_count = 0U;
static DS18B20_DebugState_t ds18b20_debug_state = {0};

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef DS18B20_InitGPIO(void);
static HAL_StatusTypeDef DS18B20_InitDWT(void);
static HAL_StatusTypeDef DS18B20_DiscoverSensorsUnlocked(void);
static DS18B20_Status_t DS18B20_ReadAllTemperaturesLocked(int16_t *temperatures_c, uint8_t max_count, uint8_t *sensor_count);
static DS18B20_Status_t DS18B20_ReadSensorScratchpad(DS18B20_Sensor_t *sensor);
static void DS18B20_ClearSensors(void);
static void DS18B20_SortSensors(void);
static void DS18B20_SwapSensors(DS18B20_Sensor_t *sensor_a, DS18B20_Sensor_t *sensor_b);
static int16_t DS18B20_RawToIntegerCelsius(int16_t raw_temperature);
static void DS18B20_SendCANMessage(const int16_t *thermistor_temps_c);
static void DS18B20_SendDebugCANMessage(DS18B20_Status_t overall_status, uint8_t sensor_count);
static bool OW_Search(uint8_t *rom, uint8_t *last_discrepancy, bool *last_device_flag);
static void OW_MatchROM(const uint8_t *rom);
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

    if (DS18B20_DiscoverSensorsUnlocked() != HAL_OK) {
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
    int16_t temperatures_c[DS18B20_MAX_SENSORS] = {0};
    uint8_t sensor_count = 0U;
    DS18B20_Status_t status;

    if (temperature_c == NULL) {
        return DS18B20_STATUS_INVALID_PARAM;
    }

    *temperature_c = DS18B20_INVALID_TEMP_C;

    status = DS18B20_ReadAllTemperaturesC(temperatures_c, DS18B20_MAX_SENSORS, &sensor_count);
    if (sensor_count > 0U) {
        *temperature_c = temperatures_c[0];
        return ds18b20_sensors[0].last_status;
    }

    return status;
}

/**
  * @brief  Read all discovered DS18B20 temperatures with integer degree Celsius resolution
  * @param  temperatures_c: Array to store signed integer Celsius readings
  * @param  max_count: Maximum number of readings the array can store
  * @param  sensor_count: Pointer to store number of discovered sensors
  * @retval DS18B20_Status_t
  */
DS18B20_Status_t DS18B20_ReadAllTemperaturesC(int16_t *temperatures_c, uint8_t max_count, uint8_t *sensor_count)
{
    DS18B20_Status_t status = DS18B20_STATUS_NO_PRESENCE;

    if ((temperatures_c == NULL) || (sensor_count == NULL) || (max_count == 0U)) {
        return DS18B20_STATUS_INVALID_PARAM;
    }

    *sensor_count = 0U;

    for (uint8_t slot = 0U; slot < max_count; slot++) {
        temperatures_c[slot] = DS18B20_INVALID_TEMP_C;
    }

    if (ds18b20_mutex == NULL) {
        return DS18B20_STATUS_NOT_INITIALIZED;
    }

    osMutexAcquire(ds18b20_mutex, osWaitForever);

    if (ds18b20_sensor_count == 0U) {
        (void)DS18B20_DiscoverSensorsUnlocked();
    }

    for (uint8_t attempt = 0U; attempt < DS18B20_MAX_READ_RETRIES; attempt++) {
        status = DS18B20_ReadAllTemperaturesLocked(temperatures_c, max_count, sensor_count);
        if (status == DS18B20_STATUS_OK) {
            break;
        }

        osDelay(DS18B20_RETRY_DELAY_MS);
    }

    osMutexRelease(ds18b20_mutex);

    return status;
}

/**
    * @brief  Get number of DS18B20 sensors discovered at initialization
    * @retval Number of discovered sensors, capped at DS18B20_MAX_SENSORS
    */
uint8_t DS18B20_GetSensorCount(void)
{
        return ds18b20_sensor_count;
}

/**
    * @brief  Rediscover DS18B20 sensors on the 1-Wire bus
    * @retval HAL_StatusTypeDef
    */
HAL_StatusTypeDef DS18B20_DiscoverSensors(void)
{
        HAL_StatusTypeDef status;

        if (ds18b20_mutex == NULL) {
                return HAL_ERROR;
        }

        osMutexAcquire(ds18b20_mutex, osWaitForever);
        status = DS18B20_DiscoverSensorsUnlocked();
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
    uint8_t sensor_count = 0U;
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
        for (uint8_t slot = 0U; slot < DS18B20_CAN_THERMISTOR_SLOTS; slot++) {
            thermistor_temps_c[slot] = DS18B20_INVALID_TEMP_C;
        }

        status = DS18B20_ReadAllTemperaturesC(thermistor_temps_c, DS18B20_CAN_THERMISTOR_SLOTS, &sensor_count);

        DS18B20_SendCANMessage(thermistor_temps_c);
        DS18B20_SendDebugCANMessage(status, sensor_count);
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
  * @brief  Discover DS18B20 sensors on the bus without taking the mutex
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef DS18B20_DiscoverSensorsUnlocked(void)
{
    uint8_t rom[DS18B20_ROM_SIZE] = {0};
    uint8_t last_discrepancy = 0U;
    bool last_device_flag = false;

    DS18B20_ClearSensors();
    if (ds18b20_debug_state.discovery_attempts < UINT8_MAX) {
        ds18b20_debug_state.discovery_attempts++;
    }
    ds18b20_debug_state.search_passes = 0U;
    ds18b20_debug_state.rom_crc_errors = 0U;
    ds18b20_debug_state.family_mismatch_errors = 0U;
    ds18b20_debug_state.present_mask = 0U;

    while ((ds18b20_sensor_count < DS18B20_MAX_SENSORS) &&
           OW_Search(rom, &last_discrepancy, &last_device_flag)) {
        uint8_t calculated_rom_crc = OW_ComputeCRC8(rom, DS18B20_ROM_SIZE - 1U);

        if (ds18b20_debug_state.search_passes < UINT8_MAX) {
            ds18b20_debug_state.search_passes++;
        }

        if ((rom[0] == DS18B20_FAMILY_CODE) &&
            (calculated_rom_crc == rom[DS18B20_ROM_SIZE - 1U])) {
            memcpy(ds18b20_sensors[ds18b20_sensor_count].rom, rom, DS18B20_ROM_SIZE);
            ds18b20_sensors[ds18b20_sensor_count].temperature_c = DS18B20_INVALID_TEMP_C;
            ds18b20_sensors[ds18b20_sensor_count].raw_temperature = DS18B20_DEBUG_INVALID_RAW;
            ds18b20_sensors[ds18b20_sensor_count].last_status = DS18B20_STATUS_NOT_INITIALIZED;
            ds18b20_sensors[ds18b20_sensor_count].scratchpad_crc_read = 0U;
            ds18b20_sensors[ds18b20_sensor_count].scratchpad_crc_calculated = 0U;
            ds18b20_sensors[ds18b20_sensor_count].rom_crc = rom[DS18B20_ROM_SIZE - 1U];
            ds18b20_sensors[ds18b20_sensor_count].consecutive_errors = 0U;
            ds18b20_sensors[ds18b20_sensor_count].present = 1U;
            ds18b20_debug_state.present_mask |= (uint8_t)(1U << ds18b20_sensor_count);
            ds18b20_sensor_count++;
        } else if (rom[0] != DS18B20_FAMILY_CODE) {
            if (ds18b20_debug_state.family_mismatch_errors < UINT8_MAX) {
                ds18b20_debug_state.family_mismatch_errors++;
            }
        } else {
            if (ds18b20_debug_state.rom_crc_errors < UINT8_MAX) {
                ds18b20_debug_state.rom_crc_errors++;
            }
        }
    }

    DS18B20_SortSensors();

    return HAL_OK;
}

/**
  * @brief  Read all discovered DS18B20 sensors while the mutex is held
  * @param  temperatures_c: Array to store signed integer Celsius readings
  * @param  max_count: Maximum number of readings the array can store
  * @param  sensor_count: Pointer to store number of discovered sensors
  * @retval DS18B20_Status_t
  */
static DS18B20_Status_t DS18B20_ReadAllTemperaturesLocked(int16_t *temperatures_c, uint8_t max_count, uint8_t *sensor_count)
{
    DS18B20_Status_t last_status = DS18B20_STATUS_NO_PRESENCE;
    uint8_t successful_reads = 0U;

    *sensor_count = ds18b20_sensor_count;
    ds18b20_debug_state.successful_read_mask = 0U;
    ds18b20_debug_state.no_presence_mask = 0U;
    ds18b20_debug_state.crc_error_mask = 0U;

    if (ds18b20_sensor_count == 0U) {
        ds18b20_debug_state.last_overall_status = DS18B20_STATUS_NO_PRESENCE;
        return DS18B20_STATUS_NO_PRESENCE;
    }

    if (!OW_Reset()) {
        for (uint8_t sensor_index = 0U; sensor_index < ds18b20_sensor_count; sensor_index++) {
            ds18b20_sensors[sensor_index].temperature_c = DS18B20_INVALID_TEMP_C;
            ds18b20_sensors[sensor_index].raw_temperature = DS18B20_DEBUG_INVALID_RAW;
            ds18b20_sensors[sensor_index].last_status = DS18B20_STATUS_NO_PRESENCE;
            ds18b20_sensors[sensor_index].scratchpad_crc_read = 0U;
            ds18b20_sensors[sensor_index].scratchpad_crc_calculated = 0U;
            if (ds18b20_sensors[sensor_index].consecutive_errors < UINT8_MAX) {
                ds18b20_sensors[sensor_index].consecutive_errors++;
            }
            ds18b20_debug_state.no_presence_mask |= (uint8_t)(1U << sensor_index);
        }

        ds18b20_debug_state.last_overall_status = DS18B20_STATUS_NO_PRESENCE;
        return DS18B20_STATUS_NO_PRESENCE;
    }

    OW_WriteByte(DS18B20_CMD_SKIP_ROM);
    OW_WriteByte(DS18B20_CMD_CONVERT_T);

    osDelay(DS18B20_CONVERT_TIME_MS);

    for (uint8_t sensor_index = 0U;
         (sensor_index < ds18b20_sensor_count) && (sensor_index < max_count);
         sensor_index++) {
        last_status = DS18B20_ReadSensorScratchpad(&ds18b20_sensors[sensor_index]);
        temperatures_c[sensor_index] = ds18b20_sensors[sensor_index].temperature_c;

        if (last_status == DS18B20_STATUS_OK) {
            successful_reads++;
            ds18b20_debug_state.successful_read_mask |= (uint8_t)(1U << sensor_index);
        } else if (last_status == DS18B20_STATUS_NO_PRESENCE) {
            ds18b20_debug_state.no_presence_mask |= (uint8_t)(1U << sensor_index);
        } else if (last_status == DS18B20_STATUS_CRC_ERROR) {
            ds18b20_debug_state.crc_error_mask |= (uint8_t)(1U << sensor_index);
        }
    }

    ds18b20_debug_state.last_overall_status = (successful_reads > 0U) ? DS18B20_STATUS_OK : last_status;

    return (DS18B20_Status_t)ds18b20_debug_state.last_overall_status;
}

/**
  * @brief  Read scratchpad from one addressed DS18B20 sensor
  * @param  sensor: Sensor record containing ROM code and state
  * @retval DS18B20_Status_t
  */
static DS18B20_Status_t DS18B20_ReadSensorScratchpad(DS18B20_Sensor_t *sensor)
{
    uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE] = {0};
    int16_t raw_temperature;
    uint8_t calculated_crc;

    if ((sensor == NULL) || (sensor->present == 0U)) {
        return DS18B20_STATUS_INVALID_PARAM;
    }

    sensor->temperature_c = DS18B20_INVALID_TEMP_C;
    sensor->raw_temperature = DS18B20_DEBUG_INVALID_RAW;
    sensor->scratchpad_crc_read = 0U;
    sensor->scratchpad_crc_calculated = 0U;

    if (!OW_Reset()) {
        sensor->last_status = DS18B20_STATUS_NO_PRESENCE;
        if (sensor->consecutive_errors < UINT8_MAX) {
            sensor->consecutive_errors++;
        }
        return sensor->last_status;
    }

    OW_MatchROM(sensor->rom);
    OW_WriteByte(DS18B20_CMD_READ_SCRATCHPAD);

    for (uint8_t index = 0U; index < DS18B20_SCRATCHPAD_SIZE; index++) {
        scratchpad[index] = OW_ReadByte();
    }

    raw_temperature = (int16_t)(((uint16_t)scratchpad[1] << 8U) | scratchpad[0]);
    calculated_crc = OW_ComputeCRC8(scratchpad, DS18B20_SCRATCHPAD_CRC_INDEX);
    sensor->raw_temperature = raw_temperature;
    sensor->scratchpad_crc_read = scratchpad[DS18B20_SCRATCHPAD_CRC_INDEX];
    sensor->scratchpad_crc_calculated = calculated_crc;

    if (calculated_crc != scratchpad[DS18B20_SCRATCHPAD_CRC_INDEX]) {
        sensor->last_status = DS18B20_STATUS_CRC_ERROR;
        if (sensor->consecutive_errors < UINT8_MAX) {
            sensor->consecutive_errors++;
        }
        return sensor->last_status;
    }

    sensor->temperature_c = DS18B20_RawToIntegerCelsius(raw_temperature);
    sensor->last_status = DS18B20_STATUS_OK;
    sensor->consecutive_errors = 0U;

    return sensor->last_status;
}

/**
  * @brief  Clear discovered sensor state
  * @retval None
  */
static void DS18B20_ClearSensors(void)
{
    memset(ds18b20_sensors, 0, sizeof(ds18b20_sensors));
    ds18b20_sensor_count = 0U;
    ds18b20_debug_state.present_mask = 0U;
}

/**
  * @brief  Sort discovered sensors by ROM code for stable CAN slot mapping
  * @retval None
  */
static void DS18B20_SortSensors(void)
{
    for (uint8_t outer_index = 0U; outer_index < ds18b20_sensor_count; outer_index++) {
        for (uint8_t inner_index = (uint8_t)(outer_index + 1U); inner_index < ds18b20_sensor_count; inner_index++) {
            if (memcmp(ds18b20_sensors[outer_index].rom, ds18b20_sensors[inner_index].rom, DS18B20_ROM_SIZE) > 0) {
                DS18B20_SwapSensors(&ds18b20_sensors[outer_index], &ds18b20_sensors[inner_index]);
            }
        }
    }
}

/**
  * @brief  Swap two sensor records
  * @param  sensor_a: First sensor record
  * @param  sensor_b: Second sensor record
  * @retval None
  */
static void DS18B20_SwapSensors(DS18B20_Sensor_t *sensor_a, DS18B20_Sensor_t *sensor_b)
{
    DS18B20_Sensor_t temp_sensor = *sensor_a;

    *sensor_a = *sensor_b;
    *sensor_b = temp_sensor;
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
    * @brief  Send one DS18B20 debug summary frame over CAN
    * @param  overall_status: Status from the most recent read cycle
    * @param  sensor_count: Number of discovered sensors reported by the read cycle
    * @retval None
    */
static void DS18B20_SendDebugCANMessage(DS18B20_Status_t overall_status, uint8_t sensor_count)
{
        uint8_t tx_data[8] = {0};

        tx_data[0] = sensor_count;
        tx_data[1] = (uint8_t)overall_status;
        tx_data[2] = ds18b20_debug_state.present_mask;
        tx_data[3] = ds18b20_debug_state.successful_read_mask;
        tx_data[4] = ds18b20_debug_state.no_presence_mask;
        tx_data[5] = ds18b20_debug_state.crc_error_mask;
        tx_data[6] = ds18b20_debug_state.search_passes;
        tx_data[7] = (uint8_t)((ds18b20_debug_state.rom_crc_errors & 0x0FU) |
                                                     ((ds18b20_debug_state.family_mismatch_errors & 0x0FU) << 4U));

        (void)CAN_SendMessage(CAN_DS18B20_DEBUG_ID, tx_data, 8U, CAN_PRIORITY_LOW);
}

/**
  * @brief  Execute one Maxim 1-Wire Search ROM pass
  * @param  rom: ROM buffer; must retain previous search result between calls
  * @param  last_discrepancy: Search discrepancy state
  * @param  last_device_flag: Set true when the last device has been found
  * @retval true if a ROM code was found, false otherwise
  */
static bool OW_Search(uint8_t *rom, uint8_t *last_discrepancy, bool *last_device_flag)
{
    uint8_t id_bit_number = 1U;
    uint8_t last_zero = 0U;
    uint8_t rom_byte_number = 0U;
    uint8_t rom_byte_mask = 1U;
    uint8_t search_direction;
    bool id_bit;
    bool complement_id_bit;

    if ((rom == NULL) || (last_discrepancy == NULL) || (last_device_flag == NULL)) {
        return false;
    }

    if (*last_device_flag) {
        return false;
    }

    if (!OW_Reset()) {
        *last_discrepancy = 0U;
        *last_device_flag = false;
        return false;
    }

    OW_WriteByte(DS18B20_CMD_SEARCH_ROM);

    while (rom_byte_number < DS18B20_ROM_SIZE) {
        id_bit = OW_ReadBit();
        complement_id_bit = OW_ReadBit();

        if (id_bit && complement_id_bit) {
            break;
        }

        if (id_bit != complement_id_bit) {
            search_direction = id_bit ? 1U : 0U;
        } else {
            if (id_bit_number < *last_discrepancy) {
                search_direction = ((rom[rom_byte_number] & rom_byte_mask) != 0U) ? 1U : 0U;
            } else {
                search_direction = (id_bit_number == *last_discrepancy) ? 1U : 0U;
            }

            if (search_direction == 0U) {
                last_zero = id_bit_number;
            }
        }

        if (search_direction != 0U) {
            rom[rom_byte_number] |= rom_byte_mask;
        } else {
            rom[rom_byte_number] &= (uint8_t)~rom_byte_mask;
        }

        OW_WriteBit(search_direction != 0U);

        id_bit_number++;
        rom_byte_mask <<= 1U;

        if (rom_byte_mask == 0U) {
            rom_byte_number++;
            rom_byte_mask = 1U;
        }
    }

    if (id_bit_number < 65U) {
        return false;
    }

    *last_discrepancy = last_zero;
    if (last_zero == 0U) {
        *last_device_flag = true;
    }

    return true;
}

/**
  * @brief  Select one DS18B20 sensor by ROM code
  * @param  rom: 8-byte sensor ROM code
  * @retval None
  */
static void OW_MatchROM(const uint8_t *rom)
{
    OW_WriteByte(DS18B20_CMD_MATCH_ROM);

    for (uint8_t index = 0U; index < DS18B20_ROM_SIZE; index++) {
        OW_WriteByte(rom[index]);
    }
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
  * @brief  Disabled DS18B20 all-temperature read stub
  * @param  temperatures_c: Array to store invalid markers
  * @param  max_count: Maximum number of readings the array can store
  * @param  sensor_count: Pointer to store zero discovered sensors
  * @retval DS18B20_Status_t
  */
DS18B20_Status_t DS18B20_ReadAllTemperaturesC(int16_t *temperatures_c, uint8_t max_count, uint8_t *sensor_count)
{
    if (temperatures_c != NULL) {
        for (uint8_t slot = 0U; slot < max_count; slot++) {
            temperatures_c[slot] = DS18B20_INVALID_TEMP_C;
        }
    }

    if (sensor_count != NULL) {
        *sensor_count = 0U;
    }

    return DS18B20_STATUS_DISABLED;
}

/**
  * @brief  Disabled DS18B20 sensor count stub
  * @retval Zero discovered sensors
  */
uint8_t DS18B20_GetSensorCount(void)
{
    return 0U;
}

/**
  * @brief  Disabled DS18B20 rediscovery stub
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef DS18B20_DiscoverSensors(void)
{
    return HAL_OK;
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