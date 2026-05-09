/**
  * @file           : ds18b20_handler.h
  * @brief          : DS18B20 1-Wire digital thermometer handler header
  */

#ifndef __DS18B20_HANDLER_H
#define __DS18B20_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
#define DS18B20_READ_INTERVAL_MS       5000U
#define DS18B20_STARTUP_DELAY_MS       2000U
#define DS18B20_CONVERT_TIME_MS        750U
#define DS18B20_MAX_READ_RETRIES       3U
#define DS18B20_RETRY_DELAY_MS         20U
#define DS18B20_INVALID_TEMP_C         ((int16_t)0x8000)
#define DS18B20_CAN_THERMISTOR_SLOTS   6U
#define DS18B20_CAN_INVALID_TEMP_C     ((int8_t)0)
#define DS18B20_ROM_SIZE               8U
#define DS18B20_MAX_SENSORS            DS18B20_CAN_THERMISTOR_SLOTS

/* Types ---------------------------------------------------------------------*/
typedef enum {
    DS18B20_STATUS_OK = 0U,             /**< Temperature read succeeded */
    DS18B20_STATUS_NO_PRESENCE = 1U,    /**< Sensor did not send presence pulse */
    DS18B20_STATUS_CRC_ERROR = 2U,      /**< Scratchpad CRC check failed */
    DS18B20_STATUS_INVALID_PARAM = 3U,  /**< Caller provided invalid argument */
    DS18B20_STATUS_NOT_INITIALIZED = 4U,/**< Handler was not initialized */
    DS18B20_STATUS_DISABLED = 5U        /**< Feature is disabled at compile time */
} DS18B20_Status_t;

typedef struct {
    uint8_t rom[DS18B20_ROM_SIZE];       /**< Unique DS18B20 ROM code */
    int16_t temperature_c;               /**< Last integer Celsius reading */
    int16_t raw_temperature;             /**< Last raw 1/16 degC DS18B20 reading */
    DS18B20_Status_t last_status;        /**< Last read status */
    uint8_t scratchpad_crc_read;         /**< Scratchpad CRC byte read from sensor */
    uint8_t scratchpad_crc_calculated;   /**< Locally calculated scratchpad CRC */
    uint8_t rom_crc;                     /**< ROM CRC byte read during discovery */
    uint8_t consecutive_errors;          /**< Consecutive read failures */
    uint8_t present;                     /**< Sensor discovered on bus */
} DS18B20_Sensor_t;

/* Function prototypes -------------------------------------------------------*/

/**
  * @brief  Initialize DS18B20 GPIO, DWT timing, and bus mutex
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef DS18B20_Init(void);

/**
  * @brief  Read DS18B20 temperature with integer degree Celsius resolution
  * @param  temperature_c: Pointer to store signed integer Celsius reading
  * @retval DS18B20_Status_t
  */
DS18B20_Status_t DS18B20_ReadTemperatureC(int16_t *temperature_c);

/**
  * @brief  Read all discovered DS18B20 temperatures with integer degree Celsius resolution
  * @param  temperatures_c: Array to store signed integer Celsius readings
  * @param  max_count: Maximum number of readings the array can store
  * @param  sensor_count: Pointer to store number of discovered sensors
  * @retval DS18B20_Status_t
  */
DS18B20_Status_t DS18B20_ReadAllTemperaturesC(int16_t *temperatures_c, uint8_t max_count, uint8_t *sensor_count);

/**
  * @brief  Get number of DS18B20 sensors discovered at initialization
  * @retval Number of discovered sensors, capped at DS18B20_MAX_SENSORS
  */
uint8_t DS18B20_GetSensorCount(void);

/**
  * @brief  Rediscover DS18B20 sensors on the 1-Wire bus
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef DS18B20_DiscoverSensors(void);

/**
  * @brief  Main DS18B20 monitoring task
  * @param  argument: Not used
  * @retval None
  */
void DS18B20_MonitorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __DS18B20_HANDLER_H */