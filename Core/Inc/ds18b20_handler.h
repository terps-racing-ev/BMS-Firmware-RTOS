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

/* Types ---------------------------------------------------------------------*/
typedef enum {
    DS18B20_STATUS_OK = 0U,             /**< Temperature read succeeded */
    DS18B20_STATUS_NO_PRESENCE = 1U,    /**< Sensor did not send presence pulse */
    DS18B20_STATUS_CRC_ERROR = 2U,      /**< Scratchpad CRC check failed */
    DS18B20_STATUS_INVALID_PARAM = 3U,  /**< Caller provided invalid argument */
    DS18B20_STATUS_NOT_INITIALIZED = 4U,/**< Handler was not initialized */
    DS18B20_STATUS_DISABLED = 5U        /**< Feature is disabled at compile time */
} DS18B20_Status_t;

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
  * @brief  Main DS18B20 monitoring task
  * @param  argument: Not used
  * @retval None
  */
void DS18B20_MonitorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __DS18B20_HANDLER_H */