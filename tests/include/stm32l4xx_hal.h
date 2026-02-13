#ifndef TEST_STM32L4XX_HAL_H
#define TEST_STM32L4XX_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t dummy;
} I2C_HandleTypeDef;

typedef struct
{
    uint32_t dummy;
} ADC_HandleTypeDef;

typedef struct
{
    uint32_t dummy;
} CAN_HandleTypeDef;

#define GPIO_PIN_0   ((uint16_t)0x0001U)
#define GPIO_PIN_1   ((uint16_t)0x0002U)
#define GPIO_PIN_3   ((uint16_t)0x0008U)
#define GPIO_PIN_4   ((uint16_t)0x0010U)
#define GPIO_PIN_5   ((uint16_t)0x0020U)
#define GPIO_PIN_6   ((uint16_t)0x0040U)
#define GPIO_PIN_7   ((uint16_t)0x0080U)
#define GPIO_PIN_8   ((uint16_t)0x0100U)

typedef struct
{
    uint32_t dummy;
} GPIO_TypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

typedef struct
{
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
    uint32_t SingleDiff;
    uint32_t OffsetNumber;
    uint32_t Offset;
} ADC_ChannelConfTypeDef;

#define ADC_CHANNEL_5 5U
#define ADC_CHANNEL_6 6U
#define ADC_CHANNEL_8 8U
#define ADC_CHANNEL_9 9U
#define ADC_CHANNEL_10 10U
#define ADC_CHANNEL_11 11U
#define ADC_CHANNEL_15 15U

#define ADC_REGULAR_RANK_1 1U
#define ADC_SAMPLETIME_640CYCLES_5 640U
#define ADC_SINGLE_ENDED 0U
#define ADC_OFFSET_NONE 0U

#define HAL_MAX_DELAY 0xFFFFFFFFU

extern GPIO_TypeDef *GPIOA;
extern GPIO_TypeDef *GPIOB;

HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *config);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout);
uint16_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

#ifdef __cplusplus
}
#endif

#endif
