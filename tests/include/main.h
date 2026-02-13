#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

extern osSemaphoreId_t BMSResetSemHandle;
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
