#ifndef TEST_CMSIS_OS_H
#define TEST_CMSIS_OS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* osMutexId_t;
typedef void* osSemaphoreId_t;
typedef void* osMessageQueueId_t;

typedef struct
{
    const char *name;
    uint32_t attr_bits;
    void *cb_mem;
    uint32_t cb_size;
} osMutexAttr_t;

typedef enum
{
    osOK = 0,
    osError = -1
} osStatus_t;

#define osWaitForever 0xFFFFFFFFU

osMutexId_t osMutexNew(const osMutexAttr_t *attr);
osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex_id);
uint32_t osKernelGetTickCount(void);
void osDelay(uint32_t ticks);

#ifdef __cplusplus
}
#endif

#endif
