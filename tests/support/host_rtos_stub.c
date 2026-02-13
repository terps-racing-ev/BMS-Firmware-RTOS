#include "cmsis_os.h"
#include "test_support.h"

static uint32_t g_tick_ms = 0U;
static bool g_mutex_create_should_fail = false;
static uint32_t g_mutex_create_count = 0U;

static uint8_t g_mutex_object = 0U;

osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    (void)attr;
    g_mutex_create_count++;
    if (g_mutex_create_should_fail)
    {
        return NULL;
    }
    return (osMutexId_t)&g_mutex_object;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    (void)mutex_id;
    (void)timeout;
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    (void)mutex_id;
    return osOK;
}

uint32_t osKernelGetTickCount(void)
{
    return g_tick_ms;
}

void osDelay(uint32_t ticks)
{
    g_tick_ms += ticks;
}

void TestSupport_SetTick(uint32_t tick)
{
    g_tick_ms = tick;
}

void TestSupport_AdvanceTick(uint32_t delta_ms)
{
    g_tick_ms += delta_ms;
}

void TestSupport_SetMutexCreateShouldFail(bool should_fail)
{
    g_mutex_create_should_fail = should_fail;
}

uint32_t TestSupport_GetMutexCreateCount(void)
{
    return g_mutex_create_count;
}

void TestSupport_ResetRtosStubs(void)
{
    g_tick_ms = 0U;
    g_mutex_create_should_fail = false;
    g_mutex_create_count = 0U;
}
