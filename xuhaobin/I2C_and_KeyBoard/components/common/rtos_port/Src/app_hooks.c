/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#include <FreeRTOS.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <task.h>
#include <timers.h>
#include "hosal_uart.h"
#include "mcu.h"
#include "uart_stdio.h"
#include "app_hooks.h"
#include "hosal_timer.h"
#include "hosal_slow_timer.h"
#include "hosal_lpm.h"
#include "hosal_sysctrl.h"
#include "FreeRTOSConfig.h"

extern uint8_t _heap_start;
extern uint8_t _heap_size;
extern uint8_t _heap2_start;
extern uint8_t _heap2_size;

static HeapRegion_t xHeapRegions[] = {
    {&_heap_start, (unsigned int)&_heap_size}, //set on runtime
    {&_heap2_start, (unsigned int)&_heap2_size},
    {NULL, 0}, /* Terminates the array. */
    {NULL, 0}  /* Terminates the array. */
};

void __attribute__((weak)) vHeapRegionsInt(void) {
                                                         
   vPortDefineHeapRegions(xHeapRegions);
}

void __attribute__((weak)) vApplicationStackOverflowHook(TaskHandle_t xTask,
                                                         char* pcTaskName) {
    puts("Stack Overflow checked\r\n");
    if (pcTaskName) {
        printf("Stack name %s\r\n", pcTaskName);
    }
    taskDISABLE_INTERRUPTS();
    NVIC_DisableIRQ(Wdt_IRQn);
    while (1) {
        /*empty here*/
    }
}

void __attribute__((weak)) vApplicationMallocFailedHook(void) {
    printf("Memory Allocate Failed. Current left size is %d bytes\r\n",
           xPortGetFreeHeapSize());
    taskDISABLE_INTERRUPTS();
    NVIC_DisableIRQ(Wdt_IRQn);
    while (1) {
        /*empty here*/
    }
}

void __attribute__((weak)) vApplicationIdleHook(void) {
    
    /*empty*/
}

#if (configUSE_TICKLESS_IDLE != 0)
void __attribute__((weak)) vApplicationSleep(TickType_t xExpectedIdleTime) {
    /*empty*/
}
#endif

#if (configUSE_TICK_HOOK != 0)
void __attribute__((weak)) vApplicationTickHook(void) { /*empty*/ }
#endif

void __attribute__((weak))
vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                              StackType_t** ppxIdleTaskStackBuffer,
                              uint32_t* pulIdleTaskStackSize) {
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void __attribute__((weak))
vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                               StackType_t** ppxTimerTaskStackBuffer,
                               uint32_t* pulTimerTaskStackSize) {
    /* If the buffers to be provided to the Timer task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void __attribute__((weak)) vAssertCalled(const char* const pcFileName,
                                         unsigned long ulLine) {
    char* current_task_name = (char*)pcTaskGetTaskName(
        xTaskGetCurrentTaskHandle());
    printf("assert: [%s] %s:%ld\n", current_task_name, pcFileName, ulLine);
    taskDISABLE_INTERRUPTS();
    while (1) {}
}

#ifdef CONFIG_HOSAL_SOC_IDLE_SLEEP
#define RT_SLOW_TIMER   CONFIG_HOSAL_SOC_SLEEP_TIMER_ID

uint8_t __attribute__((weak)) SystemReadyToSleep(void)
{
    return 1;
}



void RTOS_Wakeup_timer_Init() {

    uint32_t timer_id = RT_SLOW_TIMER;

    hosal_slow_timer_config_t rt_slow_timer ;
    hosal_slow_timer_tick_config_t rt_slow_timer_tick;

    /* This setting is 500ms timeout */
    rt_slow_timer.counting_mode = HOSAL_TIMER_DOWN_COUNTING;
    rt_slow_timer.int_enable = HOSAL_SLOW_TIMER_INT_ENABLE;
    rt_slow_timer.mode = HOSAL_SLOW_TIMER_FREERUN_MODE;
    rt_slow_timer.one_shot_mode = HOSAL_SLOW_TIMER_ONE_SHOT_DISABLE;
    rt_slow_timer.prescale = HOSAL_SLOW_TIMER_PRESCALE_1;
    rt_slow_timer.user_prescale = 0;
    rt_slow_timer.repeat_delay = 0;
 
    rt_slow_timer_tick.timeload_ticks = 0;
    rt_slow_timer_tick.timeout_ticks = 0;

    #if defined(CONFIG_RT584H) || defined(CONFIG_RT584L)
    timer_id = RT_SLOW_TIMER - 3;
    #if (CONFIG_HOSAL_SOC_SLEEP_TIMER_ID == 4)
    NVIC_DisableIRQ((IRQn_Type)(SlowTimer1_IRQn));
    NVIC_SetPriority((IRQn_Type)(SlowTimer1_IRQn), 1);
    #elif (CONFIG_HOSAL_SOC_SLEEP_TIMER_ID == 3)
    NVIC_DisableIRQ((IRQn_Type)(SlowTimer0_IRQn));
    NVIC_SetPriority((IRQn_Type)(SlowTimer0_IRQn), 1);
    #endif
    #else
    timer_id = RT_SLOW_TIMER;
    #if (CONFIG_HOSAL_SOC_SLEEP_TIMER_ID == 4)
    NVIC_DisableIRQ((IRQn_Type)(Timer4_IRQn));
    NVIC_SetPriority((IRQn_Type)(Timer4_IRQn), 1);
    #elif (CONFIG_HOSAL_SOC_SLEEP_TIMER_ID == 3)
    NVIC_DisableIRQ((IRQn_Type)(Timer3_IRQn));
    NVIC_SetPriority((IRQn_Type)(Timer3_IRQn), 1);
    #endif
    #endif
    
    hosal_slow_timer_init(timer_id, rt_slow_timer, NULL);
    hosal_slow_timer_stop(timer_id);

    lpm_set_low_power_level(LOW_POWER_LEVEL_SLEEP0);
    lpm_enable_low_power_wakeup(LOW_POWER_WAKEUP_GPIO);
    lpm_enable_low_power_wakeup(LOW_POWER_WAKEUP_SLOW_TIMER);
    lpm_enable_low_power_wakeup(LOW_POWER_WAKEUP_RTC_TIMER);
}

void RTOS_PreSleepProcessing(uint32_t xExpectedIdleTime_ms)
{
    uint32_t  clock_tick_k;
    uint32_t timer_id = RT_SLOW_TIMER;

    hosal_slow_timer_config_t rt_slow_timer ;
    hosal_slow_timer_tick_config_t rt_slow_timer_tick;

    RTOS_Wakeup_timer_Init();
    hosal_get_rco_clock_tick(&clock_tick_k);
    
    rt_slow_timer.counting_mode = HOSAL_TIMER_DOWN_COUNTING;
    rt_slow_timer.int_enable = HOSAL_SLOW_TIMER_INT_ENABLE;
    rt_slow_timer.mode = HOSAL_SLOW_TIMER_FREERUN_MODE;
    rt_slow_timer.one_shot_mode = HOSAL_SLOW_TIMER_ONE_SHOT_DISABLE;
    rt_slow_timer.prescale = HOSAL_SLOW_TIMER_PRESCALE_1;
    rt_slow_timer.user_prescale = 0;
    rt_slow_timer.repeat_delay = 0;
 
    rt_slow_timer_tick.timeload_ticks = (clock_tick_k * xExpectedIdleTime_ms) - 1;
    rt_slow_timer_tick.timeout_ticks = 0;

    #if defined(CONFIG_RT584H) || defined(CONFIG_RT584L)
    timer_id = RT_SLOW_TIMER - 3;
    #if (CONFIG_HOSAL_SOC_SLEEP_TIMER_ID == 4)
    NVIC_EnableIRQ((IRQn_Type)(SlowTimer1_IRQn));
    #elif (CONFIG_HOSAL_SOC_SLEEP_TIMER_ID == 3)
    NVIC_EnableIRQ((IRQn_Type)(SlowTimer0_IRQn));
    #endif
    #else
    timer_id = RT_SLOW_TIMER;
    #if (CONFIG_HOSAL_SOC_SLEEP_TIMER_ID == 4)
    NVIC_EnableIRQ((IRQn_Type)(Timer4_IRQn));
    #elif (CONFIG_HOSAL_SOC_SLEEP_TIMER_ID == 3)
    NVIC_EnableIRQ((IRQn_Type)(Timer3_IRQn));
    #endif
    #endif

    hosal_slow_timer_start(timer_id, rt_slow_timer_tick);
}

void RTOS_PostSleepProcessing(uint32_t* xnow_v)
{
    
    uint32_t clock_tick_k = 0, tick_value;
    uint32_t timer_id = RT_SLOW_TIMER;

    #if defined(CONFIG_RT584H) || defined(CONFIG_RT584L)
    timer_id = RT_SLOW_TIMER - 3;
    #endif

    hosal_get_rco_clock_tick(&clock_tick_k);
    hosal_slow_timer_stop(timer_id);
    hosal_slow_timer_clear_int(timer_id);
    hosal_delay_us(250);

    hosal_slow_timer_current_get(timer_id, &tick_value);

    *xnow_v = ( tick_value / clock_tick_k);
    
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime_ms) {
    TickType_t xModifiableIdleTime;
    uint32_t now_v, para;
    uint32_t clock_tick_k =0;

    __disable_irq();

    if(!SystemReadyToSleep())
    {
        __enable_irq();
        return;
    }
    if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
        __enable_irq();
        return;
    }

    hosal_get_rco_clock_tick(&clock_tick_k);
    if ( xExpectedIdleTime_ms > ((uint32_t)-1 / clock_tick_k) )
    {
        xExpectedIdleTime_ms = (uint32_t)-1 / clock_tick_k;
    }
    if (xExpectedIdleTime_ms > 0) {
        hosal_get_lpm_ioctrl(HOSAL_LPM_GET_MASK, &para);
        if (lpm_get_low_power_mask_status() != LOW_POWER_NO_MASK) {
            __enable_irq();
            return;
        } else {
            RTOS_PreSleepProcessing(xExpectedIdleTime_ms);

            lpm_enter_low_power_mode();

            RTOS_PostSleepProcessing(&now_v);

            if (now_v > (xExpectedIdleTime_ms)) {
                now_v = 0;
            }
            xModifiableIdleTime = (xExpectedIdleTime_ms)-now_v;

            vTaskStepTick(xModifiableIdleTime);
        }
    }

    __enable_irq();
}
#endif