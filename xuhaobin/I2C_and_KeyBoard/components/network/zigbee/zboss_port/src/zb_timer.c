/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * @file zb_timer.c
 * @author Rex Huang (rex.huang@rafaelmicro.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-24
 * 
 * 
 */
//=============================================================================
//                Include
//=============================================================================
#include "FreeRTOS.h"
#include "timers.h"

#include "mcu.h"
#include "hosal_timer.h"
#include "log.h"

#include "zb_common.h"
#include "zigbee_platform.h"
//=============================================================================
//                Private Definitions of const value
//=============================================================================
#define TIMER_ID 0
#define USE_HW_TIMER 0
//=============================================================================
//                Private ENUM
//=============================================================================

//=============================================================================
//                Private Struct
//=============================================================================

//=============================================================================
//                Private Global Variables
//=============================================================================
static TimerHandle_t zbAlarm_timerHandle = NULL;
static uint8_t timer_started = 0;
//=============================================================================
//                Functions
//=============================================================================
#if USE_HW_TIMER
static void zigbee_timer_callback (uint32_t timer_id)
#else
static void zigbee_timer_callback (TimerHandle_t xTimer)
#endif
{
    ZB_TIMER_CTX().timer++;
#if USE_HW_TIMER
    ZB_NOTIFY_ISR(ZB_SYSTEM_EVENT_ALARM_MS_EXPIRED);
#else
    ZB_NOTIFY(ZB_SYSTEM_EVENT_ALARM_MS_EXPIRED);
#endif
}

void zigbee_timer_init(void)
{
#if USE_HW_TIMER
    hosal_timer_config_t timer_config;

    timer_config.counting_mode = HOSAL_TIMER_DOWN_COUNTING;
    timer_config.int_en = HOSAL_TIMER_INT_ENABLE;
    timer_config.mode = HOSAL_TIMER_PERIODIC_MODE;
    timer_config.oneshot_mode = HOSAL_TIMER_ONE_SHOT_DISABLE;
    timer_config.prescale = HOSAL_TIMER_PRESCALE_32;
    timer_config.user_prescale = 0;


    hosal_timer_init(TIMER_ID, timer_config, zigbee_timer_callback);
#else
    zbAlarm_timerHandle = xTimerCreate("zb_timer", 15, pdTRUE, 
            (void *)zbAlarm_timerHandle, zigbee_timer_callback);
#endif
}

void zigbee_timer_start(void)
{
#if USE_HW_TIMER
    hosal_timer_tick_config_t tick_cfg;
    tick_cfg.timeload_ticks = 15360;
    tick_cfg.timeout_ticks = 0;
    NVIC_EnableIRQ((IRQn_Type)(Timer0_IRQn));
    NVIC_SetPriority((IRQn_Type)(Timer0_IRQn), 5);

    hosal_timer_start(TIMER_ID, tick_cfg);
#else
    BaseType_t pxHigherPriorityTaskWoken = pdTRUE;
    if (xPortIsInsideInterrupt())
    {
        xTimerStartFromISR( zbAlarm_timerHandle, &pxHigherPriorityTaskWoken);
    }
    else
    {
        xTimerStart(zbAlarm_timerHandle, 0 );
    }
#endif
    timer_started = 1;
}
void zigbee_timer_stop(void)
{
#if USE_HW_TIMER
    hosal_timer_stop(TIMER_ID);
#else
    BaseType_t pxHigherPriorityTaskWoken = pdTRUE;
    if (xPortIsInsideInterrupt())
    {
        if (xTimerIsTimerActive(zbAlarm_timerHandle) == pdTRUE) 
            xTimerStopFromISR( zbAlarm_timerHandle, &pxHigherPriorityTaskWoken);
    }
    else
    {
        if (xTimerIsTimerActive(zbAlarm_timerHandle) == pdTRUE) 
            xTimerStop(zbAlarm_timerHandle, 0 );
    }
#endif
    timer_started = 0;
}
zb_time_t osif_transceiver_time_get(void)
{
    return 0;
}
zb_time_t osif_sub_trans_timer(zb_time_t t2, zb_time_t t1)
{
    return ZB_TIME_SUBTRACT(t2, t1);
}
uint32_t zigbee_timer_status(void)
{
    return timer_started == 1;
}
void osif_sleep_using_transc_timer(zb_time_t timeout)
{

}