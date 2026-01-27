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
#ifdef ZB_USE_SLEEP
//=============================================================================
//                Include
//=============================================================================
#include "FreeRTOS.h"
#include "timers.h"

#include "mcu.h"
#include "log.h"

#include "zb_common.h"
#include "zigbee_platform.h"
//=============================================================================
//                Private Definitions of const value
//=============================================================================

//=============================================================================
//                Private ENUM
//=============================================================================

//=============================================================================
//                Private Struct
//=============================================================================

//=============================================================================
//                Private Global Variables
//=============================================================================
TimerHandle_t zigbee_sleep_timer;
//=============================================================================
//                Functions
//=============================================================================

void zb_timer_set(void)
{
    if (ZB_TIMER_CTX().canstop)
    {
        ZB_TIMER_CTX().started = ZB_FALSE;
        zigbee_timer_stop();
    }
}

uint8_t sleep_set_1st = 0;
void radio_sleep_set(void)
{
    if (sleep_set_1st == 0)
    {
        ZB_TRANSCEIVER_SET_RX_ON_OFF(0);
        sleep_set_1st = 1;
    }
}

void radio_idle_set(void)
{
    //ZB_TRANSCEIVER_SET_RX_ON_OFF(1);
}

void ZB_sleep_callback(TimerHandle_t timer_ptr)
{
    /* Optionally do something if the timer_ptr parameter is NULL. */
    configASSERT(timer_ptr);

    zboss_resume();

}

void zb_sleep_timer_set(zb_uint32_t sleep_tmo)
{

    if (zigbee_sleep_timer == NULL)
    {
        zigbee_sleep_timer = xTimerCreate
                             ( /* Just a text name, not used by the RTOS kernel. */
                                 "zb_sleep_timer",
                                 /* The timer period in ticks, must be greater than 0. */
                                 pdMS_TO_TICKS(sleep_tmo),
                                 /* The timers will auto-reload themselves when they expire. */
                                 pdFALSE,
                                 /* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
                                 (void *) 0,
                                 /* Each timer calls the same callback when it expires. */
                                 ZB_sleep_callback
                             );

    }

    if (zigbee_sleep_timer != NULL)
    {
        /* Start the timer.  No block time is specified, and
        even if one was it would be ignored because the RTOS
        scheduler has not yet been started. */
        xTimerChangePeriod(zigbee_sleep_timer, pdMS_TO_TICKS(sleep_tmo), 0);
        if (xTimerStart(zigbee_sleep_timer, 0) != pdPASS)
        {
            /* The timer could not be set into the Active state. */
        }
    }

}

zb_uint32_t zb_osif_sleep(zb_uint32_t sleep_tmo)
{
    TickType_t current_time, resume_time, slept_time;

    zb_timer_set();
    current_time = xTaskGetTickCount();

    //log_info("JJ:S(%d)\n", sleep_tmo);

    radio_sleep_set();
    //console_sleep();
    //gpio_pin_clear(LED1);

    zb_sleep_timer_set(sleep_tmo);

    zboss_suspend();

    //system wakeup
    xTimerStop(zigbee_sleep_timer, 0);
    resume_time = xTaskGetTickCount();
    if (resume_time > current_time)
    {
        slept_time = resume_time - current_time;
    }
    else
    {
        slept_time = (0xffffffff - current_time) + resume_time;
    }

    //TRACE_MSG(TRACE_APP1, "JJ:W(%d %d)", (FMT__0, slept_time, sleep_tmo));

    radio_idle_set();
    //gpio_pin_set(LED1);
    //ZB_START_HW_TIMER();
    zigbee_timer_start();

    return slept_time;
}

void zb_osif_wake_up()
{
    //aes_fw_init();  /* GP add, disable which rtos wake up should call AES FW init */
}
#endif