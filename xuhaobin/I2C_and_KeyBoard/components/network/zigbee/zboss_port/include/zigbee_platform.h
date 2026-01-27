/* ZBOSS Zigbee software protocol stack
 *
 * Copyright (c) 2012-2020 DSR Corporation, Denver CO, USA.
 * http://www.dsr-zboss.com
 * http://www.dsr-corporation.com
 * All rights reserved.
 *
 * This is unpublished proprietary source code of DSR Corporation
 * The copyright notice does not evidence any actual or intended
 * publication of such source code.
 *
 * ZBOSS is a registered trademark of Data Storage Research LLC d/b/a DSR
 * Corporation
 *

       * This software—whether modified or not—may be used only in 
 * conjunction with a Rafael Microelectronics, Inc. product. Under this 
 * license, it must not be reverse-engineered, decompiled, modified, or 
 * disassembled.

     */
/* PURPOSE: Platform specific
*/
#ifndef ZIGBEE_PLATFORMT_H
#define ZIGBEE_PLATFORMT_H

#include <string.h>             /* memcpy */
#include <stdio.h>
//#include "zb_types.h"
#include "mac_rafael.h"
#include "zboss_api_nwk.h"
#include "zboss_api_af.h"

#ifdef ZB_HAVE_SERIAL
#define ZB_HAVE_IOCTX
#endif

#define __weak                      __attribute__ ((weak))

#define ZB_VOLATILE
#define ZB_SDCC_XDATA
#define ZB_CALLBACK
#define ZB_SDCC_BANKED
#define ZB_KEIL_REENTRANT

/* At ARM all types from 1 to 4 bytes are passed to vararg with casting to 4 bytes */
typedef zb_uint32_t zb_minimal_vararg_t;

/* use macros to be able to redefine */
#define ZB_MEMCPY   memcpy
#define ZB_MEMMOVE  memmove
#define ZB_MEMSET   memset
#define ZB_MEMCMP   memcmp

#define ZB_BZERO(s,l) ZB_MEMSET((char*)(s), 0, (l))
#define ZB_BZERO2(s) ZB_BZERO(s, 2)
#define ZVUNUSED(v) (void)v

#ifndef ZB_TASK_SIZE
#define ZB_TASK_SIZE 4096
#endif

#ifndef ZB_TASK_PRORITY
#define ZB_TASK_PRORITY configMAX_PRIORITIES - 10
#endif

#define ZIGBEE_CHANNEL_MASK(n)    (1l << n)
#define ZIGBEE_CHANNEL_ALL_MASK() (0x7FFF800l)
/*** Frame Client/Server Directions ***/
#define ZCL_FRAME_CLIENT_SERVER_DIR 0x00
#define ZCL_FRAME_SERVER_CLIENT_DIR 0x01
#define ZIGBEE_ZCL_DATA_REQ(data_req_name, daddr, daddrmode, dep, sep, cid, c, \
                            sp, disdefrsp, dir, mcode, len)                    \
    do {                                                                       \
        data_req_name = pvPortMalloc(sizeof(zcl_data_req_t) + len);            \
        if (data_req_name == NULL)                                             \
            break;                                                             \
        data_req_name->dstAddr = daddr;                                        \
        data_req_name->addrmode = daddrmode;                                   \
        data_req_name->dstEndpint = dep;                                       \
        data_req_name->srcEndPoint = sep;                                      \
        data_req_name->clusterID = cid;                                        \
        data_req_name->cmd = c;                                                \
        data_req_name->specific = sp;                                          \
        data_req_name->disableDefaultRsp = disdefrsp;                          \
        data_req_name->direction = dir;                                        \
        data_req_name->manuCode = mcode;                                       \
        data_req_name->cmdFormatLen = len;                                     \
    } while (0);

typedef enum _zb_system_event {
    ZB_SYSTEM_EVENT_NONE                                = 0,

    ZB_SYSTEM_EVENT_ZB_TASKLET                          = 0x00000001,

    ZB_SYSTEM_EVENT_ALARM_MS_EXPIRED                    = 0x00000002,
    ZB_SYSTEM_EVENT_ALARM_US_EXPIRED                    = 0x00000004,
    ZB_SYSTEM_EVENT_ALARM_ALL_MASK                      = ZB_SYSTEM_EVENT_ALARM_MS_EXPIRED | ZB_SYSTEM_EVENT_ALARM_US_EXPIRED,

    ZB_SYSTEM_EVENT_UART_TXR                            = 0x00000010,
    ZB_SYSTEM_EVENT_UART_TXD                            = 0x00000020,
    ZB_SYSTEM_EVENT_UART_RXD                            = 0x00000040,
    ZB_SYSETM_EVENT_UART_ALL_MASK                       = ZB_SYSTEM_EVENT_UART_TXR | ZB_SYSTEM_EVENT_UART_TXD | ZB_SYSTEM_EVENT_UART_RXD,

    ZB_SYSTEM_EVENT_RADIO_TX_DONE_NO_ACK_REQ            = 0x00000100,
    ZB_SYSTEM_EVENT_RADIO_TX_ERROR                      = 0x00000200,
    ZB_SYSTEM_EVENT_RADIO_TX_ACKED                      = 0x00000400,
    ZB_SYSTEM_EVENT_RADIO_TX_ACKED_PD                   = 0x00000800,
    ZB_SYSTEM_EVENT_RADIO_TX_NO_ACK                     = 0x00001000,
    ZB_SYSTEM_EVENT_RADIO_TX_CCA_FAIL                   = 0x00002000,
    ZB_SYSTEM_EVENT_RADIO_TX_ALL_MASK                   = ZB_SYSTEM_EVENT_RADIO_TX_DONE_NO_ACK_REQ | 
        ZB_SYSTEM_EVENT_RADIO_TX_ERROR | ZB_SYSTEM_EVENT_RADIO_TX_ACKED | ZB_SYSTEM_EVENT_RADIO_TX_ACKED_PD | 
        ZB_SYSTEM_EVENT_RADIO_TX_NO_ACK | ZB_SYSTEM_EVENT_RADIO_TX_CCA_FAIL,

    ZB_SYSTEM_EVENT_RADIO_RX_NO_BUFF                    = 0x00002000,
    ZB_SYSTEM_EVENT_RADIO_RX_DONE                       = 0x00004000,
    ZB_SYSTEM_EVENT_RADIO_RX_CRC_FIALED                 = 0x00008000,
    ZB_SYSTEM_EVENT_RADIO_RX_ALL_MASK                   = ZB_SYSTEM_EVENT_RADIO_RX_NO_BUFF | 
        ZB_SYSTEM_EVENT_RADIO_RX_DONE | ZB_SYSTEM_EVENT_RADIO_RX_CRC_FIALED,
    ZB_SYSTEM_EVENT_RADIO_ALL_MASK                      = ZB_SYSTEM_EVENT_RADIO_TX_ALL_MASK | ZB_SYSTEM_EVENT_RADIO_RX_ALL_MASK,

    ZB_SYSTEM_EVENT_APP                                 = 0xff000000,

    ZB_SYSTEM_EVENT_ALL                                 = 0xffffffff,
} zb_system_event_t;

typedef void(pf_evt_indication)(uint32_t data_len);

typedef struct ZIGBEE_CONFIG_T
{
    zb_af_device_ctx_t *p_zigbee_device_contex_t;
    pf_evt_indication *pf_evt_indication;
} zigbee_cfg_t;

typedef struct {
    uint32_t dstAddr     : 16;
    uint32_t dstEndpint  : 8;
    uint32_t srcEndPoint : 8;

    uint32_t clusterID         : 16;
    uint32_t cmd               : 8;
    uint32_t specific          : 1;
    uint32_t disableDefaultRsp : 1;
    uint32_t direction         : 1;
    uint32_t addrmode          : 3;
    uint32_t                   : 2;

    uint32_t manuCode          : 16;
    uint32_t cmdFormatLen      : 16;

    uint32_t specific_seq_num  : 1;
    uint32_t seq_num           : 2;

    uint8_t cmdFormat[];

} zcl_data_req_t;
extern zb_system_event_t zb_system_event_var;
extern uint32_t zboss_start_run;

void zbSysEventSignalPending(void);

void zbLock(void);

void zbUnlock(void);

void zbStart(void);

void zboss_suspend(void);

void zboss_resume(void);
#ifdef ZB_USE_SLEEP
zb_uint32_t zb_osif_sleep(zb_uint32_t sleep_tmo);

void zb_osif_wake_up();
#endif
#define ZB_THREAD_SAFE(...)                 \
    zbLock();                              \
    do                                      \
    {                                       \
        __VA_ARGS__;                        \
    } while (0);                            \
    zbUnlock();                            

#define ZB_THREAD_SAFE_RET(ret, ...)        \
    do                                      \
    {                                       \
        zbLock();                          \
        (ret) = __VA_ARGS__;                \
        zbUnlock();                        \
    } while (0)                             


#define ZB_ENTER_CRITICAL()                 taskENTER_CRITICAL()
#define ZB_ENTER_CRITICAL_ISR()             taskENTER_CRITICAL_FROM_ISR()
#define ZB_EXIT_CRITICAL()                  taskEXIT_CRITICAL()
#define ZB_EXIT_CRITICAL_ISR(x)             taskEXIT_CRITICAL_FROM_ISR(x)

#define ZB_NOTIFY_ISR(ebit)                 (zb_system_event_var |= ebit); zbSysEventSignalPending()
#define ZB_NOTIFY(ebit)                     ZB_ENTER_CRITICAL(); zb_system_event_var |= ebit; ZB_EXIT_CRITICAL(); zbSysEventSignalPending()
#define ZB_GET_NOTIFY(ebit)                 ZB_ENTER_CRITICAL(); ebit = zb_system_event_var; zb_system_event_var = ZB_SYSTEM_EVENT_NONE; ZB_EXIT_CRITICAL()

#define ZB_APP_NOTIFY_ISR(ebit)             (zb_system_event_var |= ((zb_system_event_t)ebit & ZB_SYSTEM_EVENT_APP)); zbSysEventSignalPending()
#define ZB_APP_NOTIFY(ebit)                 ZB_ENTER_CRITICAL(); zb_system_event_var |=  ((zb_system_event_t)ebit & ZB_SYSTEM_EVENT_APP); ZB_EXIT_CRITICAL(); zbSysEventSignalPending()


#define ZB_PLATFORM_INIT()

#if defined ZB_TRACE_OVER_USART && defined ZB_TRACE_LEVEL
#define ZB_SERIAL_FOR_TRACE
#endif

#define ZB_CHECK_TIMER_IS_ON()          zigbee_timer_status()

#define ZB_START_HW_TIMER() 
#define ZB_STOP_HW_TIMER()    

#define ZB_TRANSPORT_NONBLOCK_ITERATION()
#define ZB_ENABLE_ALL_INTER()           ZB_EXIT_CRITICAL()  
#define ZB_DISABLE_ALL_INTER()          ZB_ENTER_CRITICAL()

#define ZB_OSIF_GLOBAL_LOCK()           ZB_ENTER_CRITICAL()
#define ZB_OSIF_GLOBAL_UNLOCK()         ZB_EXIT_CRITICAL()  

#define ZB_RADIO_INT_DISABLE()          ZB_ENTER_CRITICAL()
#define ZB_RADIO_INT_ENABLE()           ZB_EXIT_CRITICAL()  

#define ZB_ABORT 
//#define ZB_GO_IDLE() CPUwfi()
/* Have problems with sleep when working without a debugger. Switch it iff temporary */
#define ZB_GO_IDLE()

void zigbee_app_zcl_send_command(zcl_data_req_t* pt_data_req);
void zb_radioInit(void);
void zigbee_timer_init(void);
void zigbee_timer_start(void);
void zigbee_timer_stop(void);
uint32_t zigbee_timer_status(void);
void zb_radioTask(zb_system_event_t trxEvent);
void zbStartRun(void);
void zigbee_do_factory_reset(void);
void register_app_cb(void* cb);
void register_zcl_cb(void* cb);
#endif