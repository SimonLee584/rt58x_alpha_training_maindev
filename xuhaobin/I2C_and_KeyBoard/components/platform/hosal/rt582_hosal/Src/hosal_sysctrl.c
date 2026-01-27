/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * \file            hosal_sysctrl.c
 * \brief           Hosal system control driver file
 */
/*
 * Author:          Kc.tseng
 */
#include <stdio.h>
#include <stdint.h>
#include "mcu.h"
#include "sysctrl.h"
#include "dwt.h"
#include "hosal_sysctrl.h"
#include "hosal_status.h"

void hosal_delay_us(volatile uint32_t us){ 
    delay_us (us);
}

void hosal_delay_ms(volatile uint32_t ms) {
     delay_ms(ms);
}

uint32_t hosal_pin_get_mode(uint32_t pin_number) {
    return pin_get_mode(pin_number);
}

void hosal_pin_set_mode(uint32_t pin_number, uint32_t mode) {
    pin_set_mode(pin_number, mode);
}

void hosal_pin_set_pullopt(uint32_t pin_number, uint32_t mode) {
    pin_set_pullopt(pin_number, mode);
}

void hosal_enable_pin_opendrain(uint32_t pin_number) {
    enable_pin_opendrain(pin_number);
}

void hosal_disable_pin_opendrain(uint32_t pin_number) {
    disable_pin_opendrain(pin_number);
}

void hosal_config_peripherl_clock(uint32_t per_clk, void* cfg_para) {

     if(cfg_para) {
        
        enable_perclk(per_clk);
     }
     else  {

        disable_perclk(per_clk);
     }
}


int hosal_get_rco_clock_tick(uint32_t* rco_tick)
{
    if(rco_tick == NULL){
        
        return HOSAL_STATUS_INVALID_PARAM;
    }

    *rco_tick = 40;

    return HOSAL_STATUS_SUCCESS;
}

int hosal_sysctrl_ioctrl(hosal_sys_dwt_t *sys_dwt,int ctl, void* para) {

     int ret= HOSAL_STATUS_SUCCESS;
    

     switch(ctl){
        case HOSAL_SYSCTRL_DELAY_INIT:{
            ret =  dwt_init();
            break;
        }
        case HOSAL_SYSCTRL_DELAY_UNINIT:{
            ret =  dwt_uninit();
            break;
        }
        case HOSAL_SYSCTRL_TIMEOUT_START: {
             uint32_t timeout = *(uint32_t*)para;
             ret =  dwt_timeoutstart(&sys_dwt->tmo, timeout, sys_dwt->tmo.unit, sys_dwt->tmo.callback);
        }
            break; 

        case HOSAL_SYSCTRL_TIMEOUT_CHECK:{
            ret =  dwt_timeoutcheck(&sys_dwt->tmo);
            break; 
        }

        case HOSAL_SYSCTRL_GET_TIMEOUT_REMAINING:{
            ret =  dwt_timeoutremaining(&sys_dwt->tmo,(uint32_t*)para);
            break;    
        }
     }

     return ret;

}

