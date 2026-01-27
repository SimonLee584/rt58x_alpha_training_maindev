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
#include "stdio.h"
#include <stdint.h>
#include "mcu.h"
#include "hosal_sysctrl.h"
#include "hosal_status.h"

void hosal_delay_us(volatile uint32_t us) { 
    delay_us (us);
}

void hosal_delay_ms(volatile uint32_t ms) {
    delay_ms(ms);
}


uint32_t hosal_pin_get_mode(uint32_t pin_number) {
    return pin_get_mode(pin_number);
}

uint32_t hosal_pin_set_mode(uint32_t pin_number, uint32_t mode) {
    pin_set_mode(pin_number, mode);
    return HOSAL_STATUS_SUCCESS;
}

uint32_t hosal_get_periperhal_clock() {
    return get_peri_clk();
}


int hosal_get_rco_clock_tick(uint32_t* rco_tick)
{

        if(sys_slow_clk_mode()==RCO20K_MODE)
        {
            *rco_tick = 20;
        }
        else if(sys_slow_clk_mode()==RCO32K_MODE)
        {
            *rco_tick = 32;
        }
        else if(sys_slow_clk_mode()==RCO16K_MODE)
        {
            *rco_tick = 16;
        }
        else
        {
            *rco_tick = 32;
        }

    
    return HOSAL_STATUS_SUCCESS;
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

void hosal_pin_enable_schmitt(uint32_t pin_number) {
    pin_enable_schmitt(pin_number);
}

void hosal_pin_disable_schmitt(uint32_t pin_number) {
    pin_disable_schmitt(pin_number);
}

void hosal_pin_enable_filter(uint32_t pin_number) {
    pin_enable_filter(pin_number);
}

void hosal_pin_disable_filter(uint32_t pin_number) {
    pin_disable_filter(pin_number);
}

void hosal_config_peripherl_clock(uint32_t per_clk, void* cfg_para) {

     if(cfg_para) {
        
        enable_perclk(per_clk);
     }
     else  {

        disable_perclk(per_clk);
     }
}



void hosal_tx_power_level_ctrl(int ctl, void* cfg_para) {

     switch(ctl){
        case HOSAL_SET_TX_POWER_LEVEL:
            set_sys_txpower_default(*(txpower_default_cfg_t*)cfg_para);
            break;
        case HOSAL_GET_TX_POWER_LEVEL:
            (*(uint32_t*)cfg_para) = sys_txpower_getdefault();
            break;
        case HOSAL_SLOW_CLOCK_CALIBRATION:
            slow_clock_calibration(*(slow_clock_select_t*)cfg_para);
            break;
     }   
}


int hosal_sysctrl_ioctrl(hosal_sys_dwt_t *sys_dwt,int ctl, void* para) {

     int ret= HOSAL_STATUS_SUCCESS;
    

     switch(ctl){
        case HOSAL_SYSCTRL_DELAY_INIT:
            ret =  dwt_init();
            break;
        case HOSAL_SYSCTRL_DELAY_UNINIT:
            ret =  dwt_uninit();
            break;
        case HOSAL_SYSCTRL_TIMEOUT_START:
             uint32_t timeout = *(uint32_t*)para;  // Åª¨ú­È
             ret =  dwt_timeoutstart(&sys_dwt->tmo, timeout, sys_dwt->tmo.unit, sys_dwt->tmo.callback);
            break; 

        case HOSAL_SYSCTRL_TIMEOUT_CHECK:
            ret =  dwt_timeoutcheck(&sys_dwt->tmo);
            break; 

        case HOSAL_SYSCTRL_GET_TIMEOUT_REMAINING:
            ret =  dwt_timeoutremaining(&sys_dwt->tmo,(uint32_t*)para);
            break;    

     }

     return ret;

}