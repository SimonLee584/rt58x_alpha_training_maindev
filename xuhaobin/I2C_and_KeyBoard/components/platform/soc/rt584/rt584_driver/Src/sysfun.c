/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * \file           sysfunc.c
 * \brief          system function driver 
 */
/*
 * This file is part of library_name.
 * Author: ives.lee
 */



#include "mcu.h"
#include "assert_help.h"
#include "system_mcu.h"
#include "sysfun.h"
#include "sysctrl.h"
#include "wdt.h"
#include "flashctl.h"
#include "pmu_reg.h"
#if defined(CONFIG_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif

/**
 * \brief           Nest of critical section.
 */
static int critical_counter = 0;
static txpower_default_cfg_t   tx_pwr_level;
void enter_critical_section(void) {

#if defined(CONFIG_FREERTOS)
    if (portNVIC_INT_CTRL_REG & 0xFF) {
        return;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        __disable_irq();
    } else {
        vPortEnterCritical();
    }
#else
    __disable_irq();
#endif
    critical_counter++;
}
/**
 * \brief           Nest of critical section.
 */
void leave_critical_section(void) {

#if defined(CONFIG_FREERTOS)
    if (portNVIC_INT_CTRL_REG & 0xFF) {
        return;
    }
    critical_counter--;
    assert_param(critical_counter >= 0);

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vPortExitCritical();
    }

    if (critical_counter == 0) {

        if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
            __enable_irq();
        }
    }
#else
    critical_counter--;
    assert_param(critical_counter >= 0);
    if (critical_counter == 0) {
        __enable_irq();
    }
#endif
}

/**
 * \brief           Nest of critical section.
 */
uint32_t version_check(void) {

    uint32_t version_info;

    version_info = SYSCTRL->soc_chip_info.reg;

    return version_info;
}

void sys_software_reset(void) {
    wdt_ctrl_t controller;


    controller.reg = 0;
    if (WDT->control.bit.lockout) {
        while (1);
    }

    WDT->clear = 1;/*clear interrupt*/

    controller.bit.int_en = 0;
    controller.bit.reset_en = 1;
    controller.bit.wdt_en = 1;

    WDT->win_max = 0x2;
    WDT->control.reg = controller.reg;
    while (1) {}
}

/**
 * \brief           Get otp version
 */
chip_model_t getotpversion() {
    uint32_t   otp_rd_buf_addr[64];
    uint32_t     i;
    uint8_t    buf_Tmp[4];

    otp_version_t otp_version;
    chip_model_t  chip_model;

    chip_model.type = CHIP_TYPE_UNKNOW;
    chip_model.version = CHIP_VERSION_UNKNOW;

    if (flash_read_otp_sec_page((uint32_t)otp_rd_buf_addr) != STATUS_SUCCESS) {
        return  chip_model;
    }

    for (i = 0; i < 8; i++) {
        *(uint32_t *)buf_Tmp = otp_rd_buf_addr[(i / 4)];
        otp_version.buf[i] = buf_Tmp[(i % 4)];
    }

    //otp version flag  
    if (otp_version.buf[0] == 0xFF) {
        return chip_model;
    }

    /*ASCII Value*/
    otp_version.buf[5] -= 0x30; /* ascii 0~9 0x30~0x39 */
    otp_version.buf[6] -= 0x40; /* ascii A~Z 0x41~0x5A */

    /*reference chip_define.h
     CHIP_ID(TYPE,VER)                   ((TYPE << 8) | VER)
     CHIP_MODEL                           CHIP_ID(CHIP_TYPE,CHIP_VERSION)
    */
    chip_model.type = (chip_type_t)otp_version.buf[5];

    chip_model.version = (chip_version_t)otp_version.buf[6];
    return chip_model;
}

/**
 * \brief           Set the System PMU Mode
 */
void sys_pmu_setmode(pmu_mode_cfg_t pmu_mode) {
    if (pmu_mode == PMU_MODE_DCDC) {
        PMU_CTRL->pmu_en_control.bit.en_ldomv_nm    = 1;
        PMU_CTRL->pmu_en_control.bit.en_dcdc_nm     = 1;
        PMU_CTRL->pmu_en_control.bit.en_ldomv_nm    = 0;
    } else if (pmu_mode == PMU_MODE_LDO) {
        PMU_CTRL->pmu_en_control.bit.en_ldomv_nm    = 1;
        PMU_CTRL->pmu_en_control.bit.en_dcdc_nm     = 1;
        PMU_CTRL->pmu_en_control.bit.en_dcdc_nm     = 0;
    }
}

pmu_mode_cfg_t sys_pmu_getmode(void) {
    pmu_mode_cfg_t Mode;

    Mode = PMU_MODE_DCDC;

    if ( (PMU_CTRL->pmu_en_control.bit.en_ldomv_nm == 0) 
      && (PMU_CTRL->pmu_en_control.bit.en_dcdc_nm == 1)) {
        Mode = PMU_MODE_DCDC;
    } else if ( (PMU_CTRL->pmu_en_control.bit.en_ldomv_nm == 1) 
             && (PMU_CTRL->pmu_en_control.bit.en_dcdc_nm == 0)) {
        Mode = PMU_MODE_LDO;
    }

    return Mode;
}

/**
 * \brief  Get the System slow clock mode
 */
slow_clock_mode_cfg_t sys_slow_clk_mode(void)
{
    slow_clock_mode_cfg_t mode;

    if ((RCO32K_CAL->cal32k_cfg0.bit.cfg_cal32k_target == 204800) ||
            (RCO32K_CAL->cal32k_cfg0.bit.cfg_cal32k_target == 102400))   //per_clk = 32MHz, target ~= 20KHz is 204800'd
    {
        //per_clk = 16MHz, target ~= 20KHz is 102400'd
        mode = RCO20K_MODE;

    }
    else if ((RCO32K_CAL->cal32k_cfg0.bit.cfg_cal32k_target == 128000) ||
             (RCO32K_CAL->cal32k_cfg0.bit.cfg_cal32k_target == 64000))   //per_clk = 32MHz, target ~= 20KHz is 128000'd
    {
        //per_clk = 16MHz, target ~= 20KHz is 64000'd
        if (SYSCTRL->sys_clk_ctrl2.bit.en_rco32k_div2 == 1)
        {
            mode = RCO16K_MODE;
        }
        else
        {
            mode = RCO32K_MODE;
        }
    }
    else
    {
        mode = RCO_MODE;        //unkonw slow clock mode
    }

    return mode;
}

/**
 * \brief  Get the System TX Power Default
 */
txpower_default_cfg_t sys_txpower_getdefault(void)
{
#if defined(CONFIG_RF_POWER_14DBM) || defined(CONFIG_RF_POWER_0DBM) || defined(CONFIG_RF_POWER_20DBM) || defined(CONFIG_RF_POWER_10DBM)

    txpower_default_cfg_t    PowerDefault;

#if defined(CONFIG_RF_POWER_20DBM)
    PowerDefault = TX_POWER_20DBM_DEF;
#elif defined(CONFIG_RF_POWER_14DBM)
    PowerDefault = TX_POWER_14DBM_DEF;
#elif defined(CONFIG_RF_POWER_10DBM)
    PowerDefault = TX_POWER_10DBM_DEF;    
#else
    PowerDefault = TX_POWER_0DBM_DEF;
#endif

    return PowerDefault;

#else

    uint32_t read_addr=0, i=0;
    uint16_t soc_chip_id = SYSCTRL->soc_chip_info.bit.chip_id;
    uint8_t  txpwrlevel;

    if (flash_size() == FLASH_1024K)
    {
        #if defined(CONFIG_FLASHCTRL_SECURE_EN)
                read_addr = 0x100FFFD8;
        #else
                read_addr = 0x000FFFD8;
        #endif
    }
    else if (flash_size() == FLASH_2048K)
    {
        #if defined(CONFIG_FLASHCTRL_SECURE_EN)
                read_addr = 0x101FFFD8;
        #else
                read_addr = 0x001FFFD8;
        #endif
    }

    i = (read_addr + 7); //FD8~FDF, 8bytes

    for (read_addr = read_addr; read_addr < i; read_addr++)
    {

        txpwrlevel = (*(uint8_t *)(read_addr));

        if ((txpwrlevel == TX_POWER_20DBM_DEF) || (txpwrlevel == TX_POWER_14DBM_DEF) || (txpwrlevel == TX_POWER_0DBM_DEF)  || (txpwrlevel == TX_POWER_10DBM_DEF))
        {

            break;
        }
        else
        {
            if(soc_chip_id==0x0584)
            {
                txpwrlevel = TX_POWER_14DBM_DEF;
            }
            else if(soc_chip_id==0x1584)
            {
                txpwrlevel = TX_POWER_10DBM_DEF;   
            }
            else if(soc_chip_id==0x3584)
            {
                txpwrlevel = TX_POWER_20DBM_DEF;
            }
           else
            {
                txpwrlevel = TX_POWER_14DBM_DEF;
            }
                 
        }
    }

    return txpwrlevel;

#endif

}

/**
 * \brief  Get the System TX Power Default
 */
void set_sys_txpower_default(txpower_default_cfg_t txpwrdefault)
{
    tx_pwr_level = txpwrdefault;
}
