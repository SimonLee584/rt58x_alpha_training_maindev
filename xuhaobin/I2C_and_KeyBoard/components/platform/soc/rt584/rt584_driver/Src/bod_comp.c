/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * \file            bod_comp.c
 * \brief           bod comparator driver file
 */
/*
 * Author:          Kc.tseng
 */

#include "bod_comp.h"
#include "lpm.h"


static bod_comp_proc_cb  bod_notify_cb;

uint32_t bod_comp_register_callback(bod_comp_proc_cb bod_comp_callback) {
    if( bod_comp_callback != NULL) {
        bod_notify_cb = bod_comp_callback;
    }
    return STATUS_SUCCESS;
}

uint32_t bod_comp_ana_init(void) {
    BOD_COMP->comp_ana_ctrl.bit.bod_ib = 0;
    /* bod_hys defined Interval range */
    BOD_COMP->comp_ana_ctrl.bit.bod_hys = 3;

    /* bod_div_sel defined threshold */
    BOD_COMP->comp_ana_ctrl.bit.bod_div_sel = 15;

    return STATUS_SUCCESS;
}

uint32_t bod_comp_open(bod_comp_config_t bod_cfg, bod_comp_proc_cb bod_comp_callback) {
    uint32_t rval;

    rval = bod_comp_ana_init();
    
    BOD_COMP->comp_dig_ctrl0.bit.debounce_en = bod_cfg.debounce_en;
    BOD_COMP->comp_dig_ctrl0.bit.debounce_sel = bod_cfg.debounce_sel;

    BOD_COMP->comp_dig_ctrl0.bit.counter_mode_edge = bod_cfg.counter_mode_edge;
    BOD_COMP->comp_dig_ctrl1.bit.en_intr_counter = bod_cfg.counter_mode_int_en;
    BOD_COMP->comp_dig_ctrl0.bit.counter_trigger_th = bod_cfg.counter_mode_threshold;
    BOD_COMP->comp_dig_ctrl0.bit.counter_mode_en = bod_cfg.counter_mode_en;

    BOD_COMP->comp_dig_ctrl1.bit.comp_settle_time = 14;

    BOD_COMP->comp_dig_ctrl1.bit.clr_intr_falling = 1;
    BOD_COMP->comp_dig_ctrl1.bit.clr_intr_rising = 1;
    BOD_COMP->comp_dig_ctrl1.bit.clr_intr_counter = 1;
    BOD_COMP->comp_dig_ctrl1.bit.clr_counter = 1;

    BOD_COMP->comp_dig_ctrl1.bit.en_intr_rising = bod_cfg.rising_edge_int_en;
    BOD_COMP->comp_dig_ctrl1.bit.en_intr_falling = bod_cfg.falling_edge_int_en;

    SYSCTRL->sys_clk_ctrl2.bit.en_ck32_bodcomp = 1;

    rval = bod_comp_register_callback(bod_comp_callback);

    return rval;
}

uint32_t bod_comp_normal_start(void) {
    BOD_COMP->comp_dig_ctrl0.bit.comp_en_nm = 1;
    return STATUS_SUCCESS;
}

uint32_t bod_comp_normal_stop(void) {
    BOD_COMP->comp_dig_ctrl0.bit.comp_en_nm = 0;
    return STATUS_SUCCESS;
}

uint32_t bod_comp_sleep_start(void) {
    BOD_COMP->comp_dig_ctrl0.bit.comp_en_sp = 1;
    return STATUS_SUCCESS;
}

uint32_t bod_comp_sleep_stop(void) {
    BOD_COMP->comp_dig_ctrl0.bit.comp_en_sp = 0;
    return STATUS_SUCCESS;
}

uint32_t bod_comp_deep_sleep_start(void) {
    BOD_COMP->comp_dig_ctrl0.bit.comp_en_ds = 1;
    return STATUS_SUCCESS;
}

uint32_t bod_comp_deep_sleep_stop(void) {
    BOD_COMP->comp_dig_ctrl0.bit.comp_en_ds = 0;
    return STATUS_SUCCESS;
}

uint32_t bod_comp_setup_deep_sleep_enable_clock(void) {
    SYSCTRL->sram_lowpower_3.bit.cfg_ds_rco32k_off = 0;
    SYSCTRL->sys_clk_ctrl2.bit.en_ck32_bodcomp = 1;

    return STATUS_SUCCESS;
}

uint32_t bod_comp_setup_deep_sleep_disable_clock(uint8_t wakeup_level) {
    BOD_COMP->comp_dig_ctrl0.bit.comp_en_nm = 1;

    BOD_COMP->comp_dig_ctrl0.bit.ds_wakeup_pol = wakeup_level;
    BOD_COMP->comp_dig_ctrl0.bit.ds_wakeup_en = 1;

    lpm_enable_bod_level();

    return STATUS_SUCCESS;
}

uint32_t get_bod_comp_counter_count(uint32_t *count) {
    *count = BOD_COMP->comp_dig_ctrl2.bit.counter_cnt;
    return STATUS_SUCCESS;
}

uint32_t clear_bod_comp_counter_count(void) {
    BOD_COMP->comp_dig_ctrl1.bit.clr_counter = 1;
    return STATUS_SUCCESS;
}

void bod_comp_handler(void) {
    uint32_t int_status = 0;
    int_status = BOD_COMP->comp_dig_ctrl2.reg & 0x07;

    BOD_COMP->comp_dig_ctrl1.bit.clr_intr_counter =
        BOD_COMP->comp_dig_ctrl2.bit.sta_intr_counter;
    BOD_COMP->comp_dig_ctrl1.bit.clr_intr_falling =
        BOD_COMP->comp_dig_ctrl2.bit.sta_intr_falling;
    BOD_COMP->comp_dig_ctrl1.bit.clr_intr_rising =
        BOD_COMP->comp_dig_ctrl2.bit.sta_intr_rising;

    if (bod_notify_cb != NULL) {
        bod_notify_cb(int_status);
    }
}
