/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * \file            aux_com.c
 * \brief           aux comparator driver 
 */
/*
 * Author:          Kc.tseng
 */

#include "aux_comp.h"
#include "lpm.h"


static aux_comp_proc_cb  aux_notify_cb;

uint32_t aux_comp_register_callback(aux_comp_proc_cb aux_comp_callback) {
    if( aux_comp_callback != NULL) {
        aux_notify_cb = aux_comp_callback;
    }
    return STATUS_SUCCESS;
}

uint32_t aux_comp_ana_init(void) {
    AUX_COMP->comp_ana_ctrl.bit.comp_selref = 0;
    AUX_COMP->comp_ana_ctrl.bit.comp_selinput = 0;
    AUX_COMP->comp_ana_ctrl.bit.comp_pw = 3;
    /* comp_selhys defined Interval range */
    AUX_COMP->comp_ana_ctrl.bit.comp_selhys = 0;
    AUX_COMP->comp_ana_ctrl.bit.comp_swdiv = 1;
    AUX_COMP->comp_ana_ctrl.bit.comp_psrr = 0;
    /* vsel defined threshold */
    AUX_COMP->comp_ana_ctrl.bit.comp_vsel = 15;
    AUX_COMP->comp_ana_ctrl.bit.comp_refsel = 0;
    AUX_COMP->comp_ana_ctrl.bit.comp_chsel = 0;
    AUX_COMP->comp_ana_ctrl.bit.comp_tc = 0;

    return STATUS_SUCCESS;
}

uint32_t aux_comp_open(aux_comp_config_t aux_cfg, aux_comp_proc_cb aux_comp_callback) {
    uint32_t rval;

    rval = aux_comp_ana_init();
    
    AUX_COMP->comp_dig_ctrl0.bit.debounce_en = aux_cfg.debounce_en;
    AUX_COMP->comp_dig_ctrl0.bit.debounce_sel = aux_cfg.debounce_sel;

    AUX_COMP->comp_dig_ctrl0.bit.counter_mode_edge = aux_cfg.counter_mode_edge;
    AUX_COMP->comp_dig_ctrl1.bit.en_intr_counter = aux_cfg.counter_mode_int_en;
    AUX_COMP->comp_dig_ctrl0.bit.counter_trigger_th = aux_cfg.counter_mode_threshold;
    AUX_COMP->comp_dig_ctrl0.bit.counter_mode_en = aux_cfg.counter_mode_en;

    AUX_COMP->comp_dig_ctrl1.bit.comp_settle_time = 14;

    AUX_COMP->comp_dig_ctrl1.bit.clr_intr_falling = 1;
    AUX_COMP->comp_dig_ctrl1.bit.clr_intr_rising = 1;
    AUX_COMP->comp_dig_ctrl1.bit.clr_intr_counter = 1;
    AUX_COMP->comp_dig_ctrl1.bit.clr_counter = 1;

    AUX_COMP->comp_dig_ctrl1.bit.en_intr_rising = aux_cfg.rising_edge_int_en;
    AUX_COMP->comp_dig_ctrl1.bit.en_intr_falling = aux_cfg.falling_edge_int_en;

    rval = aux_comp_register_callback(aux_comp_callback);

    SYSCTRL->sys_clk_ctrl2.bit.en_ck32_auxcomp = 1;
    return rval;
}

uint32_t aux_comp_normal_start(void) {
    AUX_COMP->comp_dig_ctrl0.bit.comp_en_nm = 1;
    return STATUS_SUCCESS;
}

uint32_t aux_comp_normal_stop(void) {
    AUX_COMP->comp_dig_ctrl0.bit.comp_en_nm = 0;
    return STATUS_SUCCESS;
}

uint32_t aux_comp_sleep_start(void) {
    AUX_COMP->comp_dig_ctrl0.bit.comp_en_sp = 1;
    return STATUS_SUCCESS;
}

uint32_t aux_comp_sleep_stop(void) {
    AUX_COMP->comp_dig_ctrl0.bit.comp_en_sp = 0;
    return STATUS_SUCCESS;
}

uint32_t aux_comp_deep_sleep_start(void) {
    AUX_COMP->comp_dig_ctrl0.bit.comp_en_ds = 1;
    return STATUS_SUCCESS;
}

uint32_t aux_comp_deep_sleep_stop(void) {
    AUX_COMP->comp_dig_ctrl0.bit.comp_en_ds = 0;
    return STATUS_SUCCESS;
}

uint32_t aux_comp_setup_deep_sleep_enable_clock(void) {
    SYSCTRL->sram_lowpower_3.bit.cfg_ds_rco32k_off = 0;
    SYSCTRL->sys_clk_ctrl2.bit.en_ck32_auxcomp = 1;

    return STATUS_SUCCESS;
}

uint32_t aux_comp_setup_deep_sleep_disable_clock(uint8_t wakeup_level) {
    AUX_COMP->comp_dig_ctrl0.bit.comp_en_nm = 1;
    
    AUX_COMP->comp_dig_ctrl0.bit.ds_wakeup_pol = wakeup_level;
    AUX_COMP->comp_dig_ctrl0.bit.ds_wakeup_en = 1;

    lpm_enable_aux_level();

    return STATUS_SUCCESS;
}

uint32_t get_aux_comp_counter_count(uint32_t *count) {
    *count = AUX_COMP->comp_dig_ctrl2.bit.counter_cnt;
    return STATUS_SUCCESS;
}

uint32_t clear_aux_comp_counter_count(void) {
    AUX_COMP->comp_dig_ctrl1.bit.clr_counter = 1;
    return STATUS_SUCCESS;
}


void aux_comp_handler(void) {
    uint32_t int_status = 0;
    int_status = AUX_COMP->comp_dig_ctrl2.reg & 0x07;

    AUX_COMP->comp_dig_ctrl1.bit.clr_intr_counter =
        AUX_COMP->comp_dig_ctrl2.bit.sta_intr_counter;
    AUX_COMP->comp_dig_ctrl1.bit.clr_intr_falling =
        AUX_COMP->comp_dig_ctrl2.bit.sta_intr_falling;
    AUX_COMP->comp_dig_ctrl1.bit.clr_intr_rising =
        AUX_COMP->comp_dig_ctrl2.bit.sta_intr_rising;

    if (aux_notify_cb != NULL) {
        aux_notify_cb(int_status);
    }
}
