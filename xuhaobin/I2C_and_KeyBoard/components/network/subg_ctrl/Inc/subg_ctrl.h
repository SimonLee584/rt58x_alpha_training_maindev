/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * @file subg_ctrl.h
 * @author Rex Huang (rex.huang@rafaelmicro.com)
 * @brief SubG defination & API header file.
 * @version 0.1
 * @date 2023-08-10
 */

#ifndef __SUBG_CTRL_H__
#define __SUBG_CTRL_H__

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

 /**
 * \defgroup subg_ctrl SubG API Definition
 * \ingroup SUBG_group
 * \brief Define SubG API definitions, structures, and functions.
 * @{
 */
/**************************************************************************************************
 *    CONSTANTS AND DEFINES
 *************************************************************************************************/
#define FSK_MAX_RF_LEN 2063 //2047+16   /*!< Maximum RF length of FSK packets.*/
#define OQPSK_MAX_RF_LEN 142 //127+15   /*!< Maximum RF length of OQPSK packets.*/
#define MAX_RF_LEN FSK_MAX_RF_LEN       /*!< Maximum RF length of SubG packets.*/

/**************************************************************************************************
 *    TYPEDEFS
 *************************************************************************************************/
/**
 * \brief           SubG modulation enum definitions.
 */
typedef enum {
    SUBG_CTRL_MODU_FSK = 0,   /*!< SubG FSK modulation */
    SUBG_CTRL_MODU_OPQSK,     /*!< SubG OQPSK modulation */
} subg_ctrl_modulation_t;

/**
 * \brief           SubG data rate enum definitions.
 */
typedef enum {
    SUBG_CTRL_DATA_RATE_2M = 0,   /*!< SubG 2Mbps data rate */
    SUBG_CTRL_DATA_RATE_1M,       /*!< SubG 1Mbps data rate */
    SUBG_CTRL_DATA_RATE_500K,     /*!< SubG 500kbps data rate */
    SUBG_CTRL_DATA_RATE_200K,     /*!< SubG 200kbps data rate */
    SUBG_CTRL_DATA_RATE_100K,     /*!< SubG 100kbps data rate */
    SUBG_CTRL_DATA_RATE_50K,      /*!< SubG 50kbps data rate */
    SUBG_CTRL_DATA_RATE_300K,     /*!< SubG 300kbps data rate */
    SUBG_CTRL_DATA_RATE_150K,     /*!< SubG 150kbps data rate */
    SUBG_CTRL_DATA_RATE_75K,      /*!< SubG 75kbps data rate */
} subg_ctrl_data_rate_t;

/**
 * \brief           SubG OQPSK data rate enum definitions.
 */
typedef enum {
    SUBG_CTRL_DATA_RATE_25K = 3,     /*!< SubG OQPSK 25kbps data rate */
    SUBG_CTRL_DATA_RATE_12P5K = 4,   /*!< SubG OQPSK 12.5kbps data rate */
    SUBG_CTRL_DATA_RATE_6P25K = 5,   /*!< SubG OQPSK 6.25kbps data rate */
} subg_ctrl_oqpsk_data_rate_t;

/**
 * \brief           SubG CRC type enum definitions.
 */
typedef enum {
    SUBG_CTRL_CRC_TYPE_16 = 0,   /*!< SubG 16-bit CRC */
    SUBG_CTRL_CRC_TYPE_32,       /*!< SubG 32-bit CRC */
} subg_ctrl_crc_type_t;

/**
 * \brief           SubG whitening enum definitions.
 */
typedef enum {
    SUBG_CTRL_WHITEN_DISABLE = 0,   /*!< SubG whitening disabled */
    SUBG_CTRL_WHITEN_ENABLE         /*!< SubG whitening enabled */
} subg_ctrl_whiten_t;

/**
 * \brief           SubG FSK modulation index enum definitions.
 */
typedef enum {
    SUBG_CTRL_FSK_MOD_0P5 = 0,   /*!< SubG FSK modulation index 0.5 */
    SUBG_CTRL_FSK_MOD_1,         /*!< SubG FSK modulation index 1 */
    SUBG_CTRL_FSK_MOD_UNDEF      /*!< SubG FSK modulation index unused, for OQPSK */
} subg_ctrl_fsk_mod_t;

/**
 * \brief           SubG filter type enum definitions.
 */
typedef enum {
    SUBG_CTRL_FILTER_TYPE_FSK = 0,   /*!< SubG FSK filter type */
    SUBG_CTRL_FILTER_TYPE_GFSK,      /*!< SubG GFSK filter type */
    SUBG_CTRL_FILTER_TYPE_OQPSK      /*!< SubG OQPSK filter type */
} subg_ctrl_filter_type_t;

/**************************************************************************************************
 *    GLOBAL PROTOTYPES
 *************************************************************************************************/
/**
 * \brief           Setting SubG initialization.
 * \param[in]       modulation: modulation type.
 * \param[in]       data_rate: data rate.
 * \param[in]       modulation_index: modulation index, see subg_ctrl_fsk_mod_t.
 * \return          None
 */
void subg_ctrl_modem_config_set(subg_ctrl_modulation_t modulation,
                                subg_ctrl_data_rate_t data_rate,
                                uint8_t modulation_index);

/**
 * \brief           Setting SubG MAC parameters.
 * \param[in]       modulation: modulation type.
 * \param[in]       crc_type: CRC type.
 * \param[in]       whiten_enable: whitening function enable flag.
 * \return          None
 */
void subg_ctrl_mac_set(subg_ctrl_modulation_t modulation,
                       subg_ctrl_crc_type_t crc_type,
                       subg_ctrl_whiten_t whiten_enable);

/**
 * \brief           Setting SubG preamble length.
 * \param[in]       modulation: modulation type.
 * \param[in]       preamble_len: preamble length in bytes.
 * \return          None
 */
void subg_ctrl_preamble_set(subg_ctrl_modulation_t modulation,
                            uint32_t preamble_len);

/**
 * \brief           Setting SubG RF frequency.
 * \param[in]       frequency: frequency in kHz.
 * \return          None
 */
void subg_ctrl_frequency_set(uint32_t frequency);

/**
 * \brief           Setting SubG SFD.
 * \param[in]       sfd: 4-byte frame start field.
 * \return          None
 */
void subg_ctrl_sfd_set(subg_ctrl_modulation_t modulation, uint32_t sfd);

/**
 * \brief           Setting SubG SFD.
 * \param[in]       modulation: modulation type.
 * \param[in]       filter: filter type, see subg_ctrl_filter_type_t.
 * \return          None
 */
void subg_ctrl_filter_set(subg_ctrl_modulation_t modulation, uint32_t filter);

/**
 * \brief           Setting RF sleeping.
 * \param[in]       enable: RF sleeping enable flag.
 * \return          None
 */
void subg_ctrl_sleep_set(uint32_t enable);

/**
 * \brief           Setting RF idle.
 * \param[in]       None
 * \return          None
 */
void subg_ctrl_idle_set(void);

/**
 * \brief           Start RF RX for a while
 * \param[in]       rx_timeout: RX on time in micro second.
 * \return          None
 */
void subg_ctrl_rx_start_set(uint32_t rx_timeout);

/*@}*/ /* end of SUBG_group subg_ctrl */

#endif
