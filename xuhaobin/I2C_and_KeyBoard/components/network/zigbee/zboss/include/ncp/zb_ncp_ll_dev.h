/* ZBOSS Zigbee software protocol stack
 *
 * Copyright (c) 2012-2020 DSR Corporation, Denver CO, USA.
 * www.dsr-zboss.com
 * www.dsr-corporation.com
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
/*  PURPOSE: NCP Low level protocol definitions for the device side
*/
#ifndef NCP_LL_DEV_H
#define NCP_LL_DEV_H 1

#include "zb_common.h"

/* LL protocol API for NCP (== inside ZBOSS) */

typedef void (*ncp_ll_packet_received_cb_t)(void  *data, zb_uint16_t len);
typedef void (*ncp_ll_tx_ready_cb_t)(void);

void ncp_ll_proto_init(ncp_ll_packet_received_cb_t packet_received_cb, ncp_ll_tx_ready_cb_t tx_ready_cb);
zb_int_t ncp_ll_send_packet(void *data, zb_uint32_t len);
void ncp_ll_quant(zb_uint8_t unused);

#endif /* NCP_LL_DEV_H */
