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
/*  PURPOSE: Header file for NWK MultiMAC
*/
#ifndef ZB_NWK_MM_H
#define ZB_NWK_MM_H 1

#define ZB_NWK_MAC_IFACE_SCAN_TYPE_ACTIVE     0U
#define ZB_NWK_MAC_IFACE_SCAN_TYPE_ENH_ACTIVE 1U

void zb_nwk_mm_mac_iface_table_init(void);

zb_uint8_t zb_nwk_mm_get_freq_band(void);

void zb_nlme_set_interface_request(zb_uint8_t param);

void zb_nlme_get_interface_request(zb_uint8_t param);

#endif /* ZB_NWK_MM_H */
