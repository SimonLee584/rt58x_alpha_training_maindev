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
/* PURPOSE: storing manufacturer-specific prefixes from IEEE address
*/

#ifndef ZB_MANUF_PREFIXES_H
#define ZB_MANUF_PREFIXES_H 1

#ifdef ZB_NWK_CHECK_MULTIHOP_TRANSMISSIONS_FROM_POS

#define ZB_MANUF_PREFIX_QUIRKY_INC                      {0x24U, 0xe5U, 0x7cU}

#endif /* ZB_NWK_CHECK_MULTIHOP_TRANSMISSIONS_FROM_POS */

#endif /* ZB_MANUF_PREFIXES_H */
