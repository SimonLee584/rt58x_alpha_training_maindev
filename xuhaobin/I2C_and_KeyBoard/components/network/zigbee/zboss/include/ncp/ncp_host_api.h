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
/*  PURPOSE: NCP Host API declarations
*/
#ifndef NCP_HOST_API_H
#define NCP_HOST_API_H 1

#include "zb_common.h"

/**
 * Initializes NCP Host subsystem
 */
void ncp_host_init(void);

/**
 * Get the firmware version read from soc
 */
zb_uint32_t ncp_host_get_firmware_version(void);

/**
 * Runs the bootloader
 *
 * @return status of operation
 */
zb_ret_t ncp_host_ota_run_bootloader(void);

#endif /* NCP_HOST_API_H */
