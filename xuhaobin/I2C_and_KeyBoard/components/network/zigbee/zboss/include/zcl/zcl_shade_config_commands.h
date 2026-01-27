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
/* PURPOSE: ZCL Shade Configuration cluster specific command declarations
*/

#ifndef ZCL_LEVEL_SHADE_CONFIG_COMMANDS_H
#define ZCL_LEVEL_SHADE_CONFIG_COMMANDS_H 1

void shade_normal_processing(zb_uint8_t ep_id, zb_uint16_t cluster_id, zb_uint8_t value);
void shade_config_processing(zb_uint8_t ep_id);

#endif /* ZCL_LEVEL_CONTROL_COMMANDS_H */
