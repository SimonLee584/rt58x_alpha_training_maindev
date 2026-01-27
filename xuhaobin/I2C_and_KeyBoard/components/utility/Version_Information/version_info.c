/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#include "version_api.h"

void lib_version_init(void)
{
    volatile version_entry_t *pt_lib_ver = {0};

#if defined(CONFIG_BOOTLOADER_APP)
    bootloader_ver_get(pt_lib_ver);
#else
    ble_lib_ver_get(pt_lib_ver);
    ble_mesh_lib_ver_get(pt_lib_ver);
    zigbee_lib_ver_get(pt_lib_ver);
    thread_lib_ver_get(pt_lib_ver);
    matter_lib_ver_get(pt_lib_ver);
#endif
}


