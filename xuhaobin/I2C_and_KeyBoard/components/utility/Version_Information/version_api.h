/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#ifndef __VERSION_API_H__
#define __VERSION_API_H__

typedef struct __version_entry_s {
    const char* build_date;
    const char* version_info;
    const char* version_tag;
} version_entry_t;

#if defined(CONFIG_BOOTLOADER_APP)
void bootloader_ver_get(volatile version_entry_t *pt_ver_info);
#else
void ble_lib_ver_get(volatile version_entry_t *pt_ver_info);
void ble_mesh_lib_ver_get(volatile version_entry_t *pt_ver_info);
void zigbee_lib_ver_get(volatile version_entry_t *pt_ver_info);
void thread_lib_ver_get(volatile version_entry_t *pt_ver_info);
void matter_lib_ver_get(volatile version_entry_t *pt_ver_info);
#endif

#endif

