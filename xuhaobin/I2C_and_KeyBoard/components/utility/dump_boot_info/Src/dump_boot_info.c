/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#include <FreeRTOS.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "uart_stdio.h"
#include "sysctrl.h"
#include "sysfun.h"

void _dump_boot_info(void) {
    puts("\r\n");
    puts("------------------------------------------------------------\r\n");
    printf("ARM %s", getotpversion().type == CHIP_TYPE_584 ? "Cortex-M33" : "Cortex-M3");
    puts(" SoC: ");
    puts(CONFIG_CHIP);
    puts("\r\n");
    puts("Target Board: ");
    puts(CONFIG_BOARD);
    puts("\r\n");

#ifdef HOSAL_SOC_TARGET_CUSTOMER
    puts(" Customer: ");
    puts(HOSAL_SOC_TARGET_CUSTOMER);
    puts("\r\n");
#endif

    puts("Build Version: ");
    puts(RAFAEL_SDK_VER);
    puts("\r\n");

    puts("Build Date: ");
    puts(__DATE__);
    puts("\r\n");
    puts("Build Time: ");
    puts(__TIME__);
    puts("\r\n");

    printf("System clock: %s\r\n",
           (get_ahb_system_clk() == SYS_CLK_32MHZ) ? "32MHz"
           : (get_ahb_system_clk() == SYS_CLK_48MHZ) ? "48MHz"
           : "64MHz");
    puts("------------------------------------------------------------\r\n");
}
