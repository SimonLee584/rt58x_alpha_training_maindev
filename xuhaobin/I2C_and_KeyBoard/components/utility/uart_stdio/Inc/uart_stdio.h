/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * @file uart_stdio.h
 * @author Rex Huang (rex.huang@rafaelmicro.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-21
 * 
 * 
 */

#ifndef __UART_STDIO__
#define __UART_STDIO__

#ifdef __cplusplus
extern "C" {
#endif

int uart_stdio_init(void);
int uart_stdio_write(uint8_t* p_data, uint32_t length);
int uart_stdio_read(uint8_t* p_data, uint32_t length);
int uart_stdio_deinit(void);

/** @} */
#endif
#ifdef __cplusplus
}
#endif
