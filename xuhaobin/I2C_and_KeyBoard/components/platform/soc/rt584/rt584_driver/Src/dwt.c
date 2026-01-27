/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * \file            dwt.c
 * \brief           dwt driver file
 */

/*
 * This file is part of library_name.
 * Author:     
 */
#include "mcu.h"
#include "dwt.h"
#include "status.h"

static uint8_t dwt_initialized = 0;


uint32_t dwt_init(void)
{
    if (dwt_initialized) return STATUS_SUCCESS;

    CoreDebug->DEMCR |= DWT_CR_TRCENA;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CR_CYCCNTENA;

    dwt_initialized = 1;
    return STATUS_SUCCESS;
}


uint32_t dwt_uninit(void)
{
    if (!dwt_initialized) return STATUS_NO_INIT;

    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    dwt_initialized = 0;
    return STATUS_SUCCESS;
}


uint32_t dwt_timeoutstart(dwttimeout_t *tmo, uint32_t timeout, dwt_timeout_unit_t unit, void (*cb)(void))
{
    if (!dwt_initialized || tmo == NULL || tmo->initialized)
        return STATUS_NO_INIT;

    tmo->start_cycle   = DWT->CYCCNT;
    tmo->callback      = cb;
    tmo->initialized   = 1;
    tmo->timeout_value = timeout;
    tmo->unit          = unit;

    if (unit == DWT_TIMEOUT_UNIT_US)
        tmo->timeout_cycle = (SystemCoreClock / 1000000UL) * timeout;
    else
        tmo->timeout_cycle = (SystemCoreClock / 1000UL) * timeout;

    return STATUS_SUCCESS;
}


uint32_t dwt_timeoutcheck(dwttimeout_t *tmo)
{
    if (!dwt_initialized || tmo == NULL || !tmo->initialized)
        return STATUS_NO_INIT;

    uint32_t elapsed = DWT->CYCCNT - tmo->start_cycle;
    if (elapsed >= tmo->timeout_cycle)
    {
        if (tmo->callback) tmo->callback();
        return STATUS_TIMEOUT;
    }

    return STATUS_SUCCESS;
}


uint32_t dwt_timeoutremaining(dwttimeout_t *tmo, uint32_t *remaining)
{
    if (!dwt_initialized || tmo == NULL || !tmo->initialized || remaining == NULL)
        return STATUS_NO_INIT;

    uint32_t elapsed = DWT->CYCCNT - tmo->start_cycle;
    if (elapsed >= tmo->timeout_cycle)
    {
        *remaining = 0;
        return STATUS_TIMEOUT;
    }

    uint32_t remaining_cycles = tmo->timeout_cycle - elapsed;
    *remaining = (tmo->unit == DWT_TIMEOUT_UNIT_US)
                 ? remaining_cycles / (SystemCoreClock / 1000000UL)
                 : remaining_cycles / (SystemCoreClock / 1000UL);

    return STATUS_SUCCESS;
}


uint32_t dwt_delay(dwttimeout_t *tmo)
{
    return dwt_timeoutcheck(tmo);
}

uint32_t dwt_delay_us(uint32_t us)
{
    if (!dwt_initialized) dwt_init();

    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000UL) * us;

    while ((DWT->CYCCNT - start) < cycles);
    return STATUS_SUCCESS;
}

uint32_t dwt_delay_ms(uint32_t ms)
{
    if (!dwt_initialized) dwt_init();

    for (uint32_t i = 0; i < ms; i++)
        dwt_delay_us(1000);

    return STATUS_SUCCESS;
}


uint32_t dwt_delay_us_nonblocking(dwttimeout_t *tmo, uint32_t us)
{
    return dwt_timeoutstart(tmo, us, DWT_TIMEOUT_UNIT_US, NULL);
}

uint32_t dwt_delay_ms_nonblocking(dwttimeout_t *tmo, uint32_t ms)
{
    return dwt_timeoutstart(tmo, ms, DWT_TIMEOUT_UNIT_MS, NULL);
}