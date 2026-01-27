/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * \file            dwt.h
 * \brief           dwt header file
 */

/*
 * This file is part of library_name.
 * Author:     
 */

#ifndef ___DWT_H__
#define ___DWT_H__

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * \defgroup DWT Dwt
 * \ingroup RT584_DRIVER
 * \brief  Define Dwt comp definitions, structures, and functions
 * @{
 */

#define     DWT_CR_TRCENA           (1<<24)
#define     DWT_CR_CYCCNTENA        (1<<0)


/**
 * \brief           Timeout unit
 */
typedef enum {
    DWT_TIMEOUT_UNIT_US = 0,   /**< microsecond */
    DWT_TIMEOUT_UNIT_MS        /**< millisecond */
} dwt_timeout_unit_t;


/**
 * \brief           DWT timeout structure
 */
typedef struct {
    uint32_t start_cycle;          /**< DWT start cycle */
    uint32_t timeout_cycle;        /**< DWT timeout cycle */
    void (*callback)(void);        /**< optional callback on timeout */
    uint32_t initialized;          /**< is timeout initialized */
    uint32_t timeout_value;        /**< original timeout value */
    dwt_timeout_unit_t unit;       /**< timeout unit */
} dwttimeout_t;

/**
 * \brief           Initialize DWT counter.
 * \return
 * \retval          STATUS_SUCCESS       DWT initialized successfully.
 */
uint32_t dwt_init(void);

/**
 * \brief           Uninitialize DWT counter.
 * \return
 * \retval          STATUS_SUCCESS       DWT uninitialized successfully.
 * \retval          STATUS_NO_INIT      DWT not initialized.
 */
uint32_t dwt_uninit(void);

/**
 * \brief           Start a DWT timeout counter.
 * \param[in]       tmo         Timeout struct pointer.
 * \param[in]       timeout     Timeout value.
 * \param[in]       unit        Timeout unit (us/ms).
 * \param[in]       cb          Callback when timeout occurs (can be NULL).
 * \return
 * \retval          STATUS_NO_INIT       DWT not initialized or tmo already initialized.
 * \retval          STATUS_SUCCESS       Timeout started successfully.
 */
uint32_t dwt_timeoutstart(dwttimeout_t *tmo, uint32_t timeout, dwt_timeout_unit_t unit, void (*cb)(void));

/**
 * \brief           Check if timeout occurred.
 * \param[in]       tmo         Timeout struct pointer.
 * \return
 * \retval          STATUS_SUCCESS       Timeout not yet occurred.
 * \retval          STATUS_TIMEOUT       Timeout occurred.
 * \retval          STATUS_NO_INIT       DWT or tmo not initialized.
 */
uint32_t dwt_timeoutcheck(dwttimeout_t *tmo);

/**
 * \brief           Get remaining timeout.
 * \param[in]       tmo         Timeout struct pointer.
 * \param[out]      remaining   Remaining time (unit depends on tmo->unit).
 * \return
 * \retval          STATUS_SUCCESS       Timeout not yet occurred.
 * \retval          STATUS_TIMEOUT       Timeout occurred.
 * \retval          STATUS_NO_INIT       DWT or tmo not initialized.
 */
uint32_t dwt_timeoutremaining(dwttimeout_t *tmo, uint32_t *remaining);

/**
 * \brief           Blocking delay in microseconds.
 * \param[in]       us          Delay time in microseconds.
 * \return
 * \retval          STATUS_SUCCESS       Delay completed.
 */
uint32_t dwt_delay_us(uint32_t us);

/**
 * \brief           Blocking delay in milliseconds.
 * \param[in]       ms          Delay time in milliseconds.
 * \return
 * \retval          STATUS_SUCCESS       Delay completed.
 */
uint32_t dwt_delay_ms(uint32_t ms);

/**
 * \brief           Non-blocking delay in microseconds.
 * \param[in]       tmo         Timeout struct pointer.
 * \param[in]       us          Delay time in microseconds.
 * \return
 * \retval          STATUS_SUCCESS       Timeout started successfully.
 */
uint32_t dwt_delay_us_nonblocking(dwttimeout_t *tmo, uint32_t us);

/**
 * \brief           Non-blocking delay in milliseconds.
 * \param[in]       tmo         Timeout struct pointer.
 * \param[in]       ms          Delay time in milliseconds.
 * \return
 * \retval          STATUS_SUCCESS       Timeout started successfully.
 */
uint32_t dwt_delay_ms_nonblocking(dwttimeout_t *tmo, uint32_t ms);

/*@}*/ /* end of RT584_DRIVER DWT */

#ifdef __cplusplus
}
#endif

#endif /* __DWT_H__ */


