/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 *
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * \file            hosal_flash.h
 * \brief           hosal_flash include file
 */

/*
 * This file is part of library_name.
 * Author:  ives.lee 
 */
#ifndef HOSAL_FLASH_H
#define HOSAL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mcu.h"


/**
 * \defgroup HOSAL_FLASH Hosal flash
 * \ingroup RT58X_HOSAL
 * \brief  Define Hosal flash definitions, structures, and functions
 * @{
 */

/**
* \brief           flash read command
 */
#define HOSAL_FLASH_READ_BYTE     0x00
#define HOSAL_FLASH_READ_PAGE     0x01
#define HOSAL_FLASH_SECURITY_READ 0x02
#define HOSAL_FLASH_READ_STATUS   0x03
#define HOSAL_FLASH_READ_ID       0x04

/**
* \brief           flash write command
 */
#define HOSAL_FLASH_WRITE_BYTE     0x10
#define HOSAL_FLASH_WRITE_PAGE     0x11
#define HOSAL_FLASH_SECURITY_WRITE 0x12
#define HOSAL_FLASH_WRITE_STATUS   0x13

/**
 * \brief           flash erase command 
 */
#define HOSAL_FLASH_ERASE_PAGE          0x20
#define HOSAL_FLASH_ERASE_SECTOR        0x21
#define HOSAL_FLASH_ERASE_32K_SECTOR    0x22
#define HOSAL_FLASH_ERASE_64K_SECTOR    0x23
#define HOSAL_FLASH_ERASE_CHIP          0x24
#define HOSAL_FLASH_ERASE_SECURITY_PAGE 0x25

/**
 * \brief           flash io control command 
 */
#define HOSAL_FLASH_ENABLE_SUSPEND  0x30
#define HOSAL_FLASH_DISABLE_SUSPEND 0x31
#define HOSAL_FLASH_CACHE           0x32
#define HOSAL_FLASH_BUSY            0x33
#define HOSAL_FLASH_GET_INFO        0x34

/**
* \brief            Flash size definitions.
*/
typedef enum {
    HOSAL_FLASH_NOT_SUPPORT = 0x00,
    HOSAL_FLASH_512K = 0x13,                          /*!< 512K size   */
    HOSAL_FLASH_1024K = 0x14,                         /*!< 1024K size   */
    HOSAL_FLASH_2048K = 0x15,                         /*!< 2048K size   */
} hosal_flash_size_t;

/**
 * \brief               flash conifg struct table
 *                      This structure is used to calculate all point
 *                      related stuff
 */
typedef struct {
    uint32_t address;     /*!< flash address */
    uint32_t length;      /*!< flash data length  */
    uint32_t buf_address; /*!< flash data buffer address  */
} hosal_flash_config_t;

/**
 * \brief           enable internal flash dual mode
 * \param[in]       NONE
 * \return          function status
 */
int hosal_flash_enable_qe(void);

/**
 * \brief           hosal_flash_init
 * \param[in]       NONE
 * \return          function status
 */
int hosal_flash_init(void);

/**
 * \brief           flash io control function
 * \param[in]       ctl: control command
 * \param[in]       address: flash address
 * \param[in]       buf: flash buffer address
 * \return          function status
 */
int hosal_flash_read(int ctl, uint32_t address, uint8_t* buf);

/**
 * \brief           flash io control function
 * \param[in]       ctl: control command
 * \param[in]       address: flash address
 * \param[in]       buf: flash buffer address
 * \return          function status
 */
int hosal_flash_write(int ctl, uint32_t address, uint8_t* buf);

/**
 * \brief           flash io control function
 * \param[in]       ctl: control command
 * \param[in]       address: flash address
 * \return          function status
 */
int hosal_flash_erase(int ctl, uint32_t address);

/**
 * \brief           flash io control function
 * \param[in]       ctl: control command
 * \param[in]       address: flash address
 * \param[in]       buf: flash buffer address
 * \return          function status
 */
int hosal_flash_ioctrl(int ctl, void* buf);

/**
 * \brief           flash io control function
 * \param[in]       length: read id lenght
 * \param[in]       buf: flash buffer address
 * \return          function status
 */
int hosal_flash_read_id(uint32_t length, uint8_t* buf);

/*@}*/ /* end of RT58X_HOSAL HOSAL_FLASH */

#ifdef __cplusplus
}
#endif

#endif /* End of HOSAL_FLASH_H */
