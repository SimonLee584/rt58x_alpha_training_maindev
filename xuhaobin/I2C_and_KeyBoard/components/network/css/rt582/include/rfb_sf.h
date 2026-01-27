/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/**
 * @file rfb_sf.h
 * @author
 * @date
 * @brief rfb special function header.
 *
 * More detailed description can go here
 *
 *
 * @see http://
 */

#ifndef __RFB_SF__
#define __RFB_SF__

/**************************************************************************************************
*    INCLUDES
*************************************************************************************************/


/**************************************************************************************************
 *    CONSTANTS AND DEFINES
 *************************************************************************************************/


/**************************************************************************************************
 *    GLOBAL FUNCTIONS
 *************************************************************************************************/
uint16_t rfb_sf_hdlr(uint8_t *p_input_buf, uint8_t *p_output_buf, uint16_t max_uart_len);

#endif
