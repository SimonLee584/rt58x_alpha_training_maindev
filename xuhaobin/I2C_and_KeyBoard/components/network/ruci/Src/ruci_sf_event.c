/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/******************************************************************************
*
* @File         ruci_sf_event.c
* @Version
* $Revision:6351
* $Date: 2023-11-17
* @Brief
* @Note
*
*****************************************************************************/

/******************************************************************************
* INCLUDES
******************************************************************************/
#include "ruci_sf_event.h"

#if (RUCI_ENDIAN_INVERSE)
#if (RUCI_ENABLE_SF)

/******************************************************************************
* GLOBAL PARAMETERS
******************************************************************************/
// RUCI: sf_cnf_event ----------------------------------------------------------
const uint8_t ruci_elmt_type_sf_cnf_event[] =
{
    1, 1, 1, 1, 1, 1
};
const uint8_t ruci_elmt_num_sf_cnf_event[] =
{
    1, 1, 1, 1, 1, 1
};

#endif /* RUCI_ENABLE_SF */
#endif /* RUCI_ENDIAN_INVERSE */
