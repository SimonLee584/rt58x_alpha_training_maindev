/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */







void sec_aes_enc(uint8_t *key, uint8_t *din, uint8_t *dout);

void sec_obfuscation(uint8_t *pKey, uint8_t *pIv, uint8_t *pText, uint16_t len);

