/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#include <string.h>

void *memmove(void *dest, void const *src, size_t n)
{
    register char *dp = dest;
    register char const *sp = src;
    if (dp < sp) {
        while(n-- > 0) {
            *(dp++) = *(sp++);
        }
    } else {
        dp += n;
        sp += n;
        while(n-- > 0) {
            *(--dp) = *(--sp);
        }
    }

    return dest;
}
