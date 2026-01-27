/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/*
 * Derived from:
 * http://www.kernel.org/pub/linux/libs/klibc/
 */
/*
 * sscanf()
 */

#include <stdio.h>
#include <stdarg.h>

int sscanf(const char *str, const char *format, ...)
{
	va_list ap;
	int rv;

	va_start(ap, format);
	rv = vsscanf(str, format, ap);
	va_end(ap);

	return rv;
}
