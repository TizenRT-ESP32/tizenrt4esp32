/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 *
 * Copyright © 2005-2014 Rich Felker, et al.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ***************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <tinyara/compiler.h>
#include <math.h>
#include <stdint.h>

/************************************************************************
 * Public Functions
 ************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE
double remquo(double x, double y, int *quo)
{
	union {
		double f;
		uint64_t i;
	} ux = { x }, uy = { y };
	int ex = ux.i >> 52 & 0x7ff;
	int ey = uy.i >> 52 & 0x7ff;
	int sx = ux.i >> 63;
	int sy = uy.i >> 63;
	uint32_t q;
	uint64_t i;
	uint64_t uxi = ux.i;

	*quo = 0;
	if (uy.i << 1 == 0 || isnan(y) || ex == 0x7ff) {
		return (x * y) / (x * y);
	}
	if (ux.i << 1 == 0) {
		return x;
	}

	/* normalize x and y */
	if (!ex) {
		for (i = uxi << 12; i >> 63 == 0; ex--, i <<= 1) ;
		uxi <<= -ex + 1;
	} else {
		uxi &= -1ULL >> 12;
		uxi |= 1ULL << 52;
	}
	if (!ey) {
		for (i = uy.i << 12; i >> 63 == 0; ey--, i <<= 1) ;
		uy.i <<= -ey + 1;
	} else {
		uy.i &= -1ULL >> 12;
		uy.i |= 1ULL << 52;
	}

	q = 0;
	if (ex < ey) {
		if (ex + 1 == ey) {
			goto end;
		}
		return x;
	}

	/* x mod y */
	for (; ex > ey; ex--) {
		i = uxi - uy.i;
		if (i >> 63 == 0) {
			uxi = i;
			q++;
		}
		uxi <<= 1;
		q <<= 1;
	}
	i = uxi - uy.i;
	if (i >> 63 == 0) {
		uxi = i;
		q++;
	}
	if (uxi == 0) {
		ex = -60;
	} else {
		for (; uxi >> 52 == 0; uxi <<= 1, ex--) ;
	}
end:
	/* scale result and decide between |x| and |x|-|y| */
	if (ex > 0) {
		uxi -= 1ULL << 52;
		uxi |= (uint64_t)ex << 52;
	} else {
		uxi >>= -ex + 1;
	}
	ux.i = uxi;
	x = ux.f;
	if (sy) {
		y = -y;
	}
	if (ex == ey || (ex + 1 == ey && (2 * x > y || (2 * x == y && q % 2)))) {
		x -= y;
		q++;
	}
	q &= 0x7fffffff;
	*quo = sx ^ sy ? -(int)q : (int)q;

	return sx ? -x : x;
}
#endif
