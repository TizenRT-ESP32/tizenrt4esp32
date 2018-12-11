/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
 * apps/system/readline/readline_common.c
 *
 *   Copyright (C) 2007-2008, 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/ascii.h>
#include <tinyara/vt100.h>

#include <apps/readline.h>
#include "readline.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* <esc>[K is the VT100 command erases to the end of the line. */
#ifdef CONFIG_READLINE_ECHO
static const char g_erasetoeol[] = VT100_CLEAREOL;
#endif
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: readline_common
 *
 *   readline() reads in at most one less than 'buflen' characters from
 *   'instream' and stores them into the buffer pointed to by 'buf'.
 *   Characters are echoed on 'outstream'.  Reading stops after an EOF or a
 *   newline.  If a newline is read, it is stored into the buffer.  A null
 *   terminator is stored after the last character in the buffer.
 *
 *   This version of realine assumes that we are reading and writing to
 *   a VT100 console.  This will not work well if 'instream' or 'outstream'
 *   corresponds to a raw byte steam.
 *
 *   This function is inspired by the GNU readline but is an entirely
 *   different creature.
 *
 * Input Parameters:
 *   buf       - The user allocated buffer to be filled.
 *   buflen    - the size of the buffer.
 *   instream  - The stream to read characters from
 *   outstream - The stream to each characters to.
 *
 * Returned values:
 *   On success, the (positive) number of bytes transferred is returned.
 *   EOF is returned to indicate either an end of file condition or a
 *   failure.
 *
 **************************************************************************/

ssize_t readline_common(FAR struct rl_common_s *vtbl, FAR char *buf, int buflen)
{
	int escape;
	int nch;

	/* Sanity checks */

	DEBUGASSERT(buf && buflen > 0);

	if (buflen < 2) {
		*buf = '\0';
		return 0;
	}

	/* <esc>[K is the VT100 command that erases to the end of the line. */

#ifdef CONFIG_READLINE_ECHO
	RL_WRITE(vtbl, g_erasetoeol, sizeof(g_erasetoeol));
#endif

	/* Read characters until we have a full line. On each the loop we must
	 * be assured that there are two free bytes in the line buffer:  One for
	 * the next character and one for the null terminator.
	 */

	escape = 0;
	nch = 0;

	for (;;) {
		/* Get the next character. readline_rawgetc() returns EOF on any
		 * errors or at the end of file.
		 */

		int ch = RL_GETC(vtbl);

		/* Check for end-of-file or read error */

		if (ch == EOF) {
			/* Did we already received some data? */

			if (nch > 0) {
				/* Yes.. Terminate the line (which might be zero length)
				 * and return the data that was received.  The end-of-file
				 * or error condition will be reported next time.
				 */

				buf[nch] = '\0';
				return nch;
			}

			return EOF;
		}

		/* Are we processing a VT100 escape sequence */

		else if (escape) {
			/* Yes, is it an <esc>[, 3 byte sequence */

			if (ch != ASCII_LBRACKET || escape == 2) {
				/* We are finished with the escape sequence */

				escape = 0;
			} else {
				/* The next character is the end of a 3-byte sequence.
				 * NOTE:  Some of the <esc>[ sequences are longer than
				 * 3-bytes, but I have not encountered any in normal use
				 * yet and, so, have not provided the decoding logic.
				 */

				escape = 2;
			}
		}

		/* Check for backspace
		 *
		 * There are several notions of backspace, for an elaborate summary see
		 * http://www.ibb.net/~anne/keyboard.html. There is no clean solution.
		 * Here both DEL and backspace are treated like backspace here.  The
		 * Unix/Linux screen terminal by default outputs  DEL (0x7f) when the
		 * backspace key is pressed.
		 */

		else if (ch == ASCII_BS || ch == ASCII_DEL) {
			/* Eliminate that last character in the buffer. */

			if (nch > 0) {
				nch--;

#ifdef CONFIG_READLINE_ECHO
				/* Echo the backspace character on the console.  Always output
				 * the backspace character because the VT100 terminal doesn't
				 * understand DEL properly.
				 */

				RL_PUTC(vtbl, ASCII_BS);
				RL_WRITE(vtbl, g_erasetoeol, sizeof(g_erasetoeol));
#endif
			}
		}

		/* Check for the beginning of a VT100 escape sequence */

		else if (ch == ASCII_ESC) {
			/* The next character is escaped */

			escape = 1;
		}

		/* Check for end-of-line.  This is tricky only in that some
		 * environments may return CR as end-of-line, others LF, and
		 * others both.
		 */

#if  defined(CONFIG_EOL_IS_LF) || defined(CONFIG_EOL_IS_BOTH_CRLF)
		else if (ch == '\n')
#elif defined(CONFIG_EOL_IS_CR)
		else if (ch == '\r')
#elif defined(CONFIG_EOL_IS_EITHER_CRLF)
		else if (ch == '\n' || ch == '\r')
#endif
		{
			/* The newline is stored in the buffer along with the null
			 * terminator.
			 */

			buf[nch++] = '\n';
			buf[nch] = '\0';

#ifdef CONFIG_READLINE_ECHO
			/* Echo the newline to the console */

			RL_PUTC(vtbl, '\n');
#endif
			return nch;
		}

		/* Otherwise, check if the character is printable and, if so, put the
		 * character in the line buffer
		 */

		else if (isprint(ch)) {
			buf[nch++] = ch;

#ifdef CONFIG_READLINE_ECHO
			/* Echo the character to the console */

			RL_PUTC(vtbl, ch);
#endif
			/* Check if there is room for another character and the line's
			 * null terminator.  If not then we have to end the line now.
			 */

			if (nch + 1 >= buflen) {
				buf[nch] = '\0';
				return nch;
			}
		}
	}
}
