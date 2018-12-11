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

/**
 * @defgroup UART UART
 * @brief Provides APIs for UART (Universal Asynchronous Receiver/Transmitter)
 * @ingroup IOTBUS
 * @{
 */

/**
 * @file iotbus/iotbus_uart.h
 * @brief Iotbus APIs for UART
 */

#ifndef IOTBUS_UART_H_
#define IOTBUS_UART_H_

#include <stdint.h>

/**
 * @brief Enumeration of UART parity type
 * @details
 * Enumeration Details:\n
 * IOTBUS_UART_PARITY_NONE\n
 * IOTBUS_UART_PARITY_EVEN\n
 * IOTBUS_UART_PARITY_ODD\n
 */
typedef enum {
	IOTBUS_UART_PARITY_NONE = 0,
	IOTBUS_UART_PARITY_EVEN,
	IOTBUS_UART_PARITY_ODD,
} iotbus_uart_parity_e;

struct _iotbus_uart_s;

/**
 * @brief Pointer definition to the internal struct _iotbus_uart_wrapper_s
 */
typedef struct _iotbus_uart_wrapper_s *iotbus_uart_context_h;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief initializes uart_context.
 *
 * @details @b #include <iotbus/iotbus_uart.h>
 * @param[in] path uart device node path
 * @return On success, handle of uart_context is returned. On failure, NULL is returned.
 * @since TizenRT v1.0
 */
iotbus_uart_context_h iotbus_uart_init(const char *path);

/**
 * @brief closes uart_context.
 *
 * @details @b #include <iotbus/iotbus_uart.h>
 * @param[in] hnd handle of uart_context
 * @return On success, 0 is returned. On failure, a negative value is returned.
 * @since TizenRT v1.0
 */
int iotbus_uart_stop(iotbus_uart_context_h hnd);

#ifdef CONFIG_SERIAL_TERMIOS
/**
 * @brief flushes uart buffer.
 *
 * @details @b #include <iotbus/iotbus_uart.h>
 * @param[in] hnd handle of uart_context
 * @return On success, 0 is returned. On failure, a negative value is returned.
 * @since TizenRT v1.0
 */
int iotbus_uart_flush(iotbus_uart_context_h hnd);

/**
 * @brief sets uart baud rate.
 *
 * @details @b #include <iotbus/iotbus_uart.h>
 * @param[in] hnd handle of uart_context
 * @param[in] baud uart baud rate
 * @return On success, 0 is returned. On failure, a negative value is returned.
 * @since TizenRT v1.0
 */
int iotbus_uart_set_baudrate(iotbus_uart_context_h hnd, unsigned int baud);

/**
 * @brief sets byte size, parity bit and stop bits.
 *
 * @details @b #include <iotbus/iotbus_uart.h>
 * @param[in] hnd handle of uart_context
 * @param[in] bytesize uart byte size
 * @param[in] parity uart parity type
 * @param[in] stopbits uart stop bits
 * @return On success, 0 is returned. On failure, a negative value is returned.
 * @since TizenRT v1.0
 */
int iotbus_uart_set_mode(iotbus_uart_context_h hnd, int bytesize, iotbus_uart_parity_e parity, int stopbits);

/**
 * @brief set flow control settings.
 *
 * @details @b #include <iotbus/iotbus_uart.h>
 * @param[in] hnd handle of uart_context
 * @param[in] xonxoff ixon/ixoff
 * @param[in] rtscts rts/cts
 * @return On success, 0 is returned. On failure, a negative value is returned.
 * @since TizenRT v1.0
 */
int iotbus_uart_set_flowcontrol(iotbus_uart_context_h hnd, int xonxoff, int rtscts);
#endif /* CONFIG_SERIAL_TERMIOS */

/**
 * @brief reads data over uart bus.
 *
 * @details @b #include <iotbus/iotbus_uart.h>
 * @param[in] hnd handle of uart_context
 * @param[in] buf the pointer of data buffer
 * @param[in] length size to read
 * @return On success, size is returned. On failure, a negative value is returned.
 * @since TizenRT v1.0
 */
int iotbus_uart_read(iotbus_uart_context_h hnd, char *buf, unsigned int length);

/**
 * @brief writes data over uart bus.
 *
 * @details @b #include <iotbus/iotbus_uart.h>
 * @param[in] hnd handle of uart_context
 * @param[in] buf the pointer of data buffer
 * @param[in] length size to write
 * @return On success, size is returned. On failure, a negative value is returned.
 * @since TizenRT v1.0
 */
int iotbus_uart_write(iotbus_uart_context_h hnd, const char *buf, unsigned int length);

#ifdef __cplusplus
}
#endif

#endif /* IOTBUS_UART_H_ */

/** @} */ // end of UART group
