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
 * arch/arm/src/tiva/lm3s_ethernet.c
 *
 *   Copyright (C) 2009-2010, 2014 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_TIVA_ETHERNET)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <tinyara/arch.h>
#include <tinyara/wdog.h>
#include <tinyara/irq.h>
#include <tinyara/wqueue.h>
#include <arch/board/board.h>

#include <tinyara/net/netdev.h>
#include <net/lwip/netif/etharp.h>
#include <tinyara/net/ethernet.h>

#include <net/if.h>
#include <net/lwip/opt.h>
#include <net/lwip/netif.h>
#include <net/lwip/tcpip.h>

#ifdef CONFIG_NET_PKT
#include <tinyara/net/pkt.h>
#endif

#include "chip.h"
#include "up_arch.h"

#include "tiva_gpio.h"
#include "tiva_ethernet.h"
#include "chip/tiva_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_NET_MULTIBUFFER
#error CONFIG_NET_MULTIBUFFER should not be selected
#endif

/* Half duplex can be forced if CONFIG_TIVA_ETHHDUPLEX is defined. */

#ifdef CONFIG_TIVA_ETHHDUPLEX
#define TIVA_DUPLEX_SETBITS 0
#define TIVA_DUPLEX_CLRBITS MAC_TCTL_DUPLEX
#else
#define TIVA_DUPLEX_SETBITS MAC_TCTL_DUPLEX
#define TIVA_DUPLEX_CLRBITS 0
#endif

/* Auto CRC generation can be suppressed if CONFIG_TIVA_ETHNOAUTOCRC is definde */

#ifdef CONFIG_TIVA_ETHNOAUTOCRC
#define TIVA_CRC_SETBITS 0
#define TIVA_CRC_CLRBITS MAC_TCTL_CRC
#else
#define TIVA_CRC_SETBITS MAC_TCTL_CRC
#define TIVA_CRC_CLRBITS 0
#endif

/* Tx padding can be suppressed if CONFIG_TIVA_ETHNOPAD is defined */

#ifdef CONFIG_TIVA_ETHNOPAD
#define TIVA_PADEN_SETBITS 0
#define TIVA_PADEN_CLRBITS MAC_TCTL_PADEN
#else
#define TIVA_PADEN_SETBITS MAC_TCTL_PADEN
#define TIVA_PADEN_CLRBITS 0
#endif

#define TIVA_TCTCL_SETBITS (TIVA_DUPLEX_SETBITS|TIVA_CRC_SETBITS|TIVA_PADEN_SETBITS)
#define TIVA_TCTCL_CLRBITS (TIVA_DUPLEX_CLRBITS|TIVA_CRC_CLRBITS|TIVA_PADEN_CLRBITS)

/* Multicast frames can be enabled by defining CONFIG_TIVA_MULTICAST */

#ifdef CONFIG_TIVA_MULTICAST
#define TIVA_AMUL_SETBITS MAC_RCTL_AMUL
#define TIVA_AMUL_CLRBITS 0
#else
#define TIVA_AMUL_SETBITS 0
#define TIVA_AMUL_CLRBITS MAC_RCTL_AMUL
#endif

/* Promiscuous mode can be enabled by defining CONFIG_TIVA_PROMISCUOUS */

#ifdef CONFIG_TIVA_PROMISCUOUS
#define TIVA_PRMS_SETBITS MAC_RCTL_PRMS
#define TIVA_PRMS_CLRBITS 0
#else
#define TIVA_PRMS_SETBITS 0
#define TIVA_PRMS_CLRBITS MAC_RCTL_PRMS
#endif

/* Bad CRC rejection can be enabled by define CONFIG_TIVA_BADCRC */

#ifdef CONFIG_TIVA_BADCRC
#define TIVA_BADCRC_SETBITS MAC_RCTL_BADCRC
#define TIVA_BADCRC_CLRBITS 0
#else
#define TIVA_BADCRC_SETBITS 0
#define TIVA_BADCRC_CLRBITS MAC_RCTL_BADCRC
#endif

#define TIVA_RCTCL_SETBITS (TIVA_AMUL_SETBITS|TIVA_PRMS_SETBITS|TIVA_BADCRC_SETBITS)
#define TIVA_RCTCL_CLRBITS (TIVA_AMUL_CLRBITS|TIVA_PRMS_CLRBITS|TIVA_BADCRC_CLRBITS)

/* CONFIG_TIVA_DUMPPACKET will dump the contents of each packet to the console. */

#ifdef CONFIG_TIVA_DUMPPACKET
#define tiva_dumppacket(m, a, n) lib_dumpbuffer(m, a, n)
#else
#define tiva_dumppacket(m, a, n)
#endif

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define TIVA_WDDELAY   (1*CLK_TCK)
#define TIVA_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define TIVA_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define ETHBUF ((struct eth_hdr_s *)priv->ld_dev.d_buf)

#define TIVA_MAX_MDCCLK 2500000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* EMAC statistics (debug only) */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
struct tiva_statistics_s {
	uint32_t rx_int;			/* Number of Rx interrupts received */
	uint32_t rx_packets;		/* Number of packets received (sum of the following): */
#ifdef CONFIG_NET_IPv4
	uint32_t rx_ip;				/* Number of Rx IPv4 packets received */
#endif
#ifdef CONFIG_NET_IPv6
	uint32_t rx_ipv6;			/* Number of Rx IPv6 packets received */
#endif
	uint32_t rx_arp;			/* Number of Rx ARP packets received */
	uint32_t rx_dropped;		/* Number of dropped, unsupported Rx packets */
	uint32_t rx_pktsize;		/* Number of dropped, too small or too big */
	uint32_t rx_errors;			/* Number of Rx errors (reception error) */
	uint32_t rx_ovrerrors;		/* Number of Rx FIFO overrun errors */
	uint32_t tx_int;			/* Number of Tx interrupts received */
	uint32_t tx_packets;		/* Number of Tx packets queued */
	uint32_t tx_errors;			/* Number of Tx errors (transmission error) */
	uint32_t tx_timeouts;		/* Number of Tx timeout errors */
};
#define EMAC_STAT(priv, name) priv->ld_stat.name++
#else
#define EMAC_STAT(priv, name)
#endif

/* The tiva_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct tiva_driver_s {
	/* The following fields would only be necessary on chips that support
	 * multiple Ethernet controllers.
	 */

#if TIVA_NETHCONTROLLERS > 1
	uint32_t ld_base;			/* Ethernet controller base address */
	int ld_irq;					/* Ethernet controller IRQ */
#endif

	bool ld_bifup;				/* true:ifup false:ifdown */
	WDOG_ID ld_txpoll;			/* TX poll timer */
	WDOG_ID ld_txtimeout;		/* TX timeout timer */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
	struct tiva_statistics_s ld_stat;
#endif
	struct work_s irqwork;					/* Interrupt continuation work queue support */

	/* This holds the information visible to LWIP */
	struct netif ld_dev;		/* Interface of network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tiva_driver_s g_lm3sdev[TIVA_NETHCONTROLLERS];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Miscellaneous low level helpers */

#if TIVA_NETHCONTROLLERS > 1
static uint32_t tiva_ethin(struct tiva_driver_s *priv, int offset);
static void tiva_ethout(struct tiva_driver_s *priv, int offset, uint32_t value);
#else
static inline uint32_t tiva_ethin(struct tiva_driver_s *priv, int offset);
static inline void tiva_ethout(struct tiva_driver_s *priv, int offset, uint32_t value);
#endif
static void tiva_ethreset(struct tiva_driver_s *priv);

/* Common TX logic */

static int tiva_transmit(struct tiva_driver_s *priv);

/* Interrupt handling */

static void tiva_receive(struct tiva_driver_s *priv);
static void tiva_txdone(struct tiva_driver_s *priv);
static int tiva_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void tiva_polltimer(int argc, uint32_t arg, ...);
static void tiva_txtimeout(int argc, uint32_t arg, ...);

/* callback functions */

static int tiva_ifup(struct netif *dev);
static int tiva_ifdown(struct netif *dev);
static int tiva_txavail(struct netif *dev);

static void tiva_irqworker(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tiva_ethin
 *
 * Description:
 *   Read a register from the Ethernet module
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Byte offset of the register from the ethernet base address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS > 1
static uint32_t tiva_ethin(struct tiva_driver_s *priv, int offset)
{
	return getreg32(priv->ld_base + offset);
}
#else
static inline uint32_t tiva_ethin(struct tiva_driver_s *priv, int offset)
{
	return getreg32(TIVA_ETHCON_BASE + offset);
}
#endif

/****************************************************************************
 * Function: tiva_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Parameters:
 *   arg     - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static void tiva_irqworker(FAR void *arg)
{
	FAR struct tiva_driver_s *priv = (FAR struct tiva_driver_s *)arg;

	DEBUGASSERT(priv);

	tiva_receive(priv);
}

/* ****************************************************************************/
static int tiva_handle_recv_interrupt(void)
{
	register FAR struct tiva_driver_s *priv = &g_lm3sdev[0];

	return work_queue(HPWORK, &priv->irqwork, tiva_irqworker, (FAR void *)priv, 0);
}


/****************************************************************************
 * Function: tiva_ethout
 *
 * Description:
 *   Write a register to the Ethernet module
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Byte offset of the register from the ethernet base address
 *   value  - The value to write the Ethernet register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS > 1
static void tiva_ethout(struct tiva_driver_s *priv, int offset, uint32_t value)
{
	putreg32(value, priv->ld_base + offset);
}
#else
static inline void tiva_ethout(struct tiva_driver_s *priv, int offset, uint32_t value)
{
	putreg32(value, TIVA_ETHCON_BASE + offset);
}
#endif

/****************************************************************************
 * Function: tiva_ethreset
 *
 * Description:
 *   Configure and reset the Ethernet module, leaving it in a disabled state.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_ethreset(struct tiva_driver_s *priv)
{
	irqstate_t flags;
	uint32_t regval;

#if TIVA_NETHCONTROLLERS > 1
#error "If multiple interfaces are supported, this function would have to be redesigned"
#endif

	/* Make sure that clocking is enabled for the Ethernet (and PHY) peripherals */

	flags = irqsave();
	regval = getreg32(TIVA_SYSCON_RCGC2);
	regval |= (SYSCON_RCGC2_EMAC0 | SYSCON_RCGC2_EPHY0);
	putreg32(regval, TIVA_SYSCON_RCGC2);
	nllvdbg("RCGC2: %08x\n", regval);

	/* Then take the Ethernet controller out of the reset state */

	regval &= ~(SYSCON_SRCR2_EMAC0 | SYSCON_SRCR2_EPHY0);
	putreg32(regval, TIVA_SYSCON_SRCR2);
	nllvdbg("SRCR2: %08x\n", regval);

	/* Wait just a bit, again.  If we touch the ethernet too soon, we may busfault. */

	up_mdelay(2);

	/* Enable Port F for Ethernet LEDs: LED0=Bit 3; LED1=Bit 2 */

#ifdef CONFIG_TIVA_ETHLEDS
	/* Configure the pins for the peripheral function */

	tiva_configgpio(GPIO_ETHPHY_LED0 | GPIO_STRENGTH_2MA | GPIO_PADTYPE_STD);
	tiva_configgpio(GPIO_ETHPHY_LED1 | GPIO_STRENGTH_2MA | GPIO_PADTYPE_STD);
#endif

	/* Disable all Ethernet controller interrupts */

	regval = tiva_ethin(priv, TIVA_MAC_IM_OFFSET);
	regval &= ~MAC_IM_ALLINTS;
	tiva_ethout(priv, TIVA_MAC_IM_OFFSET, regval);

	/* Clear any pending interrupts (shouldn't be any) */

	regval = tiva_ethin(priv, TIVA_MAC_RIS_OFFSET);
	tiva_ethout(priv, TIVA_MAC_IACK_OFFSET, regval);
	irqrestore(flags);
}

/****************************************************************************
 * Function: tiva_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int tiva_transmit(struct tiva_driver_s *priv)
{
	irqstate_t flags;
	uint32_t regval;
	uint8_t *dbuf;
	int pktlen;
	int bytesleft;
	int ret = -EBUSY;

	/* Verify that the hardware is ready to send another packet */

	flags = irqsave();
	if ((tiva_ethin(priv, TIVA_MAC_TR_OFFSET) & MAC_TR_NEWTX) == 0) {
		/* Increment statistics */

		EMAC_STAT(priv, tx_packets);
		tiva_dumppacket("Transmit packet", priv->ld_dev.d_buf, priv->ld_dev.d_len);

		/* Transfer the packet into the Tx FIFO.  The LS 16-bits of the first
		 * 32-bit word written to the Tx FIFO contains the Ethernet payload
		 * data length.  That is the full length of the message (d_len) minus
		 * the size of the Ethernet header (14).
		 */

		pktlen = priv->ld_dev.d_len;
		nllvdbg("Sending packet, pktlen: %d\n", pktlen);
		DEBUGASSERT(pktlen > ETH_HDRLEN);

		dbuf = priv->ld_dev.d_buf;
		regval = (uint32_t)(pktlen - 14);
		regval |= ((uint32_t)(*dbuf++) << 16);
		regval |= ((uint32_t)(*dbuf++) << 24);
		tiva_ethout(priv, TIVA_MAC_DATA_OFFSET, regval);

		/* Write all of the whole, 32-bit values in the middle of the packet */

		for (bytesleft = pktlen - 2; bytesleft > 3; bytesleft -= 4, dbuf += 4) {
			/* Transfer a whole word from the user buffer.  Note, the user
			 * buffer may be un-aligned.
			 */

			tiva_ethout(priv, TIVA_MAC_DATA_OFFSET, *(uint32_t *)dbuf);
		}

		/* Write the last, partial word in the FIFO */

		if (bytesleft > 0) {
			/* Write the last word */

			regval = 0;
			switch (bytesleft) {
			case 0:
			default:
				break;

			case 3:
				regval |= ((uint32_t)dbuf[2] << 16);
			case 2:
				regval |= ((uint32_t)dbuf[1] << 8);
			case 1:
				regval |= (uint32_t)dbuf[0];
				break;
			}

			tiva_ethout(priv, TIVA_MAC_DATA_OFFSET, regval);
		}

		/* Activate the transmitter */

		tiva_ethout(priv, TIVA_MAC_TR_OFFSET, MAC_TR_NEWTX);

		/* Setup the TX timeout watchdog (perhaps restarting the timer) */

		(void)wd_start(priv->ld_txtimeout, TIVA_TXTIMEOUT, tiva_txtimeout, 1, (uint32_t)priv);
		ret = OK;
	}

	irqrestore(flags);
	return ret;
}



/****************************************************************************
 * Function: tiva_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_receive(struct tiva_driver_s *priv)
{
	uint32_t regval;
	uint8_t *dbuf;
	int pktlen;
	int bytesleft;

	/* Loop while there are incoming packets to be processed */

	while ((tiva_ethin(priv, TIVA_MAC_NP_OFFSET) & MAC_NP_MASK) != 0) {
		/* Update statistics */

		EMAC_STAT(priv, rx_packets);

		/* Copy the data data from the hardware to priv->ld_dev.d_buf.  Set
		 * amount of data in priv->ld_dev.d_len
		 */

		dbuf = priv->ld_dev.d_buf;

		/* The packet frame length begins in the LS 16-bits of the first
		 * word from the FIFO followed by the Ethernet header beginning
		 * in the MS 16-bits of the first word.
		 *
		 * Pick off the packet length from the first word.  This packet length
		 * includes the len/type field (size 2) and the FCS (size 4).
		 */

		regval = tiva_ethin(priv, TIVA_MAC_DATA_OFFSET);
		pktlen = (int)(regval & 0x0000ffff);
		nllvdbg("Receiving packet, pktlen: %d\n", pktlen);

		/* Check if the pktlen is valid.  It should be large enough to hold
		 * an Ethernet header and small enough to fit entirely in the I/O
		 * buffer.Six is subtracted to acount for the 2-byte length/type
		 * and 4 byte FCS that are not copied into the packet.
		 */

		if (pktlen > (CONFIG_NET_ETH_MTU + 6) || pktlen <= (ETH_HDRLEN + 6)) {
			int wordlen;

			/* We will have to drop this packet */

			nlldbg("Bad packet size dropped (%d)\n", pktlen);
			EMAC_STAT(priv, rx_pktsize);

			/* The number of bytes and words left to read is pktlen - 4 (including,
			 * the final, possibly partial word) because we've already read 4 bytes.
			 */

			wordlen = (pktlen - 1) >> 2;

			/* Read and discard the remaining words in the FIFO */

			while (wordlen--) {
				(void)tiva_ethin(priv, TIVA_MAC_DATA_OFFSET);
			}

			/* Check for another packet */

			continue;
		}

		/* Save the first two bytes from the first word */

		*dbuf++ = (uint8_t)((regval >> 16) & 0xff);
		*dbuf++ = (uint8_t)((regval >> 24) & 0xff);

		/* Read all of the whole, 32-bit values in the middle of the packet.
		 * We've already read the length (2 bytes) plus the first two bytes
		 * of data.
		 */

		for (bytesleft = pktlen - 4; bytesleft > 7; bytesleft -= 4, dbuf += 4) {
			/* Transfer a whole word to the user buffer.  Note, the user
			 * buffer may be un-aligned.
			 */

			*(uint32_t *)dbuf = tiva_ethin(priv, TIVA_MAC_DATA_OFFSET);
		}

		/* Handle the last, partial word in the FIFO (0-3 bytes) and discard
		 * the 4-byte FCS.
		 */

		for (; bytesleft > 0; bytesleft -= 4) {
			/* Read the last word.  And transfer all but the last four
			 * bytes of the FCS into the user buffer.
			 */

			regval = tiva_ethin(priv, TIVA_MAC_DATA_OFFSET);
			switch (bytesleft) {
			default:
				break;

			case 7:
				dbuf[2] = (regval >> 16) & 0xff;
			case 6:
				dbuf[1] = (regval >> 8) & 0xff;
			case 5:
				dbuf[0] = regval & 0xff;
				break;
			}
		}

		/* Pass the packet length MINUS 2 bytes for the length and
		 * 4 bytes for the FCS.
		 */

		priv->ld_dev.d_len = pktlen - 6;
		tiva_dumppacket("Received packet", priv->ld_dev.d_buf, priv->ld_dev.d_len);

#ifdef CONFIG_NET_PKT
		/* When packet sockets are enabled, feed the frame into the packet tap */

		pkt_input(&priv->ld_dev);
#endif

		/* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
		if (ETHBUF->type == HTONS(ETHTYPE_IP)) {
			nllvdbg("IPv4 frame\n");

			/* Handle ARP on input then give the IPv4 packet to the network
			 * layer
			 */

			EMAC_STAT(priv, rx_ip);

			while (1) {
				int res = ethernetif_input(&priv->ld_dev);
				if (res == 0) {
					break;
				} else {
					nlldbg("trying to handle again\n");
				}
				usleep(2000);
			}

			/* If the above function invocation resulted in data that should be
			 * sent out on the network, the field  d_len will set to a value > 0.
			 */

			if (priv->ld_dev.d_len > 0) {
				/* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
				neighbor_out(&priv->ld_dev);
#endif

				/* And send the packet */

				tiva_transmit(priv);
			}
		}
#endif
#ifdef CONFIG_NET_IPv6
			if (ETHBUF->type == HTONS(ETHTYPE_IP6)) {
				nllvdbg("Iv6 frame\n");

				/* Give the IPv6 packet to the network layer */

				EMAC_STAT(priv, rx_ipv6);
				arp_ipin(&priv->ld_dev);
				ipv6_input(&priv->ld_dev);

				/* If the above function invocation resulted in data that should be
				 * sent out on the network, the field  d_len will set to a value > 0.
				 */

				if (priv->dev.d_len > 0) {
					/* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
					if (IFF_IS_IPv4(priv->ld_dev.d_flags)) {
						arp_out(&priv->ld_dev);
					} else
#endif
#ifdef CONFIG_NET_IPv6
					neighbor_out(&priv->ld_dev);
#endif

					/* And send the packet */

					tiva_transmit(priv);
				}
			}
#endif
#ifdef CONFIG_NET_ARP
			if (ETHBUF->type == htons(ETHTYPE_ARP)) {
				nllvdbg("ARP packet received (%02x)\n", ETHBUF->type);
				EMAC_STAT(priv, rx_arp);

				/* If the above function invocation resulted in data that should be
				 * sent out on the network, the field  d_len will set to a value > 0.
				 */

				while (1) {
					int res = ethernetif_input(&priv->ld_dev);

					if (res == 0) {
						break;
					} else {
						nlldbg("trying to handle again\n");
					}
					usleep(2000);
				}

					if (priv->ld_dev.d_len > 0) {
						tiva_transmit(priv);
					}
				} else
#endif
				{
					nlldbg("Unsupported packet type dropped (%02x)\n", htons(ETHBUF->type));
					EMAC_STAT(priv, rx_dropped);
				}
	}
}

/****************************************************************************
 * Function: tiva_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_txdone(struct tiva_driver_s *priv)
{
	/* Cancel the TX timeout */

	wd_cancel(priv->ld_txtimeout);

	/* Verify that the Tx FIFO is not in use.  The NEWTX bit initiates an
	 * Ethernet transmission once the packet has been placed in the TX FIFO.
	 * This bit is cleared once the transmission has been completed.  Since
	 * we get here because of of TXEMP which indicates that the packet was
	 * transmitted and that the TX FIFO is empty, NEWTX should always be zero
	 * at this point.
	 */

	DEBUGASSERT((tiva_ethin(priv, TIVA_MAC_TR_OFFSET) & MAC_TR_NEWTX) == 0)
}

/****************************************************************************
 * Function: tiva_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_interrupt(int irq, FAR void *context, FAR void *arg)
{
	register struct tiva_driver_s *priv;
	uint32_t ris;

#if TIVA_NETHCONTROLLERS > 1
#error "A mechanism to associate and interface with an IRQ is needed"
#else
	priv = &g_lm3sdev[0];
#endif

	/* Read the raw interrupt status register */

	ris = tiva_ethin(priv, TIVA_MAC_RIS_OFFSET);

	/* Clear all pending interrupts */

	tiva_ethout(priv, TIVA_MAC_IACK_OFFSET, ris);

	/* Check for errors */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
	if ((ris & MAC_RIS_TXER) != 0) {
		EMAC_STAT(priv, tx_errors);	/* Number of Tx errors */
	}

	if ((ris & MAC_RIS_FOV) != 0) {
		EMAC_STAT(priv, rx_ovrerrors);	/* Number of Rx FIFO overrun errors */
	}

	if ((ris & MAC_RIS_RXER) != 0) {
		EMAC_STAT(priv, rx_errors);	/* Number of Rx errors */
	}
#endif

	/* Handle (unmasked) interrupts according to status bit settings */

	ris &= tiva_ethin(priv, TIVA_MAC_IM_OFFSET);

	/* Is this an Rx interrupt (meaning that a packet has been received)? */

	if ((ris & MAC_RIS_RXINT) != 0) {
		/* Handle the incoming packet */

		EMAC_STAT(priv, rx_int);
		tiva_handle_recv_interrupt();
	}

	/* Is this an Tx interrupt (meaning that the Tx FIFO is empty)? */

	if ((ris & MAC_RIS_TXEMP) != 0) {
		/* Handle the complete of the transmission */

		EMAC_STAT(priv, tx_int);
		tiva_txdone(priv);
	}

	/* Enable Ethernet interrupts (perhaps excluding the TX done interrupt if
	 * there are no pending transmissions).
	 */

	return OK;
}

/****************************************************************************
 * Function: tiva_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_txtimeout(int argc, uint32_t arg, ...)
{
	struct tiva_driver_s *priv = (struct tiva_driver_s *)arg;

	/* Increment statistics */

	nlldbg("Tx timeout\n");
	EMAC_STAT(priv, tx_timeouts);

	/* Then reset the hardware */

	DEBUGASSERT(priv->ld_bifup);
	tiva_ifdown(&priv->ld_dev);
	tiva_ifup(&priv->ld_dev);
}

/****************************************************************************
 * Function: tiva_txpoll
 *
 * Description:
 *   The transmitter is available, check if it has any outgoing packets ready
 *   to send.  This is a callback from devif_poll().  devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int tiva_txpoll(struct netif dev)
{
	FAR struct tiva_driver_s *priv;
	priv = (FAR struct tiva_driver_s *)dev.d_private;

	DEBUGASSERT(priv->ld_dev.d_buf != NULL);

	/* If the polling resulted in data that should be sent out on the network,
	 * the field d_len is set to a value > 0.
	 */

	if (priv->ld_dev.d_len > 0) {
		/* Look up the destination MAC address and add it to the Ethernet
		 * header.
		 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
		else
#endif
		{
			neighbor_out(&priv->ld_dev);
		}
#endif							/* CONFIG_NET_IPv6 */

		/* Send the packet */

		tiva_transmit(priv);
	}

	/* If zero is returned, the polling will continue until all connections have
	 * been examined.
	 */

	return 0;
}

/****************************************************************************
 * Function: tiva_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int tiva_txavail(struct netif *dev)
{
	struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;
	irqstate_t flags;
	/* Ignore the notification if the interface is not yet up or if the Tx FIFO
	 * hardware is not available at this time.  The NEWTX bit initiates an
	 * Ethernet transmission once the packet has been placed in the TX FIFO.
	 * This bit is cleared once the transmission has been completed.  When the
	 * transmission completes, tiva_txdone() will be called and the Tx polling
	 * will occur at that time.
	 */

	flags = irqsave();
	if (priv->ld_bifup && (tiva_ethin(priv, TIVA_MAC_TR_OFFSET) & MAC_TR_NEWTX) == 0) {
		/* If the interface is up and we can use the Tx FIFO, then poll
		 * for new Tx data
		 */
		tiva_txpoll(priv->ld_dev);
	}

	irqrestore(flags);
	return OK;
}
/****************************************************************************
 * Function: tiva_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_polltimer(int argc, uint32_t arg, ...)
{
	struct tiva_driver_s *priv = (struct tiva_driver_s *)arg;

	/* Check if we can send another Tx packet now.  The NEWTX bit initiates an
	 * Ethernet transmission once the packet has been placed in the TX FIFO.
	 * This bit is cleared once the transmission has been completed.
	 *
	 * NOTE: This can cause missing poll cycles and, hence, some timing
	 * inaccuracies.
	 */

	if ((tiva_ethin(priv, TIVA_MAC_TR_OFFSET) & MAC_TR_NEWTX) == 0) {
		/* Setup the watchdog poll timer again */

		(void)wd_start(priv->ld_txpoll, TIVA_WDDELAY, tiva_polltimer, 1, arg);
	}
}

/****************************************************************************
 * Function: tiva_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_ifup(struct netif *dev)
{
	struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;
	irqstate_t flags;
	uint32_t regval;
	uint32_t div;

	nlldbg("Bringing up: %d.%d.%d.%d\n", dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff, (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

	/* Enable and reset the Ethernet controller */

	flags = irqsave();
	tiva_ethreset(priv);

	/* Set the management clock divider register for access to the PHY
	 * register set. The MDC clock is divided down from the system clock per:
	 *
	 *   MDCCLK_FREQUENCY = SYSCLK_FREQUENCY / (2 * (div + 1))
	 *   div = (SYSCLK_FREQUENCY / 2 / MDCCLK_FREQUENCY) - 1
	 *
	 * Where the maximum value for MDCCLK_FREQUENCY is 2,500,000.  We will
	 * add 1 to assure the max TIVA_MAX_MDCCLK is not exceeded.
	 */

	div = SYSCLK_FREQUENCY / 2 / TIVA_MAX_MDCCLK;
	tiva_ethout(priv, TIVA_MAC_MDV_OFFSET, div);
	nllvdbg("MDV:   %08x\n", div);

	/* Then configure the Ethernet Controller for normal operation
	 *
	 * Setup the transmit control register (Full duplex, TX CRC Auto Generation,
	 * TX Padding Enabled).
	 */

	regval = tiva_ethin(priv, TIVA_MAC_TCTL_OFFSET);
	regval &= ~TIVA_TCTCL_CLRBITS;
	regval |= TIVA_TCTCL_SETBITS;
	tiva_ethout(priv, TIVA_MAC_TCTL_OFFSET, regval);
	nllvdbg("TCTL:  %08x\n", regval);

	/* Setup the receive control register (Disable multicast frames, disable
	 * promiscuous mode, disable bad CRC rejection).
	 */

	regval = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
	regval &= ~TIVA_RCTCL_CLRBITS;
	regval |= TIVA_RCTCL_SETBITS;
	tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);
	nllvdbg("RCTL:  %08x\n", regval);

	/* Setup the time stamp configuration register */

#ifdef TIVA_ETHTS
	regval = tiva_ethin(priv, TIVA_MAC_TS_OFFSET);
#ifdef CONFIG_TIVA_TIMESTAMP
	regval |= MAC_TS_EN;
#else
	regval &= ~(MAC_TS_EN);
#endif
	tiva_ethout(priv, TIVA_MAC_TS_OFFSET, regval);
	nllvdbg("TS:    %08x\n", regval);
#endif

	/* Wait for the link to come up.  This following is not very conservative
	 * of system resources -- it really should wait gracefully on a semaphore
	 * and the interrupt handler should post the semaphore when LINKSTATUS is
	 * set
	 */

	nlldbg("Waiting for link\n");

	nlldbg("Link established\n");

	/* Reset the receive FIFO */

	regval = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
	regval |= MAC_RCTL_RSTFIFO;
	tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

	/* Enable the Ethernet receiver */

	regval = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
	regval |= MAC_RCTL_RXEN;
	tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

	/* Enable the Ethernet transmitter */

	regval = tiva_ethin(priv, TIVA_MAC_TCTL_OFFSET);
	regval |= MAC_TCTL_TXEN;
	tiva_ethout(priv, TIVA_MAC_TCTL_OFFSET, regval);

	/* Reset the receive FIFO (again) */

	regval = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
	regval |= MAC_RCTL_RSTFIFO;
	tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

	/* Enable the Ethernet interrupt */

#if TIVA_NETHCONTROLLERS > 1
	up_enable_irq(priv->irq);
#else
	up_enable_irq(TIVA_IRQ_ETHCON);
#endif

	/* Enable the Ethernet RX packet receipt interrupt */

	regval = tiva_ethin(priv, TIVA_MAC_IM_OFFSET);
	regval |= MAC_IM_RXINTM;
	tiva_ethout(priv, TIVA_MAC_IM_OFFSET, regval);

	/* Program the hardware with it's MAC address (for filtering) */

	regval = (uint32_t)priv->ld_dev.d_mac.ether_addr_octet[3] << 24 | (uint32_t)priv->ld_dev.d_mac.ether_addr_octet[2] << 16 | (uint32_t)priv->ld_dev.d_mac.ether_addr_octet[1] << 8 | (uint32_t)priv->ld_dev.d_mac.ether_addr_octet[0];
	tiva_ethout(priv, TIVA_MAC_IA0_OFFSET, regval);

	regval = (uint32_t)priv->ld_dev.d_mac.ether_addr_octet[5] << 8 | (uint32_t)priv->ld_dev.d_mac.ether_addr_octet[4];
	tiva_ethout(priv, TIVA_MAC_IA1_OFFSET, regval);

	/* Set and activate a timer process */

	(void)wd_start(priv->ld_txpoll, TIVA_WDDELAY, tiva_polltimer, 1, (uint32_t)priv);

	priv->ld_bifup = true;
	irqrestore(flags);
	return OK;
}

/****************************************************************************
 * Function: tiva_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.  The only way to restore normal
 *   behavior is to call tiva_ifup().
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_ifdown(struct netif *dev)
{
	struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;
	irqstate_t flags;
	uint32_t regval;

	nlldbg("Taking down: %d.%d.%d.%d\n", dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff, (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

	/* Cancel the TX poll timer and TX timeout timers */

	flags = irqsave();
	wd_cancel(priv->ld_txpoll);
	wd_cancel(priv->ld_txtimeout);

	/* Disable the Ethernet interrupt */

#if TIVA_NETHCONTROLLERS > 1
	up_disable_irq(priv->irq);
#else
	up_disable_irq(TIVA_IRQ_ETHCON);
#endif

	/* Disable all Ethernet controller interrupt sources */

	regval = tiva_ethin(priv, TIVA_MAC_IM_OFFSET);
	regval &= ~MAC_IM_ALLINTS;
	tiva_ethout(priv, TIVA_MAC_IM_OFFSET, regval);

	/* Reset the receive FIFO */

	regval = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
	regval |= MAC_RCTL_RSTFIFO;
	tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

	/* Disable the Ethernet receiver */

	regval = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
	regval &= ~MAC_RCTL_RXEN;
	tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

	/* Disable the Ethernet transmitter */

	regval = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
	regval &= ~MAC_TCTL_TXEN;
	tiva_ethout(priv, TIVA_MAC_TCTL_OFFSET, regval);

	/* Reset the receive FIFO (again) */

	regval = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
	regval |= MAC_RCTL_RSTFIFO;
	tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

	/* Clear any pending interrupts */

	regval = tiva_ethin(priv, TIVA_MAC_RIS_OFFSET);
	tiva_ethout(priv, TIVA_MAC_IACK_OFFSET, regval);

	/* The interface is now DOWN */

	priv->ld_bifup = false;
	irqrestore(flags);
	return OK;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tiva_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS > 1
int tiva_ethinitialize(int intf)
#else
static inline int tiva_ethinitialize(int intf)
#endif
{
	struct tiva_driver_s *priv = &g_lm3sdev[intf];
	int ret;

	/* Check if the Ethernet module is present */

	ndbg("Setting up eth%d\n", intf);

#if TIVA_NETHCONTROLLERS > 1
#error "This debug check only works with one interface"
#else
	DEBUGASSERT((getreg32(TIVA_SYSCON_DC4) & (SYSCON_DC4_EMAC0 | SYSCON_DC4_EPHY0)) == (SYSCON_DC4_EMAC0 | SYSCON_DC4_EPHY0));
#endif
	DEBUGASSERT((unsigned)intf < TIVA_NETHCONTROLLERS);

	/* Initialize the driver structure */

	memset(priv, 0, sizeof(struct tiva_driver_s));
	priv->ld_dev.d_ifup = tiva_ifup;	/* I/F down callback */
	priv->ld_dev.d_ifdown = tiva_ifdown;	/* I/F up (new IP address) callback */
	priv->ld_dev.d_txavail = tiva_txavail;	/* New TX data callback */
	priv->ld_dev.d_private = (void *)priv;	/* Used to recover private state from dev */

	/* Create a watchdog for timing polling for and timing of transmissions */

#if TIVA_NETHCONTROLLERS > 1
#error "A mechanism to associate base address an IRQ with an interface is needed"
	priv->ld_base = ? ? ;		/* Ethernet controller base address */
	priv->ld_irq = ? ? ;			/* Ethernet controller IRQ number */
#endif
	priv->ld_txpoll = wd_create();	/* Create periodic poll timer */
	priv->ld_txtimeout = wd_create();	/* Create TX timeout timer */

	/* If the board can provide us with a MAC address, get the address
	 * from the board now.  The MAC will not be applied until tiva_ifup()
	 * is called (and the MAC can be overwritten with a netdev ioctl call).
	 */

#ifdef CONFIG_TIVA_BOARDMAC
	tiva_ethernetmac(&priv->ld_dev.d_mac);
#endif

	/* Perform minimal, one-time initialization -- just reset the controller and
	 * leave it disabled.  The Ethernet controller will be reset and properly
	 * re-initialized each time tiva_ifup() is called.
	 */

	tiva_ethreset(priv);
	tiva_ifdown(&priv->ld_dev);

	/* Attach the IRQ to the driver */

#if TIVA_NETHCONTROLLERS > 1
	ret = irq_attach(priv->irq, tiva_interrupt, NULL);
#else
	ret = irq_attach(TIVA_IRQ_ETHCON, tiva_interrupt, NULL);
#endif
	if (ret != 0) {
		/* We could not attach the ISR to the IRQ */

		return -EAGAIN;
	}

	/* Register the device with the OS so that socket IOCTLs can be performed */

	struct ip4_addr ipaddr;
	struct ip4_addr netmask;
	struct ip4_addr gw;

	/* Start LWIP network thread */
	ipaddr.addr = inet_addr("0.0.0.0");
	netmask.addr = inet_addr("255.255.255.255");
	gw.addr = inet_addr("0.0.0.0");

	netif_set_default(&priv->ld_dev);

	netif_add(&priv->ld_dev, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);
	return OK;
}

/************************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one interface
 *   in the chip, then board-specific logic will have to provide this function to
 *   determine which, if any, Ethernet controllers should be initialized.
 *
 ************************************************************************************/

#if TIVA_NETHCONTROLLERS == 1
void up_netinitialize(void)
{
	(void)tiva_ethinitialize(0);
}
#endif

#endif	/* CONFIG_NET && CONFIG_TIVA_ETHERNET */
