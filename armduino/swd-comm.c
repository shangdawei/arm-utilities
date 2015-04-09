/* ARM/STM32 pseudo-serial over SWD client driver. */
/*
  This driver emulates a communication port (serial port) over the
  Single Wire Debug (SWD) interface on ARM processors.
  It interacts with 'stlink-terminal' on the host, a specifically written
  program that interacts with the STLink download/debugging interface
  found on STM8 and STM32VLDiscovery devices.
  These interfaces provide a good debug and trace interface, but lack
  a serial interface typically found on development boards.

  Written 2010-2011 by Donald Becker, Hugo Becker and William Carlson.
  This program may be used under the terms of the Gnu General Public License,
  (GPL) v2 or v3.  Distribution under other terms requires an explicit
  license from the authors.

Notes:
 The STLink is a miserable hassle to deal with.  See the host side of
 this system for details.
*/

/* $Vbase: 1.54 14 January 2011 00:25:11 becker$ */
static const char version_msg[] =
"ARM SWD pseudo-terminal interface $Id:$  Copyright Donald Becker";

/* Our internal protocol is based on initially communicating through the
 * DCRDR Debug Control Data Register.
 *
 * We first signal to the host that there is a client running on the
 * target microcontroller by setting a magic number in the DCRDR.
 * Alternately, we wait for the host to raise an interrupt and respond
 * with our magic number, which encodes the protocol version.  The host
 * verifies support for a compatible interface, then procedes to query
 * the client for the interface settings.
 *
 * For the basic memory based ring buffer interface, those settings are
 * base address and size of the Tx and Rx ring buffers.
 *
 * The payload data is sent through the ring buffers using a host driven
 * poll-for-not-zero approach.  The target
 * initializes both ring buffers with zeros. The host sends
 * data to the target by writing non-zero bytes to the Rx ring buffer,
 * then optionally raising an interrupt.  The target receives the data
 * by polling for a non-zero read from the queue head.  As it consumes
 * the bytes, it overwrites the location with a zero.  When the host runs
 * out of known-empty queue locations, it reads the next block of data
 * and extends its queue tail index by scanning for zeros.
 *
 * The target transmits by writing non-zero bytes to the Tx ring buffer.
 * The host similarly does a polling read and scans for the non-zero
 * locations.
 * 
 * A 0x00 byte (unlikely with command line text, common during data
 * transfer) is encoded as 0x80 followed by the count of inserted zeros.
 * The simplest client implementation just uses the sequence 0x80 0x01,
 * repeated for multiple zero bytes.
 */

/* A unique indicator for host commands in the DCRDR. */
#define STLINK_PSEUDO_MAGIC 0xDB196500

/* Our default interrupt vector.  The FSMC is rarely used, so we snarf it.
 * Omit the suffix from the name.  _IRQHander/_Intr is added by the macros. */
#if ! defined(PSEUDO_SERIAL_VECTOR)
#define PSEUDO_SERIAL_VECTOR FSMC
#endif
/* Must eval twice to effect a CPP paste. */
#define _CONCAT2(a, b) a ## b
#define _CONCAT1(a, b) _CONCAT2(a, b)
#define PSEUDO_SERIAL_INTR _CONCAT1(PSEUDO_SERIAL_VECTOR, _Intr)

#if defined(STM32)
#include <armduino.h>
#else
#warning This source file is for the STM32F100 series.
#endif

/* Information that we need to communicate to the host:
 * Interrupt vector we listen on
 * Addresses for communication queues
 */



/* We communicate with a sequence of bytes, thus do not need to deal with
 * with partity, character size and stop bits.
 * Set the size of the receive and transmit buffers.
 * The size must fit within the queue index range.
 * The transmit buffer should be able to buffer a whole line, but can
 * smaller at the cost of busy-waiting.
 */
#define SERIAL_RXBUF_SIZE 16
#define SERIAL_TXBUF_SIZE 128

/* Public statistics for the serial port - interrupt counts. */
volatile unsigned long serial_txbytes = 0;
volatile unsigned long serial_rxbytes = 0;

typedef unsigned char q_index;

/* Queue state structure for a single direction, two per UART. */
struct serial_rx_fifo {
	volatile q_index head;
	volatile q_index tail;
	unsigned char buf[SERIAL_RXBUF_SIZE];
} uart_rx;
struct serial_tx_fifo {
	volatile q_index head;
	volatile q_index tail;
	unsigned char buf[SERIAL_TXBUF_SIZE];
} uart_tx;

/* The STLink host generated interrupt.
 * This indicates that the host has sent characters, or has consumed
 * characters when the queue might have been full.
 * We can use an arbitrary unused interrupt level.
 */
ISREVAL(PSEUDO_SERIAL_VECTOR)
{
	static int intr_count = 0;

	/* For testing we just toggle the VLDiscovery Blue LED on PC8. */
/* Definitions for the Discovery board. */
#define LED_BLUE	(1<<8) // pin 8
#define LED_GREEN	(1<<9) // pin 9
/* Toggle VLDiscovery LEDs */
#define TOGGLE_LED GPIOC_ODR = PORTC ^ (LED_BLUE | LED_GREEN);
	TOGGLE_LED;
	DCRDR = STLINK_PSEUDO_MAGIC + intr_count++;

	return;
}

/* Get the next character from the serial input FIFO.
 * Return -1 if the FIFO is empty.
 */
int serial_getchar(void)
{
	unsigned char c;
	q_index i, j;

	i = uart_rx.tail;
	j = uart_rx.head;			/* Must be atomic. */
	if (i != j) {
		c = uart_rx.buf[i++];
		if (i >= sizeof(uart_rx.buf)) i = 0;
		uart_rx.tail = i;		/* Must be atomic. */
		return c;
	}
	return -1;
}

/*
 * Put character C on the serial transmit queue.
 * Return 0 on success, -1 if the queue is full.
 */
char serial_putchar(char c)
{
	q_index i, j;

	i = uart_tx.head + 1;
	if (i >= sizeof(uart_tx.buf)) i = 0;
	j = uart_tx.tail;			/* Must be atomic. */
	if (i == j)					/* Queue full, report failure. */
		return -1;
	uart_tx.buf[uart_tx.head] = c;
	uart_tx.head = i;			/* Must be atomic. */
	/* ToDo: Signal a non-empty Tx FIFO, enable TX buffer emptied interrupt. */
	return 0;
}


/* Configure serial communication.
 * With a hardware interface this would be setting the config registers.
 * With the SWD interface there is no baud rate, etc to config, just
 * checking if we have a host connection.
 */
void serial_setup(void)
{
	/* Re-initialize counts when called. */
	serial_txbytes = serial_rxbytes = 0;
	uart_rx.head = uart_tx.head = uart_rx.tail = uart_tx.tail = 0;

	DCRDR = STLINK_PSEUDO_MAGIC;
	/* Enable the pseduo-serial interrupt.
	 * Nothing will happen unless interrupts are globally enabled.
	 */
	INTR_SETENA(PSEUDO_SERIAL_INTR);

	return;
}



/*
 * Local variables:
 *  compile-command: "make swd-comm.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
