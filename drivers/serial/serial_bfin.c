/*
 * U-boot - serial.c Blackfin Serial Driver
 *
 * Copyright (c) 2005-2008 Analog Devices Inc.
 *
 * Copyright (c) 2003	Bas Vermeulen <bas@buyways.nl>,
 *			BuyWays B.V. (www.buyways.nl)
 *
 * Based heavily on:
 * blkfinserial.c: Serial driver for BlackFin DSP internal USRTs.
 * Copyright(c) 2003	Metrowerks	<mwaddel@metrowerks.com>
 * Copyright(c)	2001	Tony Z. Kou	<tonyko@arcturusnetworks.com>
 * Copyright(c)	2001-2002 Arcturus Networks Inc. <www.arcturusnetworks.com>
 *
 * Based on code from 68328 version serial driver imlpementation which was:
 * Copyright (C) 1995       David S. Miller    <davem@caip.rutgers.edu>
 * Copyright (C) 1998       Kenneth Albanowski <kjahds@kjahds.com>
 * Copyright (C) 1998, 1999 D. Jeff Dionne     <jeff@uclinux.org>
 * Copyright (C) 1999       Vladimir Gurevich  <vgurevic@cisco.com>
 *
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * Licensed under the GPL-2 or later.
 */

/* Anomaly notes:
 *  05000086 - we don't support autobaud
 *  05000099 - we only use DR bit, so losing others is not a problem
 *  05000100 - we don't use the UART_IIR register
 *  05000215 - we poll the uart (no dma/interrupts)
 *  05000225 - no workaround possible, but this shouldnt cause errors ...
 *  05000230 - we tweak the baud rate calculation slightly
 *  05000231 - we always use 1 stop bit
 *  05000309 - we always enable the uart before we modify it in anyway
 *  05000350 - we always enable the uart regardless of boot mode
 *  05000363 - we don't support break signals, so don't generate one
 */

#include <common.h>
#include <post.h>
#include <watchdog.h>
#include <serial.h>
#include <linux/compiler.h>
#include <asm/blackfin.h>
#include <asm/serial.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_UART_CONSOLE

#ifdef CONFIG_DEBUG_SERIAL
static uart_lsr_t cached_lsr[256];
static uart_lsr_t cached_rbr[256];
static size_t cache_count;

/* The LSR is read-to-clear on some parts, so we have to make sure status
 * bits aren't inadvertently lost when doing various tests.  This also
 * works around anomaly 05000099 at the same time by keeping a cumulative
 * tally of all the status bits.
 */
static uart_lsr_t uart_lsr_save;
static uart_lsr_t uart_lsr_read(uint32_t uart_base)
{
	uart_lsr_t lsr = _lsr_read(pUART);
	uart_lsr_save |= (lsr & (OE|PE|FE|BI));
	return lsr | uart_lsr_save;
}
/* Just do the clear for everyone since it can't hurt. */
static void uart_lsr_clear(uint32_t uart_base)
{
	uart_lsr_save = 0;
	_lsr_write(pUART, -1);
}
#else
/* When debugging is disabled, we only care about the DR bit, so if other
 * bits get set/cleared, we don't really care since we don't read them
 * anyways (and thus anomaly 05000099 is irrelevant).
 */
static inline uart_lsr_t uart_lsr_read(uint32_t uart_base)
{
	return _lsr_read(pUART);
}
static void uart_lsr_clear(uint32_t uart_base)
{
	_lsr_write(pUART, -1);
}
#endif

static void uart_putc(uint32_t uart_base, const char c)
{
	/* send a \r for compatibility */
	if (c == '\n')
		serial_putc('\r');

	WATCHDOG_RESET();

	/* wait for the hardware fifo to clear up */
	while (!(uart_lsr_read(uart_base) & THRE))
		continue;

	/* queue the character for transmission */
	bfin_write(&pUART->thr, c);
	SSYNC();

	WATCHDOG_RESET();
}

static int uart_tstc(uint32_t uart_base)
{
	WATCHDOG_RESET();
	return (uart_lsr_read(uart_base) & DR) ? 1 : 0;
}

static int uart_getc(uint32_t uart_base)
{
	uint16_t uart_rbr_val;

	/* wait for data ! */
	while (!uart_tstc(uart_base))
		continue;

	/* grab the new byte */
	uart_rbr_val = bfin_read(&pUART->rbr);

#ifdef CONFIG_DEBUG_SERIAL
	/* grab & clear the LSR */
	uart_lsr_t uart_lsr_val = uart_lsr_read(uart_base);

	cached_lsr[cache_count] = uart_lsr_val;
	cached_rbr[cache_count] = uart_rbr_val;
	cache_count = (cache_count + 1) % ARRAY_SIZE(cached_lsr);

	if (uart_lsr_val & (OE|PE|FE|BI)) {
		printf("\n[SERIAL ERROR]\n");
		do {
			--cache_count;
			printf("\t%3zu: RBR=0x%02x LSR=0x%02x\n", cache_count,
				cached_rbr[cache_count], cached_lsr[cache_count]);
		} while (cache_count > 0);
		return -1;
	}
#endif
	uart_lsr_clear(uart_base);

	return uart_rbr_val;
}

#if CONFIG_POST & CONFIG_SYS_POST_UART
# define LOOP(x) x
#else
# define LOOP(x)
#endif

#if BFIN_UART_HW_VER < 4

LOOP(
static void uart_loop(uint32_t uart_base, int state)
{
	u16 mcr;

	/* Drain the TX fifo first so bytes don't come back */
	while (!(uart_lsr_read(uart_base) & TEMT))
		continue;

	mcr = bfin_read(&pUART->mcr);
	if (state)
		mcr |= LOOP_ENA | MRTS;
	else
		mcr &= ~(LOOP_ENA | MRTS);
	bfin_write(&pUART->mcr, mcr);
}
)

#else

LOOP(
static void uart_loop(uint32_t uart_base, int state)
{
	u32 control;

	/* Drain the TX fifo first so bytes don't come back */
	while (!(uart_lsr_read(uart_base) & TEMT))
		continue;

	control = bfin_read(&pUART->control);
	if (state)
		control |= LOOP_ENA | MRTS;
	else
		control &= ~(LOOP_ENA | MRTS);
	bfin_write(&pUART->control, control);
}
)

#endif

static inline void __serial_set_baud(uint32_t uart_base, uint32_t baud)
{
#ifdef CONFIG_DEBUG_EARLY_SERIAL
	serial_early_set_baud(uart_base, baud);
#else
	uint16_t divisor = (get_uart_clk() + (baud * 8)) / (baud * 16)
			- ANOMALY_05000230;

	/* Program the divisor to get the baud rate we want */
	serial_set_divisor(uart_base, divisor);
#endif
}

static void uart_puts(uint32_t uart_base, const char *s)
{
	while (*s)
		uart_putc(uart_base, *s++);
}

#define DECL_BFIN_UART(n) \
static int uart##n##_init(void) \
{ \
	const unsigned short pins[] = { _P_UART(n, RX), _P_UART(n, TX), 0, }; \
	peripheral_request_list(pins, "bfin-uart"); \
	uart_init(MMR_UART(n)); \
	__serial_set_baud(MMR_UART(n), gd->baudrate); \
	uart_lsr_clear(MMR_UART(n)); \
	return 0; \
} \
\
static int uart##n##_uninit(void) \
{ \
	return serial_early_uninit(MMR_UART(n)); \
} \
\
static void uart##n##_setbrg(void) \
{ \
	__serial_set_baud(MMR_UART(n), gd->baudrate); \
} \
\
static int uart##n##_getc(void) \
{ \
	return uart_getc(MMR_UART(n)); \
} \
\
static int uart##n##_tstc(void) \
{ \
	return uart_tstc(MMR_UART(n)); \
} \
\
static void uart##n##_putc(const char c) \
{ \
	uart_putc(MMR_UART(n), c); \
} \
\
static void uart##n##_puts(const char *s) \
{ \
	uart_puts(MMR_UART(n), s); \
} \
\
LOOP( \
static void uart##n##_loop(int state) \
{ \
	uart_loop(MMR_UART(n), state); \
} \
) \
\
struct serial_device bfin_serial##n##_device = { \
	.name   = "bfin_uart"#n, \
	.start  = uart##n##_init, \
	.stop   = uart##n##_uninit, \
	.setbrg = uart##n##_setbrg, \
	.getc   = uart##n##_getc, \
	.tstc   = uart##n##_tstc, \
	.putc   = uart##n##_putc, \
	.puts   = uart##n##_puts, \
	LOOP(.loop = uart##n##_loop) \
};

#ifdef UART0_RBR
DECL_BFIN_UART(0)
#endif
#ifdef UART1_RBR
DECL_BFIN_UART(1)
#endif
#ifdef UART2_RBR
DECL_BFIN_UART(2)
#endif
#ifdef UART3_RBR
DECL_BFIN_UART(3)
#endif

__weak struct serial_device *default_serial_console(void)
{
#if CONFIG_UART_CONSOLE == 0
	return &bfin_serial0_device;
#elif CONFIG_UART_CONSOLE == 1
	return &bfin_serial1_device;
#elif CONFIG_UART_CONSOLE == 2
	return &bfin_serial2_device;
#elif CONFIG_UART_CONSOLE == 3
	return &bfin_serial3_device;
#endif
}

void bfin_serial_initialize(void)
{
#ifdef UART0_RBR
	serial_register(&bfin_serial0_device);
#endif
#ifdef UART1_RBR
	serial_register(&bfin_serial1_device);
#endif
#ifdef UART2_RBR
	serial_register(&bfin_serial2_device);
#endif
#ifdef UART3_RBR
	serial_register(&bfin_serial3_device);
#endif
}

#ifdef CONFIG_DEBUG_EARLY_SERIAL
inline void uart_early_putc(uint32_t uart_base, const char c)
{
	/* send a \r for compatibility */
	if (c == '\n')
		uart_early_putc(uart_base, '\r');

	/* wait for the hardware fifo to clear up */
	while (!(_lsr_read(pUART) & THRE))
		continue;

	/* queue the character for transmission */
	bfin_write(&pUART->thr, c);
	SSYNC();
}

void uart_early_puts(const char *s)
{
	while (*s)
		uart_early_putc(UART_BASE, *s++);
}

/* Symbol for our assembly to call. */
void _serial_early_set_baud(uint32_t baud)
{
	serial_early_set_baud(UART_BASE, baud);
}

/* Symbol for our assembly to call. */
void _serial_early_init(void)
{
	serial_early_init(UART_BASE);
}
#endif

#elif defined(CONFIG_UART_MEM)

char serial_logbuf[CONFIG_UART_MEM];
char *serial_logbuf_head = serial_logbuf;

int serial_mem_init(void)
{
	serial_logbuf_head = serial_logbuf;
	return 0;
}

void serial_mem_setbrg(void)
{
}

int serial_mem_tstc(void)
{
	return 0;
}

int serial_mem_getc(void)
{
	return 0;
}

void serial_mem_putc(const char c)
{
	*serial_logbuf_head = c;
	if (++serial_logbuf_head == serial_logbuf + CONFIG_UART_MEM)
		serial_logbuf_head = serial_logbuf;
}

void serial_mem_puts(const char *s)
{
	while (*s)
		serial_putc(*s++);
}

struct serial_device bfin_serial_mem_device = {
	.name   = "bfin_uart_mem",
	.start  = serial_mem_init,
	.setbrg = serial_mem_setbrg,
	.getc   = serial_mem_getc,
	.tstc   = serial_mem_tstc,
	.putc   = serial_mem_putc,
	.puts   = serial_mem_puts,
};


__weak struct serial_device *default_serial_console(void)
{
	return &bfin_serial_mem_device;
}

void bfin_serial_initialize(void)
{
	serial_register(&bfin_serial_mem_device);
}
#endif /* CONFIG_UART_MEM */
