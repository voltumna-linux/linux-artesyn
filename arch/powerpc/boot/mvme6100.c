// SPDX-License-Identifier: GPL-2.0
/*
 * Motorola MVME6100 board boot wrapper code.
 *
 * Copyright 2008 Alessio Igor Bogani
 *
 * Based on prpmc2800.c by Mark A. Greer <mgreer@mvista.com>.
 * Incorporates initialization from arch/ppc/boot/simple/misc-mvme6100.S.
 */

#include <stddef.h>
#include "types.h"
#include "page.h"
#include "string.h"
#include "stdio.h"
#include "io.h"
#include "ops.h"
#include "mv64x60.h"

#define KB	1024U
#define MB	(KB*KB)

#define MV64x60_ETH_RX_QUEUE_CMD(port)	(0x2680 + ((port) << 10))

BSS_STACK(16*KB);

static u8 *bridge_base;

/*
 * Stop GigE Rx DMA engines on all 3 ports.
 *
 * Ported from arch/ppc/boot/simple/misc-mvme6100.S.  The firmware may
 * have left the DMA engines running; we must stop them before
 * decompressing the kernel to prevent spurious DMA activity that could
 * corrupt memory.
 */
static void mvme6100_stop_gige_dma(void)
{
	int port;

	for (port = 0; port < 3; port++) {
		out_le32((u32 *)(bridge_base + MV64x60_ETH_RX_QUEUE_CMD(port)),
			 0xff00);	/* Stop all Rx queues */
	}
}

/*
 * Flush and invalidate caches.
 *
 * Ported from arch/ppc/boot/simple/misc-mvme6100.S.  Touch and
 * invalidate the first 2MB of memory to ensure the cache is in a
 * known state before decompression.
 */
static void mvme6100_init_caches(void)
{
	unsigned long i;

	for (i = 0; i < 0x200000; i += 32) {
		asm volatile("dcbf %0,%1" : : "r"(0), "r"(i) : "memory");
		asm volatile("icbi %0,%1" : : "r"(0), "r"(i) : "memory");
	}
	asm volatile("sync" : : : "memory");
	asm volatile("isync" : : : "memory");
}

/*
 * Configure the MV64360 bridge windows for the MVME6100.
 *
 * This sets up CPU-to-PCI windows for both PCI buses, based on the
 * device tree ranges properties.
 */
static void mvme6100_bridge_setup(u32 mem_size)
{
	u32 v[12], enables, acc_bits;
	u32 pci_base_hi, pci_base_lo, size, buf[2];
	unsigned long cpu_base;
	int rc, i;
	void *devp, *mv64360_devp;
	u8 *bridge_pbase, is_coherent;
	struct mv64x60_cpu2pci_win *tbl;

	bridge_pbase = mv64x60_get_bridge_pbase();
	is_coherent = mv64x60_is_coherent();

	if (is_coherent)
		acc_bits = MV64x60_PCI_ACC_CNTL_SNOOP_WB
			| MV64x60_PCI_ACC_CNTL_SWAP_NONE
			| MV64x60_PCI_ACC_CNTL_MBURST_32_BYTES
			| MV64x60_PCI_ACC_CNTL_RDSIZE_32_BYTES;
	else
		acc_bits = MV64x60_PCI_ACC_CNTL_SNOOP_NONE
			| MV64x60_PCI_ACC_CNTL_SWAP_NONE
			| MV64x60_PCI_ACC_CNTL_MBURST_128_BYTES
			| MV64x60_PCI_ACC_CNTL_RDSIZE_256_BYTES;

	mv64x60_config_ctlr_windows(bridge_base, bridge_pbase, is_coherent);

	mv64360_devp = find_node_by_compatible(NULL, "marvell,mv64360");
	if (mv64360_devp == NULL)
		fatal("Error: Missing marvell,mv64360 device tree node\n\r");

	enables = in_le32((u32 *)(bridge_base + MV64x60_CPU_BAR_ENABLE));
	enables |= 0x0007fe00; /* Disable all cpu->pci windows */
	out_le32((u32 *)(bridge_base + MV64x60_CPU_BAR_ENABLE), enables);

	/* Configure PCI Bus 0 */
	devp = find_node_by_compatible(NULL, "marvell,mv64360-pci");
	if (devp != NULL) {
		mv64x60_config_pci_windows(bridge_base, bridge_pbase, 0, 0,
				mem_size, acc_bits);

		rc = getprop(devp, "ranges", v, sizeof(v));
		if (rc == sizeof(v)) {
			for (i = 0; i < 12; i += 6) {
				switch (v[i] & 0xff000000) {
				case 0x01000000: /* I/O */
					tbl = mv64x60_cpu2pci_io;
					break;
				case 0x02000000: /* MEM */
					tbl = mv64x60_cpu2pci_mem;
					break;
				default:
					continue;
				}

				pci_base_hi = v[i+1];
				pci_base_lo = v[i+2];
				cpu_base = v[i+3];
				size = v[i+5];

				buf[0] = cpu_base;
				buf[1] = size;

				if (!dt_xlate_addr(mv64360_devp, buf,
						sizeof(buf), &cpu_base))
					fatal("Error: Can't translate PCI 0"
						" address 0x%x\n\r",
						(u32)cpu_base);

				mv64x60_config_cpu2pci_window(bridge_base, 0,
						pci_base_hi, pci_base_lo,
						cpu_base, size, tbl);
			}
		}
	}

	enables &= ~0x00000600; /* Enable cpu->pci0 i/o, cpu->pci0 mem0 */
	out_le32((u32 *)(bridge_base + MV64x60_CPU_BAR_ENABLE), enables);

	/* Configure PCI Bus 1 */
	if (devp != NULL)
		devp = find_node_by_compatible(devp, "marvell,mv64360-pci");
	if (devp != NULL) {
		mv64x60_config_pci_windows(bridge_base, bridge_pbase, 1, 0,
				mem_size, acc_bits);

		rc = getprop(devp, "ranges", v, sizeof(v));
		if (rc == sizeof(v)) {
			for (i = 0; i < 12; i += 6) {
				switch (v[i] & 0xff000000) {
				case 0x01000000:
					tbl = mv64x60_cpu2pci_io;
					break;
				case 0x02000000:
					tbl = mv64x60_cpu2pci_mem;
					break;
				default:
					continue;
				}

				pci_base_hi = v[i+1];
				pci_base_lo = v[i+2];
				cpu_base = v[i+3];
				size = v[i+5];

				buf[0] = cpu_base;
				buf[1] = size;

				if (!dt_xlate_addr(mv64360_devp, buf,
						sizeof(buf), &cpu_base))
					fatal("Error: Can't translate PCI 1"
						" address 0x%x\n\r",
						(u32)cpu_base);

				mv64x60_config_cpu2pci_window(bridge_base, 1,
						pci_base_hi, pci_base_lo,
						cpu_base, size, tbl);
			}
		}

		enables &= ~0x00018000; /* Enable cpu->pci1 i/o, cpu->pci1 mem0 */
		out_le32((u32 *)(bridge_base + MV64x60_CPU_BAR_ENABLE),
				enables);
	}
}

/*
 * Device tree fixups.
 *
 * Detect actual memory size from the bridge controller and update
 * the device tree accordingly.
 */
static void mvme6100_fixups(void)
{
	u32 v[2], mem_size;
	void *devp;

	mem_size = mv64x60_get_mem_size(bridge_base);
	mvme6100_bridge_setup(mem_size);

	/* Set /memory/reg size */
	devp = finddevice("/memory");
	if (devp == NULL)
		fatal("Error: Missing /memory device tree node\n\r");
	v[0] = 0;
	v[1] = mem_size;
	setprop(devp, "reg", v, sizeof(v));
}

/*
 * Board reset.
 *
 * The MVME6100 resets by writing 0x80 to board status register 3
 * at physical address 0xf1100002.
 */
#define MVME6100_BOARD_STATUS_REG_3	0xf1100002
#define MVME6100_BOARD_RESET_MASK	0x80

static void mvme6100_reset(void)
{
	u8 *status_reg;

	udelay(5000000);

	status_reg = (u8 *)MVME6100_BOARD_STATUS_REG_3;
	out_8(status_reg, MVME6100_BOARD_RESET_MASK);

	for (;;)
		;
}

/*
 * Early debug output via NS16550 UART at 0xf1120000.
 * MOTLoad has already initialized this UART, so we can write directly.
 */
#define MVME6100_UART_BASE	((volatile u8 *)0xf1120000)
#define UART_THR	0	/* Transmit Holding Register */
#define UART_LSR	5	/* Line Status Register */
#define UART_LSR_THRE	0x20	/* THR Empty */

static void uart_putc(char c)
{
	while (!(MVME6100_UART_BASE[UART_LSR] & UART_LSR_THRE))
		;
	MVME6100_UART_BASE[UART_THR] = c;
}

static void uart_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			uart_putc('\r');
		uart_putc(*s++);
	}
}

void platform_init(unsigned long r3, unsigned long r4, unsigned long r5)
{
	u32 heapsize;
	unsigned long msr;

	uart_puts("MVME6100: platform_init entered\n");

	/* Disable external interrupts */
	asm volatile("mfmsr %0" : "=r"(msr));
	msr &= ~(1UL << 15);	/* Clear MSR_EE */
	asm volatile("sync; mtmsr %0; isync" : : "r"(msr));

	heapsize = 0x1000000 - (u32)_end;	/* 16M heap */
	simple_alloc_init(_end, heapsize, 32, 64);
	fdt_init(_dtb_start);

	uart_puts("MVME6100: bridge init\n");
	bridge_base = mv64x60_get_bridge_base();

	/* Ported from arch/ppc/boot/simple: early hardware init */
	mvme6100_init_caches();
	mvme6100_stop_gige_dma();

	platform_ops.fixups = mvme6100_fixups;
	platform_ops.exit = mvme6100_reset;

	uart_puts("MVME6100: serial console init\n");
	if (serial_console_init() < 0)
		uart_puts("MVME6100: serial_console_init failed\n");

	uart_puts("MVME6100: boot wrapper ready\n");
}
