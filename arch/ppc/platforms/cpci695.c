/*
 * cpci695.c
 *
 * Board setup routines for the Motorola CPCI695 Board.
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2007 Motorola, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/ide.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/initrd.h>
#include <linux/bootmem.h>
#include <linux/mv643xx.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/time.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/smp.h>
#include <asm/todc.h>
#include <asm/open_pic.h>
#include <asm/i8259.h>
#include <asm/pci-bridge.h>
#include <asm/bootinfo.h>
#include <asm/mv64x60.h>
#include <asm/kgdb.h>

#include "cpci695.h"

extern char cmd_line[];
extern unsigned long total_memory;

extern void gen550_progress(char *, unsigned short);
extern void gen550_init(int, struct uart_port *);

static unsigned long cpci695_find_end_of_memory(void);

static struct mv64x60_handle bh;
static void __iomem *sram_base;

static void __iomem *board_reg_base;
static void __iomem *todc_base;

TODC_ALLOC();

/*
 * Motorola CPCI695 Board PCI interrupt routing.
 */

static int __init
cpci695_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	struct pci_controller *hose = pci_bus_to_hose(dev->bus->number);

	if (hose->index == 0) {
		static char pci_irq_table[][4] = {
			/*
			 *      PCI IDSEL/INTPIN->INTLINE
			 *         A   B   C   D
			 */
			/* IDSEL 29/13 (PMC 2 - secondary PCI agent) */
			{CPCI695_PCI0_INTC_IRQ, CPCI695_PCI0_INTD_IRQ,
			 CPCI695_PCI0_INTA_IRQ, CPCI695_PCI0_INTB_IRQ},
			/* IDSEL 30/14 (PMC 2) */
			{CPCI695_PCI0_INTC_IRQ, CPCI695_PCI0_INTD_IRQ,
			 CPCI695_PCI0_INTA_IRQ, CPCI695_PCI0_INTB_IRQ},
			/* IDSEL 31/15 (MV64360 #0) */
			{CPCI695_PCI0_INTD_IRQ, 0, 0, 0},
		};

		const long min_idsel = 13, max_idsel = 15, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else {
		static char pci_irq_table[][4] = {
			/*
			 *      PCI IDSEL/INTPIN->INTLINE
			 *         A   B   C   D
			 */
			/* IDSEL 28/12 (PMC 1 - secondary PCI agent) */
			{CPCI695_PCI1_INTB_IRQ, CPCI695_PCI1_INTC_IRQ,
			 CPCI695_PCI1_INTD_IRQ, CPCI695_PCI1_INTA_IRQ},
			/* IDSEL 29/13 (PMC 1) */
			{CPCI695_PCI1_INTB_IRQ, CPCI695_PCI1_INTC_IRQ,
			 CPCI695_PCI1_INTD_IRQ, CPCI695_PCI1_INTA_IRQ},
			/* IDSEL 30/14 Sentinel64 Bridge */
			{CPCI695_PCI1_INTC_IRQ, CPCI695_PCI1_INTD_IRQ,
			 CPCI695_PCI1_INTA_IRQ, CPCI695_PCI1_INTB_IRQ},
			/* IDSEL 31/15 (MV64360 #1) */
			{CPCI695_PCI1_INTD_IRQ, 0, 0, 0},
		};

		const long min_idsel = 12, max_idsel = 15, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}
}

#if 0
static int __init
cpci695_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
//        struct pci_controller   *hose = pci_bus_to_hose(dev->bus->number);
	struct pci_controller *hose = (struct pci_controller *)(dev->sysdata);
	if (hose->index == 0) {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE
		     *         A   B   C   D
		     */
		{

			{90, 90, 90, 90},	/* IDSEL 29 - PCI bus 0 */
			{90, 91, 88, 89},	/* IDSEL 30 - PCI bus 0 */
			{91, 91, 91, 91},	/* IDSEL 31 - PCI bus 0 */
		};

		const long min_idsel = 13, max_idsel = 15, irqs_per_slot = 4;

		return PCI_IRQ_TABLE_LOOKUP;
	} else {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE
		     *         A   B   C   D
		     */
		{
			{77, 78, 79, 76},	/* IDSEL 12 - PCI bus 1 */
			{77, 78, 79, 76},	/* IDSEL 13 - PCI bus 1 */
			{78, 79, 76, 77},	/* IDSEL 14 - PCI bus 1 */
			{79, 79, 79, 79},	/* IDSEL 15 - PCI bus 1 */
		};

		const long min_idsel = 12, max_idsel = 15, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}
}
#endif

static int cpci695_get_bus_speed(void)
{
	return 133333333;
}

static int cpci695_get_cpu_speed(void)
{
	unsigned long hid1_pce;
	unsigned long cpufreq = cpci695_get_bus_speed();

	hid1_pce = mfspr(HID1) >> 27;

	if ((hid1_pce > 3) && (hid1_pce < 20)) {
		cpufreq = cpufreq * hid1_pce / 2;
	} else if ((hid1_pce >= 20) && (hid1_pce < 31)) {
		cpufreq = cpufreq * (hid1_pce - 10);
	}

	return cpufreq;
}

#define MV_REG_WRITE(offset,data) *(volatile unsigned int *) (CONFIG_MV64X60_NEW_BASE + offset) = cpu_to_le32 (data)

static void __init cpci695_cleanup_bits(void)
{
	/*
	 * Force firmware, for some reason, enables a bunch of intr sources
	 * for the PCI INT output pins.  Must mask them off b/c the PCI0/1
	 * Int pins are wired to INTD# for their respective buses.
	 */

	MV_REG_WRITE(MV64360_IC_CPU0_INTR_MASK_LO, 0x0);
	MV_REG_WRITE(MV64360_IC_CPU0_INTR_MASK_HI, 0x0);
	MV_REG_WRITE(MV64360_IC_CPU1_INTR_MASK_LO, 0x0);
	MV_REG_WRITE(MV64360_IC_CPU1_INTR_MASK_HI, 0x0);
	MV_REG_WRITE(MV64360_IC_INT0_MASK_LO, 0x0);
	MV_REG_WRITE(MV64360_IC_INT0_MASK_HI, 0x0);
	MV_REG_WRITE(MV64360_IC_INT1_MASK_LO, 0x0);
	MV_REG_WRITE(MV64360_IC_INT1_MASK_HI, 0x0);
	MV_REG_WRITE(MV64x60_PCI0_ERR_MASK, 0x0);
	MV_REG_WRITE(MV64x60_PCI1_ERR_MASK, 0x0);
	MV_REG_WRITE(MV64x60_PCI0_ERR_SERR_MASK, 0x0);
	MV_REG_WRITE(MV64x60_PCI1_ERR_SERR_MASK, 0x0);
}

static void __init cpci695_setup_bridge(void)
{
	struct mv64x60_setup_info si;
	int i;

	memset(&si, 0, sizeof(si));

	si.phys_reg_base = CONFIG_MV64X60_NEW_BASE;

	si.pci_0.enable_bus = 1;

	si.pci_0.pci_io.cpu_base = CPCI695_PCI_0_IO_START_PROC;
	si.pci_0.pci_io.pci_base_hi = 0;
	si.pci_0.pci_io.pci_base_lo = CPCI695_PCI_0_IO_START;
	si.pci_0.pci_io.size = CPCI695_PCI_0_IO_SIZE;
	si.pci_0.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_0.pci_mem[0].cpu_base = CPCI695_PCI_0_MEM_START_PROC;
	si.pci_0.pci_mem[0].pci_base_hi = 0;
	si.pci_0.pci_mem[0].pci_base_lo = CPCI695_PCI_0_MEM_START;
	si.pci_0.pci_mem[0].size = CPCI695_PCI_0_MEM_SIZE;
	si.pci_0.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_0.pci_cmd_bits = 0;
	si.pci_0.latency_timer = 0x80;

	si.pci_1.enable_bus = 1;

	si.pci_1.pci_io.cpu_base = CPCI695_PCI_1_IO_START_PROC;
	si.pci_1.pci_io.pci_base_hi = 0;
	si.pci_1.pci_io.pci_base_lo = CPCI695_PCI_1_IO_START;
	si.pci_1.pci_io.size = CPCI695_PCI_1_IO_SIZE;
	si.pci_1.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_1.pci_mem[0].cpu_base = CPCI695_PCI_1_MEM_START_PROC;
	si.pci_1.pci_mem[0].pci_base_hi = 0;
	si.pci_1.pci_mem[0].pci_base_lo = CPCI695_PCI_1_MEM_START;
	si.pci_1.pci_mem[0].size = CPCI695_PCI_1_MEM_SIZE;
	si.pci_1.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_1.pci_cmd_bits = 0;
	si.pci_1.latency_timer = 0x80;

	for (i = 0; i < MV64x60_CPU2MEM_WINDOWS; i++) {
#if defined(CONFIG_NOT_COHERENT_CACHE)
		si.cpu_prot_options[i] = 0;
		si.enet_options[i] = MV64360_ENET2MEM_SNOOP_NONE;
		si.mpsc_options[i] = MV64360_MPSC2MEM_SNOOP_NONE;
		si.idma_options[i] = MV64360_IDMA2MEM_SNOOP_NONE;

		si.pci_1.acc_cntl_options[i] =
		    MV64360_PCI_ACC_CNTL_SNOOP_NONE |
		    MV64360_PCI_ACC_CNTL_SWAP_NONE |
		    MV64360_PCI_ACC_CNTL_MBURST_128_BYTES |
		    MV64360_PCI_ACC_CNTL_RDSIZE_256_BYTES;
#else
		si.cpu_prot_options[i] = 0;

		si.enet_options[i] = MV64360_ENET2MEM_SNOOP_WB;
		si.mpsc_options[i] = MV64360_MPSC2MEM_SNOOP_WB;
		si.idma_options[i] = MV64360_IDMA2MEM_SNOOP_WB;

		si.pci_0.acc_cntl_options[i] =
		    MV64360_PCI_ACC_CNTL_SNOOP_WB |
		    MV64360_PCI_ACC_CNTL_SWAP_NONE |
		    MV64360_PCI_ACC_CNTL_MBURST_32_BYTES |
		    MV64360_PCI_ACC_CNTL_RDSIZE_32_BYTES;

		si.pci_1.acc_cntl_options[i] =
		    MV64360_PCI_ACC_CNTL_SNOOP_WB |
		    MV64360_PCI_ACC_CNTL_SWAP_NONE |
		    MV64360_PCI_ACC_CNTL_MBURST_32_BYTES |
		    MV64360_PCI_ACC_CNTL_RDSIZE_32_BYTES;
#endif
	}

	if (mv64x60_init(&bh, &si))
		printk(KERN_WARNING "Bridge initialization failed.\n");

	if (ppc_md.progress)
		ppc_md.progress("cpci695_setuparch: after mv64x60_init", 0);

	pci_dram_offset = 0;	/* sys mem at same addr on PCI & cpu bus */
	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = cpci695_map_irq;
	ppc_md.pci_exclude_device = mv64x60_pci_exclude_device;

	mv64x60_set_bus(&bh, 0, 0);
	if (ppc_md.progress)
		ppc_md.progress("cpci695_setuparch: after set_bus", 0);
	bh.hose_a->first_busno = 0;
	bh.hose_a->last_busno = 0xff;
	bh.hose_a->last_busno = pciauto_bus_scan(bh.hose_a, 0);

	bh.hose_b->first_busno = bh.hose_a->last_busno + 1;
	mv64x60_set_bus(&bh, 1, bh.hose_b->first_busno);
	bh.hose_b->last_busno = 0xff;
	bh.hose_b->last_busno = pciauto_bus_scan(bh.hose_b,
						 bh.hose_b->first_busno);

	return;
}

/* Bridge & platform setup routines */
void __init cpci695_intr_setup(void)
{
	if (ppc_md.progress)
		ppc_md.progress("cpci695_intr_setup: enter", 0);

	/*
	 * Configure the following MPP pins to indicate a level
	 * triggered interrupt
	 * MPP9  - UART0  (active high)
	 * MPP10 - UART1  (active high)
	 * MPP12 - PCI0 Slot 0 (active low)
	 * MPP13 - PCI0 Slot 1 (active low)
	 * MPP14 - PCI0 Slot 2 (active low)
	 * MPP15 - PCI0 Slot 3 (active low)
	 * MPP24 - PCI1 Slot 0 (active low)
	 * MPP25 - PCI1 Slot 1 (active low)
	 * MPP26 - PCI1 Slot 2 (active low)
	 * MPP27 - PCI1 Slot 3 (active low)
	 */

#define GPP_EXTERNAL_INTERRUPTS \
                ((BIT9)  | (BIT10) |  \
                 (BIT12) | (BIT13) | (BIT14) | (BIT15) | \
                 (BIT24) | (BIT25) | (BIT26) | (BIT27))

	/* Route MPP interrupt inputs to GPP */
	mv64x60_clr_bits(&bh, MV64x60_MPP_CNTL_1, 
			BIT7 | BIT6 | BIT5 | BIT4 |	/* MPP 9 */
			BIT11 | BIT10 | BIT9 | BIT8 |	/* MPP 10 */
			BIT19 | BIT18 | BIT17 | BIT16 |	/* MPP 12 */
			BIT23 | BIT22 | BIT21 | BIT20 |	/* MPP 13 */
			BIT27 | BIT26 | BIT25 | BIT24 |	/* MPP 14 */
			BIT31 | BIT30 | BIT29 | BIT28);	/* MPP 15 */

	mv64x60_clr_bits(&bh, MV64x60_MPP_CNTL_3,
			 BIT0 | BIT1 | BIT2 | BIT3 |
			 BIT4 | BIT5 | BIT6 | BIT7 |
			 BIT8 | BIT9 | BIT10 | BIT11 |
			 BIT12 | BIT13 | BIT14 | BIT15);

	/*
	 * Define GPP 12-15, 24-27 interrupt polarity as active low
	 * input signal and level triggered
	 */
	mv64x60_clr_bits(&bh, MV64x60_GPP_IO_CNTL, GPP_EXTERNAL_INTERRUPTS);
	mv64x60_set_bits(&bh, MV64x60_GPP_LEVEL_CNTL, GPP_EXTERNAL_INTERRUPTS);
	mv64x60_clr_bits(&bh, MV64x60_GPP_LEVEL_CNTL, BIT9 | BIT10);

	/* Config GPP interrupt controller to respond to level trigger */
	mv64x60_set_bits(&bh, MV64x60_COMM_ARBITER_CNTL, BIT10);

	/* Erratum FEr PCI-#8 */
	mv64x60_clr_bits(&bh, MV64x60_PCI0_CMD, (1 << 5) | (1 << 9));
	mv64x60_clr_bits(&bh, MV64x60_PCI1_CMD, (1 << 5) | (1 << 9));

	/* Clear any pending interrupts for these inputs and enable them */
	mv64x60_write(&bh, MV64x60_GPP_INTR_CAUSE, ~GPP_EXTERNAL_INTERRUPTS);
	mv64x60_set_bits(&bh, MV64x60_GPP_INTR_MASK, GPP_EXTERNAL_INTERRUPTS);

	/* Clear any pending interrupts for these inputs */
	mv64x60_set_bits(&bh, MV64360_IC_CPU1_INTR_MASK_LO, BIT27 | BIT25);

	if (ppc_md.progress)
		ppc_md.progress("cpci695_intr_setup: exit", 0);
}

void __init cpci695_setup_peripherals(void)
{
	if (ppc_md.progress)
		ppc_md.progress("cpci695_setup_peripherals: enter", 0);

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2BOOT_WIN,
				 CPCI695_BOOT_FLASH_BASE,
				 CPCI695_BOOT_FLASH_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2BOOT_WIN);

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_0_WIN,
				 CPCI695_BOARD_REG_BASE, CPCI695_BOARD_REG_SIZE,
				 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_0_WIN);

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_1_WIN,
				 CPCI695_TODC_BASE, CPCI695_TODC_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_1_WIN);

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_2_WIN,
				 CPCI695_IPMI_BASE, CPCI695_IPMI_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_2_WIN);

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_3_WIN,
				 CPCI695_USER_FLASH_BASE,
				 CPCI695_USER_FLASH_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_3_WIN);

	/* Set up internal SRAM window */
	mv64x60_set_32bit_window(&bh, MV64x60_CPU2SRAM_WIN,
				 CPCI695_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE,
				 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2SRAM_WIN);
	sram_base = ioremap(CPCI695_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE);

	/* Set up Enet->SRAM window */
	mv64x60_set_32bit_window(&bh, MV64x60_ENET2MEM_4_WIN,
				 CPCI695_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE,
				 0x2);
	bh.ci->enable_window_32bit(&bh, MV64x60_ENET2MEM_4_WIN);

	/* Give enet r/w access to memory region */
	mv64x60_set_bits(&bh, MV64360_ENET2MEM_ACC_PROT_0, (0x3 << (4 << 1)));
	mv64x60_set_bits(&bh, MV64360_ENET2MEM_ACC_PROT_1, (0x3 << (4 << 1)));
	mv64x60_set_bits(&bh, MV64360_ENET2MEM_ACC_PROT_2, (0x3 << (4 << 1)));

	mv64x60_clr_bits(&bh, MV64x60_PCI0_PCI_DECODE_CNTL, (1 << 3));
	mv64x60_clr_bits(&bh, MV64x60_PCI1_PCI_DECODE_CNTL, (1 << 3));
	/* Turn off timer/counters */
	mv64x60_clr_bits(&bh, MV64x60_TIMR_CNTR_0_3_CNTL,
			 ((1 << 0) | (1 << 8) | (1 << 16) | (1 << 24)));

//	mv64x60_set_bits(&bh, MV64x60_PCI0_ARBITER_CNTL, (1<<31));
//	mv64x60_set_bits(&bh, MV64x60_PCI1_ARBITER_CNTL, (1<<31));

	/* Only 1 CPU */
	mv64x60_set_bits(&bh, MV64x60_CPU_MASTER_CNTL, (1 << 9));

#if defined(CONFIG_NOT_COHERENT_CACHE)
	mv64x60_write(&bh, MV64360_SRAM_CONFIG, 0x00160000);
#else
	mv64x60_write(&bh, MV64360_SRAM_CONFIG, 0x001600b2);
#endif

	/*
	 * Set the SRAM to 0. Note that this may generate parity errors on
	 * internal data path in SRAM since it is the first time accessing it
	 * (if firmware hasn't done it already).
	 */
	memset(sram_base, 0, MV64360_SRAM_SIZE);

	/* set up PCI interrupt controller */
	cpci695_intr_setup();

	todc_base = ioremap(CPCI695_TODC_BASE, CPCI695_TODC_SIZE);
	TODC_INIT(TODC_TYPE_MK48T35, 0, 0, todc_base, 8);

	if (ppc_md.progress)
		ppc_md.progress("cpci695_setup_peripherals: exit", 0);
}

#if defined(CONFIG_SERIAL_8250)
static void __init cpci695_early_serial_map(void)
{
	struct uart_port serial_req;

	memset(&serial_req, 0, sizeof(serial_req));
	serial_req.uartclk = CPCI695_BASE_BAUD;
	serial_req.irq = CPCI695_UART_0_IRQ;
	serial_req.line = 0;
	serial_req.flags = STD_COM_FLAGS;
	serial_req.iotype = UPIO_MEM;
	serial_req.membase = (u8 *) CPCI695_SERIAL_0_IOBASE;
	serial_req.regshift = 0;

#ifdef	CONFIG_SERIAL_TEXT_DEBUG
	gen550_init(0, &serial_req);
#endif

	if (early_serial_setup(&serial_req) != 0) {
		printk("Early serial init of port 0 failed\n");
	}

	/* Assume early_serial_setup() doesn't modify serial_req */
	serial_req.line = 1;
	serial_req.irq = CPCI695_UART_1_IRQ;
	serial_req.membase = (u8 *) CPCI695_SERIAL_1_IOBASE;

#ifdef	CONFIG_SERIAL_TEXT_DEBUG
	gen550_init(1, &serial_req);
#endif

	if (early_serial_setup(&serial_req) != 0) {
		printk("Early serial init of port 1 failed\n");
	}
}
#endif

static void __init cpci695_init_caches(void)
{
	uint val;

	/* Enable L2  */
	val = _get_L2CR();
	val |= L2CR_L2E;
	_set_L2CR(val);
}

static void __init cpci695_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("cpci695_setup_arch: enter", 0);

	loops_per_jiffy = 50000000 / HZ;

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else
#endif
#ifdef	CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;
#else
		ROOT_DEV = Root_SDA2;
#endif

	cpci695_init_caches();

	/* Make sure WatchDog is disabled */
	board_reg_base =
	    ioremap(CPCI695_BOARD_REG_BASE, CPCI695_BOARD_REG_SIZE);

	out_8(board_reg_base + CPCI695_FAILSAFE_CTRL,
	      in_8(board_reg_base + CPCI695_FAILSAFE_CTRL) | 0x01);
	printk(KERN_INFO "CPCI695 WATCHDOG: Disabling the Failsafe Watchdog\n");
	iounmap(board_reg_base);

#ifdef CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif

	cpci695_cleanup_bits();

	if (ppc_md.progress)
		ppc_md.progress("cpci695_setup_arch: find_bridges", 0);

	cpci695_setup_bridge();

	if (ppc_md.progress)
		ppc_md.progress("cpci695_setup_arch: find_bridges done", 0);

	cpci695_setup_peripherals();

	printk("Motorola Computer Group CPCI695 Board\n");
	printk("CPCI695 port (C) 2003-2006 Motorola, Inc.\n");

	if (ppc_md.progress)
		ppc_md.progress("cpci695_setup_arch: exit", 0);

	return;
}

unsigned long __init cpci695_find_end_of_memory(void)
{
	return mv64x60_get_mem_size(CONFIG_MV64X60_NEW_BASE,
				    MV64x60_TYPE_MV64360);
}

/*
 * Set BAT 3 to map 0xf0000000 to end of physical memory space.
 */
static __inline__ void cpci695_set_bat(void)
{
	mb();
	mtspr(SPRN_DBAT3U, 0xf0001ffe);
	mtspr(SPRN_DBAT3L, 0xf000002a);
	mb();

	return;
}

static void __init cpci695_map_io(void)
{
	io_block_mapping(CONFIG_MV64X60_NEW_BASE, CONFIG_MV64X60_NEW_BASE,
			 0x01000000, _PAGE_IO);
}

static void cpci695_restart(char *cmd)
{
	volatile ulong i = 10000000;
	u8 *sw_reset_reg_addr;

	local_irq_disable();
	sw_reset_reg_addr = ioremap(CPCI695_SW_RESET_REG, 1);
	writeb(CPCI695_SW_RESET_MASK, sw_reset_reg_addr);

	while (i-- > 0) ;
	panic("restart failed\n");
}

static void cpci695_halt(void)
{
	local_irq_disable();
	while (1) ;
	/* NOTREACHED */
}

static void cpci695_power_off(void)
{
	cpci695_halt();
	/* NOTREACHED */
}

static int cpci695_show_cpuinfo(struct seq_file *m)
{
	uint tmp;
	u8 reg_value;
	char buff[16] = "";

	seq_printf(m, "machine\t\t: CPCI695\n");
	tmp = mfspr(SPRN_PVR);
	seq_printf(m, "PVID\t\t: 0x%x, vendor: %s\n",
		   tmp, (tmp & (1 << 15) ? "IBM" : "Motorola"));

	tmp = mfspr(SPRN_HID0);
	seq_printf(m, "HID0\t\t: 0x%08x\n", tmp);
	tmp = mfspr(SPRN_HID1);
	seq_printf(m, "HID1\t\t: 0x%08x\n", tmp);
	tmp = mfspr(SPRN_L2CR);
	seq_printf(m, "L2CR\t\t: 0x%08x\n", tmp);

	seq_printf(m, "\nBOARD INFORMATION:\n\n");

	reg_value = in_8(board_reg_base + CPCI695_PCIX_STATUS);
	seq_printf(m, "PCI Bus PCI-X\t\t: %s\n",
		   ((reg_value & CPCI695_PCIX_STATUS_PCIX) ?
		    "PCI-X Mode" : "PCI Mode"));
	switch (reg_value & CPCI695_PCI_BUS_0_SPD_MASK) {
	case CPCI695_PCI_BUS_0_SPD_133:
		sprintf(buff, "133 MHz");
		break;
	case CPCI695_PCI_BUS_0_SPD_100:
		sprintf(buff, "100 MHz");
		break;
	case CPCI695_PCI_BUS_0_SPD_66:
		sprintf(buff, "66 MHz");
		break;
	case CPCI695_PCI_BUS_0_SPD_33:
	default:
		sprintf(buff, "33 MHz");
		break;
	}
	seq_printf(m, "PCI Bus 0 Speed\t\t: %s\n", buff);
	seq_printf(m, "PCI Bus 1 Speed\t\t: %s\n",
		   ((reg_value & CPCI695_PCI_BUS_1_SPD_MASK) ?
		    "66 MHz" : "33 MHz"));

	return 0;
}

void cpci695_get_mac(int port_num, char *mac_addr)
{
	if (port_num == 0)
		memcpy(mac_addr, (todc_base + CPCI695_MAC_OFFSET), 6);
	else if (port_num == 1)
		memcpy(mac_addr, (todc_base + CPCI695_MAC_OFFSET + 6), 6);
	else if (port_num == 2)
		memcpy(mac_addr, (todc_base + CPCI695_MAC_OFFSET + 12), 6);
}

#if defined(CONFIG_MV643XX_ETH)
static void __init cpci695_fixup_eth_pdata(struct platform_device *pdev)
{
	struct mv643xx_eth_platform_data *eth_pd;
	static u16 phy_addr[] = {
		CPCI695_ETH0_PHY_ADDR,
		CPCI695_ETH1_PHY_ADDR,
		CPCI695_ETH2_PHY_ADDR,
	};

	eth_pd = pdev->dev.platform_data;
	eth_pd->force_phy_addr = 1;
	eth_pd->phy_addr = phy_addr[pdev->id];
	eth_pd->tx_queue_size = CPCI695_ETH_TX_QUEUE_SIZE;
	eth_pd->rx_queue_size = CPCI695_ETH_RX_QUEUE_SIZE;
#ifdef CONFIG_MV64X60_USE_SRAM
	if (pdev->id == 0) {
		eth_pd->rx_sram_addr = CPCI695_INTERNAL_SRAM_BASE;
		eth_pd->rx_sram_size = 16000;
		eth_pd->tx_sram_addr = CPCI695_INTERNAL_SRAM_BASE + 16000;
		eth_pd->tx_sram_size = 48000;
	} else {
		eth_pd->rx_sram_addr = CPCI695_INTERNAL_SRAM_BASE + 64000;
		eth_pd->rx_sram_size = 16000;
		eth_pd->tx_sram_addr = CPCI695_INTERNAL_SRAM_BASE + 80000;
		eth_pd->tx_sram_size = 48000;
	}
#endif
}
#endif

#if defined(CONFIG_I2C_MV64XXX)
static void __init cpci695_fixup_i2c_pdata(struct platform_device *pdev)
{
	struct mv64xxx_i2c_pdata *pdata;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return;

		pdev->dev.platform_data = pdata;
	}

	/* divisors M=8, N=3 for 100kHz I2C from 133MHz system clock */
	pdata->freq_m = 8;
	pdata->freq_n = 3;
	pdata->timeout = 500;
	pdata->retries = 3;
}
#endif

static int cpci695_platform_notify(struct device *dev)
{
	static struct {
		char *bus_id;
		void ((*rtn) (struct platform_device * pdev));
	} dev_map[] = {
#if defined(CONFIG_MV643XX_ETH)
		{ MV643XX_ETH_NAME ".0", cpci695_fixup_eth_pdata },
		{ MV643XX_ETH_NAME ".1", cpci695_fixup_eth_pdata },
		{ MV643XX_ETH_NAME ".2", cpci695_fixup_eth_pdata },
#endif
	};
	struct platform_device *pdev;
	int i;

	if (dev && dev->bus_id)
		for (i = 0; i < ARRAY_SIZE(dev_map); i++)
			if (!strncmp(dev->bus_id, dev_map[i].bus_id,
				     BUS_ID_SIZE)) {

				pdev = container_of(dev,
						    struct platform_device,
						    dev);
				dev_map[i].rtn(pdev);
			}
	return 0;
}

static void __init cpci695_pci_fixups(void)
{
	struct pci_dev *dev = NULL;
	struct pci_dev *pdc20269;

	u16 val;
	u8 progif;

	for_each_pci_dev(dev) {
		pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE, 8);
		pci_write_config_byte(dev, PCI_LATENCY_TIMER, 0x80);
		if ((dev->class >> 8) == PCI_CLASS_BRIDGE_PCI)
			pci_write_config_byte(dev, PCI_SEC_LATENCY_TIMER, 0x80);
		pci_read_config_word(dev, PCI_COMMAND, &val);
		val |= PCI_COMMAND_INVALIDATE;
		pci_write_config_word(dev, PCI_COMMAND, val);
	}
	if ((pdc20269 = pci_find_device(PCI_VENDOR_ID_PROMISE,
					PCI_DEVICE_ID_PROMISE_20269, 0)) &&
	    (pdc20269->class & 5) != 5) {
		printk(KERN_INFO
		       "CPCI695: Switching PDC20269 IDE to PCI native mode\n");
		/* Enable PDC20269 PCI native IDE mode */
		pci_read_config_byte(pdc20269, PCI_CLASS_PROG, &progif);
		pci_write_config_byte(pdc20269, PCI_CLASS_PROG, progif | 0x05);
		pdc20269->class |= 0x05;
	}
}

static void __init cpci695_init_irq(void)
{
        mv64360_init_irq();
	mv64x60_set_bits(&bh, MV64360_IC_CPU1_INTR_MASK_LO, BIT27 | BIT25);
}


void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{

	parse_bootinfo(find_bootinfo());

	cpci695_set_bat();

#if 0
#ifdef CONFIG_BLK_DEV_INITRD
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif

	/* Copy cmd_line parameters */
	if (r6 && (((char *)r6) != '\0')) {
		*(char *)(r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *)(r6 + KERNELBASE));
	}
#endif

	isa_mem_base = CPCI695_ISA_MEM_BASE;
	pci_dram_offset = CPCI695_PCI_DRAM_OFFSET;

	ppc_md.setup_arch = cpci695_setup_arch;
	ppc_md.show_cpuinfo = cpci695_show_cpuinfo;
	ppc_md.init_IRQ = cpci695_init_irq;
	ppc_md.get_irq = mv64360_get_irq;

	ppc_md.restart = cpci695_restart;
	ppc_md.power_off = cpci695_power_off;
	ppc_md.halt = cpci695_halt;

	ppc_md.find_end_of_memory = cpci695_find_end_of_memory;
	ppc_md.setup_io_mappings = cpci695_map_io;

	ppc_md.time_init = todc_time_init;
	ppc_md.set_rtc_time = todc_set_rtc_time;
	ppc_md.get_rtc_time = todc_get_rtc_time;
	ppc_md.calibrate_decr = todc_calibrate_decr;

	ppc_md.nvram_read_val = todc_direct_read_val;
	ppc_md.nvram_write_val = todc_direct_write_val;

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pcibios_fixup = cpci695_pci_fixups;

#if defined(CONFIG_SERIAL_8250)
	cpci695_early_serial_map();
#endif

#ifdef	CONFIG_SERIAL_TEXT_DEBUG
	ppc_md.progress = gen550_progress;
#endif

	platform_notify = cpci695_platform_notify;
}
