/*
 * mcp905.c
 *
 * Board setup routines for the Motorola MCP905 Board.
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2003-2007 Motorola, Inc.
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
#include <linux/initrd.h>
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
#include <asm/pci-bridge.h>
#include <asm/bootinfo.h>
#include <asm/mv64x60.h>
#include <asm/kgdb.h>

#include "mcp905.h"

extern void _set_L3CR(unsigned long);
extern char cmd_line[];
extern unsigned long total_memory;

extern void gen550_progress(char *, unsigned short);
extern void gen550_init(int, struct uart_port *);

static unsigned long mcp905_find_end_of_memory(void);

static struct mv64x60_handle bh;
static void __iomem *sram_base;

TODC_ALLOC();

/*
 * Motorola MCP905 Board PCI interrupt routing.
 */
static int __init
mcp905_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	struct pci_controller *hose = (struct pci_controller *)(dev->sysdata);

	if (hose->index == 0) {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE 
		     *         A   B   C   D
		     */
		{
			{82, 83, 82, 83},	/* IDSEL 4 - PMC 2 IDSEL */
			{83, 82, 83, 82},	/* IDSEL 5 - PMC 2 IDSELB */

		};

		const long min_idsel = 4, max_idsel = 5, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE 
		     *         A   B   C   D
		     */
		{
			{80, 81, 80, 81},	/* IDSEL 4 - 16 - PMC 1 IDSEL */
			{80, 81, 80, 81},	/* IDSEL 5 - PMC 1 IDSELB */
			{81, 0, 0, 0},	/* IDSEL 6 - PCI646U2 IDE */
			{80, 0, 0, 0},	/* IDSEL 7 - 21555 Bridge */
		};

		const long min_idsel = 4, max_idsel = 7, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}
}

#define SET_PCI_COMMAND_INVALIDATE

#ifdef SET_PCI_COMMAND_INVALIDATE
static void __init set_pci_command_invalidate(void)
{
	struct pci_dev *dev = NULL;
	u16 val;

	while ((dev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) {
		pci_read_config_word(dev, PCI_COMMAND, &val);
		val |= PCI_COMMAND_INVALIDATE;
		pci_write_config_word(dev, PCI_COMMAND, val);
	}
}
#endif

static void __init mcp905_pci_fixups(void)
{
#ifdef SET_PCI_COMMAND_INVALIDATE
	set_pci_command_invalidate();
#endif
}

static void __init mcp905_setup_bridge(void)
{
	struct mv64x60_setup_info si;
	int i;

	memset(&si, 0, sizeof(si));

	si.phys_reg_base = CONFIG_MV64X60_NEW_BASE;

	si.pci_0.enable_bus = 1;

	si.pci_0.pci_io.cpu_base = MCP905_PCI_0_IO_START_PROC;
	si.pci_0.pci_io.pci_base_hi = 0;
	si.pci_0.pci_io.pci_base_lo = MCP905_PCI_0_IO_START;
	si.pci_0.pci_io.size = MCP905_PCI_0_IO_SIZE;
	si.pci_0.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_0.pci_mem[0].cpu_base = MCP905_PCI_0_MEM_START_PROC;
	si.pci_0.pci_mem[0].pci_base_hi = 0;
	si.pci_0.pci_mem[0].pci_base_lo = MCP905_PCI_0_MEM_START;
	si.pci_0.pci_mem[0].size = MCP905_PCI_0_MEM_SIZE;
	si.pci_0.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_0.pci_cmd_bits = 0;
	si.pci_0.latency_timer = 0x80;

	si.pci_1.enable_bus = 1;

	si.pci_1.pci_io.cpu_base = MCP905_PCI_1_IO_START_PROC;
	si.pci_1.pci_io.pci_base_hi = 0;
	si.pci_1.pci_io.pci_base_lo = MCP905_PCI_1_IO_START;
	si.pci_1.pci_io.size = MCP905_PCI_1_IO_SIZE;
	si.pci_1.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_1.pci_mem[0].cpu_base = MCP905_PCI_1_MEM_START_PROC;
	si.pci_1.pci_mem[0].pci_base_hi = 0;
	si.pci_1.pci_mem[0].pci_base_lo = MCP905_PCI_1_MEM_START;
	si.pci_1.pci_mem[0].size = MCP905_PCI_1_MEM_SIZE;
	si.pci_1.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_1.pci_cmd_bits = 0;
	si.pci_1.latency_timer = 0x80;
	for (i = 0; i < MV64x60_CPU2MEM_WINDOWS; i++) {
#if defined(CONFIG_NOT_COHERENT_CACHE)
		si.cpu_prot_options[i] = 0;
		si.enet_options[i] = MV64360_ENET2MEM_SNOOP_NONE;
		si.mpsc_options[i] = MV64360_MPSC2MEM_SNOOP_NONE;
		si.idma_options[i] = MV64360_IDMA2MEM_SNOOP_NONE;

		si.pci_0.acc_cntl_options[i] =
		    MV64360_PCI_ACC_CNTL_SNOOP_NONE |
		    MV64360_PCI_ACC_CNTL_SWAP_NONE |
		    MV64360_PCI_ACC_CNTL_MBURST_128_BYTES |
		    MV64360_PCI_ACC_CNTL_RDSIZE_256_BYTES;

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

	pci_dram_offset = 0;	/* sys mem at same addr on PCI & cpu bus */
	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = mcp905_map_irq;
	ppc_md.pci_exclude_device = mv64x60_pci_exclude_device;

	mv64x60_set_bus(&bh, 0, 0);
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
void __init mcp905_intr_setup(void)
{
	if (ppc_md.progress)
		ppc_md.progress("mcp905_intr_setup: enter", 0);

	/* MPP 7 */
	mv64x60_clr_bits(&bh, MV64x60_MPP_CNTL_0,
			 BIT28 | BIT29 | BIT30 | BIT31);
	/* MPP 16-23 */
	mv64x60_clr_bits(&bh, MV64x60_MPP_CNTL_2, 0xffffffff);

	/*
	 * Define GPP 7, 16-23 interrupt polarity as active low
	 * input signal and level triggered
	 */
	mv64x60_set_bits(&bh, MV64x60_GPP_LEVEL_CNTL,
			 BIT7 | BIT16 | BIT17 | BIT18 | BIT19 | BIT20 | BIT21 |
			 BIT22 | BIT23);
	mv64x60_clr_bits(&bh, MV64x60_GPP_IO_CNTL,
			 BIT5 | BIT7 | BIT16 | BIT17 | BIT18 | BIT19 | BIT20 |
			 BIT21 | BIT22 | BIT23);

	/* Config GPP interrupt controller to respond to level trigger */

	mv64x60_set_bits(&bh, MV64x60_COMM_ARBITER_CNTL, (1 << 10));

	/* Erratum FEr PCI-#8 */
	mv64x60_clr_bits(&bh, MV64x60_PCI0_CMD, (1 << 5) | (1 << 9));
	mv64x60_clr_bits(&bh, MV64x60_PCI1_CMD, (1 << 5) | (1 << 9));

	/*
	 * Dismiss and then enable interrupt on GPP interrupt cause
	 * for CPU #0
	 */
	mv64x60_write(&bh, MV64x60_GPP_INTR_CAUSE,
		      ~(BIT7 | BIT16 | BIT17 | BIT18 | BIT19 | BIT20 |
			BIT21 | BIT22 | BIT23));

	mv64x60_set_bits(&bh, MV64x60_GPP_INTR_MASK,
			 BIT7 | BIT16 | BIT17 | BIT18 | BIT19 | BIT20 |
			 BIT21 | BIT22 | BIT23);

	/*
	 * Dismiss and then enable interrupt on CPU #0 high cause reg
	 * BIT26 summarizes GPP interrupts 16-23 (Need MPP 16-23)
	 * BIT24 summarizes GPP interrupts 0-7 (Need MPP 7)
	 */
	mv64x60_set_bits(&bh, MV64360_IC_CPU0_INTR_MASK_HI, BIT26 | BIT24);

	if (ppc_md.progress)
		ppc_md.progress("mcp905_intr_setup: exit", 0);
}

void __init mcp905_setup_peripherals(void)
{
	int bank_b_boot;
	u8 *bankp;
	u32 boot_bank_base;
	u32 boot_bank_size;
	u32 cs0_bank_base;
	u32 cs0_bank_size;

	if (ppc_md.progress)
		ppc_md.progress("mcp905_setup_peripherals: enter", 0);

	bankp = ioremap(MCP905_BOARD_STATUS_REG_1, 1);
	bank_b_boot = readb(bankp) & MCP905_BOARD_BANK_SEL_MASK;
	if (bank_b_boot) {
		cs0_bank_base = MCP905_BANK_A_FLASH_BASE;
		cs0_bank_size = MCP905_BANK_A_FLASH_SIZE;
		boot_bank_base = MCP905_BANK_B_FLASH_BASE;
		boot_bank_size = MCP905_BANK_B_FLASH_SIZE;
	} else {
		cs0_bank_base = MCP905_BANK_B_FLASH_BASE;
		cs0_bank_size = MCP905_BANK_B_FLASH_SIZE;
		boot_bank_base = MCP905_BANK_A_FLASH_BASE;
		boot_bank_size = MCP905_BANK_A_FLASH_SIZE;
	}

	/* Set up windows for flash banks */
	mv64x60_set_32bit_window(&bh, MV64x60_CPU2BOOT_WIN,
				 boot_bank_base, boot_bank_size, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2BOOT_WIN);

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_0_WIN,
				 cs0_bank_base, cs0_bank_size, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_0_WIN);

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_1_WIN,
				 MCP905_DEVICE_CS1_BASE, MCP905_DEVICE_CS1_SIZE,
				 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_1_WIN);

	/* Disable unused windows */
	mv64x60_write(&bh, MV64x60_CPU2DEV_2_BASE, 0);
	mv64x60_write(&bh, MV64x60_CPU2DEV_2_SIZE, 0);
	mv64x60_write(&bh, MV64x60_CPU2DEV_3_BASE, 0);
	mv64x60_write(&bh, MV64x60_CPU2DEV_3_SIZE, 0);

	/* Set up internal SRAM window */
	mv64x60_set_32bit_window(&bh, MV64x60_CPU2SRAM_WIN,
				 MCP905_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE,
				 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2SRAM_WIN);
	sram_base = ioremap(MCP905_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE);

	/* Set up Enet->SRAM window */
	mv64x60_set_32bit_window(&bh, MV64x60_ENET2MEM_4_WIN,
				 MCP905_INTERNAL_SRAM_BASE, MV64360_SRAM_SIZE,
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
	mcp905_intr_setup();

	if (ppc_md.progress)
		ppc_md.progress("mcp905_setup_peripherals: exit", 0);
}

#if defined(CONFIG_SERIAL_8250) &&  defined(CONFIG_SERIAL_TEXT_DEBUG)
static void __init mcp905_early_serial_map(void)
{
	struct uart_port serial_req;

	memset(&serial_req, 0, sizeof(serial_req));
	serial_req.uartclk = MCP905_BASE_BAUD;
	serial_req.irq = MCP905_UART_0_IRQ;
	serial_req.line = 0;
	serial_req.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
	serial_req.iotype = UPIO_MEM;
	serial_req.membase = (u_char *) MCP905_SERIAL_0;
	serial_req.regshift = 0;

	gen550_init(0, &serial_req);

	if (early_serial_setup(&serial_req) != 0) {
		printk("Early serial init of port 0 failed\n");
	}

	/* Assume early_serial_setup() doesn't modify serial_req */
	serial_req.line = 1;
	serial_req.irq = MCP905_UART_1_IRQ;
	serial_req.membase = (u_char *) MCP905_SERIAL_1;

	gen550_init(1, &serial_req);

	if (early_serial_setup(&serial_req) != 0) {
		printk("Early serial init of port 1 failed\n");
	}
}
#endif

static void __init mcp905_init_caches(void)
{
	uint val;

	/* Enable L2 and L3 caches (if 745x) */
	val = _get_L2CR();
	val |= L2CR_L2E;
	_set_L2CR(val);

	val = _get_L3CR();
	val |= L3CR_L3E;
	_set_L3CR(val);

#if 0
	val = mfspr(HID1);
	val &= ~HID0_EBD;
	mtspr(HID1, val);
	asm volatile ("isync");
	asm volatile ("sync");
#endif
}

static void __init mcp905_misc_init(void)
{
	u8 *status_reg_2_addr;
	u8 status_reg_2;

	status_reg_2_addr = ioremap(MCP905_BOARD_STATUS_REG_2, 1);
	status_reg_2 = readb(status_reg_2_addr);
	status_reg_2 &= ~MCP905_BOARD_FAIL_MASK;
	status_reg_2 &= ~MCP905_BOARD_FLASH_WP_MASK;
	writeb(status_reg_2, status_reg_2_addr);
	iounmap(status_reg_2_addr);

	return;
}

static void __init mcp905_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("mcp905_setup_arch: enter", 0);

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

	mcp905_init_caches();

	if (ppc_md.progress)
		ppc_md.progress("mcp905_setup_arch: find_bridges", 0);

	mcp905_setup_bridge();

	if (ppc_md.progress)
		ppc_md.progress("mcp905_setup_arch: find_bridges done", 0);

	mcp905_setup_peripherals();

	TODC_INIT(TODC_TYPE_MK48T37, 0, 0,
		  ioremap(MCP905_TODC_BASE, MCP905_TODC_SIZE), 8);

	mcp905_misc_init();

	printk("Motorola Computer Group MCP905 Board\n");
	printk("MCP905 port (C) 2002-2006 Motorola, Inc.\n");

	if (ppc_md.progress)
		ppc_md.progress("mcp905_setup_arch: exit", 0);

	return;
}

/*
 * Set BAT 3 to map 0xf0000000 to end of physical memory space.
 */
static __inline__ void mcp905_set_bat(void)
{
	mb();
	mtspr(SPRN_DBAT1U, 0xf0001ffe);
	mtspr(SPRN_DBAT1L, 0xf000002a);
	mb();

	return;
}

unsigned long __init mcp905_find_end_of_memory(void)
{
	return mv64x60_get_mem_size(CONFIG_MV64X60_NEW_BASE,
				    MV64x60_TYPE_MV64360);
}

static void __init mcp905_map_io(void)
{
	io_block_mapping(0xf1000000, 0xf1000000, 0x01000000, _PAGE_IO);
	io_block_mapping(MCP905_INTERNAL_SRAM_BASE,
			 MCP905_INTERNAL_SRAM_BASE,
			 MCP905_INTERNAL_SRAM_SIZE, _PAGE_IO);
}

static void mcp905_restart(char *cmd)
{
	volatile ulong i = 10000000;
	u8 *status_reg_3_addr;

	local_irq_disable();
	status_reg_3_addr = ioremap(MCP905_BOARD_STATUS_REG_3, 1);
	writeb(MCP905_BOARD_RESET_MASK, status_reg_3_addr);

	while (i-- > 0) ;
	panic("restart failed\n");
}

static void mcp905_halt(void)
{
	local_irq_disable();
	while (1) ;
	/* NOT REACHED */
}

static void mcp905_power_off(void)
{
	mcp905_halt();
	/* NOT REACHED */
}

static int mcp905_show_cpuinfo(struct seq_file *m)
{
	uint tmp;
	uint memsize = total_memory;
	u8 reg_value;
	u8 *reg_addr, *reg_block;
	unsigned char mcp905_geo_bits;

	seq_printf(m, "vendor\t\t: Motorola\n");
	seq_printf(m, "machine\t\t: MCP905\n");
	tmp = mfspr(SPRN_PVR);
	seq_printf(m, "PVID\t\t: 0x%x, vendor: %s\n",
		   tmp, (tmp & (1 << 15) ? "IBM" : "Motorola"));
	tmp = mfspr(SPRN_HID0);
	seq_printf(m, "HID0\t\t: 0x%08x\n", tmp);
	tmp = mfspr(SPRN_HID1);
	seq_printf(m, "HID1\t\t: 0x%08x\n", tmp);
	tmp = mfspr(SPRN_MSSCR0);
	seq_printf(m, "MSSCR0\t\t: 0x%08x, Bus Mode: %s\n",
		   tmp, ((((tmp & 0x00030000) >> 16) == 0) ? "60X" : "MPX"));
	tmp = mfspr(SPRN_MSSSR0);
	seq_printf(m, "MSSSR0\t\t: 0x%08x\n", tmp);
	tmp = mfspr(SPRN_L2CR);
	seq_printf(m, "L2CR\t\t: 0x%08x\n", tmp);
	tmp = mfspr(SPRN_L3CR);
	seq_printf(m, "L3CR\t\t: 0x%08x\n", tmp);

	seq_printf(m, "\nBOARD INFORMATION:\n\n");
	reg_block = ioremap(MCP905_BOARD_STATUS_REG_1, 8);
	reg_addr = reg_block;

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t\t: %d MB\n", memsize / (1024 * 1024));

	/* System Status Register 1 */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Reference Clock\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_REF_CLOCK) ?
		    "Bit Set" : "Bit Not Set"));
	seq_printf(m, "Flash Boot Block Select\t: %s\n",
		   ((reg_value & MCP905_BOARD_BANK_SEL_MASK) ?
		    "Flash 1 is the boot bank" : "Flash 0 is the boot bank"));
	seq_printf(m, "Safe Start Status\t: %s\n",
		   ((reg_value & MCP905_BOARD_SAFE_START) ?
		    "Safe ENV settings used" : "NVRAM ENV settings used"));
	seq_printf(m, "ABORT Status\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_ABORT_STATUS) ?
		    "ABORT Switch Not Asserted" : "ABORT Switch Asserted"));
	seq_printf(m, "Flash Busy\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_FLASH_BUSY) ?
		    "Bit Set" : "Bit Not Set"));
	seq_printf(m, "SROM Init Status\t: %s\n",
		   ((reg_value & MCP905_BOARD_SROM_INIT) ?
		    "MV64360 SROM Init Enabled" :
		    "MV64360 SROM Init Disabled"));

	/* System Status Register 2 */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Board Fail LED\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_FAIL_MASK) ?
		    "LED Lit" : "LED Not Lit"));
	seq_printf(m, "EEPROM WP Status\t: %s\n",
		   ((reg_value & MCP905_BOARD_EEPROM_WP_MASK) ?
		    "EEPROM Write Protected" :
		    "EEPROM Not SW Write Protected"));
	seq_printf(m, "Flash WP Status\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_FLASH_WP_MASK) ?
		    "Flash Write Protected" : "Flash Not Write Protected"));
	seq_printf(m, "DS1621 Thermostat\t: %s\n",
		   ((reg_value & MCP905_BOARD_TSTAT_MASK) ?
		    "Interrupt Disabled" : "Interrupt Enabled"));

	/* System Status Register 2 */
	reg_value = readb(reg_addr++);
	seq_printf(m, "ABORT Mask\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_ABORT_MASK) ?
		    "ABORT Interrupt Masked" : "ABORT Interrupt Not Masked"));

	/* Geographic Address Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Geographic Register\t: 0x%x\n", reg_value);
	mcp905_geo_bits = reg_value & MCP905_BOARD_GEO_ADDR_MASK;
	seq_printf(m, "Board in Slot\t\t: %d\n", mcp905_geo_bits);

	/* Presence Detect Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Transition Module\t: %s\n",
		   ((reg_value & MCP905_BOARD_TM_PRESENT) ?
		    "Not Installed" : "Installed"));
	seq_printf(m, "EREADY2\t\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_EREADY2) ?
		    "PMC Slot 2 Ready for Enumeration" :
		    "PMC Slot 2 Not Ready for Enumeration"));
	seq_printf(m, "EREADY1\t\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_EREADY1) ?
		    "PMC Slot 1 Ready for Enumeration" :
		    "PMC Slot 1 Not Ready for Enumeration"));
	seq_printf(m, "PMC2\t\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_PMC2) ?
		    "Not Installed" : "Installed"));
	seq_printf(m, "PMC1\t\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_PMC1) ?
		    "Not Installed" : "Installed"));

	/* Configuration Header Switch */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Configuration Bits\t: 0x%x\n", reg_value);

	/* TBEN Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "TBEN0\t\t\t: %s\n",
		   ((reg_value & MCP905_BOARD_TBEN0) ?
		    "TBEN pin driven high" : "TBEN pin driven low"));

	/* Hole */
	reg_value = readb(reg_addr++);

	/* Interrupt Status Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Thermostat Int Status\t: %s\n",
		   ((reg_value & MCP905_BOARD_TSTAT_INT) ?
		    "Interrupt NOT Asserted" : "Interrupt Asserted"));

	iounmap(reg_block);

	return 0;
}

static void __init mcp905_fixup_eth_pdata(struct platform_device *pdev)
{
	struct mv643xx_eth_platform_data *eth_pd;
	static u16 phy_addr[] = {
		MCP905_ETH0_PHY_ADDR,
		MCP905_ETH1_PHY_ADDR,
		MCP905_ETH2_PHY_ADDR,
	};

	eth_pd = pdev->dev.platform_data;
	eth_pd->force_phy_addr = 1;
	eth_pd->phy_addr = phy_addr[pdev->id];
	eth_pd->tx_queue_size = MCP905_ETH_TX_QUEUE_SIZE;
	eth_pd->rx_queue_size = MCP905_ETH_RX_QUEUE_SIZE;
#ifdef CONFIG_MV64X60_USE_SRAM
	if (pdev->id == 0) {
		eth_pd->rx_sram_addr = MCP905_INTERNAL_SRAM_BASE;
		eth_pd->rx_sram_size = 16000;
		eth_pd->tx_sram_addr = MCP905_INTERNAL_SRAM_BASE + 16000;
		eth_pd->tx_sram_size = 16000;
	} else if (pdev->id == 1) {
		eth_pd->rx_sram_addr = MCP905_INTERNAL_SRAM_BASE + 32000;
		eth_pd->rx_sram_size = 16000;
		eth_pd->tx_sram_addr = MCP905_INTERNAL_SRAM_BASE + 48000;
		eth_pd->tx_sram_size = 16000;
	} else {
		eth_pd->rx_sram_addr = MCP905_INTERNAL_SRAM_BASE + 64000;
		eth_pd->rx_sram_size = 16000;
		eth_pd->tx_sram_addr = MCP905_INTERNAL_SRAM_BASE + 80000;
		eth_pd->tx_sram_size = 16000;
	}
#endif
}
static int __init mcp905_platform_notify(struct device *dev)
{
	static struct {
		char *bus_id;
		void ((*rtn) (struct platform_device * pdev));
	} dev_map[] = {
		{ MV643XX_ETH_NAME ".0", mcp905_fixup_eth_pdata}, 
		{ MV643XX_ETH_NAME ".1", mcp905_fixup_eth_pdata}, 
		{ MV643XX_ETH_NAME ".2", mcp905_fixup_eth_pdata},
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

#if defined(CONFIG_BLK_DEV_IDE) || defined(CONFIG_BLK_DEV_IDE_MODULE)
static void __init
mcp905_ide_init_hwif_ports(hw_regs_t * hw, unsigned long data_port,
			   unsigned long ctrl_port, int *irq)
{
	unsigned long reg = data_port;
	int i;

	for (i = IDE_DATA_OFFSET; i <= IDE_STATUS_OFFSET; i++) {
		hw->io_ports[i] = reg++;
	}
	if (ctrl_port)
		hw->io_ports[IDE_CONTROL_OFFSET] = ctrl_port;
	else
		hw->io_ports[IDE_CONTROL_OFFSET] =
		    hw->io_ports[IDE_DATA_OFFSET] + 0x206;

	if (irq != NULL)
		*irq = hw->irq;
}

#endif

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	parse_bootinfo(find_bootinfo());

	mcp905_set_bat();

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

	isa_io_base = MCP905_PCI_0_IO_BASE_ADDR_PROC;
	isa_mem_base = MCP905_ISA_MEM_BASE;
	pci_dram_offset = MCP905_PCI_DRAM_OFFSET;

	ppc_md.setup_arch = mcp905_setup_arch;
	ppc_md.show_cpuinfo = mcp905_show_cpuinfo;
	ppc_md.init_IRQ = mv64360_init_irq;
	ppc_md.get_irq = mv64360_get_irq;
	ppc_md.init = NULL;

	ppc_md.restart = mcp905_restart;
	ppc_md.power_off = mcp905_power_off;
	ppc_md.halt = mcp905_halt;

	ppc_md.find_end_of_memory = mcp905_find_end_of_memory;
	ppc_md.setup_io_mappings = mcp905_map_io;

	ppc_md.time_init = todc_time_init;
	ppc_md.set_rtc_time = todc_set_rtc_time;
	ppc_md.get_rtc_time = todc_get_rtc_time;
	ppc_md.calibrate_decr = todc_calibrate_decr;

	ppc_md.nvram_read_val = todc_direct_read_val;
	ppc_md.nvram_write_val = todc_direct_write_val;

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pcibios_fixup = mcp905_pci_fixups;

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	mcp905_early_serial_map();
#endif

#ifdef  CONFIG_SERIAL_TEXT_DEBUG
	ppc_md.progress = gen550_progress;	/* Device module UART */
#endif

	platform_notify = mcp905_platform_notify;

#if defined(CONFIG_BLK_DEV_IDE) || defined(CONFIG_BLK_DEV_IDE_MODULE)
	ppc_ide_md.ide_init_hwif = mcp905_ide_init_hwif_ports;
#endif

	return;
}
