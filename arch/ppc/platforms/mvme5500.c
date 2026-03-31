/*
 * mvme5500.c
 *
 * Board setup routines for the Motorola MVME5500 Board.
 *
 * Author: Ajit Prem (Ajit.Prem@motorola.com)
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
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/initrd.h>
#include <linux/bootmem.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/machdep.h>
#include <asm/mv64x60.h>
#include <asm/todc.h>
#include <asm/time.h>

#include "mvme5500.h"

extern void _set_L3CR(unsigned long);
extern char cmd_line[];
extern unsigned long total_memory;	/* in mm/init */

unsigned long mvme5500_bank_A_flash_size;
EXPORT_SYMBOL(mvme5500_bank_A_flash_size);

static struct mv64x60_handle bh;

#ifdef CONFIG_VME_BRIDGE_BOOTMEM
void *vme_driver_bootmem;
unsigned int vme_bootmem_size;
#endif

extern void gen550_progress(char *, unsigned short);
extern void gen550_init(int, struct uart_port *);

/*
 * Motorola MVME5500 Board PCI interrupt routing.
 */
static int __init
mvme5500_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	struct pci_controller *hose = (struct pci_controller *)(dev->sysdata);

	if (hose->index == 0) {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE 
		     *         A   B   C   D
		     */
		{
			{75, 0, 0, 0},	/* IDSEL 1 - IPMC */
			{0, 0, 0, 0},	/* IDSEL 2 - */
			{0, 0, 0, 0},	/* IDSEL 3 - */
			{0, 0, 0, 0},	/* IDSEL 4 - */
			{0, 0, 0, 0},	/* IDSEL 5 - */
			{72, 73, 74, 75},	/* IDSEL 6 - PMC Slot 0 */
			{0, 0, 0, 0},	/* IDSEL 7 - */
			{0, 0, 0, 0},	/* IDSEL 8 - */
			{0, 0, 0, 0},	/* IDSEL 9 - */
			{76, 77, 78, 79},	/* IDSEL 10 - VME */
		};

		const long min_idsel = 1, max_idsel = 10, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE 
		     *         A   B   C   D
		     */
		{
			{0, 0, 0, 0},	/* IDSEL 4 */
			{0, 0, 0, 0},	/* IDSEL 5 */
			{80, 81, 82, 83},	/* IDSEL 6 - PMC Slot 1 */
			{0, 0, 0, 0},	/* IDSEL 7 - Unused Slot 1 */
			{0, 0, 0, 0},	/* IDSEL 8 - Unused Slot 1 */
			{0, 0, 0, 0},	/* IDSEL 9 - Unused Slot 1 */
			{84, 0, 0, 0},	/* IDSEL 10 - Gb Ethernet  */
		};

		const long min_idsel = 4, max_idsel = 10, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}
}

static void __init mvme5500_setup_peripherals(void)
{
	u8 *reg_addr;
	u8 reg_value;
	u32 boot_flash_base;
	u32 second_flash_base;
	u32 base, boot_flash_size, second_flash_size;

	reg_addr = ioremap(MVME5500_BOARD_STATUS_REG_1, 1);

	reg_value = readb(reg_addr);

	if (reg_value & MVME5500_BOARD_BANK_SEL_MASK) {
		boot_flash_base = MVME5500_BANK_B_FLASH_BASE;
		second_flash_base = MVME5500_BANK_A_FLASH_BASE;
	} else {
		boot_flash_base = MVME5500_BANK_A_FLASH_BASE;
		second_flash_base = MVME5500_BANK_B_FLASH_BASE;
	}
	iounmap(reg_addr);

	mv64x60_get_32bit_window(&bh, MV64x60_CPU2BOOT_WIN, &base,
				 &boot_flash_size);
	if (base != boot_flash_base) {
		printk(KERN_ERR "Boot flash base discrepancy\n");
	} else {
		printk(KERN_DEBUG "Boot Flash Base:   0x%x Size 0x%08x\n",
		       boot_flash_base, boot_flash_size);
		mv64x60_set_32bit_window(&bh, MV64x60_CPU2BOOT_WIN,
					 boot_flash_base, boot_flash_size, 0);
		bh.ci->enable_window_32bit(&bh, MV64x60_CPU2BOOT_WIN);
	}

	mv64x60_get_32bit_window(&bh, MV64x60_CPU2DEV_0_WIN, &base,
				 &second_flash_size);
	if (base != second_flash_base) {
		printk(KERN_ERR "Flash base discrepancy\n");
	} else {
		printk(KERN_DEBUG "Second Flash Base: 0x%x Size 0x%08x\n",
		       second_flash_base, second_flash_size);
		mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_0_WIN,
					 second_flash_base,
					 second_flash_size, 0);
		bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_0_WIN);
	}
	if (second_flash_size >= boot_flash_size)
		mvme5500_bank_A_flash_size = second_flash_size;
	else
		mvme5500_bank_A_flash_size = boot_flash_size;

	mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_1_WIN,
				 MVME5500_TODC_BASE, MVME5500_TODC_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_1_WIN);

#if 0
	mv64x60_set_32bit_window(&bh, MV64x60_CPU2DEV_2_WIN,
				 MVME5500_UART_BASE, MVME5500_UART_SIZE, 0);
	bh.ci->enable_window_32bit(&bh, MV64x60_CPU2DEV_2_WIN);
#endif

	mv64x60_clr_bits(&bh, MV64x60_CPU_CONFIG,
			 ((1 << 12) | (1 << 28) | (1 << 29)));
	mv64x60_set_bits(&bh, MV64x60_CPU_CONFIG, (1 << 27));

	mv64x60_set_bits(&bh, MV64x60_CPU_CONFIG, (1 << 23));

	mv64x60_set_bits(&bh, MV64x60_PCI0_PCI_DECODE_CNTL,
			 ((1 << 0) | (1 << 3)));
	mv64x60_set_bits(&bh, MV64x60_PCI1_PCI_DECODE_CNTL,
			 ((1 << 0) | (1 << 3)));

	/*
	 * Enabling of PCI internal-vs-external arbitration
	 * is a platform- and errata-dependent decision.
	 */
	if (bh.type == MV64x60_TYPE_GT64260A) {
		mv64x60_set_bits(&bh, MV64x60_PCI0_ARBITER_CNTL, (1 << 31));
		mv64x60_set_bits(&bh, MV64x60_PCI1_ARBITER_CNTL, (1 << 31));
		mv64x60_set_bits(&bh, MV64x60_COMM_ARBITER_CNTL, (1 << 10));
	}

	mv64x60_set_bits(&bh, MV64x60_CPU_MASTER_CNTL, (1 << 9));	/* Only 1 cpu */
	/*
	 * Turn off timer/counters.  Not turning off watchdog timer because
	 * can't read its reg on the 64260A so don't know if we'll be enabling
	 * or disabling.
	 */
	mv64x60_clr_bits(&bh, MV64x60_TIMR_CNTR_0_3_CNTL,
			 ((1 << 0) | (1 << 8) | (1 << 16) | (1 << 24)));
	mv64x60_clr_bits(&bh, GT64260_TIMR_CNTR_4_7_CNTL,
			 ((1 << 0) | (1 << 8) | (1 << 16) | (1 << 24)));

	/*
	 * The MVME5500 uses several Multi-Purpose Pins (MPP) on the 64260
	 * bridge as interrupt inputs (via the General Purpose Ports (GPP)
	 * register).  Need to route the MPP inputs to the GPP and set the
	 * polarity correctly.
	 *
	 */

	/* MPP Control 0
	 * MPP funct   MPP programming Usage
	 * 0  GPP[0]   MPPCTL0[3:0]   SERIAL_0/SERIAL_1 interrupt
	 * 1  GPP[1]   MPPCTL0[7:4]   Not used
	 * 2  GPP[2]   MPPCTL0[11:8]  Abort switch
	 * 3  GPP[3]   MPPCTL0[15:12] RTC/Thermostat
	 * 4  GPP[4]   MPPCTL0[19:16] Not used
	 * 5  GPP[5]   MPPCTL0[23:20] Not used
	 * 6  GPP[6]   MPPCTL0[27:24] Watchdog timer WDNMI
	 * 7  GPP[7]   MPPCTL0[31:28] LXT971A MDINT
	 */

	mv64x60_write(&bh, MV64x60_MPP_CNTL_0, 0x00000000);

	/* MPP Control 1
	 * MPP  funct  MPP programming
	 * 8   GPP[8]  MPPCTL1[3:0]   PMC 1 INTA
	 * 9   GPP[9]  MPPCTL1[7:4]   PMC 1 INTB
	 * 10  GPP[10] MPPCTL1[11:8]  PMC 1 INTC
	 * 11  GPP[11] MPPCTL1[15:12] PMC 1 INTD
	 * 12  GPP[12] MPPCTL1[19:16] VME VLINT0
	 * 13  GPP[13] MPPCTL1[23:20] VME VLINT1
	 * 14  GPP[14] MPPCTL1[27:24] VME VLINT2
	 * 15  GPP[15] MPPCTL1[31:28] VME VLINT3
	 */

	mv64x60_write(&bh, MV64x60_MPP_CNTL_1, 0x00000000);

	/* MPP Control 2
	 * MPP funct   MPP programming Usage
	 * 16  GPP[16] MPPCTL2[3:0]   PMC 2 INTA
	 * 17  GPP[17] MPPCTL2[7:4]   PMC 2 INTB
	 * 18  GPP[18] MPPCTL2[11:8]  PMC 2 INTC
	 * 19  GPP[19] MPPCTL2[15:12] PMC 2 INTD
	 * 20  GPP[20] MPPCTL2[19:16] 82544 INT
	 * 21  GPP[21] MPPCTL2[23:20] Not used
	 * 22  GPP[22] MPPCTL2[27:24] Not used
	 * 23  GPP[23] MPPCTL2[31:28] Not used
	 */

	mv64x60_write(&bh, MV64x60_MPP_CNTL_2, 0x00000000);

	/* MPP Control 3
	 * MPP funct   MPP programming Usage
	 * 24  GPP[24] MPPCTL3[3:0]   Watch dog timer WDNMI ***OUTPUT***
	 * 25  GPP[25] MPPCTL3[7:4]   Watch dog timer WDE ***OUTPUT***
	 * 26  GPP[26] MPPCTL3[11:8]  SROM InitAct
	 * 27  GPP[27] MPPCTL3[15:12] Not used
	 * 28  GPP[28] MPPCTL3[19:16] Not used
	 * 29  GPP[29] MPPCTL3[23:20] Optional external PPC bus arbiter BG1 OUT
	 * 30  GPP[30] MPPCTL3[27:24] unused
	 * 31  GPP[31] MPPCTL3[31:28] unused
	 */
	/* do not enable watch dog timer interrupts */

	mv64x60_write(&bh, MV64x60_MPP_CNTL_3, 0x00000000);

#define GPP_EXTERNAL_INTERRUPTS \
                          (1<<7)  | (1<<8)  | (1<<9)  | (1<<10) | \
                          (1<<11) | (1<<12) | (1<<13) | (1<<14) | \
                          (1<<15) | (1<<16) | (1<<17) | (1<<18) | \
                          (1<<19) | (1<<20)

	/* Set interrupt active levels */
	mv64x60_clr_bits(&bh, MV64x60_GPP_IO_CNTL, GPP_EXTERNAL_INTERRUPTS);
	mv64x60_set_bits(&bh, MV64x60_GPP_LEVEL_CNTL, GPP_EXTERNAL_INTERRUPTS);
//        mv64x60_write(&bh, MV64x60_GPP_LEVEL_CNTL, 0x081fffcc);

	/* Clear any pending interrupts for these inputs and enable them. */
	mv64x60_write(&bh, MV64x60_GPP_INTR_CAUSE, ~GPP_EXTERNAL_INTERRUPTS);
	mv64x60_set_bits(&bh, MV64x60_GPP_INTR_MASK, GPP_EXTERNAL_INTERRUPTS);

	return;
}

static void __init mvme5500_misc_init(void)
{
	u8 *status_reg_2_addr;
	u8 status_reg_2;

	status_reg_2_addr = ioremap(MVME5500_BOARD_STATUS_REG_2, 1);
	status_reg_2 = readb(status_reg_2_addr);
	status_reg_2 &= ~MVME5500_BOARD_FAIL_MASK;
	status_reg_2 &= ~MVME5500_BOARD_FLASH_WP_MASK;
	writeb(status_reg_2, status_reg_2_addr);
	iounmap(status_reg_2_addr);

	return;
}

static void __init mvme5500_setup_bridge(void)
{
	struct mv64x60_setup_info si;
	int i;

	memset(&si, 0, sizeof(si));

	si.phys_reg_base = CONFIG_MV64X60_NEW_BASE;

	si.pci_0.enable_bus = 1;
	si.pci_0.pci_io.cpu_base = MVME5500_PCI_0_IO_START_PROC;
	si.pci_0.pci_io.pci_base_hi = 0;
	si.pci_0.pci_io.pci_base_lo = MVME5500_PCI_0_IO_START;
	si.pci_0.pci_io.size = MVME5500_PCI_0_IO_SIZE;
	si.pci_0.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;

	si.pci_0.pci_mem[0].cpu_base = MVME5500_PCI_0_MEM_START_PROC;
	si.pci_0.pci_mem[0].pci_base_hi = 0;
	si.pci_0.pci_mem[0].pci_base_lo = MVME5500_PCI_0_MEM_START;
	si.pci_0.pci_mem[0].size = MVME5500_PCI_0_MEM_SIZE;
	si.pci_0.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_0.pci_cmd_bits = 0;
	si.pci_0.latency_timer = 0x80;

	si.pci_1.enable_bus = 1;
	si.pci_1.pci_io.cpu_base = MVME5500_PCI_1_IO_START_PROC;
	si.pci_1.pci_io.pci_base_hi = 0;
	si.pci_1.pci_io.pci_base_lo = MVME5500_PCI_1_IO_START;
	si.pci_1.pci_io.size = MVME5500_PCI_1_IO_SIZE;
	si.pci_1.pci_io.swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_1.pci_mem[0].cpu_base = MVME5500_PCI_1_MEM_START_PROC;
	si.pci_1.pci_mem[0].pci_base_hi = 0;
	si.pci_1.pci_mem[0].pci_base_lo = MVME5500_PCI_1_MEM_START;
	si.pci_1.pci_mem[0].size = MVME5500_PCI_1_MEM_SIZE;
	si.pci_1.pci_mem[0].swap = MV64x60_CPU2PCI_SWAP_NONE;
	si.pci_1.pci_cmd_bits = 0;
	si.pci_1.latency_timer = 0x80;

	for (i = 0; i < MV64x60_CPU2MEM_WINDOWS; i++) {
		si.cpu_prot_options[i] = 0;
		si.cpu_snoop_options[i] = GT64260_CPU_SNOOP_WB;
		si.pci_0.acc_cntl_options[i] =
		    GT64260_PCI_ACC_CNTL_PREFETCHEN |
		    GT64260_PCI_ACC_CNTL_DREADEN |
		    GT64260_PCI_ACC_CNTL_RDPREFETCH |
		    GT64260_PCI_ACC_CNTL_RDLINEPREFETCH |
		    GT64260_PCI_ACC_CNTL_RDMULPREFETCH |
		    GT64260_PCI_ACC_CNTL_SWAP_NONE |
		    GT64260_PCI_ACC_CNTL_MBURST_32_BTYES;
		si.pci_0.snoop_options[i] = GT64260_PCI_SNOOP_WB;
		si.pci_1.acc_cntl_options[i] =
		    GT64260_PCI_ACC_CNTL_PREFETCHEN |
		    GT64260_PCI_ACC_CNTL_DREADEN |
		    GT64260_PCI_ACC_CNTL_RDPREFETCH |
		    GT64260_PCI_ACC_CNTL_RDLINEPREFETCH |
		    GT64260_PCI_ACC_CNTL_RDMULPREFETCH |
		    GT64260_PCI_ACC_CNTL_SWAP_NONE |
		    GT64260_PCI_ACC_CNTL_MBURST_32_BTYES;
		si.pci_1.snoop_options[i] = GT64260_PCI_SNOOP_WB;
	}

	/* Lookup PCI host bridges */
	if (mv64x60_init(&bh, &si))
		printk(KERN_ERR "Bridge initialization failed.\n");

	pci_dram_offset = 0;	/* System mem at same addr on PCI & cpu bus */
	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = mvme5500_map_irq;
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

#if 0
	if (bh.type == MV64x60_TYPE_GT64260B) {
		mv64x60_set_bits(&bh, MV64x60_PCI0_RD_BUF_DISCARD_TIMER,
				 0x0003ffff);
		mv64x60_set_bits(&bh, MV64x60_PCI1_RD_BUF_DISCARD_TIMER,
				 0x0003ffff);
		mv64x60_set_bits(&bh, MV64x60_PCI0_CMD, 1 << 1);
		mv64x60_set_bits(&bh, MV64x60_PCI1_CMD, 1 << 1);
	}
#endif

	return;
}

static void __init mvme5500_init_caches(void)
{
	uint val;

	/* Enable L2 and L3 caches (if 745x) */
	val = _get_L2CR();
	val |= L2CR_L2E;
	_set_L2CR(val);

	val = _get_L3CR();
	val |= L3CR_L3E;
	_set_L3CR(val);
}

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
static void __init mvme5500_early_serial_map(void)
{
	struct uart_port serial_req;

	memset(&serial_req, 0, sizeof(serial_req));
	serial_req.uartclk = MVME5500_BASE_BAUD;
	serial_req.line = 0;
	serial_req.irq = MVME5500_UART_0_IRQ;
	serial_req.flags = STD_COM_FLAGS;
	serial_req.iotype = UPIO_MEM;
	serial_req.membase = (char *)MVME5500_SERIAL_0;
	serial_req.regshift = 0;

	gen550_init(0, &serial_req);

	if (early_serial_setup(&serial_req) != 0)
		printk("Early serial init of port 0 failed\n");

	/* Assume early_serial_setup() doesn't modify serial_req */
	serial_req.line = 1;
	serial_req.irq = MVME5500_UART_1_IRQ;
	serial_req.membase = (char *)MVME5500_SERIAL_1;

	gen550_init(1, &serial_req);

	if (early_serial_setup(&serial_req) != 0)
		printk("Early serial init of port 1 failed\n");

}
#endif

TODC_ALLOC();

static void __init mvme5500_setup_arch(void)
{

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	mvme5500_early_serial_map();
#endif

	if (ppc_md.progress)
		ppc_md.progress("mvme5500_setup_arch: enter", 0);

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

#ifdef CONFIG_VME_BRIDGE_BOOTMEM

	vme_bootmem_size = CONFIG_VME_BRIDGE_BOOTMEM_SIZE * 1024 * 1024;

	if (vme_bootmem_size > (total_memory / 2)) {
		printk(KERN_WARNING
		       "BOOTMEM Size Requested: 0x%x Total Memory 0x%lx\n",
		       vme_bootmem_size, total_memory);
		printk(KERN_WARNING
		       "BOOTMEM Size requested has been capped at half the total memory\n");
		vme_bootmem_size = total_memory / 2;
	}
	vme_driver_bootmem = __alloc_bootmem(vme_bootmem_size, 0x10000, 0);
	if (!vme_driver_bootmem) {
		printk(KERN_WARNING "Unable to obtain boot memory for VME\n");
		vme_bootmem_size = 0;
	} else {
		printk(KERN_INFO
		       "0x%x of boot memory reserved for setup of VME inbound window 7\n",
		       vme_bootmem_size);
	}
#endif

	mvme5500_init_caches();

	if (ppc_md.progress)
		ppc_md.progress("mvme5500_setup_arch: find_bridges", 0);

	mvme5500_setup_bridge();

	if (ppc_md.progress)
		ppc_md.progress("mvme5500_setup_arch: find_bridges done", 0);

	if (ppc_md.progress)
		ppc_md.progress("mvme5500_setup_arch: setup_peripherals", 0);

	mvme5500_setup_peripherals();

	if (ppc_md.progress)
		ppc_md.progress("mvme5500_setup_arch: setup_peripherals done",
				0);

	TODC_INIT(TODC_TYPE_MK48T37, 0, 0, MVME5500_TODC_BASE, 8);

	mvme5500_misc_init();

	printk("Motorola Computer Group MVME5500  Board\n");
	printk("MVME5500 port (C) 2003-2006 Motorola, Inc.\n");

	if (ppc_md.progress)
		ppc_md.progress("mvme5500_setup_arch: exit", 0);

	return;
}

static void __init mvme5500_init_irq(void)
{

	if (ppc_md.progress)
		ppc_md.progress("mvme5500_init_irq: enter", 0);

	gt64260_init_irq();

	irq_desc[64].status |= IRQ_LEVEL;
	irq_desc[66].status |= IRQ_LEVEL;
	irq_desc[67].status |= IRQ_LEVEL;
	irq_desc[70].status |= IRQ_LEVEL;
	irq_desc[71].status |= IRQ_LEVEL;
	irq_desc[72].status |= IRQ_LEVEL;
	irq_desc[73].status |= IRQ_LEVEL;
	irq_desc[74].status |= IRQ_LEVEL;
	irq_desc[75].status |= IRQ_LEVEL;
	irq_desc[76].status |= IRQ_LEVEL;
	irq_desc[77].status |= IRQ_LEVEL;
	irq_desc[78].status |= IRQ_LEVEL;
	irq_desc[79].status |= IRQ_LEVEL;
	irq_desc[80].status |= IRQ_LEVEL;
	irq_desc[81].status |= IRQ_LEVEL;
	irq_desc[82].status |= IRQ_LEVEL;
	irq_desc[83].status |= IRQ_LEVEL;
	irq_desc[84].status |= IRQ_LEVEL;

	if (ppc_md.progress)
		ppc_md.progress("mvme5500_init_irq: exit", 0);
}

/*
 * Set BAT 3 to map 0xf0000000 to end of physical memory space.
 */
static __inline__ void mvme5500_set_bat(void)
{
	mb();
	mtspr(SPRN_DBAT1U, 0xf0001ffe);
	mtspr(SPRN_DBAT1L, 0xf000002a);
	mb();
}

unsigned long __init mvme5500_find_end_of_memory(void)
{
	return mv64x60_get_mem_size(CONFIG_MV64X60_NEW_BASE,
				    MV64x60_TYPE_GT64260A);
}

static void __init mvme5500_map_io(void)
{
	io_block_mapping(0xf1000000, 0xf1000000, 0x01000000, _PAGE_IO);
	io_block_mapping(MVME5500_INTERNAL_SRAM_BASE,
			 MVME5500_INTERNAL_SRAM_BASE,
			 MVME5500_INTERNAL_SRAM_SIZE, _PAGE_IO);

}

static void mvme5500_restart(char *cmd)
{
	volatile ulong i = 10000000;
	u8 *status_reg_3_addr;

	local_irq_disable();
	status_reg_3_addr = ioremap(MVME5500_BOARD_STATUS_REG_3, 1);
	writeb(MVME5500_BOARD_RESET_MASK, status_reg_3_addr);

	while (i-- > 0) ;
	panic("restart failed\n");
}

static void mvme5500_halt(void)
{
	local_irq_disable();
	while (1) ;
	/* NOTREACHED */
}

static void mvme5500_power_off(void)
{
	mvme5500_halt();
	/* NOTREACHED */
}

static int mvme5500_show_cpuinfo(struct seq_file *m)
{
	uint tmp;
	uint memsize = total_memory;
	u8 reg_value;
	u8 *reg_addr, *reg_block;

	seq_printf(m, "vendor\t\t: Motorola\n");
	seq_printf(m, "machine\t\t: MVME5500\n");
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
	reg_block = ioremap(MVME5500_BOARD_STATUS_REG_1, 8);
	reg_addr = reg_block;

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t\t: %d MB\n", memsize / (1024 * 1024));

	/* System Status Register 1 */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Reference Clock\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_REF_CLOCK) ?
		    "Bit Set" : "Bit Not Set"));
	seq_printf(m, "Flash Boot Block Select\t: %s\n",
		   ((reg_value & MVME5500_BOARD_BANK_SEL_MASK) ?
		    "Flash 1 is the boot bank" : "Flash 0 is the boot bank"));
	seq_printf(m, "Safe Start Status\t: %s\n",
		   ((reg_value & MVME5500_BOARD_SAFE_START) ?
		    "Safe ENV settings used" : "NVRAM ENV settings used"));
	seq_printf(m, "Abort Status\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_ABORT_STATUS) ?
		    "Abort Switch Not Asserted" : "Abort Switch Asserted"));
	seq_printf(m, "Flash Busy\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_FLASH_BUSY) ?
		    "Bit Set" : "Bit Not Set"));
	seq_printf(m, "Fuse Status\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_FUSE_STAT) ?
		    "Fuses Functional" : "A fuse is open"));

	/* System Status Register 2 */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Board Fail LED\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_FAIL_MASK) ?
		    "LED Lit" : "LED Not Lit"));
	seq_printf(m, "EEPROM WP Status\t: %s\n",
		   ((reg_value & MVME5500_BOARD_EEPROM_WP_MASK) ?
		    "EEPROM Write Protected" : "EEPROM Not Write Protected"));
	seq_printf(m, "Flash WP Status\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_FLASH_WP_MASK) ?
		    "Flash Write Protected" : "Flash Not Write Protected"));
	seq_printf(m, "DS1621 Thermostat\t: %s\n",
		   ((reg_value & MVME5500_BOARD_TSTAT_MASK) ?
		    "Interrupt Disabled" : "Interrupt Enabled"));
	seq_printf(m, "PCI-C Bus Speed\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_PCIC_M66EN) ?
		    "66 MHz" : "33 MHz"));
	seq_printf(m, "PCI-B Bus Speed\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_PCIB_M66EN) ?
		    "66 MHz" : "33 MHz"));
	seq_printf(m, "PCI-A Bus Speed\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_PCIA_M66EN) ?
		    "66 MHz" : "33 MHz"));

	/* System Status Register 3 */
	reg_value = readb(reg_addr++);
	seq_printf(m, "ABORT Int Mask\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_ABORT_INT_MASK) ?
		    "Interrupt Masked" : "Interrupt Not Masked"));

	/* Presence Detect Register */
	reg_value = readb(reg_addr++);
	reg_value = readb(reg_addr++);
	seq_printf(m, "EREADY1\t\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_EREADY1) ?
		    "PMC Slot 2 Ready for Enumeration" :
		    "PMC Slot 2 Not Ready for Enumeration"));
	seq_printf(m, "EREADY0\t\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_EREADY0) ?
		    "PMC Slot 1 Ready for Enumeration" :
		    "PMC Slot 1 Not Ready for Enumeration"));
	seq_printf(m, "PMCSPAN\t\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_PMCSPAN) ?
		    "Not Installed" : "Installed"));
	seq_printf(m, "PMC2\t\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_PMC2) ?
		    "Not Installed" : "Installed"));
	seq_printf(m, "PMC1\t\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_PMC1) ?
		    "Not Installed" : "Installed"));

	/* Configuration Header Switch */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Configuration Bits\t: 0x%x\n", reg_value);

	/* TBEN Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "TBEN0\t\t\t: %s\n",
		   ((reg_value & MVME5500_BOARD_TBEN0) ?
		    "TBEN pin driven high" : "TBEN pin driven low"));

	/* Geographic Address Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Geographic Register\t: 0x%x\n", reg_value);

	iounmap(reg_block);

	return 0;
}

void mvme5500_pcibios_fixup_bus(struct pci_bus *bus)
{
	struct pci_dev *bridge;
	struct pci_controller *hose = (struct pci_controller *)bus->sysdata;
	struct pci_bus_region region;
	u32 l;
	
	bridge = bus->self;
	if (!bridge)
		return;

	if (hose->index == 0) {
		if (!bus->resource[1])
			return;

		pcibios_resource_to_bus(bridge, &region, bus->resource[1]);
		if (bus->resource[1]->flags & IORESOURCE_MEM) {
			struct pci_bus *parent;
			unsigned long offset = 0;

			parent = bus->parent;
			offset = hose->pci_mem_offset;

			if ((bridge->vendor == 0x3388) &&
				(bridge->device == 0x0026)) {
				region.start = MVME5500_PCI_0_MEM_BASE_ADDR + 
							0x100000;
				region.end = MVME5500_PCI_0_MEM_BASE_ADDR + 
					MVME5500_PCI_0_MEM_SIZE - 0x8000000 - 1;
				bus->resource[1]->start = region.start + offset; 
				bus->resource[1]->end = region.end + offset; 
				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else {
				pci_read_config_dword(bridge, PCI_MEMORY_BASE,
					&l);
			}
		} else {
			l = 0x0000fff0;
			printk(KERN_INFO "MEM 0 window: disabled\n");
		}
		pci_write_config_dword(bridge, PCI_MEMORY_BASE, l);

		if (!bus->resource[2])
			return;

		pcibios_resource_to_bus(bridge, &region, bus->resource[2]);
		if (bus->resource[1]->flags & IORESOURCE_MEM) {
			struct pci_bus *parent;
			unsigned long offset = 0;

			parent = bus->parent;
			offset = hose->pci_mem_offset;

			pci_write_config_dword(bridge, PCI_PREF_LIMIT_UPPER32,
				0);
			if ((bridge->vendor == 0x3388) &&
				(bridge->device == 0x0026)) {
				region.start = MVME5500_PCI_0_MEM_BASE_ADDR;
				region.end = MVME5500_PCI_0_MEM_BASE_ADDR + 
					+ 0x100000 - 1;
				bus->resource[2]->start = region.start + offset; 
				bus->resource[2]->end = region.start + offset; 
				bus->resource[2]->flags = 
					IORESOURCE_MEM | IORESOURCE_PREFETCH;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else {
				pci_read_config_dword(bridge, 
					PCI_PREF_MEMORY_BASE, &l);
			}
			pci_write_config_dword(bridge, PCI_PREF_MEMORY_BASE, l);

			pci_write_config_dword(bridge, PCI_PREF_BASE_UPPER32, 0);
			pci_write_config_word(bridge, PCI_BRIDGE_CONTROL, 
				bus->bridge_ctl);
		}
	}
}

#if 0
static void __init mvme5500_fixup_u2(struct pci_dev *dev)
{
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, 
				MVME5500_PCI0_MEM_BASE_ADDR + 
				MVME5500_PCI0_MEM_BASE_SIZE -
				0x8000000 - 0x1000);
}

DECLARE_PCI_FIXUP_HEADER(0x10e3, 0x0000, mvme5500_fixup_u2);
#endif

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	parse_bootinfo(find_bootinfo());

	mvme5500_set_bat();

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

	isa_io_base = MVME5500_PCI_0_IO_BASE_ADDR_PROC;
	isa_mem_base = MVME5500_ISA_MEM_BASE;
	pci_dram_offset = MVME5500_PCI_DRAM_OFFSET;

#ifdef CONFIG_VME_BRIDGE
	{
		extern void vmemod_setup_options(char *);

		vmemod_setup_options(cmd_line);
	}
#endif

	ppc_md.setup_arch = mvme5500_setup_arch;
	ppc_md.show_cpuinfo = mvme5500_show_cpuinfo;
	ppc_md.init_IRQ = mvme5500_init_irq;
	ppc_md.get_irq = gt64260_get_irq;
	ppc_md.init = NULL;

	ppc_md.restart = mvme5500_restart;
	ppc_md.power_off = mvme5500_power_off;
	ppc_md.halt = mvme5500_halt;

	ppc_md.find_end_of_memory = mvme5500_find_end_of_memory;
	ppc_md.setup_io_mappings = mvme5500_map_io;

	ppc_md.time_init = todc_time_init;
	ppc_md.set_rtc_time = todc_set_rtc_time;
	ppc_md.get_rtc_time = todc_get_rtc_time;
	ppc_md.calibrate_decr = todc_calibrate_decr;

	ppc_md.nvram_read_val = todc_direct_read_val;
	ppc_md.nvram_write_val = todc_direct_write_val;

	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pcibios_fixup = NULL;
	ppc_md.pcibios_fixup_bus = mvme5500_pcibios_fixup_bus;

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	mvme5500_early_serial_map();
#endif

#ifdef CONFIG_SERIAL_TEXT_DEBUG
	ppc_md.progress = gen550_progress;
#endif
	platform_notify = NULL;

	return;
}
