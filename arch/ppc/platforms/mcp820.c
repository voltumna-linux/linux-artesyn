/*
 * mcp820.c
 *
 * Board setup for the Motorola MCP820/CPCI6020 board
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2002-2007 Motorola Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/initrd.h>
#include <linux/ide.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/root_dev.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/seq_file.h>
#include <linux/harrier_defs.h>

#include <asm/byteorder.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/time.h>
#include <asm/todc.h>
#include <asm/pci-bridge.h>
#include <asm/bootinfo.h>
#include <asm/i8259.h>
#include <asm/harrier.h>

#include "mcp820.h"
#include "../syslib/open_pic_defs.h"
#include "../syslib/open_pic3.h"

extern unsigned long loops_per_jiffy;
extern int open_pic_irq_offset;
extern int open_pic2_irq_offset;
extern u_int OpenPIC_NumInitSenses;
extern u_int OpenPIC2_NumInitSenses;

extern void gen550_init(int, struct uart_port *);
extern void gen550_progress(char *, unsigned short);
extern void openpic_hookup_cascade(u_int irq, char *name,
				   int (*cascade_fn) (void));
extern int openpic2_irq(void);

#define HARRIER_ID                      0x480b1057
#define HARRIER_MPIC_IEEVP_OFF          0x00010220

#define HARRIER_1_REVI_REG	(MCP820_HARRIER_1_XCSR_BASE+HARRIER_REVI_OFF)
#define HARRIER_1_UCTL_REG	(MCP820_HARRIER_1_XCSR_BASE+HARRIER_UCTL_OFF)
#define HARRIER_1_MISC_CSR_REG  (MCP820_HARRIER_1_XCSR_BASE+HARRIER_MISC_CSR_OFF)
#define HARRIER_1_IFEVP_REG	(MCP820_HARRIER_1_MPIC_BASE+HARRIER_MPIC_IFEVP_OFF)
#define HARRIER_1_IFEDE_REG	(MCP820_HARRIER_1_MPIC_BASE+HARRIER_MPIC_IFEDE_OFF)
#define HARRIER_1_FEEN_REG	(MCP820_HARRIER_1_XCSR_BASE+HARRIER_FEEN_OFF)
#define HARRIER_1_IEEVP_REG	(MCP820_HARRIER_1_MPIC_BASE+HARRIER_MPIC_IEEVP_OFF)
#define HARRIER_1_FEMA_REG	(MCP820_HARRIER_1_XCSR_BASE+HARRIER_FEMA_OFF)

#define HARRIER_2_REVI_REG	(MCP820_HARRIER_2_XCSR_BASE+HARRIER_REVI_OFF)
#define HARRIER_2_UCTL_REG	(MCP820_HARRIER_2_XCSR_BASE+HARRIER_UCTL_OFF)
#define HARRIER_2_MISC_CSR_REG  (MCP820_HARRIER_2_XCSR_BASE+HARRIER_MISC_CSR_OFF)
#define HARRIER_2_IFEVP_REG	(MCP820_HARRIER_2_MPIC_BASE+HARRIER_MPIC_IFEVP_OFF)
#define HARRIER_2_IFEDE_REG	(MCP820_HARRIER_2_MPIC_BASE+HARRIER_MPIC_IFEDE_OFF)
#define HARRIER_2_IEEVP_REG	(MCP820_HARRIER_2_MPIC_BASE+HARRIER_MPIC_IEEVP_OFF)
#define HARRIER_2_FEEN_REG	(MCP820_HARRIER_2_XCSR_BASE+HARRIER_FEEN_OFF)
#define HARRIER_2_FEMA_REG	(MCP820_HARRIER_2_XCSR_BASE+HARRIER_FEMA_OFF)

static openpic_irq_def mcp820_openpic_irq_defs[] __initdata = {
	/* PICirq, LinuxVector, Priority, Level/Edge, Polarity, CascadeAck */
	{0, 0 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_POS, NULL},	/* 8259 */
	{1, 1 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, openpic2_irq},	/* Harrier B INT0 */
	{2, 2 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Harrier A WDT 2 */
	{3, 3 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* SYSINT_L  */
	{4, 4 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* cPCI Bus 2, INTA# */
	{5, 5 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* cPCI Bus 2, INTB# */
	{6, 6 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* cPCI Bus 2, INTC# */
	{7, 7 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* cPCI Bus 2, INTD# */
	{8, 8 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{9, 9 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{10, 10 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Primary Ethernet */
	{11, 11 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* USB Controller */
	{12, 12 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* cPCI Bus 1, INTA# */
	{13, 13 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* cPCI Bus 1, INTB# */
	{14, 14 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* cPCI Bus 1, INTC# */
	{15, 15 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* cPCI Bus 1, INTD# */
	{16, 16 + MCP820_MPIC1_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Internal */
	{-1, -1, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL}	/* End of table marker */
};

static openpic_irq_def mcp820_openpic2_irq_defs[] __initdata = {
	/* PICirq, LinuxVector, Priority, Level/Edge, Polarity, CascadeAck */
	{0, 0 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{1, 1 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{2, 2 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Harrier B, WDT 1/2 */
	{3, 3 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{4, 4 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{5, 5 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{6, 6 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{7, 7 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{8, 8 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* PMC INTA# */
	{9, 9 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* PMC INTB# */
	{10, 10 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* PMC INTC#, Sec Enet */
	{11, 11 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* PMC INTD# */
	{12, 12 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{13, 13 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{14, 14 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{15, 15 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Not Connected */
	{16, 16 + MCP820_MPIC2_VECTOR_OFFSET, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL},	/* Internal */
	{-1, -1, 8, OP_IRQ_LEVEL, OP_IRQ_NEG, NULL}	/* End of table marker */
};

ALLOC_OPENPIC_DEF(mcp820_openpic_def);
ALLOC_OPENPIC_DEF(mcp820_openpic2_def);

static inline int mcp820_map_irq(struct pci_dev *dev, unsigned char idsel,
				 unsigned char pin)
{
	struct pci_controller *hose = (struct pci_controller *)(dev->sysdata);

	if (hose->index == 0) {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE
		     *      A       B       C       D
		     */
		{
			{0, 0, 0, 0},	/* IDSEL 11  PCI-to-ISA */
			{0, 0, 0, 0},	/* IDSEL 12  */
			{0, 0, 0, 0},	/* IDSEL 13  */
			{10 + MCP820_MPIC1_VECTOR_OFFSET, 0, 0, 0},	/* IDSEL 14 ethernet  */
			{11 + MCP820_MPIC1_VECTOR_OFFSET, 8 + MCP820_MPIC1_VECTOR_OFFSET, 9 + MCP820_MPIC1_VECTOR_OFFSET, 0},	/* IDSEL 15 usb  */
			{0, 0, 0, 0},	/* IDSEL 16  */
			{0, 0, 0, 0},	/* IDSEL 17  */
			{0, 0, 0, 0},	/* IDSEL 18  */
			{0, 0, 0, 0},	/* IDSEL 19  */
			{12 + MCP820_MPIC1_VECTOR_OFFSET,
			 13 + MCP820_MPIC1_VECTOR_OFFSET,
			 14 + MCP820_MPIC1_VECTOR_OFFSET, 15 + MCP820_MPIC1_VECTOR_OFFSET},	/* IDSEL 20  P2P Bridge to local CPCI Bus */
			{0, 0, 0, 0},	/* IDSEL 21  */
			{4 + MCP820_MPIC1_VECTOR_OFFSET, 0, 0, 0},	/* IDSEL 22  HSC */
			{0, 0, 0, 0},	/* IDSEL 23  */
			{4 + MCP820_MPIC1_VECTOR_OFFSET,
			 5 + MCP820_MPIC1_VECTOR_OFFSET,
			 6 + MCP820_MPIC1_VECTOR_OFFSET, 7 + MCP820_MPIC1_VECTOR_OFFSET},	/* IDSEL 24  P2P Bridge on HA Bridge Board */
		};
		const long min_idsel = 11, max_idsel = 24, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	} else {
		static char pci_irq_table[][4] =
		    /*
		     *      PCI IDSEL/INTPIN->INTLINE
		     *      A       B       C       D
		     */
		{
			{10 + MCP820_MPIC2_VECTOR_OFFSET, 0, 0, 0},	/* IDSEL 14   eth */
			{0, 0, 0, 0},	/* IDSEL 15   */
			{8 + MCP820_MPIC2_VECTOR_OFFSET,
			 9 + MCP820_MPIC2_VECTOR_OFFSET,
			 10 + MCP820_MPIC2_VECTOR_OFFSET, 11 + MCP820_MPIC2_VECTOR_OFFSET},	/* IDSEL 16   PMC */
			{8 + MCP820_MPIC2_VECTOR_OFFSET,
			 9 + MCP820_MPIC2_VECTOR_OFFSET,
			 10 + MCP820_MPIC2_VECTOR_OFFSET, 11 + MCP820_MPIC2_VECTOR_OFFSET},	/* IDSEL 17   PMC IDSELB */
		};
		const long min_idsel = 14, max_idsel = 17, irqs_per_slot = 4;
		return PCI_IRQ_TABLE_LOOKUP;
	}
};

static void __init mcp820_pcibios_fixup(void)
{
	struct pci_dev *dev = NULL;
	unsigned short short_reg;
	unsigned char reg;

	/* Setup edge control in 8259 */
	outb(0x20, 0x4d0);	/* irq 5 level */
	outb(0x0E, 0x4d1);	/* irqs 9, 10, 11 level */

	if ((dev = pci_get_device(PCI_VENDOR_ID_WINBOND,
				  PCI_DEVICE_ID_WINBOND_83C553, NULL))) {
		/* Clear PCI Interrupt Routing Control Register. */
		short_reg = 0x0000;
		pci_write_config_word(dev, 0x44, short_reg);

		/* Route IDE interrupts to IRQ 14 */
		reg = 0xEF;
		pci_write_config_byte(dev, 0x43, reg);
	}

	if ((dev = pci_get_device(PCI_VENDOR_ID_WINBOND,
				  PCI_DEVICE_ID_WINBOND_82C105, dev))) {
		/*
		 * Disable LEGIRQ mode so PCI INTS are routed
		 * directly to the 8259 and enable both channels
		 */
		pci_write_config_dword(dev, 0x40, 0x50ff0033);

		/* Force correct IDE interrupt */
		dev->irq = 14;
		pci_write_config_byte(dev, PCI_INTERRUPT_LINE, dev->irq);
	}
	pci_dev_put(dev);

	return;
}

void __init mcp820_find_bridges(void)
{
	struct pci_controller *hose_a, *hose_b;
	int host_bridge;

	hose_a = pcibios_alloc_controller();
	if (!hose_a)
		return;

	hose_a->first_busno = 0;
	hose_a->last_busno = 0xff;
	hose_a->pci_mem_offset = MCP820_PCI_PHY_MEM_OFFSET;
	pci_init_resource(&hose_a->io_resource,
			  MCP820_PCI_IO_1_START,
			  MCP820_PCI_IO_1_END,
			  IORESOURCE_IO, "Harrier 1 host bridge");
	pci_init_resource(&hose_a->mem_resources[0],
			  MCP820_PCI_MEM_1_START,
			  MCP820_PCI_MEM_1_END,
			  IORESOURCE_MEM, "Harrier 1 host bridge");
	hose_a->io_space.start = MCP820_PCI_IO_1_START;
	hose_a->io_space.end = MCP820_PCI_IO_1_END;
	hose_a->mem_space.start = MCP820_PCI_MEM_1_START;
	hose_a->mem_space.end = MCP820_PCI_MEM_1_END;
	hose_a->io_base_virt = (void *)MCP820_ISA_IO_BASE;

	setup_indirect_pci(hose_a,
			   MCP820_PCI_CONFIG_1_ADDR, MCP820_PCI_CONFIG_1_DATA);

	early_read_config_dword(hose_a,
				hose_a->first_busno,
				PCI_DEVFN(0, 0), PCI_VENDOR_ID, &host_bridge);

	switch (host_bridge) {
	case HARRIER_ID:
		if (harrier_init(hose_a,
				 MCP820_HARRIER_1_XCSR_BASE,
				 MCP820_PROC_PCI_MEM_1_START,
				 MCP820_PROC_PCI_MEM_1_END,
				 MCP820_PROC_PCI_IO_1_START,
				 MCP820_PROC_PCI_IO_1_END,
				 MCP820_HARRIER_1_MPIC_BASE) != 0) {
			printk(KERN_CRIT
			       "Could not initialize HARRIER 1 bridge\n");
		}
		break;
	default:
		printk(KERN_CRIT "Host bridge 0x%x not supported\n",
		       host_bridge);
	}

	hose_a->last_busno = pciauto_bus_scan(hose_a, hose_a->first_busno);

	hose_b = pcibios_alloc_controller();
	if (!hose_b)
		return;

//      hose_b->first_busno = hose_a->last_busno + 1;
	hose_b->first_busno = 0;
	hose_b->last_busno = 0xff;
	hose_b->pci_mem_offset = MCP820_PCI_PHY_MEM_OFFSET;
	pci_init_resource(&hose_b->io_resource,
			  MCP820_PCI_IO_2_START,
			  MCP820_PCI_IO_2_END,
			  IORESOURCE_IO, "Harrier 2 host bridge");
	pci_init_resource(&hose_b->mem_resources[0],
			  MCP820_PCI_MEM_2_START,
			  MCP820_PCI_MEM_2_END,
			  IORESOURCE_MEM, "Harrier 2 host bridge");
	hose_b->io_space.start = MCP820_PCI_IO_2_START;
	hose_b->io_space.end = MCP820_PCI_IO_2_END;
	hose_b->mem_space.start = MCP820_PCI_MEM_2_START;
	hose_b->mem_space.end = MCP820_PCI_MEM_2_END;
	hose_b->io_base_virt = (void *)MCP820_ISA_IO_BASE;

	setup_indirect_pci(hose_b,
			   MCP820_PCI_CONFIG_2_ADDR, MCP820_PCI_CONFIG_2_DATA);

	host_bridge = 0;
	early_read_config_dword(hose_b,
				hose_b->first_busno,
				PCI_DEVFN(0, 0), PCI_VENDOR_ID, &host_bridge);

	switch (host_bridge) {
	case HARRIER_ID:
		if (harrier_init(hose_b,
				 MCP820_HARRIER_2_XCSR_BASE,
				 MCP820_PROC_PCI_MEM_2_START,
				 MCP820_PROC_PCI_MEM_2_END,
				 MCP820_PROC_PCI_IO_2_START,
				 MCP820_PROC_PCI_IO_2_END,
				 MCP820_HARRIER_2_MPIC_BASE) != 0) {
			printk(KERN_CRIT
			       "Could not initialize HARRIER 2 bridge\n");
		}
		break;
	default:
		printk(KERN_CRIT "Host bridge 0x%x not supported\n",
		       host_bridge);
	}

	hose_b->last_busno = pciauto_bus_scan(hose_b, hose_b->first_busno);

	OpenPIC_Addr = (void __iomem *)MCP820_HARRIER_1_MPIC_BASE,
	    OpenPIC2_Addr = (void __iomem *)MCP820_HARRIER_1_MPIC_BASE,
	    ppc_md.pcibios_fixup = mcp820_pcibios_fixup;
	ppc_md.pcibios_fixup_bus = NULL;
	ppc_md.pci_swizzle = common_swizzle;
	ppc_md.pci_map_irq = mcp820_map_irq;
}
static int mcp820_get_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "machine\t\t: MCP820\n");

	return 0;
}

TODC_ALLOC();

static void __init mcp820_setup_arch(void)
{
	unsigned int cpu;
	unsigned long cache;

	TODC_INIT(TODC_TYPE_MK48T37, 0, 0,
		  ioremap(MCP820_NVRAM_BASE_ADDRESS, MCP820_NVRAM_SIZE), 8);

	/* init to some ~sane value until calibrate_delay() runs */
	loops_per_jiffy = 50000000 / HZ;

	/* Lookup PCI host bridges */
	mcp820_find_bridges();

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;	/* /dev/ram */
	else
#endif
#ifdef CONFIG_ROOT_NFS
		ROOT_DEV = Root_NFS;	/* /dev/nfs pseudo device */
#else
		ROOT_DEV = Root_SDA2;	/* /dev/sda2 */
#endif

	printk("MCP820/CPCI6020 port (C) 2002-2007 Motorola Inc.\n");
	/* Identify the CPU manufacturer */
	cpu = PVR_REV(mfspr(SPRN_PVR));
	printk("CPU manufacturer: %s [rev=%04x]\n", (cpu & (1 << 15)) ? "IBM" :
	       "Motorola", cpu);
	cache = _get_L2CR();
	_set_L2CR(0);
	cache &= ~L2CR_L2DO;
	_set_L2CR(cache | L2CR_L2E);
}

static void __init mcp820_init2(void)
{
	return;
}

/*
 * Compute the MCP820's tbl frequency using the baud clock as a reference.
 */

static void __init mcp820_calibrate_decr(void)
{
	unsigned long tbl_start, tbl_end;
	unsigned long tb_ticks_per_second;
	unsigned int count;
	u8 harrier_revision;
	u8 current_state, old_state;

	harrier_revision = in_8((u8 *) HARRIER_1_REVI_REG);
	if (harrier_revision < 2) {
		/* XTAL64 was broken in harrier revision 1 */
		printk("time_init: Harrier revision %d, assuming 100 Mhz bus\n",
		       harrier_revision);
		tb_ticks_per_second = 100000000 / 4;
		tb_ticks_per_jiffy = tb_ticks_per_second / HZ;
		tb_to_us = mulhwu_scale_factor(tb_ticks_per_second, 1000000);
		return;
	}
	/*
	 * The XTAL64 bit oscillates at the 1/64 the base baud clock
	 * Set count to XTAL64 cycles per second.  Since we'll count
	 * half-cycles, we'll reach the count in half a second.
	 */
	count = MCP820_BASE_BAUD / 64;

	/* Find the first edge of the baud clock */
	old_state = in_8((u8 *) HARRIER_1_UCTL_REG) & HARRIER_XTAL64_MASK;
	do {
		current_state =
		    in_8((u8 *) HARRIER_1_UCTL_REG) & HARRIER_XTAL64_MASK;
	} while (old_state == current_state);

	old_state = current_state;

	/* Get the starting time base value */
	tbl_start = get_tbl();

	/*
	 * Loop until we have found a number of edges (half-cycles)
	 * equal to the count (half a second)
	 */
	do {
		do {
			current_state =
			    in_8((u8 *) HARRIER_1_UCTL_REG) &
			    HARRIER_XTAL64_MASK;
		} while (old_state == current_state);
		old_state = current_state;
	} while (--count);

	/* Get the ending time base value */
	tbl_end = get_tbl();

	/* We only counted for half a second, so double to get ticks/second */
	tb_ticks_per_second = (tbl_end - tbl_start) * 2;
	tb_ticks_per_jiffy = tb_ticks_per_second / HZ;
	tb_to_us = mulhwu_scale_factor(tb_ticks_per_second, 1000000);
}

static void mcp820_restart(char *cmd)
{
	ulong temp;

	local_irq_disable();
	temp = in_be32((uint *) HARRIER_1_MISC_CSR_REG);
	temp |= HARRIER_RSTOUT;
	out_be32((uint *) HARRIER_1_MISC_CSR_REG, temp);
	while (1) ;
}

static void mcp820_halt(void)
{
	local_irq_disable();
	while (1) ;
}

static void mcp820_power_off(void)
{
	mcp820_halt();
}

static void __init mcp820_init_irq(void)
{
	if (ppc_md.progress)
		ppc_md.progress("mcp820_init_irq: enter", 0);

	mcp820_openpic_def.OpenPIC_Addr =
	    (struct OpenPIC __iomem *)MCP820_HARRIER_1_MPIC_BASE;
	mcp820_openpic_def.slave_pic = 0;
	mcp820_openpic_def.IRQdef = mcp820_openpic_irq_defs;
	open_pic_irq_offset = 16;
	open_pic2_irq_offset = 33;
	OpenPIC_NumInitSenses = 17;
	openpic_set_sources(0, 16, OpenPIC_Addr + 0x10000);
	openpic_set_sources(16, 1, OpenPIC_Addr + 0x10200);

	openpic_init(&mcp820_openpic_def);

	mcp820_openpic2_def.OpenPIC_Addr =
	    (struct OpenPIC __iomem *)MCP820_HARRIER_2_MPIC_BASE;
	mcp820_openpic2_def.slave_pic = 0;
	mcp820_openpic2_def.IRQdef = mcp820_openpic2_irq_defs;
	OpenPIC2_NumInitSenses = 17;
	openpic2_set_sources(0, 16, OpenPIC2_Addr + 0x10000);
	openpic2_set_sources(16, 1, OpenPIC2_Addr + 0x10200);
	openpic2_init(&mcp820_openpic2_def);

	openpic_hookup_cascade(NUM_8259_INTERRUPTS, "82c59 cascade", i8259_irq);
	i8259_init(0, 0);

#define PRIORITY	15
#define VECTOR_1 	32
#define VECTOR_2 	49
#define PROCESSOR	0
	/* initialize the harrier A's internal interrupt priority 15, VECTOR_1 */
	out_be32((u32 *) HARRIER_1_IFEVP_REG, (PRIORITY << 16) | VECTOR_1);
	out_be32((u32 *) HARRIER_1_IFEDE_REG, (1 << PROCESSOR));

	/* enable functional exceptions for uarts and abort */
	out_8((u8 *) HARRIER_1_FEEN_REG, (HARRIER_FE_UA0 | HARRIER_FE_UA1));
	out_8((u8 *) HARRIER_1_FEMA_REG, ~(HARRIER_FE_UA0 | HARRIER_FE_UA1));

	/* Disable error interrupt */
	out_be32((u32 *) HARRIER_1_IEEVP_REG, 0x80000000);

	/* initialize the harrier B's internal interrupt priority 15, VECTOR_2 */
	out_be32((u32 *) HARRIER_2_IFEVP_REG, (PRIORITY << 16) | VECTOR_2);
	out_be32((u32 *) HARRIER_2_IFEDE_REG, (1 << PROCESSOR));

	/* enable functional exceptions for uarts and abort */
	/*
	   out_8((u8 *)HARRIER_2_FEEN_REG, (HARRIER_FE_UA0|HARRIER_FE_UA1));
	   out_8((u8 *)HARRIER_2_FEMA_REG, ~(HARRIER_FE_UA0|HARRIER_FE_UA1));
	 */

	/* Disable error interrupt */
	out_be32((u32 *) HARRIER_2_IEEVP_REG, 0x80000000);

	if (ppc_md.progress)
		ppc_md.progress("mcp820_init_irq: exit", 0);
}

/*
 * Set BAT 1 to map 0xf0000000 to end of physical memory space.
 */
static __inline__ void mcp820_set_bat(void)
{
	mb();
	mtspr(SPRN_DBAT1U, 0xf0001ffe);
	mtspr(SPRN_DBAT1L, 0xf000002a);
	mb();
}

#if defined(CONFIG_KGDB) || defined(CONFIG_SERIAL_TEXT_DEBUG)
static void __init mcp820_early_serial_map(void)
{
	struct uart_port serial_req;

	memset(&serial_req, 0, sizeof(serial_req));
	serial_req.line = 0;
	serial_req.uartclk = MCP820_BASE_BAUD;
	serial_req.irq = MCP820_HARRIER_1_SERIAL_1_IRQ;
	serial_req.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
	serial_req.iotype = UPIO_MEM;
	serial_req.membase = (u_char *) MCP820_HARRIER_1_SERIAL_1;
	serial_req.regshift = 0;

	gen550_init(0, &serial_req);

	if (early_serial_setup(&serial_req) != 0) {
		printk("Early serial init of port 0 failed\n");
	}
	serial_req.line = 1;
	serial_req.irq = MCP820_HARRIER_1_SERIAL_2_IRQ;
	serial_req.membase = (u_char *) MCP820_HARRIER_1_SERIAL_2;

	gen550_init(1, &serial_req);

	if (early_serial_setup(&serial_req) != 0) {
		printk("Early serial init of port 1 failed\n");
	}
}
#endif

/*
 * We need to read the Harrier memory controller
 * to properly determine this value
 */

static unsigned long __init mcp820_find_end_of_memory(void)
{
	unsigned long tmp1, tmp2;

	/* Cover the harrier registers with a BAT */
	mcp820_set_bat();

	/* Read the memory size from the Harrier XCSR */
	tmp1 = harrier_get_mem_size(MCP820_HARRIER_1_XCSR_BASE);
	tmp2 = harrier_get_mem_size(MCP820_HARRIER_2_XCSR_BASE);
	printk("Memory Size Harrier A  0x%lx Memory Size Harrier B 0x%lx\n",
	       tmp1, (tmp2 > 0) ? (tmp2 - tmp1) : tmp2);

	if (tmp2 > 0)
		return tmp2;
	else
		return tmp1;
}

static void __init mcp820_map_io(void)
{
	io_block_mapping(0xfe000000, 0xfe000000, 0x02000000, _PAGE_IO);
}

/* IDE functions */
#if defined(CONFIG_BLK_DEV_IDE) || defined(CONFIG_BLK_DEV_IDE_MODULE)
static void __init
mcp820_ide_init_hwif_ports(hw_regs_t * hw, unsigned long data_port,
			   unsigned long ctrl_port, int *irq)
{
	unsigned long reg = data_port;
	int i;

	for (i = IDE_DATA_OFFSET; i <= IDE_STATUS_OFFSET; i++) {
		hw->io_ports[i] = reg;
		reg += 1;
	}
	if (ctrl_port)
		hw->io_ports[IDE_CONTROL_OFFSET] = ctrl_port;
	else
		hw->io_ports[IDE_CONTROL_OFFSET] =
		    hw->io_ports[IDE_DATA_OFFSET] + 0x206;

	if (irq != NULL)
		*irq = 0;
}
#endif

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	parse_bootinfo(find_bootinfo());

	mcp820_set_bat();

#ifdef CONFIG_BLK_DEV_INITRD
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif

	/* Copy cmd_line parameters */
	if (r6) {
		*(char *)(r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *)(r6 + KERNELBASE));
	}
	isa_io_base = MCP820_ISA_IO_BASE;
	isa_mem_base = MCP820_ISA_MEM_BASE;
	pci_dram_offset = MCP820_PCI_DRAM_OFFSET;

	ISA_DMA_THRESHOLD = 0x00ffffff;
	DMA_MODE_READ = 0x44;
	DMA_MODE_WRITE = 0x48;

	ppc_md.setup_arch = mcp820_setup_arch;
	ppc_md.show_cpuinfo = mcp820_get_cpuinfo;
	ppc_md.init_IRQ = mcp820_init_irq;
	ppc_md.get_irq = openpic_get_irq;

	ppc_md.find_end_of_memory = mcp820_find_end_of_memory;
	ppc_md.setup_io_mappings = mcp820_map_io;

	ppc_md.restart = mcp820_restart;
	ppc_md.power_off = mcp820_power_off;
	ppc_md.halt = mcp820_halt;
	ppc_md.init = mcp820_init2;

	ppc_md.time_init = todc_time_init;
	ppc_md.get_rtc_time = todc_get_rtc_time;
	ppc_md.set_rtc_time = todc_set_rtc_time;
	ppc_md.calibrate_decr = mcp820_calibrate_decr;

	ppc_md.nvram_read_val = todc_direct_read_val;
	ppc_md.nvram_write_val = todc_direct_write_val;

#if defined(CONFIG_KGDB) || defined(CONFIG_SERIAL_TEXT_DEBUG)
	mcp820_early_serial_map();
#endif

#if defined(CONFIG_BLK_DEV_IDE) || defined(CONFIG_BLK_DEV_IDE_MODULE)
	ppc_ide_md.ide_init_hwif = mcp820_ide_init_hwif_ports;
#endif

#ifdef CONFIG_SERIAL_TEXT_DEBUG
	ppc_md.progress = gen550_progress;
#else
	ppc_md.progress = NULL;
#endif				/* CONFIG_SERIAL_TEXT_DEBUG */
}
