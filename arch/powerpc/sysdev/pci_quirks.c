/*
 * PCI quirks for PowerPC architecture.
 *
 * Copyright (C) 2007 Freescale Semiconductor, Inc. All rights reserved.
 * Zhang Wei <wei.zhang@freescale.com>, Jul 2007
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This file contains work-arounds or the special setting for
 * PCI hardware devices on PowerPC architecture.
 *
 * Note: any quirks for hotpluggable devices must _NOT_ be declared __init.
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <asm-generic/rtc.h>

/* The ULI1575 chip is used for MPC8641 and MPC8544 boards.
 * The PIC in ULI1575 and legacy devices should be initialized.
 */
enum pirq{PIRQA = 8, PIRQB, PIRQC, PIRQD, PIRQE, PIRQF, PIRQG, PIRQH};
const unsigned char uli1575_irq_route_table[16] = {
	0,	/* 0: Reserved */
	0x8,	/* 1: 0b1000 */
	0,	/* 2: Reserved */
	0x2,	/* 3: 0b0010 */
	0x4,	/* 4: 0b0100 */
	0x5,	/* 5: 0b0101 */
	0x7,	/* 6: 0b0111 */
	0x6,	/* 7: 0b0110 */
	0,	/* 8: Reserved */
	0x1,	/* 9: 0b0001 */
	0x3,	/* 10: 0b0011 */
	0x9,	/* 11: 0b1001 */
	0xb,	/* 12: 0b1011 */
	0,	/* 13: Reserved */
	0xd,	/* 14, 0b1101 */
	0xf,	/* 15, 0b1111 */
};

static int __devinit
get_pci_irq_from_of(struct pci_controller *hose, int slot, int pin)
{
	struct of_irq oirq;
	u32 laddr[3];
	struct device_node *hosenode = hose ? hose->arch_data : NULL;

	if (!hosenode)
		return -EINVAL;

	laddr[0] = (hose->first_busno << 16) | (PCI_DEVFN(slot, 0) << 8);
	laddr[1] = 0;
	laddr[2] = 0;
	of_irq_map_raw(hosenode, &pin, 1, laddr, &oirq);
	pr_debug("get_pci_irq_from_of: "
			"pci irq addr %x, slot %d, pin %d, irq %d\n",
			laddr[0], slot, pin, oirq.specifier[0]);
	return oirq.specifier[0];
}

static void __devinit quirk_uli1575(struct pci_dev *dev)
{
	unsigned short temp;
	struct pci_controller *hose = pci_bus_to_host(dev->bus);
	unsigned char irq2pin[16];
	unsigned long pirq_map_word = 0;
	u32 irq;
	int i;

	/*
	 * ULI1575 interrupts route setup
	 */
	memset(irq2pin, 0, 16); /* Initialize default value 0 */

	/*
	 * PIRQA -> PIRQD mapping read from OF-tree
	 *
	 * interrupts for PCI slot0 -- PIRQA / PIRQB / PIRQC / PIRQD
	 *                PCI slot1 -- PIRQB / PIRQC / PIRQD / PIRQA
	 */
	for (i = 0; i < 4; i++) {
		irq = get_pci_irq_from_of(hose, 0x1a, i + 1);
		if (irq > 0 && irq < 16)
			irq2pin[irq] = PIRQA + (i + 3) % 4;
		else
			printk(KERN_WARNING "ULI1575 device"
				"(slot %d, pin %d) irq %d is invalid.\n",
				17, i, irq);
	}

	/*
	 * PIRQE -> PIRQF mapping set manually
	 *
	 * IRQ pin   IRQ#
	 * PIRQE ---- 9
	 * PIRQF ---- 10
	 * PIRQG ---- 11
	 * PIRQH ---- 12
	 */
	for (i = 0; i < 4; i++)
		irq2pin[i + 9] = PIRQE + i;

	/* Set IRQ-PIRQ Mapping to ULI1575 */
	for (i = 0; i < 16; i++)
		if (irq2pin[i])
			pirq_map_word |= (uli1575_irq_route_table[i] & 0xf)
				<< ((irq2pin[i] - PIRQA) * 4);

	pirq_map_word |= 1<<26;	/* disable all PCI-Ex INTx in EP mode*/

	/* ULI1575 IRQ mapping conf register default value is 0xb9317542 */
	dev_dbg(&dev->dev,
		"Setup ULI1575 IRQ mapping configuration register value = "
		"0x%lx\n", pirq_map_word);
	pci_write_config_dword(dev, 0x48, pirq_map_word);

#define ULI1575_SET_DEV_IRQ(slot, pin, reg) 				\
	do { 								\
		int irq; 						\
		irq = get_pci_irq_from_of(hose, slot, pin); 		\
		if (irq > 0 && irq < 16) 				\
			pci_write_config_byte(dev, reg, irq2pin[irq]); 	\
		else							\
			printk(KERN_WARNING "ULI1575 device"		\
			    "(slot %d, pin %d) irq %d is invalid.\n",	\
			    slot, pin, irq);				\
	} while (0)

	/* USB 1.1 OHCI controller 1, slot 28, pin 1 */
	ULI1575_SET_DEV_IRQ(28, 1, 0x86);

	/* USB 1.1 OHCI controller 2, slot 28, pin 2 */
	ULI1575_SET_DEV_IRQ(28, 2, 0x87);

	/* USB 1.1 OHCI controller 3, slot 28, pin 3 */
	ULI1575_SET_DEV_IRQ(28, 3, 0x88);

	/* USB 2.0 controller, slot 28, pin 4 */
	irq = get_pci_irq_from_of(hose, 28, 4);
	if (irq >= 0 && irq <= 15)
		pci_write_config_dword(dev, 0x74, uli1575_irq_route_table[irq]);

	/* Audio controller, slot 29, pin 1 */
	ULI1575_SET_DEV_IRQ(29, 1, 0x8a);

	/* Modem controller, slot 29, pin 2 */
	ULI1575_SET_DEV_IRQ(29, 2, 0x8b);

	/* HD audio controller, slot 29, pin 3 */
	ULI1575_SET_DEV_IRQ(29, 3, 0x8c);

	/* SMB interrupt: slot 30, pin 1 */
	ULI1575_SET_DEV_IRQ(30, 1, 0x8e);

	/* PMU ACPI SCI interrupt: slot 30, pin 2 */
	ULI1575_SET_DEV_IRQ(30, 2, 0x8f);

	/* Serial ATA interrupt: slot 31, pin 1 */
	ULI1575_SET_DEV_IRQ(31, 1, 0x8d);

	/* Primary PATA IDE IRQ: 14
	 * Secondary PATA IDE IRQ: 15
	 */
	pci_write_config_byte(dev, 0x44, 0x30 | uli1575_irq_route_table[14]);
	pci_write_config_byte(dev, 0x75, uli1575_irq_route_table[15]);

	/* Set IRQ14 and IRQ15 to legacy IRQs */
	pci_read_config_word(dev, 0x46, &temp);
	temp |= 0xc000;
	pci_write_config_word(dev, 0x46, temp);

	/* Set i8259 interrupt trigger
	 * IRQ 3:  Level
	 * IRQ 4:  Level
	 * IRQ 5:  Level
	 * IRQ 6:  Level
	 * IRQ 7:  Level
	 * IRQ 9:  Level
	 * IRQ 10: Level
	 * IRQ 11: Level
	 * IRQ 12: Level
	 * IRQ 14: Edge
	 * IRQ 15: Edge
	 */
	outb(0xfa, 0x4d0);
	outb(0x1e, 0x4d1);

#undef ULI1575_SET_DEV_IRQ
#ifdef CONFIG_GEN_RTC
	/* RTC interface.
	 * If there is no initialization for rtc of ppc_md,
	 * using the sample functions in include/asm-generic/rtc.h
	 */
	if (!ppc_md.get_rtc_time)
		ppc_md.get_rtc_time = get_rtc_time;
	if (!ppc_md.set_rtc_time)
		ppc_md.set_rtc_time = set_rtc_time;
#endif
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AL, 0x1575, quirk_uli1575);

static void __devinit quirk_uli5288(struct pci_dev *dev)
{
	u8 c;
	u32 l;

	pci_read_config_byte(dev, 0x83, &c);
	c |= 0x80;		/* Unlock the writing to PCI_CLASS_REVISION */
	pci_write_config_byte(dev, 0x83, c);

	pci_read_config_dword(dev, PCI_CLASS_REVISION, &l);
	pci_write_config_dword(dev, PCI_CLASS_REVISION,
			(PCI_CLASS_STORAGE_SATA_AHCI << 8) | (l & 0xff));

	pci_read_config_byte(dev, 0x83, &c);
	c &= 0x7f;		/* Lock the writing to PCI_CLASS_REVISION */
	pci_write_config_byte(dev, 0x83, c);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AL, 0x5288, quirk_uli5288);

static void __devinit quirk_uli5229(struct pci_dev *dev)
{
	unsigned short temp;
	pci_write_config_word(dev, PCI_COMMAND, PCI_COMMAND_IO
			| PCI_COMMAND_MASTER | PCI_COMMAND_INTX_DISABLE);
	dev->class &= ~0x5;
	pci_read_config_word(dev, 0x4a, &temp);
	temp |= 0x1000;		/* Enable Native IRQ 14/15 */
	pci_write_config_word(dev, 0x4a, temp);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AL, 0x5229, quirk_uli5229);

static void __devinit early_uli5249(struct pci_dev *dev)
{
	unsigned char temp;
	pci_write_config_word(dev, PCI_COMMAND, PCI_COMMAND_IO
			| PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY);
	pci_read_config_byte(dev, 0x7c, &temp);
	pci_write_config_byte(dev, 0x7c, 0x80);	/* Unlock R/W control */
	/* set as pci-pci bridge */
	pci_write_config_byte(dev, PCI_CLASS_PROG, 0x01);
	pci_write_config_byte(dev, 0x7c, temp);	/* Lock R/W control */
	dev->class |= 0x1;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_AL, 0x5249, early_uli5249);

