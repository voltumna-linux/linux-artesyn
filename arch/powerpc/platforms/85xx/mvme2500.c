// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Board setup routines for the Emerson/Artesyn MVME2500
 *
 * Copyright 2014 Elettra-Sincrotrone Trieste S.C.p.A.
 *
 * Based on earlier code by:
 *
 *	Xianghua Xiao (x.xiao@freescale.com)
 *	Tom Armistead (tom.armistead@emerson.com)
 *	Copyright 2012 Emerson
 *
 * Author Alessio Igor Bogani <alessio.bogani@elettra.eu>
 */

#include <linux/pci.h>
#if 0
#include <linux/interrupt.h>
#endif
#include <asm/udbg.h>
#include <asm/mpic.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>

#include "mpc85xx.h"

#if 0
#define MVME2500_INTERRUPT_REG_GPIO02_OFFSET	0x95
#define MVME2500_ABORT_MASK			0x8

static void __iomem *mvme2500_fpga;
static int irq_abort, irq_power_interruption;
#endif

void __init mvme2500_pic_init(void)
{
	struct mpic *mpic = mpic_alloc(NULL, 0,
		  MPIC_BIG_ENDIAN | MPIC_SINGLE_DEST_CPU,
		0, 256, " OpenPIC  ");
	BUG_ON(mpic == NULL);
	mpic_init(mpic);
}

/*
 * Setup the architecture
 */
static void __init mvme2500_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("mvme2500_setup_arch()", 0);
	fsl_pci_assign_primary();
	pr_info("MVME2500 board from Artesyn\n");
}

machine_arch_initcall(mvme2500, mpc85xx_common_publish_devices);

#if 0
static irqreturn_t abort_timer_isr(int irq, void *context)
{
	u8 reg;

	reg = readb(mvme2500_fpga + MVME2500_INTERRUPT_REG_GPIO02_OFFSET);

	if (reg & MVME2500_ABORT_MASK) {
		printk(KERN_INFO "Requested (soft) abort\n");
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t power_interruption_isr(int irq, void *context)
{
	printk(KERN_ERR "Power interruption!\n");
	return IRQ_HANDLED;
};

static int mvme2500_setup_additional_irqs(void)
{
	struct device_node *np;
	int rc;

	np = of_find_compatible_node(NULL, NULL, "artesyn,mvme2500-fpga");
	if (np == NULL) {
		pr_err("Missing mvme2500-fpga node\n");
		return -ENOENT;
	}

	mvme2500_fpga = of_iomap(np, 0);
	of_node_put(np);
	if (mvme2500_fpga == NULL) {
		pr_err("Unable to map mvme2500-fpga io memory\n");
		return -ENOENT;
	}

	irq_abort = irq_of_parse_and_map(np, 0);
	if (irq_abort  == NO_IRQ) {
		pr_err("Fail to parse (soft) abort interrupt\n");
		return -ENOENT;
	}

	rc = request_irq(irq_abort, abort_timer_isr, IRQF_TRIGGER_LOW,
			"abort", NULL);
	if (rc) {
		pr_err("Unable to obtain (soft) abort interrupt!\n");
		return -ENOENT;
	}

	irq_power_interruption = irq_of_parse_and_map(np, 1);
	if (irq_power_interruption  == NO_IRQ) {
		pr_err("Fail to parse power interruption interrupt\n");
		return -ENOENT;
	}

	rc = request_irq(irq_power_interruption, power_interruption_isr,
			IRQF_TRIGGER_LOW, "power-interruption", NULL);
	if (rc) {
		pr_err("Unable to obtain power interruption interrupt!\n");
		return -ENOENT;
	}

	return 0;
}
machine_late_initcall(mvme2500, mvme2500_setup_additional_irqs);
#endif
define_machine(mvme2500) {
	.name			= "MVME2500",
	.compatible		= "artesyn,MVME2500",
	.setup_arch		= mvme2500_setup_arch,
	.init_IRQ		= mvme2500_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
	.pcibios_fixup_phb      = fsl_pcibios_fixup_phb,
#endif
	.get_irq		= mpic_get_irq,
	.progress		= udbg_progress,
};
