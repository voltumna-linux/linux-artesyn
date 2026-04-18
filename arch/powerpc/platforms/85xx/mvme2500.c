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

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#if 0
#include <linux/interrupt.h>
#endif
#include <asm/udbg.h>
#include <asm/mpic.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>

#include "mpc85xx.h"
#include "smp.h"

/* MPC85xx backside L2 cache controller bits (CCSR + 0x20000). */
#define MPC85xx_L2CTL_L2E	0x80000000	/* L2 enable */
#define MPC85xx_L2CTL_L2I	0x40000000	/* L2 flash invalidate */

#if 0
#define MVME2500_INTERRUPT_REG_GPIO02_OFFSET	0x95
#define MVME2500_ABORT_MASK			0x8

static void __iomem *mvme2500_fpga;
static int irq_abort, irq_power_interruption;
#endif

static void __init mvme2500_pic_init(void)
{
	struct mpic *mpic = mpic_alloc(NULL, 0,
		  MPIC_BIG_ENDIAN | MPIC_SINGLE_DEST_CPU,
		0, 256, " OpenPIC  ");
	BUG_ON(mpic == NULL);
	mpic_init(mpic);
}

/*
 * Enable the P2020 backside L2 cache if the bootloader left it disabled.
 *
 * The in-tree FSL_85XX_CACHE_SRAM driver that used to handle this was
 * removed in 6.0 (commit dc21ed2aef41). U-Boot on MVME2500 normally
 * enables L2, but do not rely on it: if L2E is clear we enable with a
 * flash invalidate. The L2CTL size and other settings are left as
 * programmed by firmware.
 */
static void __init mvme2500_enable_l2(void)
{
	struct device_node *np;
	struct resource res;
	void __iomem *l2_base;
	u32 ctl;

	np = of_find_compatible_node(NULL, NULL,
				     "fsl,p2020-l2-cache-controller");
	if (!np) {
		pr_warn("mvme2500: no L2 cache controller node in DT\n");
		return;
	}

	if (of_address_to_resource(np, 0, &res)) {
		pr_warn("mvme2500: cannot translate L2 controller reg\n");
		of_node_put(np);
		return;
	}
	of_node_put(np);

	l2_base = ioremap(res.start, resource_size(&res));
	if (!l2_base) {
		pr_warn("mvme2500: cannot ioremap L2 controller\n");
		return;
	}

	asm volatile("msync; isync");
	ctl = in_be32(l2_base);

	if (ctl & MPC85xx_L2CTL_L2E) {
		pr_info("mvme2500: L2 cache already enabled (L2CTL=%#x)\n",
			ctl);
	} else {
		pr_info("mvme2500: enabling backside L2 cache\n");
		ctl |= MPC85xx_L2CTL_L2E | MPC85xx_L2CTL_L2I;
		asm volatile("msync; isync");
		out_be32(l2_base, ctl);
		asm volatile("msync; isync");
	}

	iounmap(l2_base);
}

/*
 * Setup the architecture
 */
static void __init mvme2500_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("mvme2500_setup_arch()", 0);
	mvme2500_enable_l2();
	mpc85xx_smp_init();
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
