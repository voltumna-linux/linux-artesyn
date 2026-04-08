// SPDX-License-Identifier: GPL-2.0
/*
 * Board setup routines for the Motorola MVME6100
 *
 * Copyright 2008 Alessio Igor Bogani
 *
 * Based on prpmc2800.c by Dale Farnsworth <dale@farnsworth.org>
 * Incorporates interrupt setup from arch/ppc/platforms/mvme6100.c
 * by Ajit Prem <Ajit.Prem@motorola.com>
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/machdep.h>
#include <asm/time.h>
#include <asm/udbg.h>
#include <asm/i8259.h>

#include <mm/mmu_decl.h>

#include <sysdev/mv64x60.h>

/* MV64360 MPP & GPP register offsets (from mpp/gpp reg base) */
#define MV64x60_MPP_CNTL_0	0x0000
#define MV64x60_MPP_CNTL_2	0x0008

#define MV64x60_GPP_IO_CNTL	0x0000
#define MV64x60_GPP_LEVEL_CNTL	0x0010
#define MV64x60_GPP_VALUE_SET	0x0018

/* Board control register offsets */
#define MVME6100_BOARD_STATUS_REG_2_OFF	0x01
#define MVME6100_BOARD_STATUS_REG_3_OFF	0x02
#define MVME6100_BOARD_RESET_MASK	0x80
#define MVME6100_BOARD_FAIL_MASK	0x80
#define MVME6100_BOARD_FLASH0_SW_WP	0x20

/* GPP bit definitions */
#define GPP5	(1 << 5)
#define GPP7	(1 << 7)
#define GPP16	(1 << 16)
#define GPP17	(1 << 17)
#define GPP18	(1 << 18)
#define GPP19	(1 << 19)
#define GPP20	(1 << 20)
#define GPP21	(1 << 21)
#define GPP22	(1 << 22)
#define GPP23	(1 << 23)
#define GPP_PCI_INTS	(GPP7 | GPP16 | GPP17 | GPP18 | GPP19 | \
			 GPP20 | GPP21 | GPP22 | GPP23)
#define GPP_ALL_INTS	(GPP5 | GPP_PCI_INTS)

/* BIT macros for MPP register manipulation */
#define BIT20	(1 << 20)
#define BIT21	(1 << 21)
#define BIT22	(1 << 22)
#define BIT23	(1 << 23)

static void __iomem *mv64x60_mpp_reg_base;
static void __iomem *mv64x60_gpp_reg_base;
static void __iomem *mvme6100_board_reg_base;

#ifdef CONFIG_NOT_COHERENT_CACHE
#define MVME6100_COHERENCY_SETTING "off"
#else
#define MVME6100_COHERENCY_SETTING "on"
#endif

static void __init mvme6100_setup_arch(void)
{
	struct device_node *np;
	phys_addr_t paddr;
	const unsigned int *reg;

	/*
	 * ioremap mpp and gpp registers in case they are later
	 * needed by mvme6100_restart().
	 */
	np = of_find_compatible_node(NULL, NULL, "marvell,mv64360-mpp");
	reg = of_get_property(np, "reg", NULL);
	paddr = of_translate_address(np, reg);
	of_node_put(np);
	mv64x60_mpp_reg_base = ioremap(paddr, reg[1]);

	np = of_find_compatible_node(NULL, NULL, "marvell,mv64360-gpp");
	reg = of_get_property(np, "reg", NULL);
	paddr = of_translate_address(np, reg);
	of_node_put(np);
	mv64x60_gpp_reg_base = ioremap(paddr, reg[1]);

	/* ioremap board control registers */
	np = of_find_compatible_node(NULL, NULL,
				     "motorola,mvme6100-board-ctl");
	if (np) {
		reg = of_get_property(np, "reg", NULL);
		paddr = of_translate_address(np, reg);
		of_node_put(np);
		mvme6100_board_reg_base = ioremap(paddr, reg[1]);

		if (mvme6100_board_reg_base) {
			u8 status_reg_2;

			/* Clear board fail LED and flash SW write protect */
			status_reg_2 = in_8(mvme6100_board_reg_base +
					    MVME6100_BOARD_STATUS_REG_2_OFF);
			status_reg_2 &= ~MVME6100_BOARD_FAIL_MASK;
			status_reg_2 &= ~MVME6100_BOARD_FLASH0_SW_WP;
			out_8(mvme6100_board_reg_base +
			      MVME6100_BOARD_STATUS_REG_2_OFF, status_reg_2);
		}
	}

#ifdef CONFIG_PCI
	mv64x60_pci_init();
#endif

	pr_info("Motorola MVME6100\n");
}

/*
 * i8259 cascade via chained handler.
 *
 * The Winbond 83C553 ISA bridge on PCI Bus 1 provides an i8259
 * interrupt controller cascaded through GPP pin 5 of the MV64360.
 */
static void mvme6100_i8259_cascade(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int cascade_irq;

	cascade_irq = i8259_irq();
	if (cascade_irq)
		generic_handle_irq(cascade_irq);

	chip->irq_eoi(&desc->irq_data);
}

static void __init mvme6100_init_irq(void)
{
	struct device_node *np;
	unsigned int cascade_virq;
	u32 temp;

	/* Initialize the MV64360 interrupt controller */
	mv64x60_init_irq();

	/*
	 * Configure GPP pins for interrupt handling.
	 * Ported from arch/ppc/platforms/mvme6100.c mvme6100_intr_setup()
	 */

	/* MPP 5: configure as GPIO (clear bits 20-23) */
	temp = in_le32(mv64x60_mpp_reg_base + MV64x60_MPP_CNTL_0);
	temp &= ~(BIT20 | BIT21 | BIT22 | BIT23);
	out_le32(mv64x60_mpp_reg_base + MV64x60_MPP_CNTL_0, temp);

	/* MPP 16-23: configure as GPIO */
	out_le32(mv64x60_mpp_reg_base + MV64x60_MPP_CNTL_2, 0);

	/* GPP 5: active high (i8259 cascade) */
	temp = in_le32(mv64x60_gpp_reg_base + MV64x60_GPP_LEVEL_CNTL);
	temp &= ~GPP5;
	/* GPP 7, 16-23: active low (PCI interrupts) */
	temp |= GPP_PCI_INTS;
	out_le32(mv64x60_gpp_reg_base + MV64x60_GPP_LEVEL_CNTL, temp);

	/* Set all interrupt GPP pins as input */
	temp = in_le32(mv64x60_gpp_reg_base + MV64x60_GPP_IO_CNTL);
	temp &= ~GPP_ALL_INTS;
	out_le32(mv64x60_gpp_reg_base + MV64x60_GPP_IO_CNTL, temp);

	/*
	 * Initialize the i8259 cascade.
	 *
	 * Map the cascade IRQ BEFORE i8259_init() so that irq_find_host()
	 * returns the MV64360 PIC host (the i8259 legacy host with
	 * of_node=NULL would match all nodes and shadow the PIC host).
	 */
	np = of_find_compatible_node(NULL, NULL, "chrp,iic");
	if (!np) {
		pr_err("MVME6100: i8259 node not found in DTS\n");
		return;
	}

	cascade_virq = irq_of_parse_and_map(np, 0);
	if (!cascade_virq) {
		pr_err("MVME6100: failed to map i8259 cascade IRQ\n");
		of_node_put(np);
		return;
	}

	i8259_init(np, 0);
	of_node_put(np);

	irq_set_chained_handler(cascade_virq, mvme6100_i8259_cascade);
}

static void __noreturn mvme6100_restart(char *cmd)
{
	volatile ulong i = 10000000;

	local_irq_disable();

	if (mvme6100_board_reg_base)
		out_8(mvme6100_board_reg_base + MVME6100_BOARD_STATUS_REG_3_OFF,
		      MVME6100_BOARD_RESET_MASK);

	while (i-- > 0)
		;
	panic("restart failed\n");
}

static void mvme6100_show_cpuinfo(struct seq_file *m)
{
	uint memsize = total_memory;

	seq_printf(m, "Vendor\t\t: Motorola\n");
	seq_printf(m, "Machine\t\t: MVME6100\n");
	seq_printf(m, "Memory\t\t: %d MB\n", memsize / (1024 * 1024));
	seq_printf(m, "coherency\t: %s\n", MVME6100_COHERENCY_SETTING);
}

/*
 * Set up early NS16550 debug console.
 * mv64x60_init_early() only supports MPSC serial, but MVME6100 uses NS16550
 * on the PMC site.
 */
#define MVME6100_UART_PHYS	0xf1120000

/*
 * Called very early, device-tree isn't unflattened.
 *
 * In v5.4 there is no longer a separate .init_early callback for MVME6100;
 * the early udbg UART setup is done here in .probe().
 */
static int __init mvme6100_probe(void)
{
	void __iomem *uart;

	if (!of_machine_is_compatible("motorola,MVME6100"))
		return 0;

	/* Early udbg console setup (was previously in init_early) */
	uart = ioremap(MVME6100_UART_PHYS, 0x1000);
	if (uart) {
		udbg_uart_init_mmio(uart, 1);
		udbg_uart_setup(9600, 1843200);
	}

	/* Enable L2 and L3 caches */
	_set_L2CR(_get_L2CR() | L2CR_L2E);
	_set_L3CR(_get_L3CR() | L3CR_L3E);

	return 1;
}

define_machine(mvme6100) {
	.name			= "MVME6100",
	.probe			= mvme6100_probe,
	.setup_arch		= mvme6100_setup_arch,
	.show_cpuinfo		= mvme6100_show_cpuinfo,
	.init_IRQ		= mvme6100_init_irq,
	.get_irq		= mv64x60_get_irq,
	.restart		= mvme6100_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
};
