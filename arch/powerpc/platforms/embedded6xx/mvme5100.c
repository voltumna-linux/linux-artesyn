// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Board setup routines for the Motorola/Emerson MVME5100.
 *
 * Copyright 2013 CSC Australia Pty. Ltd.
 *
 * Based on earlier code by:
 *
 *    Matt Porter, MontaVista Software Inc.
 *    Copyright 2001 MontaVista Software Inc.
 *
 * Author: Stephen Chivers <schivers@csc.com>
 */

#include <linux/of_platform.h>
#include <linux/rtc/m48t59.h>

#include <asm/i8259.h>
#include <asm/pci-bridge.h>
#include <asm/mpic.h>
#include <asm/prom.h>
#include <mm/mmu_decl.h>
#include <asm/udbg.h>

#define HAWK_MPIC_SIZE		0x00040000U
#define MVME5100_PCI_MEM_OFFSET 0x00000000

/* Board register addresses. */
#define BOARD_STATUS_REG	0xfef88080
#define BOARD_MODFAIL_REG	0xfef88090
#define BOARD_MODRST_REG	0xfef880a0
#define BOARD_TBEN_REG		0xfef880c0
#define BOARD_SW_READ_REG	0xfef880e0
#define BOARD_GEO_ADDR_REG	0xfef880e8
#define BOARD_EXT_FEATURE1_REG	0xfef880f0
#define BOARD_EXT_FEATURE2_REG	0xfef88100

/*
 * Define the M48T37 NVRAM/RTC address strobe & data registers relative
 * to the base address of the device.
 */
#define NVRAM_AS0		0x0
#define NVRAM_AS1		0x08
#define NVRAM_DATA		0x10

static phys_addr_t pci_membase;
static u_char *restart;

static void mvme5100_8259_cascade(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int cascade_irq = i8259_irq();

	if (cascade_irq)
		generic_handle_irq(cascade_irq);

	chip->irq_eoi(&desc->irq_data);
}

static void __init mvme5100_pic_init(void)
{
	struct mpic *mpic;
	struct device_node *np;
	struct device_node *cp = NULL;
	unsigned int cirq;
	unsigned long intack = 0;
	const u32 *prop = NULL;

	np = of_find_node_by_type(NULL, "open-pic");
	if (!np) {
		pr_err("Could not find open-pic node\n");
		return;
	}

	mpic = mpic_alloc(np, pci_membase, 0, 16, 256, " OpenPIC  ");

	BUG_ON(mpic == NULL);
	of_node_put(np);

	mpic_assign_isu(mpic, 0, pci_membase + 0x10000);

	mpic_init(mpic);

	cp = of_find_compatible_node(NULL, NULL, "chrp,iic");
	if (cp == NULL) {
		pr_warn("mvme5100_pic_init: couldn't find i8259\n");
		return;
	}

	cirq = irq_of_parse_and_map(cp, 0);
	if (!cirq) {
		pr_warn("mvme5100_pic_init: no cascade interrupt?\n");
		return;
	}

	np = of_find_compatible_node(NULL, "pci", "mpc10x-pci");
	if (np) {
		prop = of_get_property(np, "8259-interrupt-acknowledge", NULL);

		if (prop)
			intack = prop[0];

		of_node_put(np);
	}

	if (intack)
		pr_debug("mvme5100_pic_init: PCI 8259 intack at 0x%016lx\n",
		   intack);

	i8259_init(cp, intack);
	of_node_put(cp);
	irq_set_chained_handler(cirq, mvme5100_8259_cascade);
}

static int __init mvme5100_add_bridge(struct device_node *dev)
{
	const int		*bus_range;
	int			len;
	struct pci_controller	*hose;
	unsigned short		devid;

	pr_info("Adding PCI host bridge %pOF\n", dev);

	bus_range = of_get_property(dev, "bus-range", &len);

	hose = pcibios_alloc_controller(dev);
	if (hose == NULL)
		return -ENOMEM;

	hose->first_busno = bus_range ? bus_range[0] : 0;
	hose->last_busno = bus_range ? bus_range[1] : 0xff;

	setup_indirect_pci(hose, 0xfe000cf8, 0xfe000cfc, 0);

	pci_process_bridge_OF_ranges(hose, dev, 1);

	early_read_config_word(hose, 0, 0, PCI_DEVICE_ID, &devid);

	if (devid != PCI_DEVICE_ID_MOTOROLA_HAWK) {
		pr_err("HAWK PHB not present?\n");
		return 0;
	}

	early_read_config_dword(hose, 0, 0, PCI_BASE_ADDRESS_1, &pci_membase);

	if (pci_membase == 0) {
		pr_err("HAWK PHB mibar not correctly set?\n");
		return 0;
	}

	pr_info("mvme5100_pic_init: pci_membase: %x\n", pci_membase);

	return 0;
}

static const struct of_device_id mvme5100_of_bus_ids[] __initconst = {
	{ .compatible = "hawk-bridge", },
	{},
};

/*
 * Setup the architecture
 */
static void __init mvme5100_setup_arch(void)
{
	struct device_node *np;

	if (ppc_md.progress)
		ppc_md.progress("mvme5100_setup_arch()", 0);

	for_each_compatible_node(np, "pci", "hawk-pci")
		mvme5100_add_bridge(np);

	restart = ioremap(BOARD_MODRST_REG, 4);
}


static void mvme5100_show_cpuinfo(struct seq_file *m)
{
	seq_puts(m, "Vendor\t\t: Motorola/Emerson\n");
	seq_puts(m, "Machine\t\t: MVME5100\n");
}

static void __noreturn mvme5100_restart(char *cmd)
{

	local_irq_disable();
	mtmsr(mfmsr() | MSR_IP);

	out_8((u_char *) restart, 0x01);

	while (1)
		;
}

/*
 * Called very early, device-tree isn't unflattened
 */
static int __init mvme5100_probe(void)
{
	return of_machine_is_compatible("MVME5100");
}

#if CONFIG_RTC_DRV_M48T59

static unsigned char nvrtc_read_byte(struct device *dev, u32 ofs)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct m48t59_plat_data *pdata = pdev->dev.platform_data;
	u8 __iomem *addr = pdata->ioaddr;
	u8 b;

	writeb(ofs, addr + NVRAM_AS0);
	writeb(ofs >> 8, addr + NVRAM_AS1);
	b = readb(addr + NVRAM_DATA);

	return b;
}

static void nvrtc_write_byte(struct device *dev, u32 ofs, u8 val)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct m48t59_plat_data *pdata = pdev->dev.platform_data;
	u8 __iomem *addr = pdata->ioaddr;

	writeb(ofs, addr + NVRAM_AS0);
	writeb(ofs >> 8, addr + NVRAM_AS1);
	writeb(val, addr + NVRAM_DATA);
}

static struct resource m48t37_resource[2];

static struct  m48t59_plat_data m48t37_data = {
	.read_byte = nvrtc_read_byte,
	.write_byte = nvrtc_write_byte,
	.type = M48T59RTC_TYPE_M48T37,
};

static struct platform_device m48t37_rtc = {
	.name		= "rtc-m48t59",
	.id		= 0,
	.num_resources	= 2,
	.resource	= m48t37_resource,
	.dev    = {
		.platform_data = &m48t37_data,
	},
};

static int __init mvme5100_rtc_init(void)
{
	int result = 0;
	int irq;
	struct device_node *np;
	struct resource *res = m48t37_rtc.resource;

	np = of_find_compatible_node(NULL, NULL, "m48t59");
	if (np == NULL) {
		pr_info("mvme5100_rtc_init: no RTC found.\n");
		return 0;
	}

	of_address_to_resource(np, 0, res);
	m48t37_data.ioaddr = ioremap(res->start, 1 + res->end - res->start);

	irq = irq_of_parse_and_map(np, 0);
	if (irq != NO_IRQ)
	{
		res[1].start = irq;
		res[1].end = irq;
		res[1].flags = IORESOURCE_IRQ;
	}
	else
		pr_warn("mvme5100_rtc_init: no IRQ for RTC\n");

	of_node_put(np);

	pr_info("Found RTC (m48t37) at i/o 0x%x irq %d\n", res->start, irq);

	result = platform_device_register(&m48t37_rtc);
	if (result < 0)
		pr_err("Failed to register RTC. Result: %d\n", result);

	return result;
}

arch_initcall(mvme5100_rtc_init);
#endif

static int __init probe_of_platform_devices(void)
{

	of_platform_bus_probe(NULL, mvme5100_of_bus_ids, NULL);
	return 0;
}

machine_device_initcall(mvme5100, probe_of_platform_devices);

define_machine(mvme5100) {
	.name			= "MVME5100",
	.probe			= mvme5100_probe,
	.setup_arch		= mvme5100_setup_arch,
	.init_IRQ		= mvme5100_pic_init,
	.show_cpuinfo		= mvme5100_show_cpuinfo,
	.get_irq		= mpic_get_irq,
	.restart		= mvme5100_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
};
