/*
 * MVME3100 setup and early boot code plus other random bits.
 *
 * Author: Ajit Prem
 *
 * Copyright 2007 Motorola Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>

#include <asm/of_device.h>
#include <asm/of_platform.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/prom.h>
#include <asm/mpic.h>
#include <mm/mmu_decl.h>
#include <asm/udbg.h>

#include <sysdev/fsl_pci.h>
#include <sysdev/fsl_soc.h>
#include "mvme3100.h"

extern void e500_enable_l1_data(void);
extern void e500_enable_l1_inst(void);

#define L2_ERR_REGS_OFFSET      0x20E20
#define L2_ERR_REGS_SIZE        0x3C

#define L2_CAPT_DATA_HI         0x20E20
#define L2_CAPT_DATA_LO         0x20E24
#define L2_CAPT_ECC             0x20E28
#define L2_ERR_DET              0x20E40
#define L2_ERR_DIS              0x20E44
#define L2_ERR_INT_EN           0x20E48
#define L2_ERR_ATTR             0x20E4C
#define L2_ERR_ADDR             0x20E50
#define L2_ERR_CTL              0x20E58

static struct of_device_id mpc85xx_ids[] = {
	{.type = "soc",},
	{.compatible = "soc",},
	{},
};

static unsigned long hb_baddr;
static unsigned int bus_frequency;

static void __iomem *l2_err_regs;
#ifdef CONFIG_MVME3100_ENABLE_L2_ERRORS
static int l2_irq;
static irqreturn_t mvme3100_l2cache_err_handler(int irq, void *dev_id);
#endif

#ifdef CONFIG_MVME3100_ENABLE_DDR_ERRORS

#define DDR_ERR_REGS_OFFSET     0x2E00
#define DDR_ERR_REGS_SIZE       0x5C

#define DDR_CAPT_DATA_HI        0x2E20
#define DDR_CAPT_DATA_LO        0x2E24
#define DDR_CAPT_ECC            0x2E28
#define DDR_ERR_DET             0x2E40
#define DDR_ERR_DIS             0x2E44
#define DDR_ERR_INT_EN          0x2E48
#define DDR_CAPT_ATTR           0x2E4C
#define DDR_CAPT_ADDR           0x2E50
#define DDR_ERR_SBE             0x2E58

static void __iomem *ddr_err_regs;
static int ddr_irq;
static irqreturn_t mvme3100_ddr_err_handler(int irq, void *dev_id);

#endif

#ifdef CONFIG_MVME3100_ENABLE_PCI_ERRORS

#define PCI_ERR_REGS_OFFSET     0x8E00
#define PCI_ERR_REGS_SIZE       0x28

#define PCI_ERR_DET             0x8E00
#define PCI_ERR_CAP_DIS         0x8E04
#define PCI_ERR_INT_EN          0x8E08
#define PCI_ERR_ATTR            0x8E0C
#define PCI_ERR_ADDR            0x8E10
#define PCI_ERR_EXT_ADDR        0x8E14
#define PCI_ERR_DL              0x8E18
#define PCI_ERR_DH              0x8E1C
#define PCI_GAS_TIMR            0x8E20
#define PCI_PCIX_TIMR           0x8E24

static int pci_errors;
static void __iomem *pci_err_regs;
static int pci_irq;
static irqreturn_t mvme3100_pci_err_handler(int irq, void *dev_id);

#endif

#ifdef CONFIG_VME_BRIDGE_BOOTMEM
void *vme_driver_bootmem;
unsigned int vme_bootmem_size;
#endif

#ifdef CONFIG_MVME3100_ENABLE_DDR_ERRORS

static irqreturn_t mvme3100_ddr_err_handler(int irq, void *dev_id)
{
	u32 err_addr, data_hi, data_lo, ecc, err_attr, err_det, sbe_count;

	/* Collect error information from the error registers */
	err_addr = readl(ddr_err_regs + DDR_CAPT_ADDR - DDR_ERR_REGS_OFFSET);
	data_hi = readl(ddr_err_regs + DDR_CAPT_DATA_HI - DDR_ERR_REGS_OFFSET);
	data_lo = readl(ddr_err_regs + DDR_CAPT_DATA_LO - DDR_ERR_REGS_OFFSET);
	err_attr = readl(ddr_err_regs + DDR_CAPT_ATTR - DDR_ERR_REGS_OFFSET);
	ecc = readl(ddr_err_regs + DDR_CAPT_ECC - DDR_ERR_REGS_OFFSET);
	sbe_count = readl(ddr_err_regs + DDR_ERR_SBE - DDR_ERR_REGS_OFFSET);
	err_det = readl(ddr_err_regs + DDR_ERR_DET - DDR_ERR_REGS_OFFSET);

	/* Clear interrupt cause and capture registers */
	writel(err_det, ddr_err_regs + DDR_ERR_DET - DDR_ERR_REGS_OFFSET);
	writel(sbe_count & ~(swab32(0xFFFF)),
	       ddr_err_regs + DDR_ERR_SBE - DDR_ERR_REGS_OFFSET);
	writel(0, ddr_err_regs + DDR_CAPT_DATA_HI - DDR_ERR_REGS_OFFSET);
	writel(0, ddr_err_regs + DDR_CAPT_DATA_LO - DDR_ERR_REGS_OFFSET);
	/* Display error information */
	printk(KERN_ERR "DDR Error!\n");
	printk(KERN_ERR "DDR ERROR DETECT REG 0x%08x\n", swab32(err_det));
	printk(KERN_ERR "DDR ERROR ADDRESS CAPTURE REG 0x%08x\n",
	       swab32(err_addr));
	printk(KERN_ERR "DDR ERROR DATA HIGH CAPTURE REG 0x%08x\n",
	       swab32(data_hi));
	printk(KERN_ERR "DDR ERROR DATA LOW CAPTURE REG 0x%08x\n",
	       swab32(data_lo));
	printk(KERN_ERR "DDR ERROR ATTRIBUTES CAPTURE REG 0x%08x\n",
	       swab32(err_attr));
	printk(KERN_ERR "DDR ERROR SYNDROME REG 0x%08x\n", swab32(ecc));
	printk(KERN_ERR "DDR ERROR SBE REG 0x%08x\n", swab32(sbe_count));

	return IRQ_HANDLED;
}

#endif

#ifdef CONFIG_MVME3100_ENABLE_L2_ERRORS

static irqreturn_t mvme3100_l2cache_err_handler(int irq, void *dev_id)
{
	u32 err_addr, data_hi, data_lo, ecc, err_attr, err_det;

	/* Collect error information from the error registers */
	err_det = readl(l2_err_regs + L2_ERR_DET - L2_ERR_REGS_OFFSET);
	err_attr = readl(l2_err_regs + L2_ERR_ATTR - L2_ERR_REGS_OFFSET);
	err_addr = readl(l2_err_regs + L2_ERR_ADDR - L2_ERR_REGS_OFFSET);
	data_hi = readl(l2_err_regs + L2_CAPT_DATA_HI - L2_ERR_REGS_OFFSET);
	data_lo = readl(l2_err_regs + L2_CAPT_DATA_LO - L2_ERR_REGS_OFFSET);
	ecc = readl(l2_err_regs + L2_CAPT_ECC - L2_ERR_REGS_OFFSET);

	/* Clear interrupt cause and capture registers */
	writel(err_det, l2_err_regs + L2_ERR_DET - L2_ERR_REGS_OFFSET);
	writel(0, l2_err_regs + L2_ERR_ATTR - L2_ERR_REGS_OFFSET);
	writel(0, l2_err_regs + L2_ERR_ADDR - L2_ERR_REGS_OFFSET);
	writel(0, l2_err_regs + L2_CAPT_DATA_HI - L2_ERR_REGS_OFFSET);
	writel(0, l2_err_regs + L2_CAPT_DATA_LO - L2_ERR_REGS_OFFSET);
	writel(0, l2_err_regs + L2_CAPT_ECC - L2_ERR_REGS_OFFSET);

	/* Display error information */
	printk(KERN_ERR "L2 Cache Error!\n");
	printk(KERN_ERR "L2 ERROR DETECT REG 0x%08x\n", swab32(err_det));
	printk(KERN_ERR "L2 ERROR ADDRESS CAPTURE REG 0x%08x\n",
	       swab32(err_addr));
	printk(KERN_ERR "L2 ERROR DATA HIGH CAPTURE REG 0x%08x\n",
	       swab32(data_hi));
	printk(KERN_ERR "L2 ERROR DATA LOW CAPTURE REG 0x%08x\n",
	       swab32(data_lo));
	printk(KERN_ERR "L2 ERROR ATTRIBUTES CAPTURE REG 0x%08x\n",
	       swab32(err_attr));
	printk(KERN_ERR "L2 ERROR SYNDROME REG 0x%08x\n", swab32(ecc));

	return IRQ_HANDLED;
}

#endif

#ifdef CONFIG_MVME3100_ENABLE_PCI_ERRORS

static irqreturn_t mvme3100_pci_err_handler(int irq, void *dev_id)
{
	u32 err_addr, data_hi, data_lo, err_attr, err_ext_addr, err_det;
	u32 gas_timr, pcix_timr;
	u16 pci_status;

	/* Collect error information from the error registers */
	err_det = readl(pci_err_regs + PCI_ERR_DET - PCI_ERR_REGS_OFFSET);
	err_addr = readl(pci_err_regs + PCI_ERR_ADDR - PCI_ERR_REGS_OFFSET);
	err_ext_addr =
	    readl(pci_err_regs + PCI_ERR_EXT_ADDR - PCI_ERR_REGS_OFFSET);
	err_attr = readl(pci_err_regs + PCI_ERR_ATTR - PCI_ERR_REGS_OFFSET);
	data_lo = readl(pci_err_regs + PCI_ERR_DL - PCI_ERR_REGS_OFFSET);
	data_hi = readl(pci_err_regs + PCI_ERR_DH - PCI_ERR_REGS_OFFSET);
	gas_timr = readl(pci_err_regs + PCI_GAS_TIMR - PCI_ERR_REGS_OFFSET);
	pcix_timr = readl(pci_err_regs + PCI_PCIX_TIMR - PCI_ERR_REGS_OFFSET);
//      pci_bus_read_config_word(0, 0, PCI_STATUS, &pci_status);
	early_read_config_word(0, 0, 0, PCI_STATUS, &pci_status);

	/* Clear interrupt cause and capture registers */
//      writel(err_det, pci_err_regs + PCI_ERR_DET - PCI_ERR_REGS_OFFSET);
	writel(0xFFFFFFFF, pci_err_regs + PCI_ERR_DET - PCI_ERR_REGS_OFFSET);
	writel(0, pci_err_regs + PCI_ERR_ADDR - PCI_ERR_REGS_OFFSET);
	writel(0, pci_err_regs + PCI_ERR_EXT_ADDR - PCI_ERR_REGS_OFFSET);
	writel(0, pci_err_regs + PCI_ERR_ATTR - PCI_ERR_REGS_OFFSET);
	writel(0, pci_err_regs + PCI_ERR_DL - PCI_ERR_REGS_OFFSET);
	writel(0, pci_err_regs + PCI_ERR_DH - PCI_ERR_REGS_OFFSET);
	if (pci_status & PCI_STATUS_PARITY) {
		pci_status &= ~PCI_STATUS_PARITY;
//              pci_bus_write_config_word(0, 0, PCI_STATUS, pci_status);
		early_write_config_word(0, 0, 0, PCI_STATUS, pci_status);
	}

	/* Display error information */
	if (pci_errors != 0) {
		printk(KERN_ERR "PCI Error!\n");
		printk(KERN_ERR "PCI ERROR DETECT REG 0x%08x\n",
		       swab32(err_det));
		printk(KERN_ERR "PCI ERROR ADDRESS REG 0x%08x\n",
		       swab32(err_addr));
		printk(KERN_ERR "PCI ERROR EXT ADDRESS REG 0x%08x\n",
		       swab32(err_ext_addr));
		printk(KERN_ERR "PCI ERROR ATTRIBUTES REG 0x%08x\n",
		       swab32(err_attr));
		printk(KERN_ERR "PCI ERROR DATA HIGH REG 0x%08x\n",
		       swab32(data_hi));
		printk(KERN_ERR "PCI ERROR DATA LOW REG 0x%08x\n",
		       swab32(data_lo));
		printk(KERN_ERR "PCI STATUS REG 0x%04x\n", swab16(pci_status));
		printk(KERN_ERR "PCI GASKET TIMER REG 0x%08x\n",
		       swab32(gas_timr));
		printk(KERN_ERR "PCI PCIX TIMER REG 0x%08x\n",
		       swab32(pcix_timr));
	}
	pci_errors++;

	return IRQ_HANDLED;
}

#endif

static int __init mpc85xx_publish_devices(void)
{
	if (!machine_is(MVME3100))
		return 0;

	of_platform_bus_probe(NULL, mpc85xx_ids, NULL);

	return 0;
}

device_initcall(mpc85xx_publish_devices);

int mvme3100_exclude_device(struct pci_controller *hose, u_char bus, u_char devfn)
{
	if (bus == 0 && PCI_SLOT(devfn) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;
	else
		return PCIBIOS_SUCCESSFUL;
}

static void __init mvme3100_misc_init(void)
{
	void __iomem *reg_addr;
	u8 reg_value;
	u32 sbe_count;

#ifdef CONFIG_MVME3100_ENABLE_DDR_ERRORS	/* Enable DDR error reporting */
	ddr_err_regs =
	    ioremap(hb_baddr + DDR_ERR_REGS_OFFSET, DDR_ERR_REGS_SIZE);
	writel(0, ddr_err_regs + DDR_ERR_DIS - DDR_ERR_REGS_OFFSET);
	/* Clear logged DDR errors */
	writel(swab32(0x8000000D),
	       ddr_err_regs + DDR_ERR_DET - DDR_ERR_REGS_OFFSET);
	/* Set single-bit error threshold */
	sbe_count = readl(ddr_err_regs + DDR_ERR_SBE - DDR_ERR_REGS_OFFSET);
	sbe_count = swab32(sbe_count) & 0xFF00FFFF;
	sbe_count |= 0x00100000;
	writel(swab32(sbe_count),
	       ddr_err_regs + DDR_ERR_SBE - DDR_ERR_REGS_OFFSET);
#endif

#ifdef CONFIG_MVME3100_ENABLE_PCI_ERRORS
	/* Enable PCI error reporting */
	pci_err_regs = ioremap(hb_baddr + PCI_ERR_REGS_OFFSET,
			       PCI_ERR_REGS_SIZE);
	writel(0, pci_err_regs + PCI_ERR_CAP_DIS - PCI_ERR_REGS_OFFSET);
#endif

#ifdef CONFIG_VME_BRIDGE
	{
		extern void vmemod_setup_options(char *);

		vmemod_setup_options(cmd_line);
	}
#endif

	reg_addr = ioremap(MVME3100_SYSTEM_CONTROL_REG, 3);

	/* Mask FEC PHY interrupts */
	reg_value = readb(reg_addr);
	reg_value |= MVME3100_FEC_PHY_MASK;
	writeb(reg_value, reg_addr);

	/* Turn off Board Fail LED */
	reg_value = readb(++reg_addr);
	reg_value &= ~MVME3100_BRD_FAIL_LED;
	writeb(reg_value, reg_addr);

	/* Turn off SW flash write-protect */
	reg_value = readb(++reg_addr);
	reg_value &= ~MVME3100_FLASH_WP_SW;
	writeb(reg_value, reg_addr);

	iounmap(reg_addr);
	return;
}

void __init mvme3100_pic_init(void)
{
	struct mpic *mpic;
	struct resource r;
	struct device_node *np = NULL;
	int irq;

	np = of_find_node_by_type(np, "open-pic");

	if (np == NULL) {
		printk(KERN_ERR "Could not find open-pic node\n");
		return;
	}

	if (of_address_to_resource(np, 0, &r)) {
		printk(KERN_ERR "Could not map mpic register space\n");
		of_node_put(np);
		return;
	}

	mpic = mpic_alloc(np, r.start,
			  MPIC_PRIMARY | MPIC_WANTS_RESET | MPIC_BIG_ENDIAN,
			  4, 60, " OpenPIC  ");
	BUG_ON(mpic == NULL);
	of_node_put(np);

	mpic_assign_isu(mpic, 0, r.start + 0x10200);
	mpic_assign_isu(mpic, 1, r.start + 0x10280);
	mpic_assign_isu(mpic, 2, r.start + 0x10300);
	mpic_assign_isu(mpic, 3, r.start + 0x10380);
	mpic_assign_isu(mpic, 4, r.start + 0x10400);
	mpic_assign_isu(mpic, 5, r.start + 0x10480);
	mpic_assign_isu(mpic, 6, r.start + 0x10500);
	mpic_assign_isu(mpic, 7, r.start + 0x10580);
	mpic_assign_isu(mpic, 8, r.start + 0x10600);
	mpic_assign_isu(mpic, 9, r.start + 0x10680);
	mpic_assign_isu(mpic, 10, r.start + 0x10700);
	mpic_assign_isu(mpic, 11, r.start + 0x10780);

	/* External Interrupts */
	mpic_assign_isu(mpic, 12, r.start + 0x10000);
	mpic_assign_isu(mpic, 13, r.start + 0x10080);
	mpic_assign_isu(mpic, 14, r.start + 0x10100);

	mpic_init(mpic);

#ifdef CONFIG_MVME3100_ENABLE_L2_ERRORS
	np = of_find_node_by_type(NULL, "l2_cache");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find l2_cache node\n");
	} else {
		l2_irq = irq_of_parse_and_map(np, 0);
		if (l2_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map L2 cache irq\n");
		of_node_put(np);
	}
#endif

#ifdef CONFIG_MVME3100_ENABLE_DDR_ERRORS
	np = of_find_node_by_type(NULL, "ddr");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find ddr node\n");
	} else {
		ddr_irq = irq_of_parse_and_map(np, 0);
		if (ddr_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map DDR irq\n");
		of_node_put(np);
	}
#endif

#ifdef CONFIG_MVME3100_ENABLE_PCI_ERRORS
	np = of_find_node_by_type(NULL, "pci");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find pci node\n");
	} else {
		pci_irq = irq_of_parse_and_map(np, 0);
		if (pci_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map PCI irq\n");
		of_node_put(np);
	}
#endif

#ifdef CONFIG_SENSORS_DS1621
	np = of_find_node_by_type(NULL, "thermostat");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find ds1621 node\n");
	} else {
		irq = irq_of_parse_and_map(np, 0);
		if (irq == NO_IRQ)
			printk(KERN_ERR "Unable to map DS1621 irq\n");
		of_node_put(np);
	}
#endif

#ifdef CONFIG_MVME3100_TICK_TIMERS
	np = of_find_node_by_type(NULL, "board_timers");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find timers node\n");
	} else {
		irq = irq_of_parse_and_map(np, 0);
		if (irq == NO_IRQ)
			printk(KERN_ERR "Unable to map timers irq\n");
		of_node_put(np);
	}
#endif

	np = of_find_node_by_path("/serial@e2011000");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find serial@e2011000 node\n");
	} else {
		irq = irq_of_parse_and_map(np, 0);
		if (irq == NO_IRQ)
			printk(KERN_ERR "Unable to map serial@e2011000 irq\n");
		of_node_put(np);
	}

	np = of_find_node_by_path("/serial@e2012000");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find serial@e2012000 node\n");
	} else {
		irq = irq_of_parse_and_map(np, 0);
		if (irq == NO_IRQ)
			printk(KERN_ERR "Unable to map serial@e2012000 irq\n");
		of_node_put(np);
	}

	np = of_find_node_by_path("/serial@e2013000");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find serial@e2013000 node\n");
	} else {
		irq = irq_of_parse_and_map(np, 0);
		if (irq == NO_IRQ)
			printk(KERN_ERR "Unable to map serial@e2013000 irq\n");
		of_node_put(np);
	}

	np = of_find_node_by_path("/serial@e2014000");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find serial@e2014000 node\n");
	} else {
		irq = irq_of_parse_and_map(np, 0);
		if (irq == NO_IRQ)
			printk(KERN_ERR "Unable to map serial@e2014000 irq\n");
		of_node_put(np);
	}

	return;
}

#ifdef CONFIG_MPC85xx_DMA
static const char *mpc85xx_dma0_intr = "mpc85xx_dma0_intr";
static const char *mpc85xx_dma1_intr = "mpc85xx_dma1_intr";
static const char *mpc85xx_dma2_intr = "mpc85xx_dma2_intr";
static const char *mpc85xx_dma3_intr = "mpc85xx_dma3_intr";

struct fsl_dma_platform_data {
	u32 device_flags;
};

static int __init fsl_dma_of_init(void)
{
	struct device_node *np = NULL;
	struct resource res[5];
	struct platform_device *dma_dev;
	struct fsl_dma_platform_data dma_data;
	int ret = 0;

	np = of_find_node_by_type(NULL, "soc_dma");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find soc_dma node\n");
		return -ENODEV;
	}
	memset(&res, 0, sizeof(res));
	memset(&dma_data, 0, sizeof(dma_data));
	ret = of_address_to_resource(np, 0, &res[0]);
	if (ret)
		goto err;
	of_irq_to_resource(np, 0, &res[1]);
	res[1].name = mpc85xx_dma0_intr;
	of_irq_to_resource(np, 1, &res[2]);
	res[2].name = mpc85xx_dma1_intr;
	of_irq_to_resource(np, 2, &res[3]);
	res[3].name = mpc85xx_dma2_intr;
	of_irq_to_resource(np, 3, &res[4]);
	res[4].name = mpc85xx_dma3_intr;
	dma_dev = platform_device_register_simple("mpc85xx_dma", 0, &res[0], 5);
	if (IS_ERR(dma_dev)) {
		ret = PTR_ERR(dma_dev);
		goto err;
	}
	dma_data.device_flags = 0;
	ret = platform_device_add_data(dma_dev, &dma_data,
				       sizeof(struct fsl_dma_platform_data));
	if (ret)
		goto err;
	of_node_put(np);
	return 0;
err:
	of_node_put(np);
	return ret;
}

arch_initcall(fsl_dma_of_init);
#endif

#if 0
static void __init mvme3100_init_early(void)
{
	settlbcam(num_tlbcam_entries - 1, 0xe1000000,
		  0xe1000000, 1024 * 1024, _PAGE_IO, 0);
}
#endif

static void __init mvme3100_pci_fixups(void)
{
	struct pci_dev *dev = NULL;
	int ret;

	while ((dev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) {
		/* Update all devices' cache line size */
		pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE,
				      L1_CACHE_BYTES >> 2);
	}

	if ((dev = pci_find_device(PCI_VENDOR_ID_NEC,
				   PCI_DEVICE_ID_NEC_USB, NULL))) {
		/* Set System Clock to 48MHz oscillator in EXT2 register */
		printk(KERN_ERR "Fixing NEC US EXT2 register\n");
		pci_write_config_byte(dev, 0xe4, 0x20);
	}

	if ((dev = pci_find_device(PCI_VENDOR_ID_DEC,
				   PCI_DEVICE_ID_DEC_21150, NULL))) {
		pci_write_config_byte(dev, PCI_LATENCY_TIMER, 0x80);
		pci_write_config_byte(dev, PCI_SEC_LATENCY_TIMER, 0x80);
	}
#ifdef CONFIG_MVME3100_ENABLE_L2_ERRORS
	if (request_irq(l2_irq, mvme3100_l2cache_err_handler,
			IRQF_DISABLED, "L2 Cache Error",
			(void *)&l2_err_regs) < 0) {
		printk(KERN_ERR "Cannot install L2 cache error handler\n");
	} else
		writel(swab32(0x1D),
		       l2_err_regs + L2_ERR_INT_EN - L2_ERR_REGS_OFFSET);
#endif

#ifdef CONFIG_MVME3100_ENABLE_DDR_ERRORS
	if ((ret = request_irq(ddr_irq, mvme3100_ddr_err_handler,
			       IRQF_DISABLED, "DDR Error",
			       (void *)&ddr_err_regs)) < 0) {
		printk(KERN_ERR "Cannot install DDR error handler. ret =0x%x\n",
		       -ret);
	} else
		writel(swab32(0xD),
		       ddr_err_regs + DDR_ERR_INT_EN - DDR_ERR_REGS_OFFSET);
#endif

#ifdef CONFIG_MVME3100_ENABLE_PCI_ERRORS
	if (request_irq(pci_irq, mvme3100_pci_err_handler,
			IRQF_DISABLED, "PCI Error",
			(void *)&pci_err_regs) < 0) {
		printk(KERN_ERR "Cannot install PCI error handler\n");
	} else
		writel(swab32(0x7FF),
		       pci_err_regs + PCI_ERR_INT_EN - PCI_ERR_REGS_OFFSET);
#endif
}

static void mvme3100_init_caches(void)
{
	void __iomem *l2_ctl_addr;
	struct device_node *dn = NULL;
	const u32 *regs;

	dn = of_find_node_by_type(dn, "soc");
	if (dn == NULL) {
		printk("Error:Cannot find soc node\n");
		return;
	}
	regs = get_property(dn, "reg", NULL);
	if (regs == NULL) {
		printk("Error:Cannot get reg property from soc node\n");
		return;
	} else {
		hb_baddr = regs[0];
	}

	l2_err_regs = ioremap(hb_baddr + L2_ERR_REGS_OFFSET, L2_ERR_REGS_SIZE);

	l2_ctl_addr = ioremap(hb_baddr + 0x20000, 4);

	/* L1 inst, L1 data, and L2 are disabled at this point */

	e500_enable_l1_inst();
	e500_enable_l1_data();
	asm volatile ("eieio");
	asm volatile ("isync");
	writel(0, l2_ctl_addr);
	readl(l2_ctl_addr);
	asm volatile ("eieio");
	writel(0x00030048, l2_ctl_addr);
	readl(l2_ctl_addr);
	asm volatile ("eieio");
	/* Enable L2 error detection */
	writel(0, l2_err_regs + L2_ERR_DIS - L2_ERR_REGS_OFFSET);
	readl(l2_ctl_addr);
	asm volatile ("eieio");
	/* Clear any logged L2 errors */
	writel(0x1D000080, l2_err_regs + L2_ERR_DET - L2_ERR_REGS_OFFSET);
	readl(l2_ctl_addr);
	asm volatile ("eieio");
	/* Enable L2 */
	writel(0x00070088, l2_ctl_addr);
	readl(l2_ctl_addr);
	asm volatile ("eieio");

	printk
	    ("L2 cache enabled (256K cache, no memory-mapped SRAM). L2CTL reg: 0x%x\n",
	     swab32(readl(l2_ctl_addr)));

	iounmap(l2_ctl_addr);
	of_node_put(dn);

	return;
}

/* Register a platform device for MTD. */
#if 0
static int __init mvme3100_register_mtd(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, "rom", "direct-mapped");
	of_platform_device_create(np, np->name, NULL);
}

arch_initcall(mvme3100_register_mtd);
#endif

#if defined(CONFIG_I2C_MPC) && defined(CONFIG_SENSORS_DS1375)
extern void ds1375_get_rtc_time(struct rtc_time *rtc_tm);
extern int ds1375_set_rtc_time(struct rtc_time *tm);

static int __init mvme3100_rtc_hookup(void)
{
	struct timespec tv;
	struct rtc_time tm;

	ppc_md.get_rtc_time = ds1375_get_rtc_time;
	ppc_md.set_rtc_time = ds1375_set_rtc_time;
#if 0
	ppc_md.nvram_read_val = ds1375_read_val;
	ppc_md.nvram_write_val = ds1375_write_val;
#endif
	tv.tv_nsec = 0;
	(ppc_md.get_rtc_time) ((struct rtc_time *)&tm);
	tv.tv_sec = mktime(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			   tm.tm_hour, tm.tm_min, tm.tm_sec);
	do_settimeofday(&tv);

	return 0;
}

late_initcall(mvme3100_rtc_hookup);
#endif

static void __init mvme3100_setup_arch(void)
{
	struct device_node *cpu;
	struct device_node *np;

	mvme3100_init_caches();

	cpu = of_find_node_by_type(NULL, "cpu");
	if (cpu != 0) {
		const unsigned int *fp;

		fp = get_property(cpu, "clock-frequency", NULL);
		if (fp != 0)
			loops_per_jiffy = *fp / HZ;
		else
			loops_per_jiffy = 50000000 / HZ;
		fp = get_property(cpu, "bus-frequency", NULL);
		if (fp != 0)
			bus_frequency = *fp;
		else
			bus_frequency = 333333333;
		of_node_put(cpu);
	}

	for (np = NULL; (np = of_find_node_by_type(np, "pci")) != NULL;)
		fsl_add_bridge(np, 1);
	ppc_md.pci_exclude_device = mvme3100_exclude_device;

#ifdef  CONFIG_ROOT_NFS
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

	mvme3100_misc_init();
}

#define SPRN_L1CFG0     0x203   /* L1 Cache Configuration Register 0  */
#define SPRN_L1CFG1     0x204   /* L1 Cache Configuration Register 1  */

void mvme3100_show_cpuinfo(struct seq_file *m)
{
	uint pvid, svid, phid1;
	uint memsize = total_memory;
	phys_addr_t immr_base;
	void __iomem *reg_addr, *reg_block;
	void __iomem *l2cache_addr;
	u8 reg_value;
	u32 reg32_value;
	u32 l1csr0, l1csr1, l2ctl;
	char buff[16] = "";

	pvid = mfspr(SPRN_PVR);
	svid = mfspr(SPRN_SVR);
	immr_base = get_immrbase();

	seq_printf(m, "Vendor\t\t: Motorola\n");
	seq_printf(m, "Machine\t\t: MVME3100\n");
	seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
	seq_printf(m, "SVR\t\t: 0x%x\n", svid);
	seq_printf(m, "HID0\t\t: 0x%08lx\n", mfspr(SPRN_HID0));
	seq_printf(m, "HID1\t\t: 0x%08lx\n", mfspr(SPRN_HID1));
	seq_printf(m, "CCSR Base\t: 0x%08lx\n", immr_base);
	seq_printf(m, "Bus Frequency\t: %08d\n", bus_frequency);

	/* Display cpu Pll setting */
	phid1 = mfspr(SPRN_HID1);
	seq_printf(m, "PLL setting\t: 0x%x\n", ((phid1 >> 24) & 0x3f));

	/* Display the cache settings */
	seq_printf(m, "\nCache Settings:\n");
	l1csr0 = mfspr(SPRN_L1CSR0);
	seq_printf(m, "L1CSR0 (L1 D-cache)\t: 0x%08x (%s)\n", l1csr0,
		   (l1csr0 & 1) ? "on" : "off");
	l1csr1 = mfspr(SPRN_L1CSR1);
	seq_printf(m, "L1CSR1 (L1 I-cache)\t: 0x%08x (%s)\n", l1csr1,
		   (l1csr1 & 1) ? "on" : "off");
	seq_printf(m, "L1CFG0\t\t\t: 0x%08lx\n", mfspr(SPRN_L1CFG0));
	seq_printf(m, "L1CFG1\t\t\t: 0x%08lx\n", mfspr(SPRN_L1CFG1));

	l2cache_addr = ioremap(immr_base + 0x20000, 4);
	l2ctl = swab32(readl(l2cache_addr));
	seq_printf(m, "L2CTL (L2 cache)\t: 0x%x (%s)\n", l2ctl,
		   (l2ctl & 0x80000000) ? "on" : "off");
	iounmap(l2cache_addr);

	seq_printf(m, "\nBOARD INFORMATION:\n");
	reg_block = ioremap(MVME3100_SYSTEM_STATUS_REG, 16);
	reg_addr = reg_block;

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t\t: %d MB\n", memsize / (1024 * 1024));

	/* System Status Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Safe Start Status\t: %s\n",
		   ((reg_value & MVME3100_SAFE_START) ?
		    "Safe ENV settings used" : "NVRAM ENV settings used"));
	seq_printf(m, "Abort Status\t\t: %s\n",
		   ((reg_value & MVME3100_ABORT_STATUS) ?
		    "Abort Switch Asserted" : "Abort Switch Not Asserted"));
	switch (reg_value & MVME3100_BOARD_TYPE_MASK) {
	case MVME3100_BOARD_TYPE_PRPMC:
		sprintf(buff, "PrPMC");
		break;
	case MVME3100_BOARD_TYPE_VME:
		sprintf(buff, "VME");
		break;
	default:
		sprintf(buff, "Unknown");
		break;
	}
	seq_printf(m, "Board Type\t\t: %s\n", buff);

	/* System Control Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "EEPROM WP Status\t: %s\n",
		   ((reg_value & MVME3100_EEPROM_WP) ?
		    "EEPROM Write Protected" : "EEPROM Not Write Protected"));
	seq_printf(m, "DS1621 Thermostat\t: %s\n",
		   ((reg_value & MVME3100_TSTAT_MASK) ?
		    "Interrupt Disabled" : "Interrupt Enabled"));

	/* Status Indicator Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Board Fail LED\t\t: %s\n",
		   ((reg_value & MVME3100_BRD_FAIL_LED) ? "Lit" : "Not Lit"));
	seq_printf(m, "USR1 LED\t\t: %s\n",
		   ((reg_value & MVME3100_USR1_LED) ? "Lit" : "Not Lit"));
	seq_printf(m, "USR2 LED\t\t: %s\n",
		   ((reg_value & MVME3100_USR2_LED) ? "Lit" : "Not Lit"));
	seq_printf(m, "USR3 LED\t\t: %s\n",
		   ((reg_value & MVME3100_USR3_LED) ? "Lit" : "Not Lit"));

	/* Flash Control/Status Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "Flash Map Select\t: %s\n",
		   ((reg_value & MVME3100_FLASH_MAP_SELECT) ?
		    "Boot Block A Mapped to Highest Address" :
		    "Flash Memory Map Controlled by Flash Boot Block Select"));
	seq_printf(m, "Flash Write Protect SW\t: %s\n",
		   ((reg_value & MVME3100_FLASH_WP_SW) ?
		    "Write Protected" : "Not Write Protected"));
	seq_printf(m, "Flash Write Protect HW\t: %s\n",
		   ((reg_value & MVME3100_FLASH_WP_HW) ?
		    "Write Protected" : "Not Write Protected"));
	seq_printf(m, "Flash Boot Block Select\t: %s\n",
		   ((reg_value & MVME3100_FLASH_BLK_SEL) ?
		    "Boot Block B Selected" : "Boot Block A Selected"));
	seq_printf(m, "Flash Ready Status\t: %s\n",
		   ((reg_value & MVME3100_FLASH_RDY) ?
		    "Bit Set" : "Bit Not Set"));

	/* PCI Bus A Status Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "PCI Bus A 64 Bit\t: %s\n",
		   ((reg_value & MVME3100_PCI_BUS_A_64B) ?
		    "64 Bit Enabled" : "32 Bit Mode"));
	seq_printf(m, "PCI Bus A PCI-X\t\t: %s\n",
		   ((reg_value & MVME3100_PCI_BUS_A_PCIX) ?
		    "PCI-X Mode" : "PCI Mode"));
	switch (reg_value & MVME3100_PCI_BUS_A_SPD_MASK) {
	case MVME3100_PCI_BUS_A_SPD_133:
		sprintf(buff, "133 MHz");
		break;
	case MVME3100_PCI_BUS_A_SPD_100:
		sprintf(buff, "100 MHz");
		break;
	case MVME3100_PCI_BUS_A_SPD_66:
		sprintf(buff, "66 MHz");
		break;
	case MVME3100_PCI_BUS_A_SPD_33:
	default:
		sprintf(buff, "33 MHz");
		break;
	}
	seq_printf(m, "PCI Bus A Speed\t\t: %s\n", buff);

	/* PCI Bus B Status Register */
	reg_value = readb(reg_addr++);
	if (reg_value & MVME3100_PCI_BUS_B_3_3V_VIO) {
		seq_printf(m, "PCI Bus B 3.3V VIO\t: %s\n",
			   "Configured for 3.3V VIO");
	};
	if (reg_value & MVME3100_PCI_BUS_B_5_0V_VIO) {
		seq_printf(m, "PCI Bus B 5.0V VIO\t: %s\n",
			   "Configured for 5.0V VIO");
	};
	seq_printf(m, "PCI Bus B ERDY2\t\t: %d\n",
		   ((reg_value & MVME3100_PCI_BUS_B_ERDY2) ? 1 : 0));
	seq_printf(m, "PCI Bus B ERDY1\t\t: %d\n",
		   ((reg_value & MVME3100_PCI_BUS_B_ERDY1) ? 1 : 0));
	seq_printf(m, "PCI Bus B 64 Bit\t: %s\n",
		   ((reg_value & MVME3100_PCI_BUS_B_64B) ?
		    "64 Bit Enabled" : "32 Bit Mode"));
	seq_printf(m, "PCI Bus B PCI-X\t\t: %s\n",
		   ((reg_value & MVME3100_PCI_BUS_B_PCIX) ?
		    "PCI-X Mode" : "PCI Mode"));
	switch (reg_value & MVME3100_PCI_BUS_B_SPD_MASK) {
	case MVME3100_PCI_BUS_B_SPD_133:
		sprintf(buff, "133 MHz");
		break;
	case MVME3100_PCI_BUS_B_SPD_100:
		sprintf(buff, "100 MHz");
		break;
	case MVME3100_PCI_BUS_B_SPD_66:
		sprintf(buff, "66 MHz");
		break;
	case MVME3100_PCI_BUS_B_SPD_33:
	default:
		sprintf(buff, "33 MHz");
		break;
	}
	seq_printf(m, "PCI Bus B Speed\t\t: %s\n", buff);

	/* PCI Bus C Status Register */
	reg_value = readb(reg_addr++);
	seq_printf(m, "PCI Bus C 64 Bit\t: %s\n",
		   ((reg_value & MVME3100_PCI_BUS_C_64B) ?
		    "64 Bit Enabled" : "32 Bit Mode"));
	seq_printf(m, "PCI Bus C PCI-X\t\t: %s\n",
		   ((reg_value & MVME3100_PCI_BUS_C_PCIX) ?
		    "PCI-X Mode" : "PCI Mode"));
	switch (reg_value & MVME3100_PCI_BUS_C_SPD_MASK) {
	case MVME3100_PCI_BUS_C_SPD_133:
		sprintf(buff, "133 MHz");
		break;
	case MVME3100_PCI_BUS_C_SPD_100:
		sprintf(buff, "100 MHz");
		break;
	case MVME3100_PCI_BUS_C_SPD_66:
		sprintf(buff, "66 MHz");
		break;
	case MVME3100_PCI_BUS_C_SPD_33:
	default:
		sprintf(buff, "33 MHz");
		break;
	}
	seq_printf(m, "PCI Bus C Speed\t\t: %s\n", buff);

	/* Interrupt Detect Register */
	reg_value = readb(reg_addr++);

	if (reg_value & MVME3100_FEC_PHY_INTERRUPT) {
		seq_printf(m, "FEC PHY Interrupt\t: %s\n", "Asserted");
	};
	if (reg_value & MVME3100_TSEC2_PHY_INTERRUPT) {
		seq_printf(m, "TSEC2 PHY Interrupt\t: %s\n", "Asserted");
	};
	if (reg_value & MVME3100_TSEC1_PHY_INTERRUPT) {
		seq_printf(m, "TSEC1 PHY Interrupt\t: %s\n", "Asserted");
	};

	/* Presence Detect Register */
	reg_value = readb(reg_addr++);

	seq_printf(m, "PMCSPAN\t\t\t: %s\n",
		   ((reg_value & MVME3100_PMCSPAN_PRESENT) ?
		    "PMCSPAN Installed" : "PMCSPAN Not Installed"));
	seq_printf(m, "PMC Site 2\t\t: %s\n",
		   ((reg_value & MVME3100_PMC2_PRESENT) ?
		    "PMC Module Installed" : "PMC Module Not Installed"));
	seq_printf(m, "PMC Site 1\t\t: %s\n",
		   ((reg_value & MVME3100_PMC1_PRESENT) ?
		    "PMC Module Installed" : "PMC Module Not Installed"));

	/* PLD Revision Register */
	reg_value = readb(reg_addr++);

	seq_printf(m, "PLD Revision\t\t: 0x%x\n", reg_value);

	/* PLD Date Code Register */
	reg_addr++;
	reg_addr++;
	reg32_value = readl((u32 *) reg_addr);
	seq_printf(m, "PLD Date Code\t\t: 0x%08x\n",
		   (((reg32_value & 0x000000ffU) << 24) |
		    ((reg32_value & 0x0000ff00U) << 8) |
		    ((reg32_value & 0x00ff0000U) >> 8) |
		    ((reg32_value & 0xff000000U) >> 24)));

	iounmap(reg_block);

	return;
}

static void mvme3100_restart(char *cmd)
{
	volatile ulong i = 10000000;
	void __iomem *system_control_reg_addr;

	local_irq_disable();
	system_control_reg_addr = ioremap(MVME3100_SYSTEM_CONTROL_REG, 1);
	writeb(MVME3100_BOARD_RESET, system_control_reg_addr);

	while (i-- > 0) ;
	panic("restart failed\n");
}

/*
 * Called very early, device-tree isn't unflattened
 */
static int __init mvme3100_probe(void)
{
	/* We always match for now, eventually we should look at the flat
	   dev tree to ensure this is the board we are supposed to run on
	 */
	return 1;
}

define_machine(mvme3100)
{
	.name = "MVME3100",
	.probe = mvme3100_probe,
	.setup_arch = mvme3100_setup_arch,
	.init_IRQ = mvme3100_pic_init,
	.show_cpuinfo = mvme3100_show_cpuinfo,
	.pcibios_fixup = mvme3100_pci_fixups,
	.get_irq = mpic_get_irq,
	.restart = mvme3100_restart,
	.calibrate_decr = generic_calibrate_decr,
	.progress = udbg_progress,
	.progress = udbg_progress,
//      .init_early             = mvme3100_init_early,
};
