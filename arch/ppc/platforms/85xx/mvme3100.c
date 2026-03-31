/*
 * arch/ppc/platforms/85xx/mvme3100.c
 *
 * MVME3100 board specific routines
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2004-2007 Motorola Inc.
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
#include <linux/major.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/serial.h>
#include <linux/tty.h>		/* for linux/serial_core.h */
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/initrd.h>
#include <linux/module.h>
#include <linux/bootmem.h>
#include <linux/fsl_devices.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/open_pic.h>
#include <asm/bootinfo.h>
#include <asm/pci-bridge.h>
#include <asm/mpc85xx.h>
#include <asm/irq.h>
#include <asm/immap_85xx.h>
#include <asm/kgdb.h>
#include <asm/ppc_sys.h>
#include <mm/mmu_decl.h>

#include <syslib/ppc85xx_setup.h>
#include <platforms/85xx/mvme3100.h>

extern void e500_enable_l1_data(void);
extern void e500_enable_l1_inst(void);

#define L2_ERR_REGS_OFFSET	0x20E20
#define L2_ERR_REGS_SIZE	0x3C

#define L2_CAPT_DATA_HI		0x20E20
#define L2_CAPT_DATA_LO		0x20E24
#define L2_CAPT_ECC		0x20E28
#define L2_ERR_DET		0x20E40
#define L2_ERR_DIS		0x20E44
#define L2_ERR_INT_EN		0x20E48
#define L2_ERR_ATTR		0x20E4C
#define L2_ERR_ADDR		0x20E50
#define L2_ERR_CTL		0x20E58

static u8 *l2_err_regs;
#ifdef CONFIG_MVME3100_ENABLE_L2_ERRORS
static irqreturn_t mvme3100_l2cache_err_handler(int irq, void *dev_id);
#endif

#ifdef CONFIG_MVME3100_ENABLE_DDR_ERRORS

#define DDR_ERR_REGS_OFFSET	0x2E00
#define DDR_ERR_REGS_SIZE	0x5C

#define DDR_CAPT_DATA_HI	0x2E20
#define DDR_CAPT_DATA_LO	0x2E24
#define DDR_CAPT_ECC		0x2E28
#define DDR_ERR_DET		0x2E40
#define DDR_ERR_DIS		0x2E44
#define DDR_ERR_INT_EN		0x2E48
#define DDR_CAPT_ATTR		0x2E4C
#define DDR_CAPT_ADDR		0x2E50
#define DDR_ERR_SBE		0x2E58

static u8 *ddr_err_regs;
static irqreturn_t mvme3100_ddr_err_handler(int irq, void *dev_id);

#endif

#ifdef CONFIG_MVME3100_ENABLE_PCI_ERRORS

#define PCI_ERR_REGS_OFFSET	0x8E00
#define PCI_ERR_REGS_SIZE	0x28

#define PCI_ERR_DET		0x8E00
#define PCI_ERR_CAP_DIS		0x8E04
#define PCI_ERR_INT_EN		0x8E08
#define PCI_ERR_ATTR		0x8E0C
#define PCI_ERR_ADDR		0x8E10
#define PCI_ERR_EXT_ADDR	0x8E14
#define PCI_ERR_DL		0x8E18
#define PCI_ERR_DH		0x8E1C
#define PCI_GAS_TIMR		0x8E20
#define PCI_PCIX_TIMR		0x8E24

static int pci_errors;
static u8 *pci_err_regs;
static irqreturn_t mvme3100_pci_err_handler(int irq, void *dev_id);

#endif

#ifndef CONFIG_PCI
unsigned long isa_io_base = 0;
unsigned long isa_mem_base = 0;
#endif

extern unsigned long total_memory;	/* in mm/init */

#ifdef CONFIG_VME_BRIDGE_BOOTMEM
void *vme_driver_bootmem;
unsigned int vme_bootmem_size;
#endif

unsigned char __res[sizeof(bd_t)];

/* Internal interrupts are all Level Sensitive, and Positive Polarity */

static u_char mvme3100_openpic_initsenses[] __initdata = {
	MPC85XX_INTERNAL_IRQ_SENSES,
#if 0
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  0: L2 Cache */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  1: ECM */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  2: DDR DRAM */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  3: LBIU */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  4: DMA 0 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  5: DMA 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  6: DMA 2 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  7: DMA 3 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  8: PCI/PCI-X */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal  9: RIO Inbound Port Write Error */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 10: RIO Doorbell Inbound */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 11: RIO Outbound Message */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 12: RIO Inbound Message */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 13: TSEC 0 Transmit */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 14: TSEC 0 Receive */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 15: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 16: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 17: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 18: TSEC 0 Receive/Transmit Error */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 19: TSEC 1 Transmit */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 20: TSEC 1 Receive */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 21: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 22: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 23: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 24: TSEC 1 Receive/Transmit Error */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 25: Fast Ethernet */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 26: DUART */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 27: I2C */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 28: Performance Monitor */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 29: Unused */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 30: CPM */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_POSITIVE),	/* Internal 31: Unused */
#endif
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 0: VME 0 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 1: VME 1 */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 2: VME2/sATA */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 3: VME3/UART */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 4: PMCSpan/PMC/USB */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 5: PMCSPAN/PMC */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 6: PMCSPAN/PMC */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 7: PMCSPAN/PMC */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 8: ABORT */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 9: Temp Sensor */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 10: PHY */
	(IRQ_SENSE_LEVEL | IRQ_POLARITY_NEGATIVE),	/* External 11: DS1375 Alm */
};

int mvme3100_show_cpuinfo(struct seq_file *m)
{
	uint pvid, svid, phid1;
	uint memsize = total_memory;
	uint ifreq, bfreq;
	u8 *reg_addr, *reg_block;
	u8 *l2cache_addr;
	bd_t *binfo = (bd_t *) __res;
	u8 reg_value;
	u32 reg32_value;
	u32 l1csr0, l1csr1, l2ctl;
	char buff[16] = "";

	pvid = mfspr(SPRN_PVR);
	svid = mfspr(SPRN_SVR);

	seq_printf(m, "Vendor\t\t: Motorola ECC\n");

	switch (pvid & 0xffff0000) {
	case PVR_8540:
		seq_printf(m, "Machine\t\t: MVME3100\n");
		break;
	default:
		seq_printf(m, "Machine\t\t: unknown\n");
		break;
	}
	ifreq = BINFO_INTFREQ;
	bfreq = BINFO_BUSFREQ;
	seq_printf(m, "int freq\t: %u.%.6u MHz\n", ifreq / 1000000,
		   ifreq % 1000000);
	seq_printf(m, "bus freq\t: %u.%.6u MHz\n", bfreq / 1000000,
		   bfreq % 1000000);
	seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
	seq_printf(m, "SVR\t\t: 0x%x\n", svid);
	seq_printf(m, "HID0\t\t: 0x%08lx\n", mfspr(SPRN_HID0));
	seq_printf(m, "HID1\t\t: 0x%08lx\n", mfspr(SPRN_HID1));
	seq_printf(m, "CCSR Base\t: 0x%08lx\n", BINFO_IMMR_BASE);

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

	l2cache_addr = ioremap(binfo->bi_immr_base + 0x20000, 4);
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

	return 0;
}

void __init mvme3100_init_IRQ(void)
{
	/* Determine the Physical Address of the OpenPIC regs */
	phys_addr_t OpenPIC_PAddr = BINFO_IMMR_BASE + MPC85xx_OPENPIC_OFFSET;

	OpenPIC_Addr = ioremap(OpenPIC_PAddr, MPC85xx_OPENPIC_SIZE);
	OpenPIC_InitSenses = mvme3100_openpic_initsenses;
	OpenPIC_NumInitSenses = sizeof(mvme3100_openpic_initsenses);

	/* Skip reserved space and internal sources */
	openpic_set_sources(0, 32, OpenPIC_Addr + 0x10200);
	/* Map PIC IRQs 0-11 */
	openpic_set_sources(48, 12, OpenPIC_Addr + 0x10000);

	openpic_init(MPC85xx_OPENPIC_IRQ_OFFSET);

	return;
}

int mpc85xx_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
	static char pci_irq_table[][4] =
	    /*
	     *      PCI IDSEL/INTPIN->INTLINE
	     *       A      B      C      D
	     */
	{
		{MPC85xx_IRQ_EXT0, MPC85xx_IRQ_EXT1, MPC85xx_IRQ_EXT2, MPC85xx_IRQ_EXT3},	/* IDSEL 1 - TSI148 */
		{MPC85xx_IRQ_EXT4, MPC85xx_IRQ_EXT5, MPC85xx_IRQ_EXT6, MPC85xx_IRQ_EXT7},	/* IDSEL 2 - PCI6520-1 */
		{MPC85xx_IRQ_EXT4, MPC85xx_IRQ_EXT5, MPC85xx_IRQ_EXT6, MPC85xx_IRQ_EXT7},	/* IDSEL 3 - PCI6520-2 */
		{MPC85xx_IRQ_EXT2, 0, 0, 0},	/* IDSEL 4 - GD31244 SATA */
		{0, 0, 0, 0},	/* IDSEL 5 */
		{0, 0, 0, 0},	/* IDSEL 6 */
		{0, 0, 0, 0},	/* IDSEL 7 */
		{0, 0, 0, 0},	/* IDSEL 8 */
		{0, 0, 0, 0},	/* IDSEL 9 */
		{0, 0, 0, 0},	/* IDSEL 10 */
		{0, 0, 0, 0},	/* IDSEL 11 */
		{0, 0, 0, 0},	/* IDSEL 12 */
		{0, 0, 0, 0},	/* IDSEL 13 */
		{0, 0, 0, 0},	/* IDSEL 14 */
		{0, 0, 0, 0},	/* IDSEL 15 */
		{0, 0, 0, 0},	/* IDSEL 16 */
		{MPC85xx_IRQ_EXT0, MPC85xx_IRQ_EXT1, MPC85xx_IRQ_EXT2, MPC85xx_IRQ_EXT3},	/* IDSEL 17 - TSI148 */
		{MPC85xx_IRQ_EXT4, MPC85xx_IRQ_EXT5, MPC85xx_IRQ_EXT6, MPC85xx_IRQ_EXT7},	/* IDSEL 18 - PCI6520-1 */
		{MPC85xx_IRQ_EXT4, MPC85xx_IRQ_EXT5, MPC85xx_IRQ_EXT6, MPC85xx_IRQ_EXT7},	/* IDSEL 19 - PCI6520-2 */
		{MPC85xx_IRQ_EXT2, 0, 0, 0},	/* IDSEL 20 - GD31244 SATA */
	};

	const long min_idsel = 1, max_idsel = 20, irqs_per_slot = 4;
	return PCI_IRQ_TABLE_LOOKUP;
}

int mpc85xx_exclude_device(u_char bus, u_char devfn)
{
	if (bus == 0 && PCI_SLOT(devfn) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;
	else
		return PCIBIOS_SUCCESSFUL;
}

static void __init mvme3100_pci_fixups(void)
{
	struct pci_dev *dev = NULL;

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

	if ((dev = pci_find_device(PCI_VENDOR_ID_DEC,
				   PCI_DEVICE_ID_DEC_21152, NULL))) {
		pci_write_config_byte(dev, PCI_LATENCY_TIMER, 0x80);
		pci_write_config_byte(dev, PCI_SEC_LATENCY_TIMER, 0x80);
	}
#ifdef CONFIG_MVME3100_ENABLE_DDR_ERRORS
	if (request_irq(MPC85xx_IRQ_DDR, mvme3100_ddr_err_handler,
			0, "DDR Error", 0) < 0) {
		printk(KERN_ERR "Cannot install DDR error handler\n");
		return;
	}

	writel(swab32(0xD),
	       ddr_err_regs + DDR_ERR_INT_EN - DDR_ERR_REGS_OFFSET);
#endif

#ifdef CONFIG_MVME3100_ENABLE_L2_ERRORS
	if (request_irq(MPC85xx_IRQ_L2CACHE, mvme3100_l2cache_err_handler,
			0, "L2 Cache Error", 0) < 0) {
		printk(KERN_ERR "Cannot install L2 cache error handler\n");
		return;
	}

	writel(swab32(0x1D), l2_err_regs + L2_ERR_INT_EN - L2_ERR_REGS_OFFSET);
#endif

#ifdef CONFIG_MVME3100_ENABLE_PCI_ERRORS
	if (request_irq(MPC85xx_IRQ_PCI1, mvme3100_pci_err_handler,
			0, "PCI Error", 0) < 0) {
		printk(KERN_ERR "Cannot install PCI error handler\n");
		return;
	}

	writel(swab32(0x7FF),
	       pci_err_regs + PCI_ERR_INT_EN - PCI_ERR_REGS_OFFSET);
#endif
}

static void mvme3100_restart(char *cmd)
{
	volatile ulong i = 10000000;
	u8 *system_control_reg_addr;

	local_irq_disable();
	system_control_reg_addr = ioremap(MVME3100_SYSTEM_CONTROL_REG, 1);
	writeb(MVME3100_BOARD_RESET, system_control_reg_addr);

	while (i-- > 0) ;
	panic("restart failed\n");
}

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

static void mvme3100_init_caches(void)
{
	u8 *l2_ctl_addr;
	bd_t *binfo = (bd_t *) __res;

	l2_err_regs = ioremap(binfo->bi_immr_base + L2_ERR_REGS_OFFSET,
			      L2_ERR_REGS_SIZE);

	l2_ctl_addr = ioremap(binfo->bi_immr_base + 0x20000, 4);

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

	return;
}

static void mvme3100_misc_init(void)
{
	u8 *reg_addr;
	u8 reg_value;
	u32 sbe_count;
	bd_t *binfo = (bd_t *) __res;

#ifdef CONFIG_MVME3100_ENABLE_DDR_ERRORS
	/* Enable DDR error reporting */
	ddr_err_regs = ioremap(binfo->bi_immr_base + DDR_ERR_REGS_OFFSET,
			       DDR_ERR_REGS_SIZE);

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
	pci_err_regs = ioremap(binfo->bi_immr_base + PCI_ERR_REGS_OFFSET,
			       PCI_ERR_REGS_SIZE);
	writel(0, pci_err_regs + PCI_ERR_CAP_DIS - PCI_ERR_REGS_OFFSET);
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

#ifdef CONFIG_SERIAL_8250
static void __init mvme3100_early_serial_map(void)
{
#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	struct uart_port serial_req;
#endif
	bd_t *binfo = (bd_t *) __res;
	struct plat_serial8250_port *pdata;

	mpc85xx_early_serial_map();

	pdata = (struct plat_serial8250_port *)ppc_sys_get_pdata(MPC85xx_DUART);

	pdata[0].uartclk = binfo->bi_busfreq;

	pdata[1].uartclk = MVME3100_BASE_BAUD;
	pdata[1].membase = ioremap(pdata[1].mapbase, 0x1000);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	memset(&serial_req, 0, sizeof(serial_req));
	serial_req.line = 1;
	serial_req.irq = MVME3100_SERIAL_IRQ;
	serial_req.uartclk = MVME3100_BASE_BAUD;
	serial_req.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;

	serial_req.iotype = UPIO_MEM;
	serial_req.mapbase = pdata[1].mapbase;
	serial_req.membase = pdata[1].membase;
	serial_req.regshift = 0;

	gen550_init(1, &serial_req);
#endif
	pdata[2].uartclk = MVME3100_BASE_BAUD;
	pdata[2].membase = ioremap(pdata[2].mapbase, 0x1000);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	serial_req.line = 2;
	serial_req.mapbase = pdata[2].mapbase;
	serial_req.membase = pdata[2].membase;

	gen550_init(2, &serial_req);
#endif

	pdata[3].uartclk = MVME3100_BASE_BAUD;
	pdata[3].membase = ioremap(pdata[3].mapbase, 0x1000);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	serial_req.line = 3;
	serial_req.mapbase = pdata[3].mapbase;
	serial_req.membase = pdata[3].membase;

	gen550_init(3, &serial_req);
#endif

	pdata[4].uartclk = MVME3100_BASE_BAUD;
	pdata[4].membase = ioremap(pdata[4].mapbase, 0x1000);

#if defined(CONFIG_SERIAL_TEXT_DEBUG) || defined(CONFIG_KGDB)
	serial_req.line = 4;
	serial_req.mapbase = pdata[4].mapbase;
	serial_req.membase = pdata[4].membase;

	gen550_init(4, &serial_req);
#endif

}
#endif

/* 
 * Setup the architecture
 */
static void __init mvme3100_setup_arch(void)
{
	bd_t *binfo = (bd_t *) __res;
	unsigned int freq;
	struct gianfar_platform_data *pdata;
	struct gianfar_mdio_data *mdata;

	if (ppc_md.progress)
		ppc_md.progress("mvme3100_setup_arch(): enter", 0);

	/* get the core frequency */
	freq = BINFO_INTFREQ;

	mvme3100_init_caches();

	/* Set loops_per_jiffy to a half-way reasonable value,
	   for use until calibrate_delay gets called. */
	loops_per_jiffy = freq / HZ;

	/* setup PCI host bridges */
	mpc85xx_setup_hose();

#ifdef CONFIG_SERIAL_8250
	mvme3100_early_serial_map();
#endif

#ifdef CONFIG_SERIAL_TEXT_DEBUG
	/* Invalidate the entry we stole earlier. The serial ports
	 * should be properly mapped */
	invalidate_tlbcam_entry(num_tlbcam_entries - 1);
#endif

        /* setup the board related info for the MDIO bus */
        mdata = (struct gianfar_mdio_data *) ppc_sys_get_pdata(MPC85xx_MDIO);
                                                                                  
        mdata->irq[1] = MPC85xx_IRQ_EXT10;
        mdata->irq[2] = MPC85xx_IRQ_EXT10;
//        mdata->irq[3] = MPC85xx_IRQ_EXT10;
        mdata->irq[3] = PHY_POLL;

	/* setup the board related information for the enet controllers */
	pdata =
	    (struct gianfar_platform_data *)ppc_sys_get_pdata(MPC85xx_TSEC1);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 1;
		/* fixup phy address */
//		pdata->phy_reg_addr += binfo->bi_immr_base;
		memcpy(pdata->mac_addr, binfo->bi_enetaddr, 6);
	}

	pdata =
	    (struct gianfar_platform_data *)ppc_sys_get_pdata(MPC85xx_TSEC2);
	if (pdata) {
		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->bus_id = 0;
		pdata->phy_id = 2;
		/* fixup phy address */
//		pdata->phy_reg_addr += binfo->bi_immr_base;
		memcpy(pdata->mac_addr, binfo->bi_enet1addr, 6);
	}

	pdata = (struct gianfar_platform_data *)ppc_sys_get_pdata(MPC85xx_FEC);
	if (pdata) {
//		pdata->board_flags = FSL_GIANFAR_BRD_HAS_PHY_INTR;
		pdata->board_flags = 0;
		pdata->bus_id = 0;
		pdata->phy_id = 3;
		/* fixup phy address */
//		pdata->phy_reg_addr += binfo->bi_immr_base;
		memcpy(pdata->mac_addr, binfo->bi_enet2addr, 6);
	}
#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		ROOT_DEV = Root_RAM0;
	else
#endif

#ifdef  CONFIG_ROOT_NFS
	ROOT_DEV = Root_NFS;
#else
	ROOT_DEV = Root_SDA2;
#endif

#ifdef CONFIG_VME_BRIDGE_BOOTMEM
        vme_bootmem_size = CONFIG_VME_BRIDGE_BOOTMEM_SIZE * 1024 * 1024;
                                                                                
        if (vme_bootmem_size > (total_memory / 2)) {
                printk(KERN_WARNING "BOOTMEM Size Requested: 0x%x Total Memory 0x%lx\n", vme_bootmem_size, total_memory);
                printk(KERN_WARNING "BOOTMEM Size requested has been capped at half the total memory\n");
                vme_bootmem_size = total_memory / 2;
	}
	vme_driver_bootmem = __alloc_bootmem(vme_bootmem_size, 0x10000, 0);
	if (!vme_driver_bootmem) {
		printk(KERN_WARNING "Unable to obtain boot memory for VME\n");
		vme_bootmem_size = 0;
	} else {
		printk(KERN_INFO "0x%x of boot memory reserved for setup of VME inbound window 7\n", vme_bootmem_size);
	}
#endif

	mvme3100_misc_init();

	if (ppc_md.progress)
		ppc_md.progress("mvme3100_setup_arch(): exit", 0);
}

#if defined(CONFIG_I2C_MPC) && defined(CONFIG_SENSORS_DS1375)
extern ulong ds1375_get_rtc_time(void);
extern int ds1375_set_rtc_time(ulong);

static int __init mvme3100_rtc_hookup(void)
{
	struct timespec tv;

	ppc_md.get_rtc_time = ds1375_get_rtc_time;
	ppc_md.set_rtc_time = ds1375_set_rtc_time;
#if 0
	ppc_md.nvram_read_val = ds1375_read_val;
	ppc_md.nvram_write_val = ds1375_write_val;
#endif
	tv.tv_nsec = 0;
	tv.tv_sec = (ppc_md.get_rtc_time) ();
	do_settimeofday(&tv);

	return 0;
}

late_initcall(mvme3100_rtc_hookup);

#endif

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{

	/* parse_bootinfo must always be called first */
	parse_bootinfo(find_bootinfo());

	/*
	 * If we were passed in a board information, copy it into the
	 * residual data area.
	 */
	if (r3) {
		memcpy((void *)__res, (void *)(r3 + KERNELBASE), sizeof(bd_t));
	}
#ifdef CONFIG_SERIAL_TEXT_DEBUG
	{
		bd_t *binfo = (bd_t *) __res;
		struct uart_port p;

		/* Use the last TLB entry to map CCSRBAR to allow access to UART regs */
		settlbcam(num_tlbcam_entries - 1, BINFO_IMMR_BASE,
			  BINFO_IMMR_BASE, MPC85xx_CCSRBAR_SIZE, _PAGE_IO, 0);

		memset(&p, 0, sizeof(p));
		p.iotype = UPIO_MEM;
		p.membase = (void *)binfo->bi_immr_base + MPC85xx_UART0_OFFSET;
		p.uartclk = binfo->bi_busfreq;

		gen550_init(0, &p);
	}
#endif

#if defined(CONFIG_BLK_DEV_INITRD)
	/*
	 * If the init RAM disk has been configured in, and there's a valid
	 * starting address for it, set it up.
	 */
	if (r4) {
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif				/* CONFIG_BLK_DEV_INITRD */

	/* Copy the kernel command line arguments to a safe place. */

	if (r6) {
		*(char *)(r7 + KERNELBASE) = 0;
		strcpy(cmd_line, (char *)(r6 + KERNELBASE));
	}

	identify_ppc_sys_by_id(mfspr(SPRN_SVR));

#ifdef CONFIG_VME_BRIDGE
	{
		extern void vmemod_setup_options(char *);

		vmemod_setup_options(cmd_line);


	}
#endif
	ppc_md.setup_arch = mvme3100_setup_arch;
	ppc_md.show_cpuinfo = mvme3100_show_cpuinfo;

	ppc_md.init_IRQ = mvme3100_init_IRQ;
	ppc_md.get_irq = openpic_get_irq;

	ppc_md.restart = mvme3100_restart;
	ppc_md.power_off = mpc85xx_power_off;
	ppc_md.halt = mpc85xx_halt;

	ppc_md.find_end_of_memory = mpc85xx_find_end_of_memory;

	ppc_md.time_init = NULL;
	ppc_md.set_rtc_time = NULL;
	ppc_md.get_rtc_time = NULL;
	ppc_md.nvram_read_val = NULL;
	ppc_md.nvram_write_val = NULL;

	ppc_md.calibrate_decr = mpc85xx_calibrate_decr;

	ppc_md.pcibios_fixup = mvme3100_pci_fixups;

#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_TEXT_DEBUG)
	ppc_md.progress = gen550_progress;
#endif				/* CONFIG_SERIAL_8250 && CONFIG_SERIAL_TEXT_DEBUG */
#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_KGDB)
	ppc_md.early_serial_map = mvme3100_early_serial_map;
#endif				/* CONFIG_SERIAL_8250 && CONFIG_KGDB */

	if (ppc_md.progress)
		ppc_md.progress("mvme3100_init(): exit", 0);

	return;
}
