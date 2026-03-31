/*
 * MVME7100 board specific routines
 *
 * Author: Ajit Prem <ajit.prem@emerson.com>
 *
 * Copyright 2008 Emerson Network Power Embedded Computing
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
#include <linux/highmem.h>

#include <asm/of_device.h>
#include <asm/of_platform.h>
#include <asm/system.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/mpc86xx.h>
#include <asm/prom.h>
#include <asm/page.h>
#include <asm/kmap_types.h>
#include <asm/edac.h>
#include <mm/mmu_decl.h>
#include <asm/udbg.h>

#include <asm/mpic.h>

#include <sysdev/fsl_pci.h>
#include <sysdev/fsl_soc.h>

#include "mpc86xx.h"
#include "mvme7100.h"

#undef DEBUG

#ifdef DEBUG
#define DBG(fmt...) do { printk(KERN_ERR fmt); } while(0)
#else
#define DBG(fmt...) do { } while(0)
#endif

static unsigned long hb_baddr;
static int serial_irq;

#ifdef CONFIG_MVME7100_ENABLE_L2_ERRORS

#define SPRN_L2CAPTDATAHI       0x3DC
#define SPRN_L2CAPTDATALO       0x3DD
#define SPRN_L2CAPTECC          0x3DE
#define SPRN_L2ERRDET           0x3DF
#define SPRN_L2ERRDIS           0x3E0
#define SPRN_L2ERRINTEN         0x3E1
#define SPRN_L2ERRATTR          0x3E2
#define SPRN_L2ERRADDR          0x3E3
#define SPRN_L2ERREADDR         0x3E4
#define SPRN_L2ERRCTL           0x3E5

static int l2_irq;
static irqreturn_t mvme7100_l2cache_err_handler(int irq, void *dev_id);

#endif

#ifdef CONFIG_MVME7100_ENABLE_DDR_ERRORS

#define DDR_ERR_REGS_OFFSET     0x2E00
#define DDR_ERR_REGS_SIZE       0x5C

#define DDR_CAPT_DATA_HI        0x2E20
#define DDR_CAPT_DATA_LO        0x2E24
#define DDR_CAPT_ECC            0x2E28
#define DDR_ERR_DET             0x2E40
#define DDR_ERR_DISABLE         0x2E44
#define DDR_ERR_INT_EN          0x2E48
#define DDR_CAPT_ATTR           0x2E4C
#define DDR_CAPT_ADDR           0x2E50
#define DDR_CAPT_EXT_ADDR       0x2E54
#define DDR_ERR_SBE             0x2E58

#define DDR_SBE             	0x4		/* single-bit ECC error */
#define DDR_MBE             	0x8		/* multi-bit ECC error */
#define DDR_MME             	0x80000000	/* multiple ECC error */

static void __iomem *ddr_err_regs;
static irqreturn_t mvme7100_ddr_err_handler(int irq, void *dev_id);
static int mpc86xx_ddr_irq;
static u32 ddr_errors;

#endif


#ifdef CONFIG_MVME7100_ENABLE_PCI_ERRORS

#define PEX_ERR_REGS_OFFSET     0x8E00
#define PEX_ERR_REGS_SIZE       0x38

#define PEX_ERR_DR		0x8E00
#define PEX_ERR_EN		0x8E08
#define PEX_ERR_DISR		0x8E10
#define PEX_ERR_CAP_STAT        0x8E20
#define PEX_ERR_CAP_R0          0x8E28
#define PEX_ERR_CAP_R1          0x8E2C
#define PEX_ERR_CAP_R2          0x8E30
#define PEX_ERR_CAP_R3          0x8E34

static u32 pci_errors;
static void __iomem *pci_err_regs;
static int pci_irq;
static irqreturn_t mvme7100_pci_err_handler(int irq, void *dev_id);

#endif

#ifdef CONFIG_VME_BRIDGE_BOOTMEM
void *vme_driver_bootmem;
unsigned int vme_bootmem_size;
#endif

#ifdef CONFIG_MVME7100_TICK_TIMERS
int mvme7100_timer_irq;
#endif

#ifdef CONFIG_SENSORS_LM90
int maxim6649_irq;
#endif


#ifdef CONFIG_MVME7100_ENABLE_DDR_ERRORS

static void mvme7100_determine_bit_location(u32 data_hi, u32 data_lo, 
					u32 stored_ecc)
{
	u32 syndrome_bits;
	u32 syndrome[8];
	u32 calculated_ecc = 0;
	int i;
	u32 data0[32], data1[32];
	unsigned char bitpos[8];

	/* Initial 32-bit data */
	for (i=0; i < 32; i++) {
		data0[i] = (data_hi >> (31 - i)) & 0x1;
		data1[i] = (data_lo >> (31 - i)) & 0x1;
	}

	for (i=0; i < 8; i++) 
		syndrome[i] = 0;

	/* Calculate 8-bit ECC for the first 32-bit data */
	for (i = 0; i < 32; ++i) {
		/* Calculate syndrome[0] */
		if ((i <= 15) || (i == 19) || (i == 23) || (i == 27) || 
			(i == 31))
			syndrome[0] = syndrome[0] ^ (data0[i] & 0x1);

		/* Calculate syndrome[1] */
		if ((i == 0) || (i == 4) || (i == 8) || (i == 12) ||
			(i >= 16))
			syndrome[1] = syndrome[1] ^ (data0[i]  & 0x1);
   
		/* Calculate syndrome[2] */
		if ((i == 1)  || (i == 5)  || (i == 9)  || (i == 13) ||
			(i == 16) || (i == 20) || (i == 24) || (i == 28))
			syndrome[2] = syndrome[2] ^ (data0[i] & 0x1);
   
		/* Calculate syndrome[3] */
		if ((i == 2)  || (i == 6)  || (i == 10) || (i == 14) ||
			(i == 17) || (i == 21) || (i == 25) || (i == 29))
			syndrome[3] = syndrome[3] ^ (data0[i] & 0x1);

		/* Calculate syndrome[4] */
		if ((i == 3)  || (i == 7)  || (i == 11) || (i == 15) ||
			(i == 18) || (i == 19) || (i == 22) || (i == 23) ||
			(i == 26) || (i == 27) || (i == 30) || (i == 31))
			syndrome[4] = syndrome[4] ^ (data0[i] & 0x1);
   
		/* Calculate syndrome[5] */
		if (((i >= 4)  && (i <= 7))  || ((i >= 12) && (i <= 15)) ||
			((i >= 20) && (i <= 23)) || ((i >= 28) && (i <= 31)))
			syndrome[5] = syndrome[5] ^ (data0[i] & 0x1);
   
		/* Calculate syndrome[6] */
		if (((i >= 8)  && (i <= 15))  || ((i >= 24) && (i <= 31)))
			syndrome[6] = syndrome[6] ^ (data0[i] & 0x1);
   
		/* Calculate syndrome[7] */
		if ((i <= 3 ) || ((i >= 12) && (i <= 18)) ||
			(i == 23) || ((i >= 27) && (i <= 30)))
			syndrome[7] = syndrome[7] ^ (data0[i] & 0x1);
	}


	/* Calculate 8-bit ECC for the second 32-bit data */
	for (i = 0; i < 32; ++i) {
		/* Calculate syndrome[0] */
		if ((i == 2)  || (i == 6)  || (i == 10) || (i == 14) ||
			(i == 19) || (i == 23) || (i == 27) || (i == 29))
			syndrome[0] = syndrome[0] ^ (data1[i] & 0x1);
   
		/* Calculate syndrome[1] */
		if ((i == 3)  || (i == 7)  || (i == 11) || (i == 15) ||
			(i == 16) || (i == 20) || (i == 24) || (i == 30))
			syndrome[1] = syndrome[1] ^ (data1[i] & 0x1);
   
		/* Calculate syndrome[2] */
		if ((i <= 15) || (i == 17) || (i == 21) || (i == 25) |
			(i == 31))
			syndrome[2] = syndrome[2] ^ (data1[i] & 0x1);
   
		/* Calculate syndrome[3] */
		if ((i == 0)  || (i == 4)  || (i == 8)  || (i == 12) ||
			(i == 18) || (i == 22) || (i == 26) || (i >= 28))
			syndrome[3] = syndrome[3] ^ (data1[i] & 0x1);
   
		/* Calculate syndrome[4] */
		if (((i >= 1)  && (i <= 3))  || ((i >= 5)  && (i <= 7))  ||
			((i >= 9)  && (i <= 11)) || ((i >= 13) && (i <= 15)) ||
			((i >= 28) && (i <= 31)))
			syndrome[4] = syndrome[4] ^ (data1[i] & 0x1);
   
		/* Calculate syndrome[5] */
		if (((i >= 4) && (i <= 7)) || ((i >= 12) && (i <= 23)))
			syndrome[5] = syndrome[5] ^ (data1[i] & 0x1);
   
		/* Calculate syndrome[6] */
		if (((i >= 8) && (i <= 19)) || ((i >= 24) && (i <= 31)))
			syndrome[6] = syndrome[6] ^ (data1[i] & 0x1);
   
		/* Calculate syndrome[7] */
		if ((i == 0) || (i == 1) || (i == 6) || (i == 7) ||
			((i >= 10) && (i <= 13)) || ((i >= 20) && (i <= 27)) ||
			((i >= 29) && (i <= 31)))
			syndrome[7] = syndrome[7] ^ (data1[i] & 0x1);
	}

	calculated_ecc = ((syndrome[7] << 0) | (syndrome[6] << 1) | 
			(syndrome[5] << 2) | (syndrome[4] << 3) | 
			(syndrome[3] << 4) | (syndrome[2] << 5) | 
			(syndrome[1] << 6) | (syndrome[0] << 7));

	syndrome_bits = calculated_ecc ^ stored_ecc;

	switch (syndrome_bits) {
		case 0xC1: 
			sprintf(bitpos, "%s", "0");
			break;
		case 0xA1: 
			sprintf(bitpos, "%s", "1");
			break;
		case 0x91: 
			sprintf(bitpos, "%s", "2");
			break;
		case 0x89: 
			sprintf(bitpos, "%s", "3");
			break;
		case 0xC4: 
			sprintf(bitpos, "%s", "4");
			break;
		case 0xA4: 
			sprintf(bitpos, "%s", "5");
			break;
		case 0x94: 
			sprintf(bitpos, "%s", "6");
			break;
		case 0x8C: 
			sprintf(bitpos, "%s", "7");
			break;
		case 0xC2: 
			sprintf(bitpos, "%s", "8");
			break;
		case 0xA2: 
			sprintf(bitpos, "%s", "9");
			break;
		case 0x92: 
			sprintf(bitpos, "%s", "10");
			break;
		case 0x8A: 
			sprintf(bitpos, "%s", "11");
			break;
		case 0xC7: 
			sprintf(bitpos, "%s", "12");
			break;
		case 0xA7: 
			sprintf(bitpos, "%s", "13");
			break;
		case 0x97: 
			sprintf(bitpos, "%s", "14");
			break;
		case 0x8F: 
			sprintf(bitpos, "%s", "15");
			break;
		case 0x61: 
			sprintf(bitpos, "%s", "16");
			break;
		case 0x51: 
			sprintf(bitpos, "%s", "17");
			break;
		case 0x49: 
			sprintf(bitpos, "%s", "18");
			break;
		case 0xC8: 
			sprintf(bitpos, "%s", "19");
			break;
		case 0x64: 
			sprintf(bitpos, "%s", "20");
			break;
		case 0x54: 
			sprintf(bitpos, "%s", "21");
			break;
		case 0x4C: 
			sprintf(bitpos, "%s", "22");
			break;
		case 0xCD: 
			sprintf(bitpos, "%s", "23");
			break;
		case 0x62: 
			sprintf(bitpos, "%s", "24");
			break;
		case 0x52: 
			sprintf(bitpos, "%s", "25");
			break;
		case 0x4A: 
			sprintf(bitpos, "%s", "26");
			break;
		case 0xCB: 
			sprintf(bitpos, "%s", "27");
			break;
		case 0x67: 
			sprintf(bitpos, "%s", "28");
			break;
		case 0x57: 
			sprintf(bitpos, "%s", "29");
			break;
		case 0x4F: 
			sprintf(bitpos, "%s", "30");
			break;
		case 0xCE: 
			sprintf(bitpos, "%s", "31");
			break;
		case 0x31: 
			sprintf(bitpos, "%s", "32");
			break;
		case 0x29: 
			sprintf(bitpos, "%s", "33");
			break;
		case 0xA8: 
			sprintf(bitpos, "%s", "34");
			break;
		case 0x68: 
			sprintf(bitpos, "%s", "35");
			break;
		case 0x34: 
			sprintf(bitpos, "%s", "36");
			break;
		case 0x2C: 
			sprintf(bitpos, "%s", "37");
			break;
		case 0xAD: 
			sprintf(bitpos, "%s", "38");
			break;
		case 0x6D: 
			sprintf(bitpos, "%s", "39");
			break;
		case 0x32: 
			sprintf(bitpos, "%s", "40");
			break;
		case 0x2A: 
			sprintf(bitpos, "%s", "41");
			break;
		case 0xAB: 
			sprintf(bitpos, "%s", "42");
			break;
		case 0x6B: 
			sprintf(bitpos, "%s", "43");
			break;
		case 0x37: 
			sprintf(bitpos, "%s", "44");
			break;
		case 0x2F: 
			sprintf(bitpos, "%s", "45");
			break;
		case 0xAE: 
			sprintf(bitpos, "%s", "46");
			break;
		case 0x6E: 
			sprintf(bitpos, "%s", "47");
			break;
		case 0x46: 
			sprintf(bitpos, "%s", "48");
			break;
		case 0x26: 
			sprintf(bitpos, "%s", "49");
			break;
		case 0x16: 
			sprintf(bitpos, "%s", "50");
			break;
		case 0x86: 
			sprintf(bitpos, "%s", "51");
			break;
		case 0x45: 
			sprintf(bitpos, "%s", "52");
			break;
		case 0x25: 
			sprintf(bitpos, "%s", "53");
			break;
		case 0x15: 
			sprintf(bitpos, "%s", "54");
			break;
		case 0x85: 
			sprintf(bitpos, "%s", "55");
			break;
		case 0x43: 
			sprintf(bitpos, "%s", "56");
			break;
		case 0x23: 
			sprintf(bitpos, "%s", "57");
			break;
		case 0x13: 
			sprintf(bitpos, "%s", "58");
			break;
		case 0x83: 
			sprintf(bitpos, "%s", "59");
			break;
		case 0x1A: 
			sprintf(bitpos, "%s", "60");
			break;
		case 0x9B: 
			sprintf(bitpos, "%s", "61");
			break;
		case 0x5B: 
			sprintf(bitpos, "%s", "62");
			break;
		case 0x3B: 
			sprintf(bitpos, "%s", "63");
			break;
		case 0x80: 
			sprintf(bitpos, "%s", "ECC-0");
			break;
		case 0x40: 
			sprintf(bitpos, "%s", "ECC-1");
			break;
		case 0x20: 
			sprintf(bitpos, "%s", "ECC-2");
			break;
		case 0x10: 
			sprintf(bitpos, "%s", "ECC-3");
			break;
		case 0x08: 
			sprintf(bitpos, "%s", "ECC-4");
			break;
		case 0x04: 
			sprintf(bitpos, "%s", "ECC-5");
			break;
		case 0x02: 
			sprintf(bitpos, "%s", "ECC-6");
			break;
		case 0x01: 
			sprintf(bitpos, "%s", "ECC-7");
			break;
		default:
			sprintf(bitpos, "%s", "UNKNOWN");
			break;
	}
	printk(KERN_ERR "DDR ERROR CALCULATED ECC: 0x%02x\n", calculated_ecc);
	printk(KERN_ERR "DDR ERROR STORED ECC: 0x%02x\n", stored_ecc);
	printk(KERN_ERR "DDR ERROR ECC SYNDROME: 0x%02x\n", 
			calculated_ecc ^ stored_ecc);
	printk(KERN_ERR "DDR ERROR BIT LOCATION: %s\n", bitpos);
}

static irqreturn_t mvme7100_ddr_err_handler(int irq, void *dev_id)
{
	u32 err_addr, data_hi, data_lo, ecc, err_attr, err_det, sbe_count;
	u32 pfn, offset;
	struct page *pg;
	void *virt_addr;
	unsigned long flags = 0;

	err_det = readl(ddr_err_regs + DDR_ERR_DET - DDR_ERR_REGS_OFFSET);
	if (!err_det)
		return IRQ_NONE;

	/* Collect error information from the error registers */
	err_addr = readl(ddr_err_regs + DDR_CAPT_ADDR - DDR_ERR_REGS_OFFSET);
	data_hi = readl(ddr_err_regs + DDR_CAPT_DATA_HI - DDR_ERR_REGS_OFFSET);
	data_lo = readl(ddr_err_regs + DDR_CAPT_DATA_LO - DDR_ERR_REGS_OFFSET);
	err_attr = readl(ddr_err_regs + DDR_CAPT_ATTR - DDR_ERR_REGS_OFFSET);
	ecc = readl(ddr_err_regs + DDR_CAPT_ECC - DDR_ERR_REGS_OFFSET);
	sbe_count = readl(ddr_err_regs + DDR_ERR_SBE - DDR_ERR_REGS_OFFSET);

	/* Keep error count */
	ddr_errors++;

	/* Display error information */
	printk(KERN_ERR "DDR Error!\n");
	printk(KERN_ERR "DDR ERROR DETECT REG: 0x%08x\n", swab32(err_det));
	printk(KERN_ERR "DDR ERROR ADDRESS CAPTURE REG: 0x%08x\n",
	       swab32(err_addr));
	printk(KERN_ERR "DDR ERROR DATA HIGH CAPTURE REG: 0x%08x\n",
	       swab32(data_hi));
	printk(KERN_ERR "DDR ERROR DATA LOW CAPTURE REG: 0x%08x\n",
	       swab32(data_lo));
	printk(KERN_ERR "DDR ERROR ATTRIBUTES CAPTURE REG: 0x%08x\n",
	       swab32(err_attr));
	printk(KERN_ERR "DDR ERROR SBE REG: 0x%08x\n", swab32(sbe_count));
	printk(KERN_ERR "DDR ERROR COUNT: %u\n", ddr_errors);

	if (err_det & DDR_SBE) {
		/* Scrub block */
		pfn = err_addr >> PAGE_SHIFT;
		offset = err_addr & ~PAGE_MASK;

		pg = pfn_to_page(pfn);

		if (PageHighMem(pg))
			local_irq_save(flags);

		virt_addr = kmap_atomic(pg, KM_BOUNCE_READ);
		atomic_scrub(virt_addr + offset, 8);
		kunmap_atomic(virt_addr, KM_BOUNCE_READ);
	
		if (PageHighMem(pg))
			local_irq_restore(flags);
	}

	/* Determine and display bit location */
	mvme7100_determine_bit_location(swab32(data_hi), 
		swab32(data_lo), 
		swab32(ecc) & 0xff);

	/* Clear interrupt cause */
	writel(err_det, ddr_err_regs + DDR_ERR_DET - DDR_ERR_REGS_OFFSET);

	return IRQ_HANDLED;
}

#endif

#ifdef CONFIG_MVME7100_ENABLE_L2_ERRORS

static irqreturn_t mvme7100_l2cache_err_handler(int irq, void *dev_id)
{
	u32 err_addr, data_hi, data_lo, ecc, err_attr, err_det, err_ctl;

	/* Collect error information from the error registers */
	err_det = mfspr(SPRN_L2ERRDET);
	err_attr = mfspr(SPRN_L2ERRATTR);
	err_addr = mfspr(SPRN_L2ERRADDR);
	data_hi = mfspr(SPRN_L2CAPTDATAHI);
	data_lo = mfspr(SPRN_L2CAPTDATALO);
	ecc = mfspr(SPRN_L2CAPTECC);
	err_ctl = mfspr(SPRN_L2ERRCTL);

	/* Clear interrupt cause and capture registers */
	mtspr(SPRN_L2ERRDET, err_det);
	mtspr(SPRN_L2ERRATTR, 0);

	/* Display error information */
	printk(KERN_ERR "L2 Cache Error!\n");
	printk(KERN_ERR "L2 ERROR DETECT REG: 0x%08x\n", err_det);
	printk(KERN_ERR "L2 ERROR ADDRESS CAPTURE REG: 0x%08x\n", err_addr);
	printk(KERN_ERR "L2 ERROR DATA HIGH CAPTURE REG: 0x%08x\n", data_hi);
	printk(KERN_ERR "L2 ERROR DATA LOW CAPTURE REG: 0x%08x\n", data_lo);
	printk(KERN_ERR "L2 ERROR ATTRIBUTES CAPTURE REG: 0x%08x\n", err_attr);
	printk(KERN_ERR "L2 ERROR SYNDROME REG: 0x%08x\n", ecc);
	printk(KERN_ERR "L2 ERROR CONTROL REG: 0x%08x\n", err_ctl);

	return IRQ_HANDLED;
}

#endif

#ifdef CONFIG_MVME7100_ENABLE_PCI_ERRORS

static u16 mvme7100_get_pci_parity_status(struct pci_dev *dev, int secondary)
{
	int where;
	u16 status;

	where = secondary ? PCI_SEC_STATUS : PCI_STATUS;
	pci_read_config_word(dev, where, &status);

	if (status == 0xFFFF) 
		return 0;

	status &= PCI_STATUS_DETECTED_PARITY | PCI_STATUS_SIG_SYSTEM_ERROR |
			PCI_STATUS_PARITY;

	if (status)
		/* reset only the bits we are interested in */
		pci_write_config_word(dev, where, status);

	return status;
}

static void mvme7100_pci_dev_parity_test(struct pci_dev *dev)
{
	u16 status;
	u8 header_type;

	status = mvme7100_get_pci_parity_status(dev, 0);

	/* read the device TYPE, looking for bridges */
	pci_read_config_byte(dev, PCI_HEADER_TYPE, &header_type);

	if (status && !dev->broken_parity_status) {
		if (status & (PCI_STATUS_SIG_SYSTEM_ERROR)) {
			printk(KERN_CRIT 
				"PCI Error: Signaled System Error on %s\n",
				pci_name(dev));
		}

		if (status & (PCI_STATUS_PARITY)) {
			printk(KERN_CRIT 
				"PCI Error: Master Data Parity Error on %s\n",
				pci_name(dev));
		}

		if (status & (PCI_STATUS_DETECTED_PARITY)) {
			printk(KERN_CRIT
				"PCI Error: Detected Parity Error on %s\n",
				pci_name(dev));
		}
	}

	if ((header_type & 0x7F) == PCI_HEADER_TYPE_BRIDGE) {
		/* On bridges, need to examine secondary status register  */
		status = mvme7100_get_pci_parity_status(dev, 1);

		if (status && !dev->broken_parity_status) {

			if (status & (PCI_STATUS_SIG_SYSTEM_ERROR)) {
				printk(KERN_CRIT "PCI Error: Bridge "
					"Signaled System Error on %s\n",
					pci_name(dev));
			}

			if (status & (PCI_STATUS_PARITY)) {
				printk(KERN_CRIT "PCI Error: Bridge "
					"Master Data Parity Error on "
					"%s\n", pci_name(dev));
			}

			if (status & (PCI_STATUS_DETECTED_PARITY)) {
				printk(KERN_CRIT "PCI Error: Bridge "
					"Detected Parity Error on %s\n",
					pci_name(dev));
			}
		}
	}
}

typedef void (*pci_parity_check_fn_t) (struct pci_dev *dev);

/*
 *   Scan the PCI device list for one pass, looking for SERRORs
 *   Master Parity ERRORS or Parity ERRORs on primary or secondary devices
 */
static void mvme7100_pci_dev_parity_iterator(pci_parity_check_fn_t fn)
{
	struct pci_dev *dev = NULL;

	while ((dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) {
		fn(dev);
	}
}

static irqreturn_t mvme7100_pci_err_handler(int irq, void *dev_id)
{
	u32 err_det, err_status, err_cap0, err_cap1, err_cap2, err_cap3;

	/* Collect error information from the error registers */
	err_det = readl(pci_err_regs + PEX_ERR_DR - PEX_ERR_REGS_OFFSET);
	if (err_det == 0)
		return IRQ_NONE;

	err_status =
	    readl(pci_err_regs + PEX_ERR_CAP_STAT - PEX_ERR_REGS_OFFSET);
	err_cap0 = readl(pci_err_regs + PEX_ERR_CAP_R0 - PEX_ERR_REGS_OFFSET);
	err_cap1 = readl(pci_err_regs + PEX_ERR_CAP_R1 - PEX_ERR_REGS_OFFSET);
	err_cap2 = readl(pci_err_regs + PEX_ERR_CAP_R2 - PEX_ERR_REGS_OFFSET);
	err_cap3 = readl(pci_err_regs + PEX_ERR_CAP_R3 - PEX_ERR_REGS_OFFSET);

	pci_errors++;
	/* Display error information */
	printk(KERN_ERR "PEX Error!\n");
	printk(KERN_ERR "PEX ERROR DETECT REG: 0x%08x\n", swab32(err_det));
	printk(KERN_ERR "PEX ERR CAP STATUS REG: 0x%08x\n", swab32(err_status));
	printk(KERN_ERR "PEX ERR CAP REG0: 0x%08x\n", swab32(err_cap0));
	printk(KERN_ERR "PEX ERR CAP REG1: 0x%08x\n", swab32(err_cap1));
	printk(KERN_ERR "PEX ERR CAP REG2: 0x%08x\n", swab32(err_cap2));
	printk(KERN_ERR "PEX ERR CAP REG3: 0x%08x\n", swab32(err_cap3));
	printk(KERN_ERR "PEX Error Count: %u\n", pci_errors);

        mvme7100_pci_dev_parity_iterator(mvme7100_pci_dev_parity_test);

	/* Clear interrupt cause and capture registers */
	writel(err_det, pci_err_regs + PEX_ERR_DR - PEX_ERR_REGS_OFFSET);
	writel(swab32(0x1),
	       pci_err_regs + PEX_ERR_CAP_STAT - PEX_ERR_REGS_OFFSET);

	return IRQ_HANDLED;
}
#endif

void mvme7100_restart(char *cmd)
{
	volatile ulong i = 10000000;
	void __iomem *system_control_reg_addr;

	local_irq_disable();
	system_control_reg_addr = ioremap(MVME7100_SYSTEM_CONTROL_REG, 1);
	writeb(MVME7100_BOARD_RESET, system_control_reg_addr);

	while (i-- > 0) ;
	panic("restart failed\n");
}

void __init mvme7100_init_irq(void)
{
	struct mpic *mpic1;
	struct device_node *np = NULL;
	struct resource res;

	/* Determine PIC address. */
	np = of_find_node_by_type(NULL, "open-pic");
	if (np == NULL) {
		printk(KERN_ERR "Could not find open-pic node\n");
		return;
	}
	if (of_address_to_resource(np, 0, &res)) {
		printk(KERN_ERR "Could not map mpic register space\n");
		of_node_put(np);
		return;
	}

	/* Alloc mpic structure and per isu has 16 INT entries. */
	mpic1 = mpic_alloc(np, res.start,
			   MPIC_PRIMARY | MPIC_WANTS_RESET | MPIC_BIG_ENDIAN,
			   16, NR_IRQS - 4, " MPIC     ");
	BUG_ON(mpic1 == NULL);

	mpic_assign_isu(mpic1, 0, res.start + 0x10000);

	mpic_assign_isu(mpic1, 1, res.start + 0x10200);
	mpic_assign_isu(mpic1, 2, res.start + 0x10400);
	mpic_assign_isu(mpic1, 3, res.start + 0x10600);

	/* 16 External interrupts
	 * Moving them from [0 - 15] to [64 - 79]
	 */
	mpic_assign_isu(mpic1, 4, res.start + 0x10000);

	mpic_init(mpic1);
	of_node_put(np);

#ifdef CONFIG_MVME7100_ENABLE_L2_ERRORS
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

#ifdef CONFIG_MVME7100_ENABLE_DDR_ERRORS
	np = of_find_node_by_type(NULL, "memory-controller");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find memory-controller node\n");
	} else {
		mpc86xx_ddr_irq = irq_of_parse_and_map(np, 0);
		if (mpc86xx_ddr_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map memory controller irq\n");
		of_node_put(np);
	}
#endif

	np = of_find_node_by_type(NULL, "pci");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find pci node\n");
	} else {
		pci_irq = irq_of_parse_and_map(np, 0);
		if (pci_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map PCI irq\n");
		of_node_put(np);
	}

#ifdef CONFIG_SENSORS_LM90
	np = of_find_node_by_type(NULL, "thermostat");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find temp sensor node\n");
	} else {
		maxim6649_irq = irq_of_parse_and_map(np, 0);
		if (maxim6649_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map MAXIM6649 irq\n");
		of_node_put(np);
	}
#endif

#ifdef CONFIG_MVME7100_TICK_TIMERS
	np = of_find_node_by_type(NULL, "board_timers");
	if (np == NULL) {
		printk(KERN_INFO "PIC init: cannot find timers node\n");
	} else {
		mvme7100_timer_irq = irq_of_parse_and_map(np, 0);
		if (mvme7100_timer_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map timers irq\n");
		of_node_put(np);
	}
#endif

	np = of_find_node_by_path("/serial@f2011000");
	if (np == NULL) {
		printk(KERN_INFO
		       "PIC init: cannot find serial@f2011000 node\n");
	} else {
		serial_irq = irq_of_parse_and_map(np, 0);
		if (serial_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map serial@f2011000 irq\n");
		of_node_put(np);
	}

	np = of_find_node_by_path("/serial@f2012000");
	if (np == NULL) {
		printk(KERN_INFO
		       "PIC init: cannot find serial@f2012000 node\n");
	} else {
		serial_irq = irq_of_parse_and_map(np, 0);
		if (serial_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map serial@f2012000 irq\n");
		of_node_put(np);
	}

	np = of_find_node_by_path("/serial@f2013000");
	if (np == NULL) {
		printk(KERN_INFO
		       "PIC init: cannot find serial@f2013000 node\n");
	} else {
		serial_irq = irq_of_parse_and_map(np, 0);
		if (serial_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map serial@f2013000 irq\n");
		of_node_put(np);
	}
	np = of_find_node_by_path("/serial@f2014000");
	if (np == NULL) {
		printk(KERN_INFO
		       "PIC init: cannot find serial@f2014000 node\n");
	} else {
		serial_irq = irq_of_parse_and_map(np, 0);
		if (serial_irq == NO_IRQ)
			printk(KERN_ERR "Unable to map serial@f2014000 irq\n");
		of_node_put(np);
	}
}

static void __init mvme7100_misc_init(void)
{
	void __iomem *reg_block;
	u8 reg_value;
	u32 val;
	struct device_node *np = NULL;
	const u32 *regs;
	unsigned long a;
#ifdef CONFIG_MVME7100_ENABLE_DDR_ERRORS
	u32 sbe_count;
#endif

	np = of_find_node_by_type(np, "soc");
	if (np == NULL) {
		printk("Error:Cannot find soc node\n");
		return;
	}
	regs = get_property(np, "reg", NULL);
	if (regs == NULL) {
		printk("Error:Cannot get reg property from soc node\n");
		of_node_put(np);
		return;
	} else
		hb_baddr = regs[0];

	of_node_put(np);

#ifdef CONFIG_MVME7100_ENABLE_L2_ERRORS

	val = _get_L2CR();
	val &= ~L2CR_L2E;
	_set_L2CR(val);
	for (a = KERNELBASE; a < KERNELBASE + 0x800000; a += 32)
		asm volatile ("dcbf 0,%0"::"r" (a):"memory");
	asm volatile ("sync");

	/* Enable L2 cache error detection */
	mtspr(SPRN_L2ERRDIS, 0);
	asm volatile ("eieio");
	asm volatile ("isync");
	/* Clear any logged L2 errors */
	mtspr(SPRN_L2ERRDET, 0x8000001C);
	asm volatile ("eieio");
	asm volatile ("isync");
#endif
	/* Enable L2 */
	val = _get_L2CR();
	val |= L2CR_L2E;
	_set_L2CR(val);
	printk("L2 cache enabled. L2CTL reg: 0x%lx\n", _get_L2CR());

#ifdef CONFIG_MVME7100_ENABLE_DDR_ERRORS	
	ddr_err_regs =
	    ioremap(hb_baddr + DDR_ERR_REGS_OFFSET, DDR_ERR_REGS_SIZE);
	/* Enable all error detection */
	writel(0, ddr_err_regs + DDR_ERR_DISABLE - DDR_ERR_REGS_OFFSET);
	/* Clear logged DDR errors */
	writel(swab32(0x8000008D),
	       ddr_err_regs + DDR_ERR_DET - DDR_ERR_REGS_OFFSET);
	/* Set single-bit error threshold */
	sbe_count = 0x00010000;
	writel(swab32(sbe_count),
	       ddr_err_regs + DDR_ERR_SBE - DDR_ERR_REGS_OFFSET);
#endif

#ifdef CONFIG_MVME7100_ENABLE_PCI_ERRORS
	/* Map in PCI Error Registers */
	pci_err_regs = ioremap(hb_baddr + PEX_ERR_REGS_OFFSET,
			       PEX_ERR_REGS_SIZE);
#endif

#ifdef CONFIG_VME_BRIDGE
	{
		extern void vmemod_setup_options(char *);

		vmemod_setup_options(cmd_line);
	}
#endif

	reg_block = ioremap(MVME7100_SYSTEM_STATUS_REG, 64);

	/* Turn off USR1 Red LED */
	reg_value = readb(reg_block + MVME7100_STATUS_INDICATOR_REG_OFFSET);
	reg_value &= ~MVME7100_USR1_RED_LED;
	writeb(reg_value, reg_block + MVME7100_STATUS_INDICATOR_REG_OFFSET);

	/* Disable RTC, Temp Sensor, and Abort interrupts */
	reg_value = readb(reg_block + MVME7100_INTERRUPT_REG_2_OFFSET);
	reg_value |= MVME7100_RTC_MASK | MVME7100_TEMP_MASK |
	    MVME7100_ABORT_MASK;
	writeb(reg_value, reg_block + MVME7100_INTERRUPT_REG_2_OFFSET);

	/* Turn off NOR flash SW write-protect */
	reg_value = readb(reg_block + MVME7100_NOR_FLASH_CTRL_STAT_REG_OFFSET);
	reg_value &= ~MVME7100_NOR_FLASH_WP_SW;
	writeb(reg_value, reg_block + MVME7100_NOR_FLASH_CTRL_STAT_REG_OFFSET);

	iounmap(reg_block);

	/* Hack for Rev A boards */
#if 0
	reg_block = ioremap(0xe1005008, 4);
	val = swab32(readl(reg_block));
	val &= ~1;
	writel(swab32(val), reg_block);
	iounmap(reg_block);
#endif

	return;
}

static void __init mvme7100_setup_arch(void)
{
	struct device_node *np;

	np = of_find_node_by_type(NULL, "cpu");
	if (np != 0) {
		const unsigned int *fp;

		fp = get_property(np, "clock-frequency", NULL);
		if (fp != 0)
			loops_per_jiffy = *fp / HZ;
		else
			loops_per_jiffy = 50000000 / HZ;
		of_node_put(np);
	}

#ifdef CONFIG_PCI
	pci_assign_all_buses = 1;

	for (np = NULL; (np = of_find_node_by_type(np, "pci")) != NULL;) {
		struct resource rsrc;

		of_address_to_resource(np, 0, &rsrc);
		if ((rsrc.start & 0xfffff) == 0x8000)
			fsl_add_bridge(np, 1);
		else
			fsl_add_bridge(np, 0);
	}
#endif

	printk
	    ("MVME7100 board from Emerson Network Power - Embedded Computing\n");

#ifdef  CONFIG_ROOT_NFS
	ROOT_DEV = Root_NFS;
#else
	ROOT_DEV = Root_SDA2;
#endif

#ifdef CONFIG_SMP
	mpc86xx_smp_init();
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

	mvme7100_misc_init();
}

void mvme7100_show_cpuinfo(struct seq_file *m)
{
	struct device_node *root;
	uint pvid, svid, phid1;
	phys_addr_t immr_base;
	uint memsize = total_memory;
	const char *model = "";
	void __iomem *reg_block;
	u8 reg_value;
	u32 reg32_value;
	char buff[16] = "";

	seq_printf(m, "Vendor\t\t: Freescale\n");

	root = of_find_node_by_path("/");
	if (root)
		model = get_property(root, "model", NULL);
	seq_printf(m, "Machine\t\t: %s\n", model);
	of_node_put(root);

	pvid = mfspr(SPRN_PVR);
	svid = mfspr(SPRN_SVR);
	immr_base = get_immrbase();

	seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
	seq_printf(m, "SVR\t\t: 0x%x\n", svid);
	seq_printf(m, "HID0\t\t: 0x%08lx\n", mfspr(SPRN_HID0));
	seq_printf(m, "HID1\t\t: 0x%08lx\n", mfspr(SPRN_HID1));
	seq_printf(m, "CCSR Base\t: 0x%08lx\n", immr_base);

	/* Display cpu Pll setting */
	phid1 = mfspr(SPRN_HID1);
	seq_printf(m, "PLL setting\t: 0x%x\n", ((phid1 >> 24) & 0x3f));

	/* Display the cache settings */
	seq_printf(m, "L2CR\t\t: 0x%lx\n", _get_L2CR());

	seq_printf(m, "\nBOARD INFORMATION:\n\n");

	reg_block = ioremap(MVME7100_SYSTEM_STATUS_REG, 64);

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t\t: %d MB\n", memsize / (1024 * 1024));

	/* System Status Register */
	reg_value = readb(reg_block);
	seq_printf(m, "SW8 State\t\t: %s\n",
		   ((reg_value & MVME7100_STATE_SW8) ? "ON" : "OFF"));
	seq_printf(m, "SW7 State\t\t: %s\n",
		   ((reg_value & MVME7100_STATE_SW7) ? "ON" : "OFF"));
	seq_printf(m, "SW6 State\t\t: %s\n",
		   ((reg_value & MVME7100_STATE_SW6) ? "ON" : "OFF"));
	seq_printf(m, "SW5 State\t\t: %s\n",
		   ((reg_value & MVME7100_STATE_SW5) ? "ON" : "OFF"));

	seq_printf(m, "Safe Start Status\t: %s\n",
		   ((reg_value & MVME7100_SAFE_START) ?
		    "Safe ENV settings used" : "NVRAM ENV settings used"));

	seq_printf(m, "PEX8525 Error Status\t: %s\n",
		   ((reg_value & MVME7100_PEX8525_ERROR) ?
		    "Fatal Error" : "No Fatal Error"));

	switch (reg_value & MVME7100_BOARD_TYPE_MASK) {
	case MVME7100_BOARD_TYPE_PRPMC:
		sprintf(buff, "PrPMC");
		break;
	case MVME7100_BOARD_TYPE_VME:
		sprintf(buff, "VME");
		break;
	default:
		sprintf(buff, "Unknown");
		break;
	}
	seq_printf(m, "Board Type\t\t: %s\n", buff);

	/* System Control Register */
	reg_value = readb(reg_block + MVME7100_SYSTEM_CONTROL_REG_OFFSET);
	seq_printf(m, "EEPROM WP Status\t: %s\n",
		   ((reg_value & MVME7100_EEPROM_WP) ?
		    "EEPROM Write Protected" : "EEPROM Not Write Protected"));

	/* Status Indicator Register */
	reg_value = readb(reg_block + MVME7100_STATUS_INDICATOR_REG_OFFSET);
	seq_printf(m, "USR1 Red LED\t\t: %s\n",
		   ((reg_value & MVME7100_USR1_RED_LED) ? "Lit" : "Not Lit"));
	seq_printf(m, "USR1 Yellow LED\t\t: %s\n",
		   ((reg_value & MVME7100_USR1_YELLOW_LED) ? "Lit" :
		    "Not Lit"));
	seq_printf(m, "USR2 LED\t\t: %s\n",
		   ((reg_value & MVME7100_USR2_LED) ? "Lit" : "Not Lit"));
	seq_printf(m, "USR3 LED\t\t: %s\n",
		   ((reg_value & MVME7100_USR3_LED) ? "Lit" : "Not Lit"));

	/* NOR Flash Control/Status Register */
	reg_value = readb(reg_block + MVME7100_NOR_FLASH_CTRL_STAT_REG_OFFSET);
	seq_printf(m, "NOR Flash Map Select\t: %s\n",
		   ((reg_value & MVME7100_NOR_FLASH_MAP_SELECT) ?
		    "Boot Block A Mapped to Highest Address" :
		    "Flash Memory Map Controlled by Flash Boot Block Select"));
	seq_printf(m, "NOR Flash WP SW\t\t: %s\n",
		   ((reg_value & MVME7100_NOR_FLASH_WP_SW) ?
		    "Write Protected" : "Not Write Protected"));
	seq_printf(m, "NOR Flash WP HW\t\t: %s\n",
		   ((reg_value & MVME7100_NOR_FLASH_WP_HW) ?
		    "Write Protected" : "Not Write Protected"));
	seq_printf(m, "NOR Flash Boot Select\t: %s\n",
		   ((reg_value & MVME7100_NOR_FLASH_BLK_SEL) ?
		    "Boot Block B Selected" : "Boot Block A Selected"));
	seq_printf(m, "NOR Flash Ready Status\t: %s\n",
		   ((reg_value & MVME7100_NOR_FLASH_RDY) ?
		    "Bit Set" : "Bit Not Set"));

	/* Interrupt Register 1 */
	reg_value = readb(reg_block + MVME7100_INTERRUPT_REG_1_OFFSET);

	seq_printf(m, "TSEC4 PHY Interrupt\t: %s\n",
		((reg_value & MVME7100_TSEC4_PHY_INTERRUPT) ?
		"Asserted" : "Not Asserted"));

	seq_printf(m, "TSEC3 PHY Interrupt\t: %s\n",
		((reg_value & MVME7100_TSEC3_PHY_INTERRUPT) ?
		"Asserted" : "Not Asserted"));

	seq_printf(m, "TSEC2 PHY Interrupt\t: %s\n",
		((reg_value & MVME7100_TSEC2_PHY_INTERRUPT) ?
		"Asserted" : "Not Asserted"));

	seq_printf(m, "TSEC1 PHY Interrupt\t: %s\n",
		((reg_value & MVME7100_TSEC1_PHY_INTERRUPT) ?
		"Asserted" : "Not Asserted"));

	/* Interrupt Register 2 */
	reg_value = readb(reg_block + MVME7100_INTERRUPT_REG_2_OFFSET);

	seq_printf(m, "RTC Mask\t\t: %s\n",
		   ((reg_value & MVME7100_RTC_MASK) ?
		    "Interrupt Disabled" : "Interrupt Enabled"));
	seq_printf(m, "Temperature Sensor Mask\t: %s\n",
		   ((reg_value & MVME7100_TEMP_MASK) ?
		    "Interrupt Disabled" : "Interrupt Enabled"));
	seq_printf(m, "ABORT Mask\t\t: %s\n",
		   ((reg_value & MVME7100_ABORT_MASK) ?
		    "Interrupt Disabled" : "Interrupt Enabled"));

	seq_printf(m, "RTC Status\t\t: %s\n",
		   ((reg_value & MVME7100_RTC_STATUS) ?
		    "RTC Output Asserted" : "RTC Output Not Asserted"));
	seq_printf(m, "Temp Sensor Status\t: %s\n",
		   ((reg_value & MVME7100_TEMP_STATUS) ?
		    "Temperature Sensor Output Asserted" :
		    "Temperature Sensor Output Not Asserted"));
	seq_printf(m, "Abort Status\t\t: %s\n",
		   ((reg_value & MVME7100_ABORT_STATUS) ?
		    "Abort Switch Asserted" : "Abort Switch Not Asserted"));

	/* Presence Detect Register */
	reg_value = readb(reg_block + MVME7100_PRESENCE_DETECT_REG_OFFSET);

	seq_printf(m, "ERDY2\t\t\t: %s\n",
		   ((reg_value & MVME7100_ERDY2) ?
		    "ERDY2 Set" : "ERDY2 Not Set"));
	seq_printf(m, "ERDY1\t\t\t: %s\n",
		   ((reg_value & MVME7100_ERDY2) ?
		    "ERDY1 Set" : "ERDY1 Not Set"));
	seq_printf(m, "XMCSPAN\t\t\t: %s\n",
		   ((reg_value & MVME7100_XMCSPAN_PRESENT) ?
		    "XMCSPAN Installed" : "XMCSPAN Not Installed"));
	seq_printf(m, "PMC Site 2\t\t: %s\n",
		   ((reg_value & MVME7100_PMC2_PRESENT) ?
		    "PMC Module Installed" : "PMC Module Not Installed"));
	seq_printf(m, "PMC Site 1\t\t: %s\n",
		   ((reg_value & MVME7100_PMC1_PRESENT) ?
		    "PMC Module Installed" : "PMC Module Not Installed"));

	/* NAND Flash1 CTRL Register */
	reg_value = readb(reg_block + MVME7100_NAND_FLASH1_CTRL_REG_OFFSET);
	seq_printf(m, "NAND 1 CONTROL Reg\t: 0x%02x\n", reg_value);

	/* NAND Flash1 SELECT Register */
	reg_value = readb(reg_block + MVME7100_NAND_FLASH1_SELECT_REG_OFFSET);
	seq_printf(m, "NAND 1 SELECT Reg\t: 0x%02x\n", reg_value);

	/* NAND Flash1 Presence Register */
	reg_value = readb(reg_block + MVME7100_NAND_FLASH1_PRESENCE_REG_OFFSET);
	seq_printf(m, "NAND 1 PRESENCE Reg\t: 0x%02x\n", reg_value);

	/* NAND Flash1 STATUS Register */
	reg_value = readb(reg_block + MVME7100_NAND_FLASH1_STATUS_REG_OFFSET);
	seq_printf(m, "NAND 1 STATUS Reg\t: 0x%02x\n", reg_value);

	/* NAND Flash2 CTRL Register */
	reg_value = readb(reg_block + MVME7100_NAND_FLASH2_CTRL_REG_OFFSET);
	seq_printf(m, "NAND 2 CONTROL Reg\t: 0x%02x\n", reg_value);

	/* NAND Flash2 SELECT Register */
	reg_value = readb(reg_block + MVME7100_NAND_FLASH2_SELECT_REG_OFFSET);
	seq_printf(m, "NAND 2 SELECT Reg\t: 0x%02x\n", reg_value);

	/* NAND Flash2 Presence Register */
	reg_value = readb(reg_block + MVME7100_NAND_FLASH2_PRESENCE_REG_OFFSET);
	seq_printf(m, "NAND 2 PRESENCE Reg\t: 0x%02x\n", reg_value);

	/* NAND Flash2 STATUS Register */
	reg_value = readb(reg_block + MVME7100_NAND_FLASH2_STATUS_REG_OFFSET);
	seq_printf(m, "NAND 2 STATUS Reg\t: 0x%02x\n", reg_value);

	/* PLD Revision Register */
	reg_value = readb(reg_block + MVME7100_PLD_REVISION_REG_OFFSET);

	seq_printf(m, "PLD Revision\t\t: 0x%x\n", reg_value);

	/* PLD Date Code Register */
	reg32_value = readl(reg_block + MVME7100_PLD_DATE_CODE_REG_OFFSET);
	seq_printf(m, "PLD Date Code\t\t: 0x%08x\n",
		   (((reg32_value & 0x000000ffU) << 24) |
		    ((reg32_value & 0x0000ff00U) << 8) |
		    ((reg32_value & 0x00ff0000U) >> 8) |
		    ((reg32_value & 0xff000000U) >> 24)));

	iounmap(reg_block);

	return;
}

/*
 * Called very early, device-tree isn't unflattened
 */
static int __init mvme7100_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (of_flat_dt_is_compatible(root, "mpc86xx"))
		return 1;	/* Looks good */

	return 0;
}

long __init mpc86xx_time_init(void)
{
	unsigned int temp;

	/* Set the time base to zero */
	mtspr(SPRN_TBWL, 0);
	mtspr(SPRN_TBWU, 0);

	temp = mfspr(SPRN_HID0);
	temp |= HID0_TBEN;
	mtspr(SPRN_HID0, temp);
	asm volatile ("isync");

	return 0;
}

void __init mvme7100_pcibios_fixup(void)
{
	struct pci_dev *dev = NULL;

	if ((dev = pci_find_device(PCI_VENDOR_ID_NEC,
				   PCI_DEVICE_ID_NEC_USB, NULL))) {
		/* Set System Clock to 48MHz oscillator in EXT2 register */
		pci_write_config_byte(dev, 0xe4, 0x20);
	}

        dev = NULL;
        for_each_pci_dev(dev) {
                pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE,
                                      L1_CACHE_BYTES >> 2);
                pci_write_config_byte(dev, PCI_LATENCY_TIMER, 0x80);
        }

        dev = NULL;
        while ((dev = pci_get_device(PCI_VENDOR_ID_PLX, 0x8114, dev)) != NULL) {
                void __iomem *vaddr;
                u16  ctl;

                /* PLX PEX8114 Errata */
                pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE, 16);

                /* Change Max Read Request Size from 512 to 4096 bytes */
                pci_read_config_word(dev, 0x70, &ctl);
                ctl = (ctl & 0x0fff) | 0x5000;
                pci_write_config_word(dev, 0x70, ctl);


                /* Enable Address Stepping; Enable Memory Read Line Multiple */
                vaddr = ioremap(pci_resource_start(dev, 0),
                                pci_resource_len(dev, 0));
                writel(readl(vaddr + 0xfa0) | 0x00003000, vaddr + 0xfa0);
                iounmap(vaddr);
        }

        dev = NULL;
        while ((dev = pci_get_device(PCI_VENDOR_ID_TUNDRA, 0x8114, dev)) != NULL) {
                u32  tmp;

                pci_read_config_dword(dev, 0xbc, &tmp);
                /* Change prefetch for Memory Read from 1/2 DWords to
		 * whatever is set in the MRL_33/MRL_66 field */
                tmp |= 0x04000000;
                pci_write_config_dword(dev, 0xbc, tmp);
        }

#ifdef CONFIG_MVME7100_ENABLE_L2_ERRORS
	if (request_irq(l2_irq, mvme7100_l2cache_err_handler,
			IRQF_DISABLED, "L2 Cache Error", (void *)&l2_irq) < 0) {
		printk(KERN_ERR "Cannot install L2 cache error handler\n");
	} else {
		/* Enable all error interrupts */
		mtspr(SPRN_L2ERRINTEN, 0x1C);
	}
#endif

#ifdef CONFIG_MVME7100_ENABLE_DDR_ERRORS
	if (request_irq(mpc86xx_ddr_irq, mvme7100_ddr_err_handler,
		IRQF_DISABLED, "DDR Error", (void *)&ddr_err_regs) < 0) {
		printk(KERN_ERR "Cannot install DDR error handler\n");
	} else {
		/* Enable all error interrupts */
		writel(swab32(0x0000000D),
		       ddr_err_regs + DDR_ERR_INT_EN - DDR_ERR_REGS_OFFSET);
	}
#endif

#ifdef CONFIG_MVME7100_ENABLE_PCI_ERRORS
	if (request_irq(pci_irq, mvme7100_pci_err_handler,
		IRQF_SHARED, "PCI Error", (void *)&pci_err_regs) < 0) {
		printk(KERN_ERR "Cannot install PCI error handler\n");
	} else {
		/* Enable detection of all errors */
		writel(0, pci_err_regs + PEX_ERR_DISR - PEX_ERR_REGS_OFFSET);
		/* Enable all error interrupts */
		writel(swab32(0x00bfff00),
		       pci_err_regs + PEX_ERR_EN - PEX_ERR_REGS_OFFSET);
	}
#endif
}

void mvme7100_pcibios_fixup_bus(struct pci_bus *bus)
{
	struct pci_dev *bridge;
	struct pci_controller *hose = (struct pci_controller *)bus->sysdata;
	struct pci_bus_region region;
	u32 l;

	fsl_pcibios_fixup_bus(bus);
	bridge = bus->self;
	if (!bridge)
		return;

	if ((bridge->vendor == PCI_VENDOR_ID_FREESCALE) &&
		(bridge->device == PCI_DEVICE_ID_MPC8641D))
		bridge->irq = pci_irq;

	if (hose->global_number == 0) {
		if (!bus->resource[1])
			return;
		pcibios_resource_to_bus(bridge, &region, bus->resource[1]);
		/* Parent bridge of the Tundra TSI148 VME Bridge has 
		 * to be allocated a lot (1G here) of prefetchable PCI MEM 
		 * which makes it necessary to adjust default allocations */
		if (bus->resource[1]->flags & IORESOURCE_MEM) {
			struct pci_bus *parent;
			unsigned long offset = 0;

			parent = bus->parent;
			offset = hose->pci_mem_offset;
			if ((bridge->vendor == 0x10b5)
			    && (bridge->device == 0x8525)
			    && (bridge->devfn == 0) && parent && parent->self
			    && (parent->self->vendor == 0x1957)
			    && (parent->self->device == 0x7011)
			    && (parent->self->devfn == 0)) {
				region.start = 0x80200000;
				region.end = 0x8fffffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if ((bridge->vendor == 0x10b5) &&
				   (bridge->device == 0x8525) &&
				   (bridge->devfn == 0) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0)) {
				region.start = 0x80200000;
				region.end = 0x802fffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if ((bridge->vendor == 0x10b5) &&
				   (bridge->device == 0x8525) &&
				   (bridge->devfn == 0x8) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0)) {
				region.start = 0x80300000;
				region.end = 0x804fffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if (((bridge->vendor == 0x10b5) || 
				    (bridge->vendor == 0x10e3)) &&
				   (bridge->device == 0x8114) &&
				   (bridge->devfn == 0x0) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0x8)) {
				region.start = 0x80400000;
				region.end = 0x804fffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if ((bridge->vendor == 0x10b5) &&
				   (bridge->device == 0x8525) &&
				   (bridge->devfn == 0x10) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0x0)) {
				region.start = 0x80500000;
				region.end = 0x806fffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if (((bridge->vendor == 0x10b5) || 
                                    (bridge->vendor == 0x10e3)) &&
				   (bridge->device == 0x8114) &&
				   (bridge->devfn == 0x0) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0x10)) {

				region.start = 0x80600000;
				region.end = 0x806fffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if ((bridge->vendor == 0x10b5) &&
				   (bridge->device == 0x8525) &&
				   (bridge->devfn == 0x48) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0x0)) {
				region.start = 0x80700000;
				region.end = 0x808fffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if (((bridge->vendor == 0x10b5) || 
				    (bridge->vendor == 0x10e3)) &&
				   (bridge->device == 0x8114) &&
				   (bridge->devfn == 0x0) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0x48)) {

				region.start = 0x80800000;
				region.end = 0x808fffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if ((bridge->vendor == 0x10b5) &&
				   (bridge->device == 0x8525) &&
				   (bridge->devfn == 0x50) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0x0)) {
				region.start = 0x80900000;
				region.end = 0x80afffff;
				bus->resource[1]->start = region.start + offset;
				bus->resource[1]->end = region.end + offset;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if ((bridge->vendor == 0x10b5) &&
				   ((bridge->device == 0x8111) ||
				    (bridge->device == 0x8112)) &&
				   (bridge->devfn == 0x0) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0x50)) {
				region.start = 0x80a00000;
				region.end = 0x80afffff;
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
			printk(KERN_DEBUG "MEM window: disabled.\n");
		}
		pci_write_config_dword(bridge, PCI_MEMORY_BASE, l);

		if (!bus->resource[2])
			return;

		pcibios_resource_to_bus(bridge, &region, bus->resource[2]);
		{
			struct pci_bus *parent;
			unsigned long offset = 0;

			parent = bus->parent;
			offset = hose->pci_mem_offset;

			/* Clear out the upper 32 bits of PREF limit.
			   If PCI_PREF_BASE_UPPER32 was non-zero, this 
			   temporarily disables PREF range, which is ok. */
			pci_write_config_dword(bridge, PCI_PREF_LIMIT_UPPER32,
					       0);

			if ((bridge->vendor == 0x10b5) &&
			    (bridge->device == 0x8525) &&
			    (bridge->devfn == 0) && parent && parent->self &&
			    (parent->self->vendor == 0x1957) &&
			    (parent->self->device == 0x7011) &&
			    (parent->self->devfn == 0)) {
				region.start = 0x90000000;
				region.end = 0xcfffffff;
				bus->resource[2]->start = region.start + offset;
				bus->resource[2]->end = region.end + offset;
				bus->resource[2]->flags =
				    IORESOURCE_MEM | IORESOURCE_PREFETCH;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if ((bridge->vendor == 0x10b5) &&
				   (bridge->device == 0x8525) &&
				   (bridge->devfn == 0x8) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0)) {
				region.start = 0x90000000;
				region.end = 0xcfffffff;
				bus->resource[2]->start = region.start + offset;
				bus->resource[2]->end = region.end + offset;
				bus->resource[2]->flags =
				    IORESOURCE_MEM | IORESOURCE_PREFETCH;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else if (((bridge->vendor == 0x10b5) || 
				    (bridge->vendor == 0x10e3)) &&
				   (bridge->device == 0x8114) &&
				   (bridge->devfn == 0x0) && parent &&
				   parent->self &&
				   (parent->self->vendor == 0x10b5) &&
				   (parent->self->device == 0x8525) &&
				   (parent->self->devfn == 0x8)) {

				region.start = 0x90000000;
				region.end = 0xcfffffff;
				bus->resource[2]->start = region.start + offset;
				bus->resource[2]->end = region.end + offset;
				bus->resource[2]->flags =
				    IORESOURCE_MEM | IORESOURCE_PREFETCH;

				l = (region.start >> 16) & 0xfff0;
				l |= region.end & 0xfff00000;
			} else
				pci_read_config_dword(bridge,
						      PCI_PREF_MEMORY_BASE, &l);
		}
		pci_write_config_dword(bridge, PCI_PREF_MEMORY_BASE, l);

		/* Clear out the upper 32 bits of PREF base. */
		pci_write_config_dword(bridge, PCI_PREF_BASE_UPPER32, 0);

		pci_write_config_word(bridge, PCI_BRIDGE_CONTROL,
				      bus->bridge_ctl);
	}
}

static void __init mvme7100_map_io(void)
{
	io_block_mapping(0xf0000000, 0xf0000000, 0x01000000, _PAGE_IO);
	io_block_mapping(0xf2400000, 0xf2400000, 0x00080000, _PAGE_IO);
}

define_machine(mvme7100)
{
	.name = "MVME7100",
	.probe = mvme7100_probe,
	.setup_arch = mvme7100_setup_arch,
	.init_IRQ = mvme7100_init_irq,
	.show_cpuinfo = mvme7100_show_cpuinfo,
	.pcibios_fixup = mvme7100_pcibios_fixup,
	.pcibios_fixup_bus = mvme7100_pcibios_fixup_bus,
	.setup_io_mappings = mvme7100_map_io,
	.get_irq = mpic_get_irq,
	.restart = mvme7100_restart,
	.time_init = mpc86xx_time_init,
	.calibrate_decr = generic_calibrate_decr,
	.progress = udbg_progress,
};

static struct of_device_id mpc86xx_of_ids[] = {
	{.type = "soc",},
	{},
};

static __init int mpc86xx_of_device_init(void)
{
	if (!machine_is(mvme7100))
		return 0;
	return of_platform_bus_probe(NULL, mpc86xx_of_ids, NULL);
}

device_initcall(mpc86xx_of_device_init);
