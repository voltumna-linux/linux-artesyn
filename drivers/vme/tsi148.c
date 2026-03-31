/*
 * tsi148.c
 *
 * Support for the Tundra TSI148 VME Bridge Chip
 *
 * Authors: Tom Armistead, Ajit Prem
 * Copyright 2004-2007 Motorola Inc.
 * Copyright 2008 Emerson Network Power Embedded Computing, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "vmedrv.h"
#include "tsi148.h"

extern struct vmeSharedData *vmechip_interboard_data;
extern dma_addr_t vmechip_interboard_datap;
extern const int vmechip_revision;
extern const int vmechip_devid;
extern const int vmechip_irq;
extern int vmechip_irq_overhead_ticks;
extern void __iomem *vmechip_baseaddr;
extern const int vme_slotnum;
extern int vme_syscon;
extern char *in_image_ba[];
extern int in_image_size[];
extern int in_image_mapped[];
extern dma_addr_t in_image_pa[];
extern unsigned int out_image_va[];
extern unsigned int vme_irqlog[8][0x100];
extern struct pci_dev *vme_pci_dev;

extern wait_queue_head_t dma_queue[];
extern wait_queue_head_t lm_queue;
extern wait_queue_head_t mbox_queue;
extern wait_queue_head_t vmeint_queue[];

extern void vme_sync_data(void);
extern void vme_flush_range(unsigned long, unsigned long);
extern int tb_speed;

unsigned int tempe_irq_time;
unsigned int tempe_dma_irq_time[2];
unsigned int tempe_lm_event;

static spinlock_t lm_lock = SPIN_LOCK_UNLOCKED;

/*
 *  add64hi - calculate upper 32 bits of 64 bit addition operation.
 */

static unsigned int
add64hi(unsigned int lo0, unsigned int hi0, unsigned int lo1, unsigned int hi1)
{
	if ((lo1 + lo0) < lo1) {
		return (hi0 + hi1 + 1);
	}
	return (hi0 + hi1);
}

/*
 *  sub64hi - calculate upper 32 bits of 64 bit subtraction operation.
 */

static int
sub64hi(unsigned int lo0, unsigned int hi0, unsigned int lo1, unsigned int hi1)
{
	if (lo0 < lo1) {
		return (hi0 - hi1 - 1);
	}
	return (hi0 - hi1);
}

/*
 *  tsi148_procinfo()
 */

int tsi148_procinfo(char *buf)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	char *p;
	unsigned int i;

	p = buf;

	p += sprintf(p, "\n");
	/* Display outbound decoders */
	p += sprintf(p, "Local Control and Status Register Group (LCSR):\n");
	p += sprintf(p, "\nOutbound Translations:\n");

	p += sprintf(p,
		     "No. otat         otsau:otsal        oteau:oteal         otofu:otofl\n");
	for (i = 0; i < 8; i += 1) {
		p += sprintf(p, "O%d: %08X  %08X:%08X  %08X:%08X:  %08X:%08X\n",
			     i,
			     p_tempe->lcsr.outboundTranslation[i].otat,
			     p_tempe->lcsr.outboundTranslation[i].otsau,
			     p_tempe->lcsr.outboundTranslation[i].otsal,
			     p_tempe->lcsr.outboundTranslation[i].oteau,
			     p_tempe->lcsr.outboundTranslation[i].oteal,
			     p_tempe->lcsr.outboundTranslation[i].otofu,
			     p_tempe->lcsr.outboundTranslation[i].otofl);
	}

	/* Display inbound decoders */
	p += sprintf(p, "\nInbound Translations:\n");
	p += sprintf(p,
		     "No. itat         itsau:itsal        iteau:iteal         itofu:itofl\n");
	for (i = 0; i < 8; i += 1) {
		p += sprintf(p, "O%d: %08X  %08X:%08X  %08X:%08X:  %08X:%08X\n",
			     i,
			     p_tempe->lcsr.inboundTranslation[i].itat,
			     p_tempe->lcsr.inboundTranslation[i].itsau,
			     p_tempe->lcsr.inboundTranslation[i].itsal,
			     p_tempe->lcsr.inboundTranslation[i].iteau,
			     p_tempe->lcsr.inboundTranslation[i].iteal,
			     p_tempe->lcsr.inboundTranslation[i].itofu,
			     p_tempe->lcsr.inboundTranslation[i].itofl);
	}
	p += sprintf(p, "\nVME Bus Control:\n");
	p += sprintf(p, "\tvmctrl  0x%08x\n", p_tempe->lcsr.vmctrl);
	p += sprintf(p, "\tvctrl   0x%08x\n", p_tempe->lcsr.vctrl);
	p += sprintf(p, "\tvstat   0x%08x\n", p_tempe->lcsr.vstat);
	p += sprintf(p, "PCI Status:\n");
	p += sprintf(p, "\tpstat 0x%08x\n", p_tempe->lcsr.pstat);
	p += sprintf(p, "VME Exception Status:\n");
	p += sprintf(p, "\tveau:veal 0x%08x:0x%08x\n", p_tempe->lcsr.veau,
		     p_tempe->lcsr.veal);
	p += sprintf(p, "\tveat 0x%08x\n", p_tempe->lcsr.veat);
	p += sprintf(p, "PCI Error Status:\n");
	p += sprintf(p, "\tedpau:edpal 0x%08x:0x%08x\n", p_tempe->lcsr.edpau,
		     p_tempe->lcsr.edpal);
	p += sprintf(p, "\tedpxa 0x%08x\n", p_tempe->lcsr.edpxa);
	p += sprintf(p, "\tedpxs 0x%08x\n", p_tempe->lcsr.edpxs);
	p += sprintf(p, "\tedpat 0x%08x\n", p_tempe->lcsr.edpat);
	p += sprintf(p, "Inbound Translation GCSR:\n");
	p += sprintf(p, "\tgbau:gbal 0x%08x:0x%08x\n", p_tempe->lcsr.gbau,
		     p_tempe->lcsr.gbal);
	p += sprintf(p, "\tgcsrat 0x%08x\n", p_tempe->lcsr.gcsrat);
	p += sprintf(p, "Inbound Translation CRG:\n");
	p += sprintf(p, "\tcbau:cbal 0x%08x:0x%08x\n", p_tempe->lcsr.cbau,
		     p_tempe->lcsr.cbal);
	p += sprintf(p, "\tcsrat 0x%08x\n", p_tempe->lcsr.csrat);
	p += sprintf(p, "Inbound Translation CR/CSR:\n");
	p += sprintf(p, "\tcrou:crol 0x%08x:0x%08x\n", p_tempe->lcsr.crou,
		     p_tempe->lcsr.crol);
	p += sprintf(p, "\tcrat 0x%08x\n", p_tempe->lcsr.crat);
	p += sprintf(p, "Inbound Translation Location Monitor:\n");
	p += sprintf(p, "\tlmbau:lmbal 0x%08x:0x%08x:\n", p_tempe->lcsr.lmbau,
		     p_tempe->lcsr.lmbal);
	p += sprintf(p, "\tlmat 0x%08x\n", p_tempe->lcsr.lmat);
	p += sprintf(p, "Interrupt Control:\n");
	p += sprintf(p, "\tvicr  0x%08x\n", p_tempe->lcsr.vicr);
	p += sprintf(p, "\tinten 0x%08x\n", p_tempe->lcsr.inten);
	p += sprintf(p, "\tinteo 0x%08x\n", p_tempe->lcsr.inteo);
	p += sprintf(p, "\tints  0x%08x\n", p_tempe->lcsr.ints);
	p += sprintf(p, "\tintc  0x%08x\n", p_tempe->lcsr.intc);
	p += sprintf(p, "\tintm1 0x%08x\n", p_tempe->lcsr.intm1);
	p += sprintf(p, "\tintm2 0x%08x\n", p_tempe->lcsr.intm2);

	p += sprintf(p, "DMA Control:\n");
	p += sprintf(p, "\tdctl0  0x%08x\n", p_tempe->lcsr.dma[0].dctl);
	p += sprintf(p, "\tdctl1  0x%08x\n", p_tempe->lcsr.dma[1].dctl);

	p += sprintf(p, "\nPCFS Register Group:\n");
	p += sprintf(p, "\tpciveni  0x%04x\n", swab16(p_tempe->pcfs.veni));
	p += sprintf(p, "\tpcidevi  0x%04x\n", swab16(p_tempe->pcfs.devi));
	p += sprintf(p, "\tpcicmd   0x%04x\n", swab16(p_tempe->pcfs.cmmd));
	p += sprintf(p, "\tpcistat  0x%04x\n", swab16(p_tempe->pcfs.stat));
	p += sprintf(p, "\tpcirev   0x%02x\n",
		     (p_tempe->pcfs.reviAndClas >> 24));
	p += sprintf(p, "\tpciclass 0x%06x\n",
		     ((p_tempe->pcfs.reviAndClas << 24) >> 8) |
		     swab16((p_tempe->pcfs.reviAndClas << 8) >> 16));
	p += sprintf(p, "\tpciclsz  0x%02x\n", p_tempe->pcfs.clsz);
	p += sprintf(p, "\tpcimlat  0x%02x\n", p_tempe->pcfs.mlat);
	p += sprintf(p, "\tpcisubv  0x%04x\n", swab16(p_tempe->pcfs.subv));
	p += sprintf(p, "\tpcisubi  0x%04x\n", swab16(p_tempe->pcfs.subi));
	p += sprintf(p, "\tpcixcap  0x%08x\n", p_tempe->pcfs.msiCapID);
	p += sprintf(p, "\tpcixstat 0x%08x\n",
		     swab32(p_tempe->pcfs.msiMsgAddrL));

	p += sprintf(p, "\nGlobal Control and Status Register Group (GCSR):\n");
	p += sprintf(p, "\tctrl  0x%04x%02x%02x\n", p_tempe->gcsr.ctrl,
		     p_tempe->gcsr.ga, p_tempe->gcsr.revid);

	p += sprintf(p, "\nCR/CSR Register Group:\n");
	p += sprintf(p, "\tbcr   0x%08x\n", p_tempe->crcsr.csrbcr);
	p += sprintf(p, "\tbsr   0x%08x\n", p_tempe->crcsr.csrbsr);

	return p - buf;
}

/*
 * Function   : tempe_setup_attribute
 *
 * Description: helper function for calculating attribute register contents.
 *
 */
static int
tempe_setup_attribute(addressMode_t addrSpace,
		      int userAccessType,
		      int dataAccessType,
		      dataWidth_t maxDataWidth,
		      int xferProtocol, vme2esstRate_t xferRate2esst)
{
	int temp_ctl = 0;

	/* Validate & initialize address space field */
	switch (addrSpace) {
	case VME_A16:
		temp_ctl |= 0x0;
		break;
	case VME_A24:
		temp_ctl |= 0x1;
		break;
	case VME_A32:
		temp_ctl |= 0x2;
		break;
	case VME_A64:
		temp_ctl |= 0x4;
		break;
	case VME_CRCSR:
		temp_ctl |= 0x5;
		break;
	case VME_USER1:
		temp_ctl |= 0x8;
		break;
	case VME_USER2:
		temp_ctl |= 0x9;
		break;
	case VME_USER3:
		temp_ctl |= 0xA;
		break;
	case VME_USER4:
		temp_ctl |= 0xB;
		break;
	default:
		return (-EINVAL);
	}

	/* Setup CTL register */
	if (userAccessType & VME_SUPER)
		temp_ctl |= 0x0020;
	if (dataAccessType & VME_PROG)
		temp_ctl |= 0x0010;
	if (maxDataWidth == VME_D16)
		temp_ctl |= 0x0000;
	if (maxDataWidth == VME_D32)
		temp_ctl |= 0x0040;
	switch (xferProtocol) {
	case VME_SCT:
		temp_ctl |= 0x000;
		break;
	case VME_BLT:
		temp_ctl |= 0x100;
		break;
	case VME_MBLT:
		temp_ctl |= 0x200;
		break;
	case VME_2eVME:
		temp_ctl |= 0x300;
		break;
	case VME_2eSST:
		temp_ctl |= 0x400;
		break;
	case VME_2eSSTB:
		temp_ctl |= 0x500;
		break;
	}
	switch (xferRate2esst) {
	case VME_SSTNONE:
	case VME_SST160:
		temp_ctl |= 0x0000;
		break;
	case VME_SST267:
		temp_ctl |= 0x800;
		break;
	case VME_SST320:
		temp_ctl |= 0x1000;
		break;
	}

	return (temp_ctl);
}

/*
 *  tempe_bus_error_chk()
 *  Return zero if no VME bus error has occured, 1 otherwise.
 *  Optionally, clear bus error status.
 */

int tempe_bus_error_chk(int clrflag)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int tmp;

	tmp = p_tempe->lcsr.veat;
	if (tmp & 0x80000000) {	/* VES is Set */
		if (clrflag)
			p_tempe->lcsr.veat = 0x20000000;
		return (1);
	}
	return (0);
}

/*
 * Function:    DMA_tempe_irqhandler
 * Description: Saves DMA completion timestamp and then wakes up DMA queue
 */

static void DMA_tempe_irqhandler(int channel_mask)
{

	if (channel_mask & 1) {
		tempe_dma_irq_time[0] = tempe_irq_time;
		wake_up(&dma_queue[0]);
	}
	if (channel_mask & 2) {
		tempe_dma_irq_time[1] = tempe_irq_time;
		wake_up(&dma_queue[1]);
	}
}

/*
 * Function   : LERR_tempe_irqhandler
 * Description: Display error & status message when LERR (PCI) exception
 *              interrupt occurs.
 *
 */

static void LERR_tempe_irqhandler(void)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;

	/* 
	 * Only print a message if this is a real error.  
	 * Dummy errors can sometimes happen here. 
   	 * A real error is indicated if the EDPST bit 
	 * is set in the EDPAT register.
	 */
	if (p_tempe->lcsr.edpat & 0x80000000) {
		printk(KERN_ERR
			 "tsi148: VME PCI Exception at address: 0x%08x:%08x, attributes: %08x\n",
			p_tempe->lcsr.edpau, p_tempe->lcsr.edpal, 
			p_tempe->lcsr.edpat);
		printk(KERN_ERR
			"tsi148: PCI-X attribute reg: %08x, PCI-X split completion reg: %08x\n",
			p_tempe->lcsr.edpxa, p_tempe->lcsr.edpxs);
	}
	/* Clear the PCI error by writing the EDPCL bit in the EDPAT register */
	p_tempe->lcsr.edpat = 0x20000000;
	vme_sync_data();

	/* Also clear the PCI error interrupt (PERRC) in the INTC register */
	p_tempe->lcsr.intc = 0x00002000;
	vme_sync_data();
}

/*
 * Function:     VERR_tempe_irqhandler
 * Description:  Display error & status when VME error interrupt occurs.
 * Not normally enabled or used.  tempe_bus_error_chk() is used instead.
 */

static void VERR_tempe_irqhandler(void)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;

	printk(KERN_ERR
	       "tsi148: VME Exception at address: 0x%08x:%08x, attributes: %08x\n",
	       p_tempe->lcsr.veau, p_tempe->lcsr.veal, p_tempe->lcsr.veat);
	p_tempe->lcsr.veat = 0x20000000;
	vme_sync_data();
}

/*
 * Function   : MB_tempe_irqhandler
 * Description: Wake up mail box queue.
 */

static void MB_tempe_irqhandler(int mboxMask)
{
	if (vmechip_irq_overhead_ticks != 0) {
		wake_up(&mbox_queue);
	}
}

/*
 * Function   : LM_tempe_irqhandler
 * Description: Wake up location monitor queue
 */

static void LM_tempe_irqhandler(int lm_mask)
{
	tempe_lm_event = lm_mask;
	wake_up(&lm_queue);
}

/*
 * Function:    VIRQ_tempe_irqhandler
 * Description: Record the level & vector from the VME bus interrupt.
 */

static void VIRQ_tempe_irqhandler(int virq_mask)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int iackvec, i;

	for (i = 7; i > 0; i--) {
		if (virq_mask & (1 << i)) {
			iackvec = p_tempe->lcsr.viack[(i * 4) + 3];
			vme_irqlog[i][iackvec]++;
			wake_up(&vmeint_queue[i]);
		}
	}
	wake_up(&vmeint_queue[0]);
}

/*
 * Function   : tempe_irqhandler
 * Description: Top level interrupt handler.  Clears appropriate interrupt
 *              status bits and then calls appropriate sub handler(s).
 */

static irqreturn_t tempe_irqhandler(int irq, void *dev_id)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	long stat, enable;

	if (dev_id != p_tempe)
		return IRQ_NONE;

	/* Save time that this IRQ occurred at */
	tempe_irq_time = get_tbl();

	/* Determine which interrupts are unmasked and active */
	enable = p_tempe->lcsr.inteo;
	stat = p_tempe->lcsr.ints;
	stat = stat & enable;

	/* Clear them */
	p_tempe->lcsr.intc = stat;
	vme_sync_data();

	/* Call subhandlers as appropriate */
	if (stat & 0x03000000)	/* DMA irqs */
		DMA_tempe_irqhandler((stat & 0x03000000) >> 24);
	if (stat & 0x00002000)	/* PCI bus error */
		LERR_tempe_irqhandler();
	if (stat & 0x00001000)	/* VME bus error */
		VERR_tempe_irqhandler();
	if (stat & 0x000F0000)	/* Mail box irqs */
		MB_tempe_irqhandler((stat & 0xF0000) >> 16);
	if (stat & 0x00F00000)	/* Location monitor irqs */
		LM_tempe_irqhandler((stat & 0xF00000) >> 20);
	if (stat & 0x000000FE)	/* VME bus irqs */
		VIRQ_tempe_irqhandler(stat & 0x0000FE);

	stat = p_tempe->lcsr.ints;
	
	return IRQ_HANDLED;
}

/*
 * Function   : tempe_generate_irq
 * Description: Generate a VME bus interrupt at the requested level & vector.
 *              Wait for system controller to ack the interrupt.
 */
int tempe_generate_irq(virqInfo_t * vmeIrq)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int timeout;
	int looptimeout;
	unsigned int tmp;

	timeout = vmeIrq->waitTime;
	if (timeout == 0) {
		timeout += 10;	/* Wait at least 10 ticks */
	}
	looptimeout = HZ / 20;	/* try for 1/20 second */

	vmeIrq->timeOutFlag = 0;

	/* Validate & setup vector register */
	tmp = p_tempe->lcsr.vicr;
	tmp &= ~0xFF;
	p_tempe->lcsr.vicr = tmp | vmeIrq->vector;
	vme_sync_data();

	/* Assert VMEbus IRQ */
	p_tempe->lcsr.vicr = tmp | (vmeIrq->level << 8) | vmeIrq->vector;
	vme_sync_data();

	/* Wait for syscon to do iack */
	while (p_tempe->lcsr.vicr & 0x800) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(looptimeout);
		timeout = timeout - looptimeout;
		if (timeout <= 0) {
			vmeIrq->timeOutFlag = 1;
			break;
		}
	}

	return (0);
}

/*
 * Function:    tempe_set_arbiter
 * Description: Set the VME bus arbiter with the requested attributes
 */

int tempe_set_arbiter(vmeArbiterCfg_t * vmeArb)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;
	int gto = 0;

	temp_ctl = p_tempe->lcsr.vctrl;
	temp_ctl &= 0xFFEFFF00;

	if (vmeArb->globalTimeoutTimer == 0xFFFFFFFF) {
		gto = 8;
	} else if (vmeArb->globalTimeoutTimer > 2048) {
		return (-EINVAL);
	} else if (vmeArb->globalTimeoutTimer == 0) {
		gto = 0;
	} else {
		gto = 1;
		while ((16 * (1 << (gto - 1))) < vmeArb->globalTimeoutTimer) {
			gto += 1;
		}
	}
	temp_ctl |= gto;

	if (vmeArb->arbiterMode != VME_PRIORITY_MODE) {
		temp_ctl |= 1 << 6;
	}

	if (vmeArb->arbiterTimeoutFlag) {
		temp_ctl |= 1 << 7;
	}

	if (vmeArb->noEarlyReleaseFlag) {
		temp_ctl |= 1 << 20;
	}
	p_tempe->lcsr.vctrl = temp_ctl;
	vme_sync_data();

	return (0);
}

/*
 * Function:    tempe_get_arbiter
 * Description: Return the attributes of the VME bus arbiter.
 */

int tempe_get_arbiter(vmeArbiterCfg_t * vmeArb)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;
	int gto = 0;

	temp_ctl = p_tempe->lcsr.vctrl;

	gto = temp_ctl & 0xF;
	if (gto != 0) {
		vmeArb->globalTimeoutTimer = (16 * (1 << (gto - 1)));
	}

	if (temp_ctl & (1 << 6)) {
		vmeArb->arbiterMode = VME_R_ROBIN_MODE;
	} else {
		vmeArb->arbiterMode = VME_PRIORITY_MODE;
	}

	if (temp_ctl & (1 << 7)) {
		vmeArb->arbiterTimeoutFlag = 1;
	}

	if (temp_ctl & (1 << 20)) {
		vmeArb->noEarlyReleaseFlag = 1;
	}

	return (0);
}

//-----------------------------------------------------------------------------
// Function   : tempe_set_requestor
// Description: Set the VME bus requestor with the requested attributes
//-----------------------------------------------------------------------------
int tempe_set_requestor(vmeRequesterCfg_t * vmeReq)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;

	temp_ctl = p_tempe->lcsr.vmctrl;
	temp_ctl &= 0xFFFF0000;

	if (vmeReq->releaseMode == 1) {
		temp_ctl |= (1 << 3);
	}

	if (vmeReq->fairMode == 1) {
		temp_ctl |= (1 << 2);
	}

	temp_ctl |= (vmeReq->timeonTimeoutTimer & 7) << 8;
	temp_ctl |= (vmeReq->timeoffTimeoutTimer & 7) << 12;
	temp_ctl |= vmeReq->requestLevel;

	p_tempe->lcsr.vmctrl = temp_ctl;
	vme_sync_data();
	return (0);
}

//-----------------------------------------------------------------------------
// Function   : tempe_get_requestor
// Description: Return the attributes of the VME bus requestor
//-----------------------------------------------------------------------------
int tempe_get_requestor(vmeRequesterCfg_t * vmeReq)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;

	temp_ctl = p_tempe->lcsr.vmctrl;

	if (temp_ctl & 0x18) {
		vmeReq->releaseMode = 1;
	}

	if (temp_ctl & (1 << 2)) {
		vmeReq->fairMode = 1;
	}

	vmeReq->requestLevel = temp_ctl & 3;
	vmeReq->timeonTimeoutTimer = (temp_ctl >> 8) & 7;
	vmeReq->timeoffTimeoutTimer = (temp_ctl >> 12) & 7;

	return (0);
}

//-----------------------------------------------------------------------------
// Function   : tempe_set_in_bound
// Description: Initialize an inbound window with the requested attributes.
//-----------------------------------------------------------------------------
int tempe_set_in_bound(vmeInWindowCfg_t * vmeIn)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;
	unsigned int win, x, granularity = 0x10000;
	struct page *page, *pend;

	/* Verify input data */
	if (vmeIn->windowNbr > 7) {
		return (-EINVAL);
	}
	win = vmeIn->windowNbr;

	/* Free previous allocation */
#ifndef CONFIG_VME_BRIDGE_BOOTMEM
	if (in_image_ba[win] != NULL) {
#else
	if ((win != 7) && (in_image_ba[win] != NULL)) {
#endif
             	 /* Undo marking the pages as reserved */
              	 pend = virt_to_page(in_image_ba[win] + in_image_size[win] - 1);
              	 for (page = virt_to_page(in_image_ba[win]); page <= pend; page++)
              	         ClearPageReserved(page);
              	 dma_free_coherent(&vme_pci_dev->dev,
              	                         in_image_size[win],
              	                         in_image_ba[win],
              	                         in_image_pa[win]);
              	 in_image_mapped[win] = 0;
       	}

	if (vmeIn->windowEnable == 0) {
		temp_ctl = p_tempe->lcsr.inboundTranslation[win].itat;
		temp_ctl &= ~0x80000000;
		p_tempe->lcsr.inboundTranslation[win].itat = temp_ctl;
		vme_sync_data();
		return 0;
	}

	switch (vmeIn->addrSpace) {
	case VME_CRCSR:
	case VME_USER1:
	case VME_USER2:
	case VME_USER3:
	case VME_USER4:
		return (-EINVAL);
	case VME_A16:
		granularity = 0x10;
		temp_ctl |= 0x00;
		break;
	case VME_A24:
		granularity = 0x1000;
		temp_ctl |= 0x10;
		break;
	case VME_A32:
		granularity = 0x10000;
		temp_ctl |= 0x20;
		break;
	case VME_A64:
		granularity = 0x10000;
		temp_ctl |= 0x40;
		break;
	}

#ifdef CONFIG_VME_BRIDGE_BOOTMEM
	if (win == 7) {
		vmeIn->windowSizeL = in_image_size[7];
		vmeIn->windowSizeU = 0;
	}
#endif 
	if ((vmeIn->vmeAddrL & (granularity - 1)) || 
		(vmeIn->windowSizeL & (granularity - 1)))  
		return (-EINVAL);

#ifndef CONFIG_VME_BRIDGE_BOOTMEM
	if ((vmeIn->windowSizeL < 0x10) || (vmeIn->windowSizeL > 0x800000)) 
#else
	if ((win != 7) && 
		((vmeIn->windowSizeL < 0x10) || (vmeIn->windowSizeL > 0x800000))) 
#endif
		return (-EINVAL);

#ifndef CONFIG_VME_BRIDGE_BOOTMEM
	if (vmeIn->windowEnable == 1) {
#else
	if ((win != 7) && (vmeIn->windowEnable == 1)) {
#endif
		in_image_ba[win] = (char *)dma_alloc_coherent((struct device *)
				&vme_pci_dev->dev, 
				vmeIn->windowSizeL,
				(dma_addr_t *)&in_image_pa[win],
				GFP_KERNEL);
                if (!in_image_ba[win]) {
                        printk(KERN_ERR "vmedrv: No memory for inbound window\n");
                        return -ENOMEM;
                }
		in_image_size[win] = vmeIn->windowSizeL;

		/* now mark the pages as reserved; otherwise */
		/* remap_pfn_range doesn't do what we want */
		pend = virt_to_page(in_image_ba[win] + in_image_size[win] - 1);
                for (page = virt_to_page(in_image_ba[win]); page <= pend; page++)
			SetPageReserved(page);
		memset(in_image_ba[win], 0, in_image_size[win]);
		vme_flush_range((unsigned long)in_image_ba[win],
					(unsigned long)(in_image_ba[win] + in_image_size[win]));
		vmeIn->pciAddrL = (unsigned int)in_image_pa[win];
		vmeIn->pciAddrU = 0;
#ifndef CONFIG_VME_BRIDGE_BOOTMEM
        }
#else
	} else if (win == 7)  {
		vmeIn->pciAddrL = (unsigned int)in_image_pa[win];
		vmeIn->pciAddrU = 0;
	}
#endif

	// Disable while we are mucking around
	p_tempe->lcsr.inboundTranslation[win].itat = 0;
	vme_sync_data();

	p_tempe->lcsr.inboundTranslation[win].itsal = vmeIn->vmeAddrL;
	p_tempe->lcsr.inboundTranslation[win].itsau = vmeIn->vmeAddrU;
	p_tempe->lcsr.inboundTranslation[win].iteal =
	    vmeIn->vmeAddrL + vmeIn->windowSizeL - granularity;

	p_tempe->lcsr.inboundTranslation[win].iteau =
	    add64hi(vmeIn->vmeAddrL, vmeIn->vmeAddrU,
		    vmeIn->windowSizeL - granularity, vmeIn->windowSizeU);
	p_tempe->lcsr.inboundTranslation[win].itofl =
	    vmeIn->pciAddrL - vmeIn->vmeAddrL;
	p_tempe->lcsr.inboundTranslation[win].itofu =
	    sub64hi(vmeIn->pciAddrL, vmeIn->pciAddrU, vmeIn->vmeAddrL,
		    vmeIn->vmeAddrU);

	/* Setup CTL register */
	temp_ctl |= (vmeIn->xferProtocol & 0x3E) << 6;
	for (x = 0; x < 4; x++) {
		if ((64 << x) >= vmeIn->prefetchSize) {
			break;
		}
	}
	if (x == 4)
		x--;
	temp_ctl |= (x << 16);

	if (vmeIn->prefetchThreshold)

		if (vmeIn->prefetchThreshold)
			temp_ctl |= 0x40000;

	switch (vmeIn->xferRate2esst) {
	case VME_SSTNONE:
		break;
	case VME_SST160:
		temp_ctl |= 0x0400;
		break;
	case VME_SST267:
		temp_ctl |= 0x1400;
		break;
	case VME_SST320:
		temp_ctl |= 0x2400;
		break;
	}

	if (vmeIn->userAccessType & VME_USER)
		temp_ctl |= 0x4;
	if (vmeIn->userAccessType & VME_SUPER)
		temp_ctl |= 0x8;
	if (vmeIn->dataAccessType & VME_DATA)
		temp_ctl |= 0x1;
	if (vmeIn->dataAccessType & VME_PROG)
		temp_ctl |= 0x2;

	/* Write ctl reg without enable */
	p_tempe->lcsr.inboundTranslation[win].itat = temp_ctl;
	vme_sync_data();

	if (vmeIn->windowEnable)
		temp_ctl |= 0x80000000;

	p_tempe->lcsr.inboundTranslation[win].itat = temp_ctl;
	vme_sync_data();

	return (0);
}

//-----------------------------------------------------------------------------
// Function   : tempe_get_in_bound
// Description: Return the attributes of an inbound window.
//-----------------------------------------------------------------------------
int tempe_get_in_bound(vmeInWindowCfg_t * vmeIn)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;
	unsigned int i, vme_end_u, vme_end_l;

	/* Verify input data */
	if (vmeIn->windowNbr > 7) {
		return (-EINVAL);
	}
	i = vmeIn->windowNbr;

	temp_ctl = p_tempe->lcsr.inboundTranslation[i].itat;

	// Get Control & BUS attributes
	if (temp_ctl & 0x80000000)
		vmeIn->windowEnable = 1;
	vmeIn->xferProtocol = ((temp_ctl & 0xF80) >> 6) | VME_SCT;
	vmeIn->prefetchSize = 64 << ((temp_ctl >> 16) & 3);
	vmeIn->wrPostEnable = 1;
	vmeIn->prefetchEnable = 1;
	if (temp_ctl & 0x40000)
		vmeIn->prefetchThreshold = 1;
	if (temp_ctl & 0x4)
		vmeIn->userAccessType |= VME_USER;
	if (temp_ctl & 0x8)
		vmeIn->userAccessType |= VME_SUPER;
	if (temp_ctl & 0x1)
		vmeIn->dataAccessType |= VME_DATA;
	if (temp_ctl & 0x2)
		vmeIn->dataAccessType |= VME_PROG;

	switch ((temp_ctl & 0x70) >> 4) {
	case 0x0:
		vmeIn->addrSpace = VME_A16;
		break;
	case 0x1:
		vmeIn->addrSpace = VME_A24;
		break;
	case 0x2:
		vmeIn->addrSpace = VME_A32;
		break;
	case 0x4:
		vmeIn->addrSpace = VME_A64;
		break;
	}

	switch ((temp_ctl & 0x7000) >> 12) {
	case 0x0:
		vmeIn->xferRate2esst = VME_SST160;
		break;
	case 0x1:
		vmeIn->xferRate2esst = VME_SST267;
		break;
	case 0x2:
		vmeIn->xferRate2esst = VME_SST320;
		break;
	}

	// Get VME inbound start & end addresses
	vmeIn->vmeAddrL = p_tempe->lcsr.inboundTranslation[i].itsal;
	vmeIn->vmeAddrU = p_tempe->lcsr.inboundTranslation[i].itsau;
	vme_end_l = p_tempe->lcsr.inboundTranslation[i].iteal;
	vme_end_u = p_tempe->lcsr.inboundTranslation[i].iteau;

	// Adjust addresses for window granularity
	switch (vmeIn->addrSpace) {
	case VME_A16:
		vmeIn->vmeAddrU = 0;
		vmeIn->vmeAddrL &= 0x0000FFF0;
		vme_end_u = 0;
		vme_end_l &= 0x0000FFF0;
		vme_end_l += 0x10;
		break;
	case VME_A24:
		vmeIn->vmeAddrU = 0;
		vmeIn->vmeAddrL &= 0x00FFF000;
		vme_end_u = 0;
		vme_end_l &= 0x00FFF000;
		vme_end_l += 0x1000;
		break;
	case VME_A32:
		vmeIn->vmeAddrU = 0;
		vmeIn->vmeAddrL &= 0xFFFF0000;
		vme_end_u = 0;
		vme_end_l &= 0xFFFF0000;
		vme_end_l += 0x10000;
		break;
	default:
		vmeIn->vmeAddrL &= 0xFFFF0000;
		vme_end_l &= 0xFFFF0000;
		vme_end_l += 0x10000;
		break;
	}

	/* Calculate size of window */
	vmeIn->windowSizeL = vme_end_l - vmeIn->vmeAddrL;
	vmeIn->windowSizeU = sub64hi(vme_end_l, vme_end_u,
				     vmeIn->vmeAddrL, vmeIn->vmeAddrU);

	/* Calculate corresponding PCI bus address */
	vmeIn->pciAddrL =
	    vmeIn->vmeAddrL + p_tempe->lcsr.inboundTranslation[i].itofl;
	vmeIn->pciAddrU =
	    add64hi(vmeIn->vmeAddrL, vmeIn->vmeAddrU,
		    p_tempe->lcsr.inboundTranslation[i].itofl,
		    p_tempe->lcsr.inboundTranslation[i].itofu);

	return (0);
}

//-----------------------------------------------------------------------------
// Function   : tempe_set_out_bound
// Description: Set the attributes of an outbound window.
//-----------------------------------------------------------------------------
int tempe_set_out_bound(vmeOutWindowCfg_t * vmeOut)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;
	unsigned int i, x;

	/* Verify input data */
	if (vmeOut->windowNbr > 7) {
		return (-EINVAL);
	}
	i = vmeOut->windowNbr;

	if (vmeOut->windowEnable == 0) {
		temp_ctl = p_tempe->lcsr.outboundTranslation[i].otat;
		temp_ctl &= ~0x80000000;
		p_tempe->lcsr.outboundTranslation[i].otat = temp_ctl;
		vme_sync_data();
		return 0;
	}

	if ((vmeOut->xlatedAddrL & 0xFFFF) ||
	    (vmeOut->windowSizeL & 0xFFFF) || (vmeOut->pciBusAddrL & 0xFFFF)) {
		return (-EINVAL);
	}

	temp_ctl = tempe_setup_attribute(vmeOut->addrSpace,
					 vmeOut->userAccessType,
					 vmeOut->dataAccessType,
					 vmeOut->maxDataWidth,
					 vmeOut->xferProtocol,
					 vmeOut->xferRate2esst);

	if (vmeOut->prefetchEnable) {
		for (x = 0; x < 4; x++) {
			if ((2 << x) >= vmeOut->prefetchSize)
				break;
		}
		if (x == 4)
			x = 3;
		temp_ctl |= (x << 16);
	} else {
		temp_ctl |= 0x40000;
	}
	// Disable while we are mucking around
	p_tempe->lcsr.outboundTranslation[i].otat = 0;
	vme_sync_data();

	p_tempe->lcsr.outboundTranslation[i].otbs = vmeOut->bcastSelect2esst;
	p_tempe->lcsr.outboundTranslation[i].otsal = vmeOut->pciBusAddrL;
	p_tempe->lcsr.outboundTranslation[i].otsau = vmeOut->pciBusAddrU;
	p_tempe->lcsr.outboundTranslation[i].oteal =
	    vmeOut->pciBusAddrL + (vmeOut->windowSizeL - 0x10000);
	p_tempe->lcsr.outboundTranslation[i].oteau =
	    add64hi(vmeOut->pciBusAddrL, vmeOut->pciBusAddrU,
		    vmeOut->windowSizeL - 0x10000, vmeOut->windowSizeU);
	p_tempe->lcsr.outboundTranslation[i].otofl =
	    vmeOut->xlatedAddrL - vmeOut->pciBusAddrL;
	if (vmeOut->addrSpace == VME_A64) {
		p_tempe->lcsr.outboundTranslation[i].otofu =
		    sub64hi(vmeOut->xlatedAddrL, vmeOut->xlatedAddrU,
			    vmeOut->pciBusAddrL, vmeOut->pciBusAddrU);
	} else {
		p_tempe->lcsr.outboundTranslation[i].otofu = 0;
	}

	// Write ctl reg without enable
	p_tempe->lcsr.outboundTranslation[i].otat = temp_ctl;
	vme_sync_data();

	if (vmeOut->windowEnable)
		temp_ctl |= 0x80000000;

	p_tempe->lcsr.outboundTranslation[i].otat = temp_ctl;
	vme_sync_data();

	return (0);
}

//-----------------------------------------------------------------------------
// Function   : tempe_get_out_bound
// Description: Return the attributes of an outbound window.
//-----------------------------------------------------------------------------
int tempe_get_out_bound(vmeOutWindowCfg_t * vmeOut)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;
	unsigned int i;

	// Verify input data
	if (vmeOut->windowNbr > 7) {
		return (-EINVAL);
	}
	i = vmeOut->windowNbr;

	// Get Control & BUS attributes
	temp_ctl = p_tempe->lcsr.outboundTranslation[i].otat;
	vmeOut->wrPostEnable = 1;
	if (temp_ctl & 0x0020)
		vmeOut->userAccessType = VME_SUPER;
	else
		vmeOut->userAccessType = VME_USER;
	if (temp_ctl & 0x0010)
		vmeOut->dataAccessType = VME_PROG;
	else
		vmeOut->dataAccessType = VME_DATA;
	if (temp_ctl & 0x80000000)
		vmeOut->windowEnable = 1;

	switch ((temp_ctl & 0xC0) >> 6) {
	case 0:
		vmeOut->maxDataWidth = VME_D16;
		break;
	case 1:
		vmeOut->maxDataWidth = VME_D32;
		break;
	}
	vmeOut->xferProtocol = 1 << ((temp_ctl >> 8) & 7);

	switch (temp_ctl & 0xF) {
	case 0x0:
		vmeOut->addrSpace = VME_A16;
		break;
	case 0x1:
		vmeOut->addrSpace = VME_A24;
		break;
	case 0x2:
		vmeOut->addrSpace = VME_A32;
		break;
	case 0x4:
		vmeOut->addrSpace = VME_A64;
		break;
	case 0x5:
		vmeOut->addrSpace = VME_CRCSR;
		break;
	case 0x8:
		vmeOut->addrSpace = VME_USER1;
		break;
	case 0x9:
		vmeOut->addrSpace = VME_USER2;
		break;
	case 0xA:
		vmeOut->addrSpace = VME_USER3;
		break;
	case 0xB:
		vmeOut->addrSpace = VME_USER4;
		break;
	}

	vmeOut->xferRate2esst = VME_SSTNONE;
	if (vmeOut->xferProtocol == VME_2eSST) {

		switch (temp_ctl & 0x1800) {
		case 0x000:
			vmeOut->xferRate2esst = VME_SST160;
			break;
		case 0x800:
			vmeOut->xferRate2esst = VME_SST267;
			break;
		case 0x1000:
			vmeOut->xferRate2esst = VME_SST320;
			break;
		}

	}
	/* Get Window mappings */

	vmeOut->bcastSelect2esst = p_tempe->lcsr.outboundTranslation[i].otbs;
	vmeOut->pciBusAddrL = p_tempe->lcsr.outboundTranslation[i].otsal;
	vmeOut->pciBusAddrU = p_tempe->lcsr.outboundTranslation[i].otsau;

	vmeOut->windowSizeL =
	    (p_tempe->lcsr.outboundTranslation[i].oteal + 0x10000) -
	    vmeOut->pciBusAddrL;
	vmeOut->windowSizeU =
	    sub64hi(p_tempe->lcsr.outboundTranslation[i].oteal + 0x10000,
		    p_tempe->lcsr.outboundTranslation[i].oteau,
		    vmeOut->pciBusAddrL, vmeOut->pciBusAddrU);
	vmeOut->xlatedAddrL =
	    p_tempe->lcsr.outboundTranslation[i].otofl + vmeOut->pciBusAddrL;
	if (vmeOut->addrSpace == VME_A64) {
		vmeOut->xlatedAddrU =
		    add64hi(p_tempe->lcsr.outboundTranslation[i].otofl,
			    p_tempe->lcsr.outboundTranslation[i].otofu,
			    vmeOut->pciBusAddrL, vmeOut->pciBusAddrU);
	} else {
		vmeOut->xlatedAddrU = 0;
	}

	return (0);
}

/*
 * Function   : tempe_setup_lm
 * Description: Set the attributes of the location monitor
 */

int tempe_setup_lm(vmeLmCfg_t * vmeLm)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;

	/* Setup CTL register */
	switch (vmeLm->addrSpace) {
	case VME_A16:
		temp_ctl |= 0x00;
		break;
	case VME_A24:
		temp_ctl |= 0x10;
		break;
	case VME_A32:
		temp_ctl |= 0x20;
		break;
	case VME_A64:
		temp_ctl |= 0x40;
		break;
	default:
		return (-EINVAL);
	}
	if (vmeLm->userAccessType & VME_USER)
		temp_ctl |= 0x4;
	if (vmeLm->userAccessType & VME_SUPER)
		temp_ctl |= 0x8;
	if (vmeLm->dataAccessType & VME_DATA)
		temp_ctl |= 0x1;
	if (vmeLm->dataAccessType & VME_PROG)
		temp_ctl |= 0x2;

	/* Disable while we are mucking around */
	p_tempe->lcsr.lmat = 0;
	vme_sync_data();

	p_tempe->lcsr.lmbal = vmeLm->addr;
	p_tempe->lcsr.lmbau = vmeLm->addrU;

	tempe_lm_event = 0;

	/* Write ctl reg and enable */
	p_tempe->lcsr.lmat = temp_ctl | 0x80;
	vme_sync_data();

	return (0);
}

/*
 * Function:    tempe_wait_lm
 * Description: Wait for location monitor to be triggered.
 */

int tempe_wait_lm(vmeLmCfg_t * vmeLm)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	unsigned long flags;
	unsigned int tmp;

	spin_lock_irqsave(&lm_lock, flags);
	tmp = tempe_lm_event;
	spin_unlock_irqrestore(&lm_lock, flags);
	if (tmp == 0) {
		if (vmeLm->lmWait < 10)
			vmeLm->lmWait = 10;
		interruptible_sleep_on_timeout(&lm_queue, vmeLm->lmWait);
	}
	p_tempe->lcsr.lmat = 0;
	vme_sync_data();
	vmeLm->lmEvents = tempe_lm_event;
	return (0);
}

//-----------------------------------------------------------------------------
// Function   : tempe_do_rmw
// Description: Perform an RMW cycle on the VME bus.
//    A VME outbound window must already be setup which maps to the desired 
//    RMW address.
//-----------------------------------------------------------------------------
int tempe_do_rmw(vmeRmwCfg_t * vmeRmw)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int temp_ctl = 0;
	unsigned int vme_end_u, vme_end_l;
	int *rmw_pci_data_ptr = NULL;
	int *va_data_ptr = NULL;
	int i;
	vmeOutWindowCfg_t vmeOut;

	if (vmeRmw->maxAttempts < 1) {
		return (-EINVAL);
	}
	// Find the PCI address that maps to the desired VME address 
	for (i = 0; i < 7; i++) {
		temp_ctl = p_tempe->lcsr.outboundTranslation[i].otat;
		if ((temp_ctl & 0x80000000) == 0) {
			continue;
		}
		memset(&vmeOut, 0, sizeof(vmeOut));
		vmeOut.windowNbr = i;
		tempe_get_out_bound(&vmeOut);
		if (vmeOut.addrSpace != vmeRmw->addrSpace) {
			continue;
		}
		if (vmeOut.xlatedAddrU > vmeRmw->targetAddrU) {
			continue;
		}
		if (vmeOut.xlatedAddrU == vmeRmw->targetAddrU) {
			if (vmeOut.xlatedAddrL > vmeRmw->targetAddr) {
				continue;
			}
		}
		vme_end_l = vmeOut.xlatedAddrL + vmeOut.windowSizeL;
		vme_end_u = add64hi(vmeOut.xlatedAddrL, vmeOut.xlatedAddrU,
				    vmeOut.windowSizeL, vmeOut.windowSizeU);
		if (sub64hi(vme_end_l, vme_end_u,
			    vmeRmw->targetAddr, vmeRmw->targetAddrU) >= 0) {
			rmw_pci_data_ptr =
			    (int *)(vmeOut.pciBusAddrL +
				    (vmeRmw->targetAddr - vmeOut.xlatedAddrL));
			va_data_ptr =
			    (int *)(out_image_va[i] +
				    (vmeRmw->targetAddr - vmeOut.xlatedAddrL));
			break;
		}
	}

	// If no window - fail.
	if (rmw_pci_data_ptr == NULL) {
		return (-EINVAL);
	}
	// Setup the RMW registers.
	p_tempe->lcsr.vmctrl &= ~0x100000;
	vme_sync_data();
	p_tempe->lcsr.rmwen = vmeRmw->enableMask;
	p_tempe->lcsr.rmwc = vmeRmw->compareData;
	p_tempe->lcsr.rmws = vmeRmw->swapData;
	p_tempe->lcsr.rmwau = 0;
	p_tempe->lcsr.rmwal = (int)rmw_pci_data_ptr;
	p_tempe->lcsr.vmctrl |= 0x100000;
	vme_sync_data();

	// Run the RMW cycle until either success or max attempts.
	vmeRmw->numAttempts = 1;
	while (vmeRmw->numAttempts <= vmeRmw->maxAttempts) {

		if ((*va_data_ptr & vmeRmw->enableMask) ==
		    (vmeRmw->swapData & vmeRmw->enableMask)) {

			break;

		}
		vmeRmw->numAttempts++;
	}

	p_tempe->lcsr.vmctrl &= ~0x100000;
	vme_sync_data();

	// If no success, set num Attempts to be greater than max attempts
	if (vmeRmw->numAttempts > vmeRmw->maxAttempts) {
		vmeRmw->numAttempts = vmeRmw->maxAttempts + 1;
	}

	return (0);
}

//-----------------------------------------------------------------------------
// Function   : dma_src_attr, dma_dst_attr
// Description: Helper functions which setup common portions of the 
//      DMA source and destination attribute registers.
//-----------------------------------------------------------------------------
static int dma_src_attr(vmeDmaPacket_t *vmeCur)
{

	int dsatreg = 0;
	// calculate source attribute register
	switch (vmeCur->srcBus) {
	case VME_DMA_PATTERN_BYTE:
		dsatreg = 0x23000000;
		break;
	case VME_DMA_PATTERN_BYTE_INCREMENT:
		dsatreg = 0x22000000;
		break;
	case VME_DMA_PATTERN_WORD:
		dsatreg = 0x21000000;
		break;
	case VME_DMA_PATTERN_WORD_INCREMENT:
		dsatreg = 0x20000000;
		break;
	case VME_DMA_PCI:
		dsatreg = 0x00000000;
		break;
	case VME_DMA_VME:
		dsatreg = 0x10000000;
		dsatreg |= tempe_setup_attribute(vmeCur->srcVmeAttr.addrSpace,
						 vmeCur->srcVmeAttr.
						 userAccessType,
						 vmeCur->srcVmeAttr.
						 dataAccessType,
						 vmeCur->srcVmeAttr.
						 maxDataWidth,
						 vmeCur->srcVmeAttr.
						 xferProtocol,
						 vmeCur->srcVmeAttr.
						 xferRate2esst);
		break;
	default:
		dsatreg = -EINVAL;
		break;
	}
	return (dsatreg);
}

static int dma_dst_attr(vmeDmaPacket_t * vmeCur)
{
	int ddatreg = 0;
	// calculate destination attribute register
	switch (vmeCur->dstBus) {
	case VME_DMA_PCI:
		ddatreg = 0x00000000;
		break;
	case VME_DMA_VME:
		ddatreg = 0x10000000;
		ddatreg |= tempe_setup_attribute(vmeCur->dstVmeAttr.addrSpace,
						 vmeCur->dstVmeAttr.
						 userAccessType,
						 vmeCur->dstVmeAttr.
						 dataAccessType,
						 vmeCur->dstVmeAttr.
						 maxDataWidth,
						 vmeCur->dstVmeAttr.
						 xferProtocol,
						 vmeCur->dstVmeAttr.
						 xferRate2esst);
		break;
	default:
		ddatreg = -EINVAL;
		break;
	}
	return (ddatreg);
}

//-----------------------------------------------------------------------------
// Function   : tempe_start_dma
// Description: Write the DMA controller registers with the contents
//    needed to actually start the DMA operation.
//    returns starting time stamp.
//
//    Starts either direct or chained mode as appropriate (based on dctlreg)
//-----------------------------------------------------------------------------
static unsigned int
tempe_start_dma(int channel,
		unsigned int dctlreg, tsi148DmaDescriptor_t * vmeLL)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	unsigned int val;

	// Setup registers as needed for direct or chained.
	if (dctlreg & 0x800000) {

		// Write registers 
		p_tempe->lcsr.dma[channel].dsau = vmeLL->dsau;
		p_tempe->lcsr.dma[channel].dsal = vmeLL->dsal;
		p_tempe->lcsr.dma[channel].ddau = vmeLL->ddau;
		p_tempe->lcsr.dma[channel].ddal = vmeLL->ddal;
		p_tempe->lcsr.dma[channel].dsat = vmeLL->dsat;
		p_tempe->lcsr.dma[channel].ddat = vmeLL->ddat;
		p_tempe->lcsr.dma[channel].dcnt = vmeLL->dcnt;
		p_tempe->lcsr.dma[channel].ddbs = vmeLL->ddbs;
	} else {
		p_tempe->lcsr.dma[channel].dnlau = 0;
		p_tempe->lcsr.dma[channel].dnlal = (unsigned int)vmeLL;
	}
	vme_sync_data();

	// Start the operation
	p_tempe->lcsr.dma[channel].dctl = dctlreg | 0x2000000;
	val = get_tbl();
	vme_sync_data();
	return (val);
}

//-----------------------------------------------------------------------------
// Function   : tempe_setup_dma
// Description: Create a linked list (possibly only 1 element long) of 
//   Tempe DMA descriptors which will perform the DMA operation described
//   by the DMA packet.  Flush descriptors from cache.
//   Return pointer to beginning of list (or 0 on failure.).
//-----------------------------------------------------------------------------
static tsi148DmaDescriptor_t *tempe_setup_dma(vmeDmaPacket_t * vmeDma)
{
	vmeDmaPacket_t *vme_cur;
	int max_per_page;
	int currentLLcount;
	tsi148DmaDescriptor_t *startLL;
	tsi148DmaDescriptor_t *currentLL;
	tsi148DmaDescriptor_t *nextLL;

	max_per_page = PAGE_SIZE / sizeof(tsi148DmaDescriptor_t) - 1;
	startLL = (tsi148DmaDescriptor_t *) __get_free_pages(GFP_KERNEL, 0);
	if (startLL == 0) {
		return (startLL);
	}
	// First allocate pages for descriptors and create linked list
	vme_cur = vmeDma;
	currentLL = startLL;
	currentLLcount = 0;
	while (vme_cur != 0) {
		if (vme_cur->pNextPacket != 0) {
			currentLL->dnlau = (unsigned int)0;
			currentLL->dnlal = (unsigned int)(currentLL + 1);
			currentLLcount++;
			if (currentLLcount >= max_per_page) {
				currentLL->dnlal =
				    __get_free_pages(GFP_KERNEL, 0);
				currentLLcount = 0;
			}
			currentLL = (tsi148DmaDescriptor_t *) currentLL->dnlal;
		} else {
			currentLL->dnlau = (unsigned int)0;
			currentLL->dnlal = (unsigned int)0;
		}
		vme_cur = vme_cur->pNextPacket;
	}

	// Next fill in information for each descriptor
	vme_cur = vmeDma;
	currentLL = startLL;
	while (vme_cur != 0) {
		currentLL->dsau = vme_cur->srcAddrU;
		currentLL->dsal = vme_cur->srcAddr;
		currentLL->ddau = vme_cur->dstAddrU;
		currentLL->ddal = vme_cur->dstAddr;
		currentLL->dsat = dma_src_attr(vme_cur);
		currentLL->ddat = dma_dst_attr(vme_cur);
		currentLL->dcnt = vme_cur->byteCount;
		currentLL->ddbs = vme_cur->bcastSelect2esst;

		currentLL = (tsi148DmaDescriptor_t *) currentLL->dnlal;
		vme_cur = vme_cur->pNextPacket;
	}

	// Convert Links to PCI addresses.
	currentLL = startLL;
	while (currentLL != 0) {
		nextLL = (tsi148DmaDescriptor_t *) currentLL->dnlal;
		currentLL->dnlal = (unsigned int)virt_to_bus(nextLL);
		if (nextLL == 0)
			currentLL->dnlal |= 1;
		vme_flush_range((unsigned long)currentLL, 
			(unsigned long)(currentLL + sizeof(tsi148DmaDescriptor_t)));
		currentLL = nextLL;
	}

	// Return pointer to descriptors list 
	return (startLL);
}

//-----------------------------------------------------------------------------
// Function   : tempe_free_dma
// Description: Free all memory that is used to hold the DMA 
//    descriptor linked list.  
//
//-----------------------------------------------------------------------------
static int tempe_free_dma(tsi148DmaDescriptor_t * startLL)
{
	tsi148DmaDescriptor_t *currentLL;
	tsi148DmaDescriptor_t *prevLL;
	tsi148DmaDescriptor_t *nextLL;

	// Convert Links to virtual addresses.
	currentLL = startLL;
	while (currentLL != 0) {
		if (currentLL->dnlal & 1) {
			currentLL->dnlal = 0;
		} else {
			currentLL->dnlal =
			    (unsigned int)bus_to_virt(currentLL->dnlal);
		}
		currentLL = (tsi148DmaDescriptor_t *) currentLL->dnlal;
	}

	// Free all pages associated with the descriptors.
	currentLL = startLL;
	prevLL = currentLL;
	while (currentLL != 0) {
		nextLL = (tsi148DmaDescriptor_t *) currentLL->dnlal;
		if (currentLL + 1 != nextLL) {
			free_pages((int)prevLL, 0);
			prevLL = nextLL;
		}
		currentLL = nextLL;
	}

	// Return pointer to descriptors list 
	return (0);
}

/*
 * Function   : tempe_do_dma
 * Description:  
 *  Sanity check the DMA request. 
 *  Setup DMA attribute register. 
 *  Create linked list of DMA descriptors.
 *  Invoke actual DMA operation.
 *  Wait for completion.  Record ending time.
 *  Free the linked list of DMA descriptors.
 */

int tempe_do_dma(vmeDmaPacket_t * vmeDma)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	unsigned int dctlreg = 0;
	int val;
	int channel, x;
	vmeDmaPacket_t *cur_dma;
	tsi148DmaDescriptor_t *dmaLL;

	/* Sanity check the VME chain */
	channel = vmeDma->channel_number;
	if (channel > 1) {
		return (-EINVAL);
	}
	cur_dma = vmeDma;
	while (cur_dma != 0) {
		if (cur_dma->byteCount == 0) {
			return (-EINVAL);
		}
		if (dma_src_attr(cur_dma) < 0) {
			return (-EINVAL);
		}
		if (dma_dst_attr(cur_dma) < 0) {
			return (-EINVAL);
		}

		cur_dma = cur_dma->pNextPacket;
		if (cur_dma == vmeDma) {	// Endless Loop!
			return (-EINVAL);
		}
	}

	/* calculate control register */
	if (vmeDma->pNextPacket != 0) {
		dctlreg = 0;
	} else {
		dctlreg = 0x800000;
	}

	for (x = 0; x < 8; x++) {	// vme block size
		if ((32 << x) >= vmeDma->maxVmeBlockSize) {
			break;
		}
	}
	if (x == 8)
		x = 7;
	dctlreg |= (x << 12);

	for (x = 0; x < 8; x++) {	// pci block size
		if ((32 << x) >= vmeDma->maxPciBlockSize) {
			break;
		}
	}
	if (x == 8)
		x = 7;
	dctlreg |= (x << 4);

	if (vmeDma->vmeBackOffTimer) {
		for (x = 1; x < 8; x++) {	// vme timer
			if ((1 << (x - 1)) >= vmeDma->vmeBackOffTimer) {
				break;
			}
		}
		if (x == 8)
			x = 7;
		dctlreg |= (x << 8);
	}

	if (vmeDma->pciBackOffTimer) {
		for (x = 1; x < 8; x++) {	// pci timer
			if ((1 << (x - 1)) >= vmeDma->pciBackOffTimer) {
				break;
			}
		}
		if (x == 8)
			x = 7;
		dctlreg |= (x << 0);
	}
	/* Setup the dma chain */
	dmaLL = tempe_setup_dma(vmeDma);

	/* Start the DMA */
	if (dctlreg & 0x800000) {
		vmeDma->vmeDmaStartTick =
		    tempe_start_dma(channel, dctlreg, dmaLL);
	} else {
		vmeDma->vmeDmaStartTick =
		    tempe_start_dma(channel, dctlreg, (tsi148DmaDescriptor_t *)
				    virt_to_phys(dmaLL));
	}

	wait_event_interruptible(dma_queue[channel],
				 (p_tempe->lcsr.dma[channel].
				  dsta & 0x1000000) == 0);
	val = p_tempe->lcsr.dma[channel].dsta;

	vmeDma->vmeDmaStopTick = tempe_dma_irq_time[channel];
	vmeDma->vmeDmaStatus = val;
	if (vmeDma->vmeDmaStopTick < vmeDma->vmeDmaStartTick) {
		vmeDma->vmeDmaElapsedTime =
		    (0xFFFFFFFF - vmeDma->vmeDmaStartTick) +
		    vmeDma->vmeDmaStopTick;
	} else {
		vmeDma->vmeDmaElapsedTime =
		    vmeDma->vmeDmaStopTick - vmeDma->vmeDmaStartTick;
	}
	vmeDma->vmeDmaElapsedTime -= vmechip_irq_overhead_ticks;
	vmeDma->vmeDmaElapsedTime /= (tb_speed / 1000000);

	vmeDma->vmeDmaStatus = 0;
	if (val & 0x10000000) {
		printk(KERN_ERR
		       "tsi148: DMA Error in DMA_tempe_irqhandler DSTA=%08X\n",
		       val);
		vmeDma->vmeDmaStatus = val;

	}
	/* Free the dma chain */
	tempe_free_dma(dmaLL);

	return (0);
}

/*
 * Function:    tsi148_shutdown
 * Description: Put VME bridge in quiescent state.  Disable all decoders,
 * clear all interrupts.
 */

void tsi148_shutdown(void)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int i;

	/*
	 *  Shutdown all inbound and outbound windows.
	 */
	for (i = 0; i < 8; i++) {
		p_tempe->lcsr.inboundTranslation[i].itat = 0;
		p_tempe->lcsr.outboundTranslation[i].otat = 0;
	}

	/*
	 *  Shutdown Location monitor.
	 */
	p_tempe->lcsr.lmat = 0;

	/*
	 *  Shutdown CRG map.
	 */
	p_tempe->lcsr.csrat = 0;

	/*
	 *  Clear error status.
	 */
	p_tempe->lcsr.edpat = 0xFFFFFFFF;
	p_tempe->lcsr.veat = 0xFFFFFFFF;
	p_tempe->lcsr.pstat = 0x07000700;

	/*
	 *  Remove VIRQ interrupt (if any)
	 */
	if (p_tempe->lcsr.vicr & 0x800) {
		p_tempe->lcsr.vicr = 0x8000;
	}

	/*
	 *  Disable and clear all interrupts.
	 */
	p_tempe->lcsr.inteo = 0;
	vme_sync_data();
	p_tempe->lcsr.intc = 0xFFFFFFFF;
	vme_sync_data();
	p_tempe->lcsr.inten = 0xFFFFFFFF;

	/*
	 *  Map all Interrupts to PCI INTA
	 */
	p_tempe->lcsr.intm1 = 0;
	p_tempe->lcsr.intm2 = 0;

	vme_sync_data();
}

/*
 * Function:    tempe_init()
 * Description: Initialize the VME chip as needed by the driver.
 *    Quiesce VME bridge.
 *    Setup default decoders.
 *    Connect IRQ handler and enable interrupts.
 *    Conduct quick sanity check of bridge.
 */

int tempe_init(void)
{
	tsi148_t *p_tempe = (tsi148_t *) vmechip_baseaddr;
	int result;
	unsigned int tmp;
	unsigned int crcsr_addr;
	u32 vme_stat_reg;
//	int overhead_ticks;
//	unsigned int irq_overhead_start;

	tsi148_shutdown();

	/* Determine syscon and slot number */
	tmp = p_tempe->lcsr.vstat;
	if (tmp & 0x100) {
		vme_syscon = 1;
	} else {
		vme_syscon = 0;
	}

	/* Initialize crcsr map */
	if (vme_slotnum != -1) {
		p_tempe->crcsr.cbar = vme_slotnum << 3;
		crcsr_addr = vme_slotnum * 512 * 1024;
		p_tempe->lcsr.crol = (unsigned long)vmechip_interboard_datap;
		p_tempe->lcsr.crou = 0;
		vme_sync_data();
		p_tempe->lcsr.crat = 0xFFFFFFFF;
	}

	result = request_irq(vmechip_irq,
				tempe_irqhandler,
				IRQF_SHARED,
				"VMEBus (Tsi148)", 
				vmechip_baseaddr);
	if (result) {
		printk(KERN_ERR
		       "tsi148: can't get assigned pci irq vector %02X\n",
		       vmechip_irq);
		return (0);
	}
	/* Enable DMA, mailbox, VIRQ (syscon only) & LM Interrupts */
	if (vme_syscon)
		tmp = 0x03FF34FE;
//		tmp = 0x03FF20FE;
	else
		tmp = 0x03FF3400;
//		tmp = 0x03FF2000;
	p_tempe->lcsr.inteo = tmp;
	p_tempe->lcsr.inten = tmp;

	/* Do a quick sanity test of the bridge */
	if (p_tempe->lcsr.inteo != tmp) {
		return (0);
	}
	/* Sanity check register access */
	for (tmp = 1; tmp < 0x80000000; tmp = tmp << 1) {
		p_tempe->lcsr.rmwen = tmp;
		p_tempe->lcsr.rmwc = ~tmp;
		vme_sync_data();
		if (p_tempe->lcsr.rmwen != tmp) {
			return (0);
		}
		if (p_tempe->lcsr.rmwc != ~tmp) {
			return (0);
		}
	}

	/* Calculate IRQ overhead */
#if 0
	irq_overhead_start = get_tbl();
	p_tempe->gcsr.mbox[0] = 0;
	vme_sync_data();
	for (tmp = 0; tmp < 10; tmp++) {
		vme_sync_data();
	}

	overhead_ticks = tempe_irq_time - irq_overhead_start;
	if (overhead_ticks > 0) {
		vmechip_irq_overhead_ticks = overhead_ticks;
	} else {
		vmechip_irq_overhead_ticks = 1;
	}
#endif
	vmechip_irq_overhead_ticks = 2;

	/* Clear board fail, and power-up reset */
	vme_stat_reg = p_tempe->lcsr.vstat;
	vme_stat_reg &= ~TSI148_LCSR_VSTAT_BRDFL;
	vme_stat_reg |= TSI148_LCSR_VSTAT_CPURST;
	p_tempe->lcsr.vstat = vme_stat_reg;

	return (1);
}
