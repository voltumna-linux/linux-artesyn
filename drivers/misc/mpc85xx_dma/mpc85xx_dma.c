/*
 * MPC85xx DMA Support
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 * Copyright 2005-2007 Motorola Inc.
 *
 * Based on work done by Jason McMullan
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/mpc85xx.h>
#include <asm/mpc85xx_dma.h>
#include <asm/semaphore.h>

#undef DMA_DEBUG

#ifdef DMA_DEBUG
# ifndef assert
#  define assert(x) do { if (!(x)) { printk("KERNEL: assertion (" #x ") failed at " __FILE__ "(%d):%s\n", __LINE__, __FUNCTION__);  \
	} } while (0)
# endif
# define DPRINTK(x, a...) printk(x , ## a)
#else
# ifndef assert
#  define assert(x) do { /**/ } while (0)
# endif
# define DPRINTK(x, a...) do { /**/ } while (0)
#endif

/* DMA Registers(0x2_1000-0x2_3003) */

struct ccsr_dma {
	/* Skip first 256 bytes */
	/* 0x21100 - DMA */
	struct {
		uint32_t mr;	/* 0x00 - DMA 0 Mode Register */
		uint32_t sr;	/* 0x04 - DMA 0 Status Register
				 */
		uint8_t res2[4];
		uint32_t clndar;	/* 0x2110c - DMA 0 Current Link Descriptor Address Register */
		uint32_t satr;	/* 0x10 - DMA 0 Source Attributes Register */
		uint32_t sar;	/* 0x14 - DMA 0 Source Address Register */
		uint32_t datr;	/* 0x18 - DMA 0 Destination Attributes Register */
		uint32_t dar;	/* 0x1c - DMA 0 Destination Address Register */
		uint32_t bcr;	/* 0x20 - DMA 0 Byte Count Register */
		uint8_t res3[4];
		uint32_t nlndar;	/* 0x28 - DMA 0 Next Link Descriptor Address Register */
		uint8_t res4[8];
		uint32_t clabdar;	/* 0x34 - DMA 0 Current List - Alternate Base Descriptor Address Register */
		uint8_t res5[4];
		uint32_t nlsdar;	/* 0x3c - DMA 0 Next List Descriptor Address Register */
		uint32_t ssr;	/* 0x40 - DMA 0 Source Stride Register */
		uint32_t dsr;	/* 0x44 - DMA 0 Destination Stride Register */
		uint8_t res6[56];
	} dma[4];
	uint32_t dgsr;		/* 0x21300 - DMA General Status Register */
};

struct mpc85xx_dma_chain {
	dma_addr_t link_pa;
	struct mpc85xx_dma_desc *link;
	int max;
	int len;
};

struct mpc85xx_dma_channel {
	mpc85xx_dma_f func;
	void *data;
	struct mpc85xx_dma_chain *chain;
} mpc85xx_dma_info[4];

struct semaphore mpc85xx_dma_lock;
static struct ccsr_dma *dma_addr;

#define DMA_R(channel,reg)	(dma_addr->dma[channel].reg)
#define DMA_W(channel,reg,val)	do { DPRINTK(KERN_INFO "DMA %d.%s = 0x%.*x\n",channel, #reg, sizeof(DMA_R(channel,reg)) * 2, val); DMA_R(channel,reg)=val; } while (0)
#define DMA_RMW(channel,reg,op,val)  do { DMA_W(channel,reg,DMA_R(channel,reg) op val);  } while (0)

static irqreturn_t mpc85xx_dma_irq(int irq, void *data)
{
	struct mpc85xx_dma_channel *info;
	int channel, i;
	int event = 0;
	uint32_t status, handled = 0;

	/* The irq can be one of four channels */
	channel = irq - MPC85xx_IRQ_DMA0;
	assert(channel >= 0 && channel < 4);

	status = DMA_R(channel, sr);
	DPRINTK("DMA%d: IRQ, status=0x%.2x (%d.sr)\n", 
			channel, status, channel);

	i = 0;
	while (status & MPC85xx_SR_CB) {
		status = DMA_R(channel, sr);
		i++;
	}
	if (i > 0)
		DPRINTK("DMA%d: Polled status %d times while busy\n", 
			channel, i);

	if (status & MPC85xx_SR_CH) {
		DPRINTK("DMA%d: Channel halted\n", channel);
		handled |= MPC85xx_SR_CH;
		event = MPC85xx_DMA_EVENT_HALTED;

	} else if (status & (MPC85xx_SR_TE | MPC85xx_SR_PE)) {
		if (status & MPC85xx_SR_TE)
			DPRINTK("DMA%d: Transfer error\n", channel);
		if (status & MPC85xx_SR_PE)
			DPRINTK("DMA%d: Programming Error\n", channel);
		handled |= (status & (MPC85xx_SR_TE | MPC85xx_SR_PE));
		event = MPC85xx_DMA_EVENT_ERROR;
	} else if (status &
		   (MPC85xx_SR_EOSI | MPC85xx_SR_EOLSI | MPC85xx_SR_EOLNI)) {
		handled |=
		    status & (MPC85xx_SR_EOSI | MPC85xx_SR_EOLSI |
			      MPC85xx_SR_EOLNI);
		event = MPC85xx_DMA_EVENT_DONE;
	}

	DPRINTK("DMA%d: status 0x%.2x, handled 0x%.2x, event %d\n", 
			channel, status, handled, event);

	if (status & MPC85xx_SR_CB) {
		printk("DMA%d: Crazy busy: status=0x%.2x\n", channel, status);
		for (;;) ;
	}

	if ((status & ~handled) != 0 || event == MPC85xx_DMA_EVENT_ERROR) {
		DPRINTK("status = %.8x handled=%.8x\n", status, handled);
		DPRINTK("DGSR  =%.8x\n", dma_addr->dgsr);
		DPRINTK("    mr=%.8x\n", DMA_R(channel, mr));
		DPRINTK("    sr=%.8x\n", DMA_R(channel, sr));
		DPRINTK("clndar=%.8x\n", DMA_R(channel, clndar));
		DPRINTK("  satr=%.8x\n", DMA_R(channel, satr));
		DPRINTK("   sar=%.8x\n", DMA_R(channel, sar));
		DPRINTK("  datr=%.8x\n", DMA_R(channel, datr));
		DPRINTK("   dar=%.8x\n", DMA_R(channel, dar));
		DPRINTK("   bcr=%.8x\n", DMA_R(channel, bcr));
		DPRINTK("nlndar=%.8x\n", DMA_R(channel, nlndar));
		DPRINTK("clbdar=%.8x\n", DMA_R(channel, clabdar));
		DPRINTK("nlsdar=%.8x\n", DMA_R(channel, nlsdar));
		DPRINTK("   ssr=%.8x\n", DMA_R(channel, ssr));
		DPRINTK("   dsr=%.8x\n", DMA_R(channel, dsr));
	}

	info = &mpc85xx_dma_info[channel];
	if (info->func != NULL) {
		DPRINTK("DMA%d: Calling event function 0x%p(0x%p)\n", channel,
			info->func, info->data);
		info->func(channel, event, info->data);
	}

	DMA_R(channel, sr) |= handled;
	DMA_R(channel, sr) &= ~handled;
	DMA_W(channel, mr, 0);
	dma_addr->dgsr |= dma_addr->dgsr;

	DPRINTK("DMA%d: status now 0x%.2x\n", channel, DMA_R(channel, sr));

	return IRQ_HANDLED;
}

/* 
 * Remove any flags that we don't want the user to be able to set
 */
static inline int clean_flags(int flags)
{
	flags &= ~(MPC85xx_MR_SWSM |
		   MPC85xx_MR_SRW |
		   MPC85xx_MR_XFE |
		   MPC85xx_MR_EOSIE |
		   MPC85xx_MR_EOLNIE |
		   MPC85xx_MR_EOLSIE |
		   MPC85xx_MR_EIE |
		   MPC85xx_MR_CA |
		   MPC85xx_MR_CTM |
		   MPC85xx_MR_CDSM | 
		   MPC85xx_MR_CC | 
		   MPC85xx_MR_CS);
	return flags;
}

EXPORT_SYMBOL(mpc85xx_dma_xfer);
int mpc85xx_dma_xfer(int channel, int flags, phys_addr_t dst, phys_addr_t src,
		     size_t bytes)
{
	flags = clean_flags(flags);

	/* Direct mode */
	flags |= MPC85xx_MR_CTM | MPC85xx_MR_EIE | MPC85xx_MR_EOSIE;

	if (bytes & ~MPC85xx_BCR_MASK)
		return -EINVAL;

	if (mpc85xx_dma_busy(channel))
		return -EBUSY;

	/* Assert that we aren't busy */
	assert((DMA_R(channel, sr) & MPC85xx_SR_CB) == 0);

	if (src > 0xf0000000UL) {
		/* Read, don't snoop local proc */
		DMA_W(channel, satr, MPC85xx_SATR_SREADTYPE_L_NO_SNOOP);
	} else {
		/* Read, snoop local proc */
		DMA_W(channel, satr, MPC85xx_SATR_SREADTYPE_L_SNOOP);
	}

	if (dst > 0xf0000000UL) {
		/* Write, don't snoop local proc */
		DMA_W(channel, datr, MPC85xx_DATR_DWRITETYPE_L_NO_SNOOP);
	} else {
		/* Write, don't snoop local proc */
		DMA_W(channel, datr, MPC85xx_DATR_DWRITETYPE_L_SNOOP);
	}
	DMA_W(channel, bcr, bytes);
	DMA_W(channel, sar, src);
	DMA_W(channel, dar, dst);

	/* If using external start, no need to set CS outselves */
	if ((flags & MPC85xx_MR_EMS_EN) == 0)
		flags |= MPC85xx_MR_CS;

	DMA_W(channel, mr, flags);
	wmb();

	return 0;
}

EXPORT_SYMBOL(mpc85xx_dma_chain_xfer);
int mpc85xx_dma_chain_xfer(int channel, int flags,
			   struct mpc85xx_dma_chain *chain)
{
	/* Chains must be 32-byte aligned */
	if (chain == NULL)
		return -EINVAL;

	if (chain->len == 0)
		return -EINVAL;

	if (mpc85xx_dma_busy(channel))
		return -EBUSY;

	/* Chain mode */
	flags = clean_flags(flags);

	flags |= MPC85xx_MR_EIE | MPC85xx_MR_EOLNIE;

	DPRINTK(KERN_INFO
		"DMA chain: channel %d at 0x%p, flags=0x%x, chain=0x%p\n",
		channel, &dma_addr->dma[channel], flags, chain);
	{
		int i;
		for (i = 0; i < chain->len; i++) {
			DPRINTK(KERN_INFO "chain[%d] vaddr 0x%p paddr 0x%.8x\n",
				i,
				&chain->link[i],
				chain->link_pa + i * sizeof(chain->link[0]));
			DPRINTK(KERN_INFO "   satr   0x%.8x\n",
				chain->link[i].satr);
			DPRINTK(KERN_INFO "   sar    0x%.8x\n",
				chain->link[i].sar);
			DPRINTK(KERN_INFO "   datr   0x%.8x\n",
				chain->link[i].datr);
			DPRINTK(KERN_INFO "   dar    0x%.8x\n",
				chain->link[i].dar);
			DPRINTK(KERN_INFO "   nlndar 0x%.8x\n",
				chain->link[i].nlndar);
			DPRINTK(KERN_INFO "   bcr    0x%.8x\n",
				chain->link[i].bcr);
		}
	}

	__dma_sync(&chain->link[0], sizeof(chain->link[0]) * chain->len,
		   PCI_DMA_TODEVICE);

	/* Assert that we aren't busy */
	assert((DMA_R(channel, sr) & MPC85xx_SR_CB) == 0);

	/* Point to the first link in the chain */
	DMA_W(channel, clndar, chain->link_pa);

	/* If using external start, no need to set CS outselves */
	if ((flags & MPC85xx_MR_EMS_EN) == 0) {
		flags |= MPC85xx_MR_CS;
		DMA_W(channel, mr, flags & ~MPC85xx_MR_CS);
	}

	DMA_W(channel, mr, flags);

	return (0);
}

EXPORT_SYMBOL(mpc85xx_dma_abort);
void mpc85xx_dma_abort(int channel)
{
	DMA_RMW(channel, mr, |, MPC85xx_MR_CA);
}

EXPORT_SYMBOL(mpc85xx_dma_halt);
void mpc85xx_dma_halt(int channel)
{
	DMA_W(channel, nlndar, 0);
}

EXPORT_SYMBOL(mpc85xx_dma_busy);
int mpc85xx_dma_busy(int channel)
{
	return (dma_addr->dma[channel].sr & MPC85xx_SR_CB) ? 1 : 0;
}


EXPORT_SYMBOL(mpc85xx_dma_chain_alloc);
struct mpc85xx_dma_chain *mpc85xx_dma_chain_alloc(int links)
{
	struct mpc85xx_dma_chain *chain;

	if (links < 0)
		return NULL;

	chain = kmalloc(sizeof(*chain), GFP_KERNEL);
	if (chain == NULL)
		return NULL;

	chain->link = (struct mpc85xx_dma_desc *)
			dma_alloc_coherent(NULL,
			    sizeof (struct mpc85xx_dma_desc) * links,
			    (dma_addr_t *) &chain-> link_pa,
			    GFP_DMA);
	if (chain->link == NULL) {
		printk(KERN_ERR "mpc85xx_dma: Unable to obtain dma region\n");
		return NULL;
	}

	chain->max = links;
	chain->len = 0;

	return chain;
}

EXPORT_SYMBOL(mpc85xx_dma_chain_clear);
void mpc85xx_dma_chain_clear(struct mpc85xx_dma_chain *chain)
{
	if (chain == NULL)
		return;

	chain->len = 0;
}

EXPORT_SYMBOL(mpc85xx_dma_chain_free);
void mpc85xx_dma_chain_free(struct mpc85xx_dma_chain *chain)
{
	if (chain == NULL)
		return;

	dma_free_coherent(NULL,
			  sizeof(struct mpc85xx_dma_desc) * chain->max,
			  (void *)chain->link, (dma_addr_t) chain->link_pa);
	kfree(chain);
}

EXPORT_SYMBOL(mpc85xx_dma_chain_append);
int mpc85xx_dma_chain_append(struct mpc85xx_dma_chain *chain,
			     phys_addr_t dst_addr,
			     phys_addr_t src_addr, size_t len_in_bytes)
{
	struct mpc85xx_dma_desc *desc;

	if (chain == NULL)
		return -EINVAL;

	if (len_in_bytes & ~MPC85xx_DMA_BYTE_MASK)
		return -EINVAL;

	if (chain->len >= chain->max) {
		printk(KERN_WARNING
		       "mpc85xx_dma_chain_append: Can't append to full chain %p\n",
		       chain);
		return -ENOMEM;
	}

	desc = &chain->link[chain->len];
	memset(desc, 0, sizeof(*desc));
	desc->bcr = len_in_bytes;
	if (src_addr > 0xf0000000UL) {
		desc->satr = MPC85xx_SATR_SREADTYPE_L_NO_SNOOP;	/* Read, snoop local proc */
	} else {
		desc->satr = MPC85xx_SATR_SREADTYPE_L_SNOOP;	/* Read, snoop local proc */
	}
	if (dst_addr > 0xf0000000UL) {
		desc->datr = MPC85xx_DATR_DWRITETYPE_L_NO_SNOOP;	/* Write, snoop local proc */
	} else {
		desc->datr = MPC85xx_DATR_DWRITETYPE_L_SNOOP;	/* Write, snoop local proc */
	}
	desc->sar = src_addr;
	desc->dar = dst_addr;
	desc->nlndar = MPC85xx_NLNDAR_EOLND | MPC85xx_NLNDAR_NDEOSIE;
	if (chain->len > 0) {
		chain->link[chain->len - 1].nlndar =
		    chain->link_pa +
		    ((chain->len) * (sizeof(struct mpc85xx_dma_desc)));
	}

	chain->len++;

	return 0;
}

EXPORT_SYMBOL(mpc85xx_dma_chain_ring);
int mpc85xx_dma_chain_ring(struct mpc85xx_dma_chain *chain)
{
	if (chain == NULL)
		return -EINVAL;

	if (chain->len == 0)
		return -EINVAL;

	chain->link[chain->len - 1].nlndar = chain->link_pa;

	return 0;
}

EXPORT_SYMBOL(mpc85xx_dma_request);
int mpc85xx_dma_request(int channel, const char *name, mpc85xx_dma_f func,
			void *data)
{
	if (channel < 0 || channel >= 4)
		return -EINVAL;

	if (data == NULL)
		return -EINVAL;

	down(&mpc85xx_dma_lock);

	if (mpc85xx_dma_info[channel].data != NULL) {
		up(&mpc85xx_dma_lock);
		return -EBUSY;
	}

	/* We use the 'wmb()' so we don't have to use
	 * spin_lock_irqsave/restore here, as the IRQ
	 * only checks .func
	 */
	mpc85xx_dma_info[channel].chain = NULL;
	mpc85xx_dma_info[channel].data = data;
	wmb();
	mpc85xx_dma_info[channel].func = func;
	wmb();

	up(&mpc85xx_dma_lock);

	return 0;
}

EXPORT_SYMBOL(mpc85xx_dma_free);
void mpc85xx_dma_free(int channel, void *data)
{
	down(&mpc85xx_dma_lock);

	if (mpc85xx_dma_info[channel].data != data) {
		printk(KERN_WARNING
		       "mpc85xx_dma_free: Channel %d not in use, or already freed\n",
		       channel);
		goto done;
	}

	/* We use the 'wmb()' so we don't have to use
	 * spin_lock_irqsave/restore here, as the IRQ
	 * only checks .func
	 */
	mpc85xx_dma_info[channel].func = NULL;
	wmb();
	mpc85xx_dma_info[channel].data = NULL;

	kfree(mpc85xx_dma_info[channel].chain);
	mpc85xx_dma_info[channel].chain = NULL;

done:
	up(&mpc85xx_dma_lock);
}

char *dma_names[4] = {
	"DMA Channel 0",
	"DMA Channel 1",
	"DMA Channel 2",
	"DMA Channel 3"
};

#ifdef CONFIG_PPC_OF

static int dma0_virq;

static int mpc85xx_dma_probe(struct platform_device *pdev)
{
	struct resource *r;
	int err = 0;
	int i;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	dma_addr = ioremap(r->start, r->end - r->start);
	if (!dma_addr) {
		printk(KERN_ERR "mpc85xx_dma: Unable to ioremap\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, dma_addr);
	init_MUTEX(&mpc85xx_dma_lock);

	for (i = 0; i < 4; i++) {
		DMA_W(i, mr, 0);
	}

	dma0_virq = platform_get_irq_byname(pdev, "mpc85xx_dma0_intr");
	for (i = 0; i < 4; i++) {
		err =
		    request_irq(dma0_virq + i, mpc85xx_dma_irq, IRQF_DISABLED,
				dma_names[i], dma_names[i]);
		if (err < 0) {
			iounmap(dma_addr);
			for (i--; i >= 0; i--)
				free_irq(dma0_virq + i, dma_names[i]);
			return err;
		}
	}
	printk(KERN_INFO "MPC85xx DMA Engine (4 channels available)\n");

	return 0;
}

static int mpc85xx_dma_remove(struct platform_device *pdev)
{
	int i;

	platform_set_drvdata(pdev, NULL);
	iounmap(dma_addr);
	for (i = 0; i < 4; i++)
		free_irq(dma0_virq + i, dma_names[i]);

	return 0;
}

static struct platform_driver mpc85xx_dma_driver = {
	.probe = mpc85xx_dma_probe,
	.remove = mpc85xx_dma_remove,
	.driver = {
		.name = "mpc85xx_dma",
	},
};

static int __init mpc85xx_dma_init(void)
{
	return platform_driver_register(&mpc85xx_dma_driver);
}


static void __exit mpc85xx_dma_exit(void)
{
	platform_driver_unregister(&mpc85xx_dma_driver);
}

#else

static int __init mpc85xx_dma_init(void)
{
	int i, err;
	bd_t *binfo = (bd_t *) __res;

	dma_addr = ioremap(binfo->bi_immr_base + 0x21000 + 256, 0x204);

	if (!dma_addr) {
		printk("mpc85xx_dma: Unable to ioremap\n");
		return -ENODEV;
	}
	init_MUTEX(&mpc85xx_dma_lock);

	for (i = 0; i < 4; i++) {
		DMA_W(i, mr, 0);
	}

	for (i = 0; i < 4; i++) {
		err = request_irq(MPC85xx_IRQ_DMA0 + i, mpc85xx_dma_irq, 0,
			dma_names[i], dma_names[i]);
		if (err < 0) {
			for (i--; i >= 0; i--)
				free_irq(MPC85xx_IRQ_DMA0 + i, dma_names[i]);
			return err;
		}
	}

	printk(KERN_INFO "MPC85xx DMA Engine (4 channels available)\n");

	return 0;
}

static void __exit mpc85xx_dma_exit(void)
{
	int i;

	iounmap(dma_addr);
	for (i = 0; i < 4; i++)
		free_irq(MPC85xx_IRQ_DMA0 + i, dma_names[i]);
}

#endif

module_init(mpc85xx_dma_init);
module_exit(mpc85xx_dma_exit);
