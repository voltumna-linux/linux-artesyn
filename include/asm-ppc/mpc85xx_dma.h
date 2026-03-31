#ifndef __MPC85xx_DMA_H
#define __MPC85xx_DMA_H

#include <asm/immap_85xx.h>
#include <asm/mpc85xx.h>

/* Mode Registers */

#define MPC85xx_MR_EMP_EN	0x00200000
#define MPC85xx_MR_EMS_EN	0x00040000
#define MPC85xx_MR_SAHTS_1	0x00000000
#define MPC85xx_MR_SAHTS_2	0x00004000
#define MPC85xx_MR_SAHTS_4	0x00008000
#define MPC85xx_MR_SAHTS_8	0x0000C000
#define MPC85xx_MR_DAHE		0x00002000
#define MPC85xx_MR_SAHE		0x00001000
#define MPC85xx_MR_SRW		0x00000400
#define MPC85xx_MR_EOSIE	0x00000200
#define MPC85xx_MR_EOLNIE	0x00000100
#define MPC85xx_MR_EOLSIE	0x00000080
#define MPC85xx_MR_EIE		0x00000040
#define MPC85xx_MR_XFE		0x00000020
#define MPC85xx_MR_CDSM		0x00000010
#define MPC85xx_MR_SWSM		0x00000010
#define MPC85xx_MR_CA		0x00000008
#define MPC85xx_MR_CTM		0x00000004
#define MPC85xx_MR_CC		0x00000002
#define MPC85xx_MR_CS		0x00000001

/* Status Register */

#define MPC85xx_SR_TE		0x00000080
#define MPC85xx_SR_CH		0x00000020
#define MPC85xx_SR_PE		0x00000010
#define MPC85xx_SR_EOLNI	0x00000008
#define MPC85xx_SR_CB		0x00000004
#define MPC85xx_SR_CB		0x00000004
#define MPC85xx_SR_EOSI		0x00000002
#define MPC85xx_SR_EOLSI	0x00000001


/* Source Attribute Registers */

#define MPC85xx_SATR_SREADTYPE_L_NO_SNOOP	0x00040000
#define MPC85xx_SATR_SREADTYPE_L_SNOOP		0x00050000


/* Destination Attribute Registers */

#define MPC85xx_DATR_DWRITETYPE_L_NO_SNOOP	0x00040000
#define MPC85xx_DATR_DWRITETYPE_L_SNOOP		0x00050000

/* Next Link Descriptor Address Registers */

#define MPC85xx_NLNDAR_NDEOSIE	0x00000008
#define MPC85xx_NLNDAR_EOLND	0x00000001

#define MPC85xx_BCR_MASK	0x03ffffff

/* Physical layout of DMA chain descriptors */
struct mpc85xx_dma_desc {
	uint32_t satr;		/* Source attributes */
	uint32_t sar;		/* Source physical address */
	uint32_t datr;		/* Dest attributes */
	uint32_t dar;		/* Dest physical address */
	uint32_t res_1;
	uint32_t nlndar;	/* Next link physical address */
	uint32_t bcr;		/* Byte count */
	uint32_t res_2;
};

#define MPC85xx_DMA_EVENT_DONE		(1 << 0)
#define MPC85xx_DMA_EVENT_HALTED	(1 << 1)
#define MPC85xx_DMA_EVENT_ERROR		(1 << 2)

#define MPC85xx_DMA_CONTROL_DST_HOLD	MPC85xx_MR_DAHE
#define MPC85xx_DMA_CONTROL_DST_1	MPC85xx_MR_DAHTS_1
#define MPC85xx_DMA_CONTROL_DST_2	MPC85xx_MR_DAHTS_2
#define MPC85xx_DMA_CONTROL_DST_4	MPC85xx_MR_DAHTS_4
#define MPC85xx_DMA_CONTROL_DST_8	MPC85xx_MR_DAHTS_8

#define MPC85xx_DMA_CONTROL_SRC_HOLD	MPC85xx_MR_SAHE
#define MPC85xx_DMA_CONTROL_SRC_1	MPC85xx_MR_SAHTS_1
#define MPC85xx_DMA_CONTROL_SRC_2	MPC85xx_MR_SAHTS_2
#define MPC85xx_DMA_CONTROL_SRC_4	MPC85xx_MR_SAHTS_4
#define MPC85xx_DMA_CONTROL_SRC_8	MPC85xx_MR_SAHTS_8

#define MPC85xx_DMA_BYTE_MASK		MPC85xx_BCR_MASK
#define MPC85xx_DMA_CONTROL_MASTER_START	MPC85xx_MR_EMS_EN
#define MPC85xx_DMA_CONTROL_MASTER_PAUSE	MPC85xx_MR_EMP_EN

#define MPC85xx_DMA_CONTROL_PAUSE_2	MPC85xx_MR_BWC_2
#define MPC85xx_DMA_CONTROL_PAUSE_4	MPC85xx_MR_BWC_4
#define MPC85xx_DMA_CONTROL_PAUSE_8	MPC85xx_MR_BWC_8
#define MPC85xx_DMA_CONTROL_PAUSE_16	MPC85xx_MR_BWC_16
#define MPC85xx_DMA_CONTROL_PAUSE_32	MPC85xx_MR_BWC_32
#define MPC85xx_DMA_CONTROL_PAUSE_64	MPC85xx_MR_BWC_64
#define MPC85xx_DMA_CONTROL_PAUSE_128	MPC85xx_MR_BWC_128
#define MPC85xx_DMA_CONTROL_PAUSE_256	MPC85xx_MR_BWC_256
#define MPC85xx_DMA_CONTROL_PAUSE_512	MPC85xx_MR_BWC_512

/** DMA Engine **/

/* 
 * 'event' is a mask of MPC85xx_DMA_EVENT_*
 */
typedef void (*mpc85xx_dma_f)(int channel, int event_mask, void *data);

/* 
 * You can request a DMA channel (0-3). Your handler will be called
 * on all events that happen on the channel.  If another
 * routine is already registered for the requested channel, this
 * call will fail. Only if you succeed in this call may you
 * call any other DMA routines!
 */
int mpc85xx_dma_request(int channel, const char *name, mpc85xx_dma_f func, void *data);
void mpc85xx_dma_free(int channel, void *data);

/* 
 * Runs a single-shot DMA transfer.
 * 'flags' is options to MPC85xx_DMA_CONTROL
 */
int mpc85xx_dma_xfer(int channel, int flags, phys_addr_t dst, phys_addr_t src, size_t bytes);

/* 
 * Stops the current DMA transfer immediately.
 */
void mpc85xx_dma_abort(int channel);

/* 
 * Halts the DMA unit after the next successful transfer.
 */
void mpc85xx_dma_halt(int channel);

/* 
 * Indicates if the DMA channel is busy
 */
int mpc85xx_dma_busy(int channel);


/************** DMA chains **************/
struct mpc85xx_dma_chain;

/* 
 * Allocate a DMA chain
 */
struct mpc85xx_dma_chain *mpc85xx_dma_chain_alloc(int links);

/* 
 * Erase all blocks in the DMA chain.
 * DO NOT USE if the chain is DMAing.
 */
void mpc85xx_dma_chain_clear(struct mpc85xx_dma_chain *chain);

/* 
 * Free a DMA chain
 * DO NOT USE if the chain is DMAing.
 */
void mpc85xx_dma_chain_free(struct mpc85xx_dma_chain *chain);

/*
 * Will return -ENOMEM if you're out of chain slots.
 * DO NOT USE if the chain is DMAing.
 */
int mpc85xx_dma_chain_append(struct mpc85xx_dma_chain *chain,
			phys_addr_t dst_addr,
			phys_addr_t src_addr,
			size_t len_in_bytes);

/* 
 * Makes a ring out of a chain.
 * Append can still be called after this function
 * (makes the ring larger).
 * DO NOT USE if the chain is DMAing.
 */
int mpc85xx_dma_chain_ring(struct mpc85xx_dma_chain *chain);

/*
 * 'flags' is options to MPC85xx_DMA_CONTROL
 * (see include/asm-ppc/mpc85xx_defs.h)
 */
int mpc85xx_dma_chain_xfer(int channel, int flags, struct mpc85xx_dma_chain *chain);

#endif /* __MPC85xx_DMA_H */
