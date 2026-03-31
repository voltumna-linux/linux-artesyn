/*
 * mv64360_dma.c: Linux driver for the Marvel MV64360 IDMA controller.
 *
 * Author:  Ajit Prem, (Ajit.Prem@motorola.com).
 * Copyright 2003-2007  Motorola
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

/* 

The MV64360 DMA driver provides support to initialize, configure, 
start, pause, resume, abort and retrieve status  of DMA operations 
on a per channel basis. Only block mode is supported. 

The arbiter setup routine may be invoked if a custom channel priority
weighting is desired. It is not necessary to program the DMA channel 
arbiter unless something other than the default weighting is needed. The
default is balanced equally across all channels.

To start DMA for a particular channel, the user-space app has to:
-- open the appropriate dma channel (/dev/mv64360_dma_x where 0<=x<=3)

-- Setup the dma parameters and invoke ioctl(MV64360_DMA_SETUP). 
   The DMA parameter setup involves providing either the source or 
   destination address (one of which is expected to be a device address), 
   the byte account, and other transfer attributes. On the call, the 
   driver will allocate a kernel buffer for either the source or 
   destination address (based on the transfer attributes) and allow the 
   user-space application to mmap() the buffer. 

   For SRAM to device DMA, the source buffer is a kernel allocated buffer 
   that the user-app can mmap(), populate, and then issue an 
   ioctl(MV64360_DMA_START) to have the driver start the DMA. 
 
   For device to SRAM DMA, the destination buffer is a kernel allocated 
   buffer that the user-app can mmap().

-- Start DMA with ioctl(MV64360_DMA_START)

-- A read() has to be issued on the device to wait for DMA completion 
   (alternatively, select() can also be used).

-- Status can be checked with ioctl(MV64360_DMA_STATUS)

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mv643xx.h>
#include <asm/io.h>
#include <asm/mv64x60_defs.h>

#ifdef CONFIG_BIGPHYS_AREA
#include <linux/bigphysarea.h>
#endif

#include "mv64360_dma.h"
#include "mv64360_dma_interface.h"

#define DRV_NAME "mv64360_dma"
#define DRV_VERSION "2.0.1"

#define ALL_MSG "mv64360_dma: "

#ifdef DMA_DEBUG
int dma_debug = DMA_DEBUG;
MODULE_PARM(dma_debug, "i");

#define DEBUG(n, fmt, args...) if (dma_debug>(n)) printk(KERN_INFO ALL_MSG fmt, ## args)
#else
#define DEBUG(n, fmt, args...)
#endif

#define IDMA_REG_SIZE sizeof(u32)
#define IDMA_CHAN_IS_ACTIVE(_X_) (((_X_) & IDMA_CNTL_CHAN_ACTIVE_MASK) == \
  IDMA_CNTL_CHAN_ACTIVE_MASK)
#define MV_REG_WRITE_PUSH(offset,data)					 \
{									\
*(volatile unsigned int *) (mv64360_base + offset) = cpu_to_le32(data); \
le32_to_cpu(*(volatile unsigned int *) (mv64360_base + offset));	\
}

/*
 * The following structure defines the control elements for a given DMA
 * channel. The register address offsets are used to conveniently access a
 * particular channel's DMA descriptor and control registers.
 */

struct mv64360_dma_cntl_t {

	/* channel number */
	u32 chan;

	/* offset of DMA byte count register */
	u32 byte_count_reg_offset;

	/* offset of DMA source address register */
	u32 source_addr_reg_offset;

	/* offset of DMA destination address register */
	u32 dest_reg_offset;

	/* offset of the next desc ptr register */
	u32 next_desc_ptr_reg_offset;

	/* offset of the current desc ptr register */
	u32 curr_desc_ptr_reg_offset;

	/* offset of the channel control register */
	u32 chan_cntl_low_reg_offset;

	/* offset of the channel control register */
	u32 chan_cntl_high_reg_offset;

	/* offset of window access control register */
	u32 chan_access_prot_offset;

	/* DMA completion status */
	int dma_complete;

	/* DMA error select code */
	u32 error_select;

	/* address causing the error */
	u32 error_addr;

	/* address of DMA buffer in kernel space */
	char *dma_src_buffer;

	/* address handed to device */
	dma_addr_t dma_src_handle;

	/* address of DMA destination buffer */
	char *dma_dest_buffer;

	/* address handed to device */
	dma_addr_t dma_dest_handle;

	/* size of DMA buffer in pages */
	int dma_buffer_npages;

	/* size of DMA buffer in bytes */
	size_t dma_buffer_size;

	/* byte count passed in by user. The computed dma_buffer_size may 
	   be greater as it is a multiple of the PAGE_SIZE */
	size_t byte_count;

	/* attributes for current transfer */
	struct dma_attributes_t dma_attributes;

	/* semaphore used to serialize dma requests on the channel */
	struct semaphore dma_available;

	/* used to signal that a started DMA operation is complete */
	wait_queue_head_t readywait;

	/* spin lock for synchronizing with ISR */
	spinlock_t dma_lock;

	/* A timer whose function will abort a started DMA transaction 
	   if it hasn't completed by a specific time */
	struct timer_list dma_device_timer;

	/* Indicates the dma_device_timer has been started */
	int dma_timer_pending;
};

#ifdef CONFIG_MV64X60_NEW_BASE
static unsigned int mv64360_reg_base = CONFIG_MV64X60_NEW_BASE;
#else
static unsigned int mv64360_reg_base = 0xf1000000;
#endif

static unsigned int mv64360_base;

static struct mv64360_dma_cntl_t mv64360_dma_cntl[IDMA_CHANNEL_COUNT];
static int dma_device_open[IDMA_CHANNEL_COUNT];

static int is_initialized;

static int major_dev;
static struct class *mv64360_dma_class;

static DECLARE_WAIT_QUEUE_HEAD(dma_wait);

static ssize_t mv64360_dma_read(struct file *file, char *buf,
				size_t count, loff_t * ppos);
static ssize_t mv64360_dma_write(struct file *file, const char *buf,
				 size_t count, loff_t * ppos);
static unsigned int mv64360_dma_poll(struct file *file,
				     struct poll_table_struct *poll_table);
static int mv64360_dma_ioctl(struct inode *inode, struct file *file,
			     unsigned int cmd, unsigned long arg);
static int mv64360_dma_mmap(struct file *file, struct vm_area_struct *vma);
static int mv64360_dma_open(struct inode *inode, struct file *file);
static int mv64360_dma_release(struct inode *inode, struct file *file);

static int mv64360_dma_arbiter_setup(unsigned int *arb_slice);
static int mv64360_dma_abort(struct mv64360_dma_cntl_t *dma_chan_cntl);
static int mv64360_dma_pause(struct mv64360_dma_cntl_t *dma_chan_cntl);
static int mv64360_dma_resume(struct mv64360_dma_cntl_t *dma_chan_cntl);

#define MV_REG_READ(offset) le32_to_cpu(* (volatile unsigned int *) (mv64360_base + offset))

/******************************************************************************
*
* mv64360_init_addr_windows - Initialize address decoding windows.
*
* This routine initializes the DMA address decoding windows to 
* cover the system address map. DRAM addresses, PCI I/O and PCI memory 
* addresses on both PCI buses are mapped for DMA access. 
* A decoding window is configured for each DRAM bank that is enabled.
* All DRAM windows are configured for write back (WB) coherency.
* DRAM bank base addresses and sizes are obtained from the
* corresponding CPU interface registers, then programmed into
* DMA base address and size registers.  PCI windows are configured for
* non-byte swapping transfers. All configured windows are enabled upon return.
*
* Note that base addresses and sizes are read from previously configured
* CPU IF registers. The base address values from these registers are 20 bits
* wide. The upper four address bits are ignored. It is not expected to have
* any addresses high enough to use these ignored bits. Only the next (upper) 
* 16 address bits are used since all DMA windows must be 64K aligned.
*
* RETURNS: N/A
*/

static void mv64360_init_addr_windows(void)
{
	u32 dma_base_enable = 0;
	u32 base_addr_enable = 0;
	u32 base_addr = 0;
	u32 size = 0;
	u32 base_addr_val = 0;
	base_addr_enable = MV_REG_READ(MV64360_CPU_BAR_ENABLE);

	/* Configure window for DRAM CS0 */
	if (!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_CS_0_MASK)) {
		base_addr =
		    MV_REG_READ(MV64x60_CPU2MEM_0_BASE) <<
		    IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size =
		    MV_REG_READ(MV64x60_CPU2MEM_0_SIZE) << IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
		    IDMA_BASE_ADDR_TARGET_DRAM | IDMA_BASE_ADDR_DRAM_BANK_CS0 |
		    IDMA_BASE_ADDR_COHERENCY_WB | (base_addr &
						   IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_0_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_0_SIZE,
				  size & IDMA_WINDOW_SIZE_MASK);

		/* Enable Window 0 */
		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
				  ~IDMA_BASE_ADDR_ENABLE_WIN0 &
				  dma_base_enable);
	}

	/* Configure window for DRAM CS1 */
	if (!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_CS_1_MASK)) {
		base_addr =
		    MV_REG_READ(MV64x60_CPU2MEM_1_BASE) <<
		    IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size =
		    MV_REG_READ(MV64x60_CPU2MEM_1_SIZE) << IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
		    IDMA_BASE_ADDR_TARGET_DRAM | IDMA_BASE_ADDR_DRAM_BANK_CS1 |
		    IDMA_BASE_ADDR_COHERENCY_WB | (base_addr &
						   IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_1_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_1_SIZE,
				  size & IDMA_WINDOW_SIZE_MASK);

		/* Enable Window 1 */
		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
				  ~IDMA_BASE_ADDR_ENABLE_WIN1 &
				  dma_base_enable);
	}

	/* Configure window for DRAM CS2 */
	if (!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_CS_2_MASK)) {
		base_addr =
		    MV_REG_READ(MV64x60_CPU2MEM_2_BASE) <<
		    IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size =
		    MV_REG_READ(MV64x60_CPU2MEM_2_SIZE) << IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
		    IDMA_BASE_ADDR_TARGET_DRAM | IDMA_BASE_ADDR_DRAM_BANK_CS2 |
		    IDMA_BASE_ADDR_COHERENCY_WB | (base_addr &
						   IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_2_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_2_SIZE,
				  size & IDMA_WINDOW_SIZE_MASK);

		/* Enable Window 2 */
		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
				  ~IDMA_BASE_ADDR_ENABLE_WIN2 &
				  dma_base_enable);
	}

	/* Configure window for DRAM CS3 */
	if (!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_CS_3_MASK)) {
		base_addr =
		    MV_REG_READ(MV64x60_CPU2MEM_3_BASE) <<
		    IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size =
		    MV_REG_READ(MV64x60_CPU2MEM_3_SIZE) << IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
		    IDMA_BASE_ADDR_TARGET_DRAM | IDMA_BASE_ADDR_DRAM_BANK_CS3 |
		    IDMA_BASE_ADDR_COHERENCY_WB | (base_addr &
						   IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_3_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_3_SIZE,
				  size & IDMA_WINDOW_SIZE_MASK);

		/* Enable Window 3 */
		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
				  ~IDMA_BASE_ADDR_ENABLE_WIN3 &
				  dma_base_enable);
	}

	/* Configure window for PCI Bus 0 I/O */
	if (!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_PCI0_IO_MASK)) {
		base_addr = MV_REG_READ(MV64x60_CPU2PCI0_IO_BASE) << 
			IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size = MV_REG_READ(MV64x60_CPU2PCI0_IO_SIZE) << 
			IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
			IDMA_BASE_ADDR_TARGET_PCI0 | 
			IDMA_BASE_ADDR_PCI_NO_SWAP |
			IDMA_BASE_ADDR_PCI_REQ64_SIZE | 
			(base_addr & IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_4_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_4_SIZE,
			  size & IDMA_WINDOW_SIZE_MASK);
		/* Enable Window 4 */
		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
			~IDMA_BASE_ADDR_ENABLE_WIN4 & dma_base_enable);
	}

	/* Configure window for PCI Bus 1 I/O */
	if (!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_PCI1_IO_MASK)) {
		base_addr = MV_REG_READ(MV64x60_CPU2PCI1_IO_BASE) <<
			IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size = MV_REG_READ(MV64x60_CPU2PCI1_IO_SIZE) << 
			IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
			IDMA_BASE_ADDR_TARGET_PCI1 | 
			IDMA_BASE_ADDR_PCI_NO_SWAP |
			IDMA_BASE_ADDR_PCI_REQ64_SIZE | 
			(base_addr & IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_5_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_5_SIZE,
			size & IDMA_WINDOW_SIZE_MASK);
		/* Enable Window 5 */
		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
			~IDMA_BASE_ADDR_ENABLE_WIN5 & dma_base_enable);
	}

	/* Configure window for PCI Bus 0 memory */
	if (!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_PCI0_MEM0_MASK)) {
		base_addr = MV_REG_READ(MV64x60_CPU2PCI0_MEM_0_BASE) <<
			IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size = MV_REG_READ(MV64x60_CPU2PCI0_MEM_0_SIZE) << 
			IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
			IDMA_BASE_ADDR_TARGET_PCI0 | 
			IDMA_BASE_ADDR_PCI_NO_SWAP |
			IDMA_BASE_ADDR_PCI_MEM_SEL | 
			IDMA_BASE_ADDR_PCI_REQ64_SIZE |
			(base_addr & IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_6_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_6_SIZE,
			  size & IDMA_WINDOW_SIZE_MASK);
		/* Enable Window 6 */
		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
			~IDMA_BASE_ADDR_ENABLE_WIN6 & dma_base_enable);
	}

	/* Configure window for PCI Bus 1 memory */
	if (!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_PCI1_MEM0_MASK)) {
		base_addr = MV_REG_READ(MV64x60_CPU2PCI1_MEM_0_BASE) <<
			IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size = MV_REG_READ(MV64x60_CPU2PCI1_MEM_0_SIZE) << 
			IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
			IDMA_BASE_ADDR_TARGET_PCI0 | 
			IDMA_BASE_ADDR_PCI_NO_SWAP |
			IDMA_BASE_ADDR_PCI_MEM_SEL | 
			IDMA_BASE_ADDR_PCI_REQ64_SIZE |
			(base_addr & IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_7_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_7_SIZE,
			size & IDMA_WINDOW_SIZE_MASK);
		/* Enable Window 7 */
		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
			~IDMA_BASE_ADDR_ENABLE_WIN7 & dma_base_enable);
	}

	/* 
	 * If a DMA address decoding window is available (one will be 
         * available if not all DRAM CS banks are enabled), then 
         * configure a window for Integrated SRAM (provided the CPU interface 
         *  has enabled Integrated SRAM).
	 */

	dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
	if ((dma_base_enable & IDMA_BASE_ADDR_ENABLE_WIN3) &&  
		!(base_addr_enable & MV64360_BASE_ADDR_ENABLE_SRAM_MASK)) {
		base_addr = MV_REG_READ(MV64360_CPU2SRAM_BASE) <<
	    		IDMA_CPUIF_ADDR_SHIFT_INDEX;
		size = MV64360_SRAM_SIZE << IDMA_WINDOW_SIZE_BIT;
		base_addr_val =
		    	IDMA_BASE_ADDR_TARGET_SRAM | 
			(base_addr & IDMA_BASE_ADDR_BASE_MASK);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_3_BASE, base_addr_val);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_3_SIZE,
			  size & IDMA_WINDOW_SIZE_MASK);

		dma_base_enable = MV_REG_READ(MV64360_IDMA2MEM_BAR_ENABLE);
		MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
			~IDMA_BASE_ADDR_ENABLE_WIN3 & dma_base_enable);
	}
}

/**************************************************************************
*
* mv64360_dma_int_handler - Handle a normal completion interrupt.
*
* This function handles a normal DMA completion interrupt for a particular
* channel. The interrupt is verified to have occurred and the channel
* status set to OK to indicate successful completion. 
*
* The interrupt is cleared by writing a zero to the appropriate channel 
* completion bit in the interrupt cause register. Other channel's bits are 
* written as ones to preserve their state (writing a one has no effect).
*
* RETURNS: N/A
*/

static irqreturn_t
mv64360_dma_int_handler(int irq, void *dev_id)
{
	struct mv64360_dma_cntl_t *dma_chan_cntl = dev_id;
	u32 int_cause = 0xFFFFFFFF;
	u32 completion_mask = 0;
	u32 error_mask = 0;
	u32 error_code = 0;
	u32 chan;
	u32 temp;

	chan = dma_chan_cntl->chan;
	DEBUG(1, "interrupt cntl 0x%p channel %d\n", dma_chan_cntl, chan);
	completion_mask = IDMA_INT_CAUSE_DMA_COMPL_MASK <<
	    (IDMA_INT_CAUSE_CHAN1_BIT * chan);
	error_mask = IDMA_INT_CAUSE_CHAN_ERR_MASK <<
	    (IDMA_INT_CAUSE_CHAN1_BIT * chan);
	temp = MV_REG_READ(MV64x60_IDMA_0_3_INTR_CAUSE);
	error_code = temp & error_mask;

	DEBUG(1,
	      "completion mask 0x%x error_mask 0x%x cause 0x%x error_code 0x%x\n",
	      completion_mask, error_mask, temp, error_code);
	/* Check channel to be sure interrupt actually occurred */
	if ((MV_REG_READ(MV64x60_IDMA_0_3_INTR_CAUSE) & completion_mask) != 0) {
		if (error_code == 0) {
			dma_chan_cntl->dma_complete = 0;
		} else {
			dma_chan_cntl->dma_complete = 1;
		}

		/*
		 * Write out cleared interrupt field to clear the completion 
		 * interrupt for the given channel only.
		 */
		int_cause &= ~completion_mask;
		MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_INTR_CAUSE, int_cause);
	}
	wake_up(&dma_chan_cntl->readywait);
	return IRQ_HANDLED;
}

/**************************************************************************
*
* mv64360_dma_err_int_handler - Handle an error interrupt.
*
* This function handles a DMA error interrupt. Since the interrupt is shared
* across all channels, all channels must be checked for an error status by
* examining the error bits in the interrupt cause register. If the error
* code is not zero, dma_complete is set to FALSE and the code is saved for 
* that channel. If the error select register matches the error code, the 
* error address is also saved for that channel. 
*
* After checking all channels, the interrupts are cleared by writing 
* zeros to the all of the interrupting channels' error bits in the 
* interrupt cause register. Other channel's bits are written as ones 
* to preserve their state (writing a one has no effect).
*
* RETURNS: N/A
*/

static irqreturn_t
mv64360_dma_err_int_handler(int irq, void *dev_id)
{
	u32 chan = 0;
	struct mv64360_dma_cntl_t *dma_chan_cntl = NULL;
	u32 error_mask = 0;
	u32 error_code = 0;
	u32 error_chan = 0;
	u32 error_cause = 0xFFFFFFFF;

	DEBUG(0, "dma_err_int:\n");
	/* Check each channel error interrupt group */
	for (chan = 0; chan < IDMA_CHANNEL_COUNT; ++chan) {
		dma_chan_cntl = &mv64360_dma_cntl[chan];
		error_mask = IDMA_INT_CAUSE_CHAN_MASK <<
		    (IDMA_INT_CAUSE_CHAN1_BIT * chan);
		error_code =
		    MV_REG_READ(MV64x60_IDMA_0_3_INTR_CAUSE) & error_mask;

		/*
		 * If the error code is not zero record the error status for the
		 * channel. Save the error address if one is available for the
		 * given error on the interrupting channel. Bits 3 & 4 of the 
		 * error select code contain the erroring channel number. This
		 * should match the channel in the interrupt cause register for
		 * the error address to be meaningful. 
		 */
		if (error_code != 0) {
			dma_chan_cntl->dma_complete = 2;
			dma_chan_cntl->error_select =
			    MV_REG_READ(MV64x60_IDMA_0_3_ERROR_SELECT);
			error_chan =
			    (dma_chan_cntl->
			     error_select & IDMA_ERROR_SEL_CHAN_MASK) >>
			    IDMA_ERROR_SEL_CHAN_BIT;
			if (chan == error_chan) {
				dma_chan_cntl->error_addr =
				    MV_REG_READ(MV64x60_IDMA_0_3_ERROR_ADDR);
			} else {
				dma_chan_cntl->error_addr = 0;
			}
			DEBUG(0,
			      "dma_err_int: chan 0x%x select 0x%x addr 0x%x\n",
			      error_chan, dma_chan_cntl->error_select,
			      dma_chan_cntl->error_addr);

			/* Clear the error interrupt field */
			error_cause &= ~error_mask;

			/* Write out cleared interrupt fields to clear the interrupts */
			MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_INTR_CAUSE, error_cause);
		}
	}
	return IRQ_HANDLED;
}

/******************************************************************************
*
* mv64360_dma_init - Initialize DMA and attach DMA Interrupt Handler.
*
* This function configures the DMA driver for all available DMA channels.
* The driver tracks information on each channel independently. Each channel
* control structure is set up to allow easy indexing by channel from the 
* driver routines. Register defaults are also initialized.
*
* Eight address decoding windows are also configured and enabled
* via this routine. Four are used to cover the DRAM addresses, one per bank.
* If a bank is not enabled, then no window is configured for it.
* Two are used for PCI I/O addresses on each PCI bus. Two are used for PCI 
* memory addresses on each PCI bus. An interrupt handler is connected to
* each DMA completion interrupt number. A different handler is connected
* to the DMA error interrupt. This routine must be called once for DMA 
* capability to be properly initialized before any other driver routine 
* is called. 
*
* RETURNS: 0 if initialization succeeded or non-zero if an error occurred.
*/

static int mv64360_dma_init(void)
{
	u32 chan = 0;
	struct mv64360_dma_cntl_t *dma_chan_cntl = NULL;
	u32 dma_cntl_val = 0;
	int status = 0;
	unsigned int arb_slice_default[] = {
		0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3
	};

	/* Set all unused registers to defaults */
	MV_REG_WRITE_PUSH(MV64360_DMA_HEADERS_RETARGET_CONTROL, 0x00);
	MV_REG_WRITE_PUSH(MV64360_DMA_HEADERS_RETARGET_BASE, 0x00);
	MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_XBAR_TO,
			  IDMA_CROSSBAR_TIMEOUT_PRESET |
			  IDMA_CROSSBAR_TIMEOUT_DISABLE);

	/* Ensure all interrupts are cleared and windows are disabled */
	MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_INTR_CAUSE, 0x00);
	MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_INTR_MASK, 0x00);
	MV_REG_WRITE_PUSH(MV64360_IDMA2MEM_BAR_ENABLE,
			  IDMA_BASE_ADDR_ENABLE_WIN_MASK);
	dma_cntl_val =
	    IDMA_CNTL_DBURST_LIMIT_32B | IDMA_CNTL_SBURST_LIMIT_32B |
	    IDMA_CNTL_BLOCK_MODE | IDMA_CNTL_NON_CHAIN_MODE |
	    IDMA_CNTL_END_OF_XFER_HALT | IDMA_CNTL_DESC_16MB;

	/* Initialize the DMA control array for all channels */
	while (chan < IDMA_CHANNEL_COUNT && status == 0) {
		dma_chan_cntl = &mv64360_dma_cntl[chan];
		dma_chan_cntl->chan = chan;
		dma_chan_cntl->byte_count_reg_offset =
		    MV64360_DMA_CHANNEL0_BYTE_COUNT + (chan * IDMA_REG_SIZE);
		dma_chan_cntl->source_addr_reg_offset =
		    MV64360_DMA_CHANNEL0_SOURCE_ADDR + (chan * IDMA_REG_SIZE);
		dma_chan_cntl->dest_reg_offset =
		    MV64360_DMA_CHANNEL0_DESTINATION_ADDR +
		    (chan * IDMA_REG_SIZE);
		dma_chan_cntl->next_desc_ptr_reg_offset =
		    MV64360_DMA_CHANNEL0_NEXT_DESCRIPTOR_POINTER +
		    (chan * IDMA_REG_SIZE);
		dma_chan_cntl->curr_desc_ptr_reg_offset =
		    MV64360_DMA_CHANNEL0_CURRENT_DESCRIPTOR_POINTER +
		    (chan * IDMA_REG_SIZE);
		dma_chan_cntl->chan_cntl_low_reg_offset =
		    MV64360_DMA_CHANNEL0_CONTROL + (chan * IDMA_REG_SIZE);
		dma_chan_cntl->chan_cntl_high_reg_offset =
		    MV64360_DMA_CHANNEL0_CONTROL_HIGH + (chan * IDMA_REG_SIZE);
		dma_chan_cntl->chan_access_prot_offset =
		    MV64360_DMA_CHANNEL0_ACCESS_PROTECTION_REG +
		    (chan * IDMA_REG_SIZE);

		/* Clear the error and status information */
		dma_chan_cntl->dma_complete = 0;
		dma_chan_cntl->error_select = 0;
		dma_chan_cntl->error_addr = 0;
		MV_REG_READ(MV64x60_IDMA_0_3_ERROR_ADDR);
		MV_REG_READ(MV64x60_IDMA_0_3_ERROR_SELECT);
		MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_ERROR_ADDR, 0);
		MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_ERROR_SELECT, 0);

		/* Initialize the fixed control settings and defaults */
		MV_REG_WRITE_PUSH(dma_chan_cntl->
				  chan_cntl_low_reg_offset, dma_cntl_val);
		MV_REG_WRITE_PUSH(dma_chan_cntl->chan_cntl_high_reg_offset,
				  0x00);
		MV_REG_WRITE_PUSH(dma_chan_cntl->chan_access_prot_offset,
				  IDMA_CHAN_ACCESS_FULL_WIN_ALL);
		MV_REG_WRITE_PUSH(dma_chan_cntl->byte_count_reg_offset, 0);
		MV_REG_WRITE_PUSH(dma_chan_cntl->source_addr_reg_offset, 0x00);
		MV_REG_WRITE_PUSH(dma_chan_cntl->dest_reg_offset, 0x00);
		MV_REG_WRITE_PUSH(dma_chan_cntl->curr_desc_ptr_reg_offset,
				  0x00);
		MV_REG_WRITE_PUSH(dma_chan_cntl->next_desc_ptr_reg_offset,
				  0x00);
		MV_REG_WRITE_PUSH(MV64360_DMA_HIGH_ADDR_REMAP_REG0 +
				  (chan * IDMA_REG_SIZE), 0x00);

		/* Connect the same int handler to each interrupt number */
		status =
		    request_irq(IDMA_CHAN0_COMPLETION_INT + chan,
				mv64360_dma_int_handler, IRQF_DISABLED,
				"MV64360 IDMA", &mv64360_dma_cntl[chan]);
		if (status != 0) {
			printk(KERN_ERR
			       "Cannot get irq for MV64360 DMA channel %d\n",
			       chan);
			return status;
		}
		++chan;
	}

	/*
	 * If status is okay, connect the DMA error interrupt and initialize
	 * the DMA base address decoding windows and arbiter.
	 */
	if (status == 0) {
		status =
		    request_irq(IDMA_ERROR_INT,
				mv64360_dma_err_int_handler, IRQF_DISABLED,
				"MV64360 IDMA error", NULL);
		if (status != 0) {
			printk(KERN_ERR
			       "Cannot get irq for MV64360_dma_err_int\n");
			return (status);
		}
		mv64360_init_addr_windows();
		status = mv64360_dma_arbiter_setup(arb_slice_default);
	}
	is_initialized = (status == 0);
	return (status);
}

/******************************************************************************
*
* mv64360_dma_start - Configure and start the DMA controller.
*
* This function sets up the DMA controller for a block mode DMA transfer.
* Then interrupts for the given channel are enabled and the transfer 
* is initiated. 
*
* The user has two options for being notified of the completion of a DMA
* transaction. One is to use a blocking read() on the device; the other is 
* to use select().
* After being notified of completion, an ioctl(MV64360_DMA_STATUS) can be 
* used to get additional status information.
*
* RETURNS: 
* 0 is returned if the channel started successfully.
* non-zero is returned if the driver is not initialized, or the 
* channel is invalid, or the dma_desc reference is null, or the
* channel is already active and busy.
*/

static int
mv64360_dma_start(struct mv64360_dma_cntl_t *dma_chan_cntl,
		  struct dma_descriptor_t *dma_desc)
{
	int chan = dma_chan_cntl->chan;
	u32 cntl_reg = 0;
	int status = 0;

	if (is_initialized && dma_desc != NULL) {
		down(&dma_chan_cntl->dma_available);

		/* Make sure channel is not active and byte count is legal */
		cntl_reg = MV_REG_READ(dma_chan_cntl->chan_cntl_low_reg_offset);
		if (!IDMA_CHAN_IS_ACTIVE(cntl_reg) &&
		    dma_chan_cntl->byte_count <=
		    IDMA_DESC_16MB_BYTE_COUNT_MASK) {

			/* Set the configuration options */
			cntl_reg &= ~IDMA_CNTL_HOLD_SRC_ADDR;
			cntl_reg &= ~IDMA_CNTL_HOLD_DEST_ADDR;

			if (dma_chan_cntl->dma_attributes.hold_source_addr) {
				cntl_reg |= IDMA_CNTL_HOLD_SRC_ADDR;
			}
			if (dma_chan_cntl->dma_attributes.hold_dest_addr) {
				cntl_reg |= IDMA_CNTL_HOLD_DEST_ADDR;
			}

			/* Set the descriptor registers */
			MV_REG_WRITE_PUSH(dma_chan_cntl->
					  byte_count_reg_offset,
					  dma_chan_cntl->byte_count &
					  IDMA_DESC_16MB_BYTE_COUNT_MASK);
			MV_REG_WRITE_PUSH(dma_chan_cntl->source_addr_reg_offset,
					  dma_desc->source_addr);
			MV_REG_WRITE_PUSH(dma_chan_cntl->dest_reg_offset,
					  dma_desc->dest_addr);
			MV_REG_WRITE_PUSH(dma_chan_cntl->
					  next_desc_ptr_reg_offset, 0);
			MV_REG_WRITE_PUSH(dma_chan_cntl->
					  curr_desc_ptr_reg_offset,
					  (u32) dma_desc);

			dma_chan_cntl->dma_complete = -1;
			dma_chan_cntl->error_select = 0;
			dma_chan_cntl->error_addr = 0;

			/* Unmask/enable all of the channel's interrupts */
			MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_INTR_MASK,
					  MV_REG_READ
					  (MV64x60_IDMA_0_3_INTR_MASK) |
					  (IDMA_INT_MASK_CHAN_MASK <<
					   (IDMA_INT_MASK_CHAN1_BIT * chan)));

			/* Start the transfer */
			cntl_reg |= IDMA_CNTL_CHAN_ENABLE;
			MV_REG_WRITE_PUSH(dma_chan_cntl->
					  chan_cntl_low_reg_offset, cntl_reg);
		} else {
			status = -1;
		}
		/* Release the device instance semaphore. */
		up(&dma_chan_cntl->dma_available);
	} else {
		status = -1;
	}
	return (status);
}

/******************************************************************************
*
* mv64360_dma_status - Read and return DMA status.
*
* For the given channel, provide the following DMA status information:
*
* The saved DMA Error Code Value;
* The saved DMA Error Address associated with the error code;
* DMA Current Source Address Register;
* DMA Current Destination Address Register;
* DMA Current Next Descriptor Register;
* DMA Channel Control Register;
*
* The status information is placed into a DMA status
* structure referenced by dma_status along with a DMA completion status
* summary of either OK or ERROR. 
*
* If status is obtained while DMA is in progress, the status
* summary will be ERROR with a zero error code and null error address.
* If DMA status is OK, the DMA transfer completed successfully. If status
* is ERROR with a non-zero error code, the transfer stopped due to the 
* indicated error. If available, the offending address for the error 
* is also returned.
*
* RETURNS: 
* OK is returned if the dma_status is valid.
* ERROR is returned if the driver is not initialized, or the 
* channel is invalid or the dma_status reference is null. The 
* dma_status contents will not be valid.
*/

static int
mv64360_dma_status(struct mv64360_dma_cntl_t *dma_chan_cntl,
		   struct dma_status_t *dma_status)
{
	int chan;
	int status = 0;

	chan = dma_chan_cntl->chan;
	if (is_initialized && chan < IDMA_CHANNEL_COUNT && dma_status != NULL) {
		dma_status->dma_complete = dma_chan_cntl->dma_complete;
		dma_status->dma_error_code = dma_chan_cntl->error_select;
		dma_status->dma_error_addr = dma_chan_cntl->error_addr;
		dma_status->cur_source_addr =
		    MV_REG_READ(dma_chan_cntl->source_addr_reg_offset);
		dma_status->cur_dest_addr =
		    MV_REG_READ(dma_chan_cntl->dest_reg_offset);
		dma_status->cur_next_desc =
		    MV_REG_READ(dma_chan_cntl->next_desc_ptr_reg_offset);
		dma_status->chan_cntl_low =
		    MV_REG_READ(dma_chan_cntl->chan_cntl_low_reg_offset);
	} else {
		status = -1;
	}
	return (status);
}

/******************************************************************************
*
* mv64360_dma_abort - Initiate an abort of the current DMA operation.
*
* For the given valid channel, if the channel is active, the channel's
* abort bit is set in its control register. The function waits for the
* abort to complete before returning.
*
* RETURNS:
* OK is returned if abort succeeded.
* ERROR is returned if the driver is not initialized or the
* channel number is invalid or should the abort fail.
*/
static int mv64360_dma_abort(struct mv64360_dma_cntl_t *dma_chan_cntl)
{
	int chan;
	u32 cntl_reg = 0;
	int status = 0;
	u32 ms_wait = 2;	/* HZ wait for abort time */

	chan = dma_chan_cntl->chan;
	DEBUG(1, "dma_abort: channel %d\n", chan);
	if (is_initialized && chan < IDMA_CHANNEL_COUNT) {
		down(&dma_chan_cntl->dma_available);
		cntl_reg = MV_REG_READ(dma_chan_cntl->chan_cntl_low_reg_offset);
		DEBUG(1, "dma_abort: cntl_reg 0x%x\n", cntl_reg);
		if ((cntl_reg & IDMA_CNTL_CHAN_ACTIVE_MASK) ==
		    IDMA_CNTL_CHAN_ACTIVE_MASK) {
			cntl_reg |= IDMA_CNTL_CHAN_ABORT;
			MV_REG_WRITE_PUSH(dma_chan_cntl->
					  chan_cntl_low_reg_offset, cntl_reg);

			/* Wait for abort to complete */
			while ((MV_REG_READ
				(dma_chan_cntl->chan_cntl_low_reg_offset) &
				IDMA_CNTL_CHAN_ABORT) && ms_wait > 0) {
				DEBUG(1, "dma_abort: ms_wait %d\n", ms_wait);
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(1);
				--ms_wait;
			}
			if (ms_wait == 0) {
				/*
				 * Channel became inactive prior to abort 
				 * command or abort could not complete.
				 */
				status = -1;
			}
		} else {
			/* Channel not active */
			status = -1;
		}
		up(&dma_chan_cntl->dma_available);
	} else {
		/* Driver not initialized or invalid parameter */
		status = -1;
	}
	DEBUG(1, "dma_abort: status %d\n", status);
	return (status);
}

/******************************************************************************
*
* mv64360_dma_pause - Initiate a pause of the current DMA operation.
*
* For the given valid channel, if the channel is busy, reset the activate
* bit in the channel's control register to pause the channel. Monitor the 
* channel active bit in the channel's control register until it resets. The
* channel will pause after completing the current transfer burst in progress.
*
* RETURNS: 
* OK is returned if pause succeeded.
* ERROR is returned if the driver is not initialized, or the channel
* is invalid, or the channel was not active.
*/
static int mv64360_dma_pause(struct mv64360_dma_cntl_t *dma_chan_cntl)
{
	u32 cntl_reg = 0;
	int status = 0;
	int chan;
	u32 ms_wait = 2;	/* HZ wait for abort time */

	chan = dma_chan_cntl->chan;
	DEBUG(1, "dma_pause: channel %d\n", chan);
	if (is_initialized && chan < IDMA_CHANNEL_COUNT) {
		down(&dma_chan_cntl->dma_available);
		cntl_reg = MV_REG_READ(dma_chan_cntl->chan_cntl_low_reg_offset);

		/* If the channel is active, pause the channel */
		if (IDMA_CHAN_IS_ACTIVE(cntl_reg)) {
			cntl_reg &= ~IDMA_CNTL_CHAN_ENABLE;
			MV_REG_WRITE_PUSH(dma_chan_cntl->
					  chan_cntl_low_reg_offset, cntl_reg);

			/* Wait for channel to pause */
			while (IDMA_CHAN_IS_ACTIVE
			       (MV_REG_READ(dma_chan_cntl->
					    chan_cntl_low_reg_offset))
			       && ms_wait > 0) {
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(1);
				--ms_wait;
			}
			if (ms_wait == 0) {
				/* Channel did not become inactive */
				status = -1;
			}
		} else {
			/* Channel already stopped */
			status = -1;
		}
		up(&dma_chan_cntl->dma_available);
	} else {
		/* Driver not initialized or invalid parameter */
		status = -1;
	}
	return (status);
}

/******************************************************************************
*
* mv64360_dma_resume - Resume a previously paused DMA operation.
*
* For a given valid channel, verify that the channel is not active and that
* there are bytes remaining to transfer (a non-zero byte count). If so, write
* the channel activate bit out to the channel's control register. The 
* channel will continue the transfer.
*
* RETURNS: 
* OK is returned if the resume succeeded.
* ERROR is returned if the driver is not initialized, or the channel
* is invalid, or the channel is not paused (i.e. inactive with a 
* a non-zero byte count descriptor register).
*/
static int mv64360_dma_resume(struct mv64360_dma_cntl_t *dma_chan_cntl)
{
	u32 cntl_reg = 0;
	u32 byte_count = 0;
	int chan;
	int status = 0;

	chan = dma_chan_cntl->chan;
	DEBUG(1, "dma_resume channel %d\n", chan);
	if (is_initialized && chan < IDMA_CHANNEL_COUNT) {
		down(&dma_chan_cntl->dma_available);
		cntl_reg = MV_REG_READ(dma_chan_cntl->chan_cntl_low_reg_offset);
		byte_count = MV_REG_READ(dma_chan_cntl->byte_count_reg_offset);

		/* If the channel is not active and bytes remain for transfer, 
		   resume the channel */
		if (!IDMA_CHAN_IS_ACTIVE(cntl_reg) && byte_count > 0) {
			cntl_reg |= IDMA_CNTL_CHAN_ENABLE;
			MV_REG_WRITE_PUSH(dma_chan_cntl->
					  chan_cntl_low_reg_offset, cntl_reg);
		} else {
			status = -1;
		}
		up(&dma_chan_cntl->dma_available);
	} else {
		status = -1;
	}
	return (status);
}

/******************************************************************************
*
* mv64360_dma_arbiter_setup - Custom program channel arbitration weighting.
*
* This function accepts an array of IDMA_ARBITER_SLICE_COUNT elements. Each
* element represents an arbiter time slice. The value of each element is a
* DMA channel number assigned to the slice. The default arbiter configuration
* provides equal weighting to all channels at reset. The default arbSlice
* array of channel numbers looks like this:
*
* arb_slice_default[] = { 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3 }
*
* Replacing channel numbers provides a custom weighting. Those channels
* that occupy more slices receive proportionately more running time when other
* channels are active simultaneously. A null array reference will have no 
* effect and results in an ERROR return status. If a channel number is not 
* a valid channel an ERROR status is returned with no action taken.
*
* RETURNS: 
* OK when arbiter has been successfully programmed.
* Error if arbSlice array is null or any channel is invalid. 
*/

static int mv64360_dma_arbiter_setup(unsigned int *arb_slice)
{
	u32 slice = 0;
	u32 arb_cntl = 0;
	int status = 0;

	if (arb_slice != NULL) {

		/* Validate the channel number in each entry slice */
		while (status == 0 && slice < IDMA_ARBITER_SLICE_COUNT) {
			if (arb_slice[slice] < IDMA_CHANNEL_COUNT) {
				++slice;
			} else {
				status = -1;
			}
		}

		/* Program the arbiter */
		if (status == 0) {
			for (slice = 0; slice < IDMA_ARBITER_SLICE_COUNT;
			     ++slice) {
				arb_cntl |= (arb_slice[slice] &
					     IDMA_ARBITER_SLICE_MASK) <<
				    (slice * IDMA_ARBITER_1_BIT);
			}
			MV_REG_WRITE_PUSH(MV64x60_IDMA_0_3_ARBITER_CNTL,
					  arb_cntl);
		}
	} else {
		status = -1;
	}
	return (status);
}

static unsigned int mv64360_dma_poll(struct file *file, poll_table *wait)
{
	unsigned int retval = 0;
	struct mv64360_dma_cntl_t *p;

	p = file->private_data;
	if (!p)
		return 0;

	poll_wait(file, &p->readywait, wait);

	if (p->dma_complete >= 0)
		retval = POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;

	return retval;
}

static int mv64360_dma_open(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct mv64360_dma_cntl_t *p;

	DEBUG(1, "open minor %d\n", minor);
	if (minor >= IDMA_CHANNEL_COUNT)
		return -ENODEV;

	if (dma_device_open[minor])
		return -EBUSY;
	dma_device_open[minor]++;

	file->private_data = &mv64360_dma_cntl[minor];
	p = file->private_data;
	p->dma_src_buffer = NULL;
	p->dma_dest_buffer = NULL;
	p->dma_complete = 0;
	p->error_select = 0;
	p->error_addr = 0;
	p->dma_buffer_npages = 0;
	p->dma_buffer_size = 0;
	p->byte_count = 0;
	p->dma_attributes.flags = 0;
	p->dma_attributes.hold_source_addr = 0;
	p->dma_attributes.hold_dest_addr = 0;
	p->dma_timer_pending = 0;
	init_MUTEX(&p->dma_available);
	init_waitqueue_head(&p->readywait);
	DEBUG(2, "open: device open count %d\n", dma_device_open[minor]);
	return 0;
}

static int mv64360_dma_release(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct mv64360_dma_cntl_t *p;
	struct page *page, *pend;
	int retval = 0;

	DEBUG(1, "release minor %d\n", minor);
	if (minor >= IDMA_CHANNEL_COUNT) {
		printk(KERN_ERR
		       "mv64360_dma: unknown device (minor %d) closed\n",
		       minor);
		retval = -ENODEV;
	}
	p = file->private_data;
	if (p->dma_src_buffer) {
		if (p->dma_attributes.flags != FIXED_SOURCE) {
			pend =
			    virt_to_page(p->dma_src_buffer +
					 p->dma_buffer_size - 1);
			for (page = virt_to_page(p->dma_src_buffer);
			     page <= pend; page++)
				ClearPageReserved(page);
#ifdef CONFIG_BIGPHYS_AREA
			bigphysarea_free_pages((caddr_t) (p->dma_src_buffer));
#else
			pci_free_consistent(NULL, p->dma_buffer_size,
					    p->dma_src_buffer,
					    p->dma_src_handle);
#endif
		}
		p->dma_src_buffer = NULL;
	}
	if (p->dma_dest_buffer) {
		if (p->dma_attributes.flags != FIXED_DESTINATION) {
			pend =
			    virt_to_page(p->dma_dest_buffer +
					 p->dma_buffer_size - 1);
			for (page = virt_to_page(p->dma_dest_buffer);
			     page <= pend; page++)
				ClearPageReserved(page);
#ifdef CONFIG_BIGPHYS_AREA
			bigphysarea_free_pages((caddr_t) (p->dma_dest_buffer));
#else
			pci_free_consistent(NULL, p->dma_buffer_size,
					    p->dma_dest_buffer,
					    p->dma_dest_handle);
#endif
		}
		p->dma_dest_buffer = NULL;
	}
	file->private_data = NULL;
	dma_device_open[minor]--;
	DEBUG(2, "close: device open count %d\n", dma_device_open[minor]);
	return (retval);
}

static ssize_t
mv64360_dma_read(struct file *file, char *buf, size_t count, loff_t * ppos)
{
	int retval = 0;
	struct mv64360_dma_cntl_t *p;
	int data;
	unsigned long flags;
	DECLARE_WAITQUEUE(wait, current);

	p = file->private_data;
	if (!p)
		return -EINVAL;
	add_wait_queue(&p->readywait, &wait);

	spin_lock_irqsave(&p->dma_lock, flags);
	data = p->dma_complete;
	spin_unlock_irqrestore(&p->dma_lock, flags);
	while (data < 0) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			break;
		}
		if (signal_pending(current)) {
			retval = -EINTR;
			break;
		}
		schedule();
		spin_lock_irqsave(&p->dma_lock, flags);
		data = p->dma_complete;
		spin_unlock_irqrestore(&p->dma_lock, flags);
	}
	remove_wait_queue(&p->readywait, &wait);
	set_current_state(TASK_RUNNING);
	if (retval == 0) {
		int status;

		status = data;
		copy_to_user(buf, &status, sizeof(status));
		retval = sizeof(status);
	}
	DEBUG(1, "read: retval 0x%x\n", retval);
	return retval;
}

static ssize_t
mv64360_dma_write(struct file *file, const char *buf, size_t count,
		  loff_t * ppos)
{
	return 0;
}

static int mv64360_dma_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long offset;
	struct mv64360_dma_cntl_t *p;
	int retval;

	DEBUG(1, "mmap: minor %d\n", MINOR(file->f_dentry->d_inode->i_rdev));

	if ((!file) || (!vma))
		return -ENXIO;
	if (vma->vm_pgoff)
		return -EINVAL;	/* want no offset */

	p = file->private_data;
	if (!p)
		return -EINVAL;

	if ((p->dma_attributes.flags == 0) ||
	    (p->dma_src_buffer == NULL) || (p->dma_dest_buffer == NULL))
		return -EINVAL;

	if (vma->vm_end - vma->vm_start > p->dma_buffer_size) {
		/* An attempt to map beyond the end of the buffer */
		printk(KERN_ERR
		       "mv64360_dma_mmap: INVALID. "
		       "Start at 0x%08lx  end 0x%08lx\n",
		       vma->vm_start, vma->vm_end);
		return (-EINVAL);
	}

	if (p->dma_attributes.flags == FIXED_DESTINATION) {
#ifdef CONFIG_NOT_COHERENT_CACHE
		offset = p->dma_src_handle;
#else
		offset = virt_to_phys(p->dma_src_buffer);
#endif
	} else {
#ifdef CONFIG_NOT_COHERENT_CACHE
		offset = p->dma_dest_handle;
#else
		offset = virt_to_phys(p->dma_dest_buffer);
#endif
	}

	retval = remap_pfn_range(vma,
			    vma->vm_start,
			    offset >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start,
			    vma->vm_page_prot);
	DEBUG(1, "mmap: start 0x%lx end 0x%lx offset 0x%lx\n",
	      vma->vm_start, vma->vm_end, offset);
	if (retval) {
		printk(KERN_WARNING "mv64360_mmap: remap_pfn_range failed");
		return (-EAGAIN);
	}
	return 0;
}

static int
mv64360_dma_ioctl(struct inode *inode, struct file *file,
		  unsigned int cmd, unsigned long arg)
{
	unsigned int minor = MINOR(inode->i_rdev);
	int retval;
	struct mv64360_dma_cntl_t *p;

	DEBUG(1, "ioctl(%x, %lx)\n", cmd, arg);
	if (minor >= IDMA_CHANNEL_COUNT) {
		printk(KERN_ERR
		       "mv64360_dma: unknown device (minor %d) ioctl\n", minor);
		return -ENODEV;
	}

	/* Permission check */
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	p = file->private_data;
	switch (cmd) {
	case MV64360_DMA_SETUP:
		{
			struct dma_setup_t dma_setup;
			int needed_pages;
			struct page *page, *pend;

			copy_from_user(&dma_setup,
				       (char *)arg, sizeof(dma_setup));
			/* Argument check */
			if ((dma_setup.dma_descriptor.source_addr == 0) &&
			    (dma_setup.dma_descriptor.dest_addr == 0)) {
				DEBUG(0,
				      "dma setup invalid 1: source addr 0x%x dest addr 0x%x\n",
				      dma_setup.dma_descriptor.source_addr,
				      dma_setup.dma_descriptor.dest_addr);
				return -EINVAL;
			}
			if ((dma_setup.dma_descriptor.source_addr != 0) &&
			    (dma_setup.dma_descriptor.dest_addr != 0)) {
				DEBUG(0,
				      "dma setup invalid 2: source addr 0x%x dest addr 0x%x\n",
				      dma_setup.dma_descriptor.source_addr,
				      dma_setup.dma_descriptor.dest_addr);
				return -EINVAL;
			}
			if ((dma_setup.dma_descriptor.dest_addr != 0) &&
			    (dma_setup.dma_descriptor.source_addr != 0)) {
				DEBUG(0,
				      "dma setup invalid 3: source addr 0x%x dest addr 0x%x\n",
				      dma_setup.dma_descriptor.source_addr,
				      dma_setup.dma_descriptor.dest_addr);
				return -EINVAL;
			}
			if (dma_setup.dma_descriptor.byte_count <= 0) {
				DEBUG(0,
				      "dma setup invalid 4: source addr 0x%x dest addr 0x%x\n",
				      dma_setup.dma_descriptor.source_addr,
				      dma_setup.dma_descriptor.dest_addr);
				return -EINVAL;
			}
			p->byte_count = dma_setup.dma_descriptor.byte_count;
			needed_pages = ((p->byte_count - 1) / PAGE_SIZE) + 1;
			if (dma_setup.dma_descriptor.dest_addr != 0) {
				/* Dest address specified => alloc source buffer */
				if (dma_setup.dma_descriptor.dest_addr <=
				    __pa(high_memory)) {
					DEBUG(0,
					      "dma setup invalid 5: source addr 0x%x dest addr 0x%x\n",
					      dma_setup.dma_descriptor.
					      source_addr,
					      dma_setup.dma_descriptor.
					      dest_addr);
					return -EINVAL;
				}
				if (p->dma_src_buffer) {
					if (needed_pages !=
					    p->dma_buffer_npages) {
						/* Mismatch; allocate new */
						pend =
						    virt_to_page(p->
								 dma_src_buffer
								 +
								 (needed_pages *
								  PAGE_SIZE) -
								 1);
						for (page =
						     virt_to_page(p->
								  dma_src_buffer);
						     page <= pend; page++)
							ClearPageReserved(page);
#ifdef CONFIG_BIGPHYS_AREA
						bigphysarea_free_pages((caddr_t)
								       (p->
									dma_src_buffer));
						p->dma_src_buffer = (char *)
						    bigphysarea_alloc_pages
						    (needed_pages, 0,
						     GFP_KERNEL);
#else
						pci_free_consistent(NULL,
								    needed_pages
								    * PAGE_SIZE,
								    p->
								    dma_src_buffer,
								    p->
								    dma_src_handle);
						if (!
						    (p->dma_src_buffer =
						     (char *)
						     pci_alloc_consistent(NULL,
									  needed_pages
									  *
									  PAGE_SIZE,
									  &p->
									  dma_src_handle)))
							return -ENOMEM;
#endif
					}
				} else {
#ifdef CONFIG_BIGPHYS_AREA
					p->dma_src_buffer =
					    (char *)bigphysarea_alloc_pages
					    (needed_pages, 0, GFP_KERNEL);
#else
					if (!(p->dma_src_buffer =
					      (char *)pci_alloc_consistent
					      (NULL, needed_pages * PAGE_SIZE,
					       &p->dma_src_handle)))
						return -ENOMEM;
#endif
				}
				pend =
				    virt_to_page(p->dma_src_buffer +
						 (needed_pages * PAGE_SIZE) -
						 1);
				for (page = virt_to_page(p->dma_src_buffer);
				     page <= pend; page++)
					SetPageReserved(page);

				p->dma_dest_buffer =
				    (char *)dma_setup.dma_descriptor.dest_addr;
				p->dma_attributes.flags = FIXED_DESTINATION;
			} else {
				/* Source address specified => alloc dest buffer */
				if (dma_setup.dma_descriptor.source_addr <=
				    __pa(high_memory)) {
					DEBUG(0,
					      "dma setup invalid 6: source addr 0x%x dest addr 0x%x\n",
					      dma_setup.dma_descriptor.
					      source_addr,
					      dma_setup.dma_descriptor.
					      dest_addr);
					return -EINVAL;
				}
				if (p->dma_dest_buffer) {
					if (needed_pages !=
					    p->dma_buffer_npages) {
						/* Mismatch; allocate new */
						pend =
						    virt_to_page(p->
								 dma_dest_buffer
								 +
								 (needed_pages *
								  PAGE_SIZE) -
								 1);
						for (page =
						     virt_to_page(p->
								  dma_dest_buffer);
						     page <= pend; page++)
							ClearPageReserved(page);
#ifdef CONFIG_BIGPHYS_AREA
						bigphysarea_free_pages((caddr_t)
								       (p->
									dma_dest_buffer));
						p->dma_dest_buffer = (char *)
						    bigphysarea_alloc_pages
						    (needed_pages, 0,
						     GFP_KERNEL);
#else
						pci_free_consistent(NULL,
								    needed_pages
								    * PAGE_SIZE,
								    p->
								    dma_dest_buffer,
								    p->
								    dma_dest_handle);
						if (!
						    (p->dma_dest_buffer =
						     (char *)
						     pci_alloc_consistent(NULL,
									  needed_pages
									  *
									  PAGE_SIZE,
									  &p->
									  dma_dest_handle)))
							return -ENOMEM;
#endif
					}
				} else {
#ifdef CONFIG_BIGPHYS_AREA
					p->dma_dest_buffer =
					    (char *)bigphysarea_alloc_pages
					    (needed_pages, 0, GFP_KERNEL);
#else
					if (!(p->dma_dest_buffer =
					      (char *)pci_alloc_consistent
					      (NULL, needed_pages * PAGE_SIZE,
					       &p->dma_dest_handle)))
						return -ENOMEM;
#endif
				}
				pend =
				    virt_to_page(p->dma_dest_buffer +
						 (needed_pages * PAGE_SIZE) -
						 1);
				for (page = virt_to_page(p->dma_dest_buffer);
				     page <= pend; page++)
					SetPageReserved(page);
				p->dma_src_buffer =
				    (char *)dma_setup.dma_descriptor.
				    source_addr;
				p->dma_attributes.flags = FIXED_SOURCE;
			}
			p->dma_buffer_npages = needed_pages;
			p->dma_buffer_size = PAGE_SIZE * needed_pages;
			if ((p->dma_src_buffer == NULL)
			    || (p->dma_dest_buffer == NULL)) {
				p->byte_count = 0;
				p->dma_buffer_size = 0;
				return -ENOMEM;
			}

			DEBUG(1,
			      "setup: src 0x%p dest 0x%p count 0x%x pages 0x%x\n",
			      p->dma_src_buffer, p->dma_dest_buffer,
			      p->byte_count, needed_pages);

			p->dma_attributes.hold_source_addr =
			    dma_setup.dma_attributes.hold_source_addr;
			p->dma_attributes.hold_dest_addr =
			    dma_setup.dma_attributes.hold_dest_addr;

			break;
		}

	case MV64360_DMA_START:
		{
			struct dma_descriptor_t dma_descriptor;

			/* Check whether setup is complete */
			if ((p->byte_count == 0) ||
			    (p->dma_src_buffer == NULL) ||
			    (p->dma_dest_buffer == NULL))
				return -EINVAL;

			dma_descriptor.byte_count = p->byte_count;
			if (p->dma_attributes.flags == FIXED_SOURCE) {
				dma_descriptor.source_addr =
				    (unsigned int)p->dma_src_buffer;
			} else {
#ifdef CONFIG_BIGPHYS_AREA
				dma_descriptor.source_addr =
				    (unsigned
				     int)(virt_to_phys(p->dma_src_buffer));
#else
				dma_descriptor.source_addr = p->dma_src_handle;
#endif
			}
			if (p->dma_attributes.flags == FIXED_DESTINATION) {
				dma_descriptor.dest_addr =
				    (unsigned int)p->dma_dest_buffer;
			} else {
#ifdef CONFIG_BIGPHYS_AREA
				dma_descriptor.dest_addr =
				    (unsigned
				     int)(virt_to_phys(p->dma_dest_buffer));
#else
				dma_descriptor.dest_addr = p->dma_dest_handle;
#endif
			}
			DEBUG(1, "dma_start: src 0x%x dest 0x%x count 0x%x\n",
			      dma_descriptor.source_addr,
			      dma_descriptor.dest_addr,
			      dma_descriptor.byte_count);

			retval = mv64360_dma_start(p, &dma_descriptor);

			return (retval);
			break;
		}
	case MV64360_DMA_ABORT:
		{
			retval = mv64360_dma_abort(p);
			return (retval);
			break;
		}
	case MV64360_DMA_STATUS:
		{
			struct dma_status_t dma_status;

			mv64360_dma_status(p, &dma_status);
			copy_to_user((char *)arg, &dma_status,
				     sizeof(dma_status));
			break;
		}
	case MV64360_DMA_PAUSE:
		{
			retval = mv64360_dma_pause(p);
			return (retval);
			break;
		}
	case MV64360_DMA_RESUME:
		{
			retval = mv64360_dma_resume(p);
			return (retval);
			break;
		}
	case MV64360_DMA_ARBITER_SETUP:
		{
			struct dma_arbiter_setup_t dma_arbiter;

			copy_from_user(&dma_arbiter,
				       (char *)arg, sizeof(dma_arbiter));
			retval = mv64360_dma_arbiter_setup((unsigned int *)
							   &dma_arbiter.values);
			return (retval);
			break;
		}
	default:
		return -EINVAL;
		break;
	}
	return 0;

}				/* mv64360_dma_ioctl */

static struct file_operations mv64360_dma_fops = {
	.owner = THIS_MODULE,
	.open = mv64360_dma_open,
	.release = mv64360_dma_release,
	.read = mv64360_dma_read,
	.write = mv64360_dma_write,
	.mmap = mv64360_dma_mmap,
	.ioctl = mv64360_dma_ioctl,
	.poll = mv64360_dma_poll,
};

static int __init mv64360_dma_init_module(void)
{
	int status;

	printk(KERN_INFO DRV_NAME " version " DRV_VERSION "\n");

	mv64360_base = (unsigned int)ioremap(mv64360_reg_base, 0x10000);

	/* Set up character device for user mode clients */
	major_dev = register_chrdev(0, "mv64360_dma", &mv64360_dma_fops);
	if (major_dev == 0) {
		printk(KERN_NOTICE
		       "mv64360_dma: Unable to find a free device # for "
		       "the MV64360 IDMA driver\n");
		iounmap((void *)mv64360_base);
		return -1;

	}
	status = mv64360_dma_init();

	if (status != 0)
		return -ENODEV;

        mv64360_dma_class = class_create(THIS_MODULE, "mv64360_dma");
        class_device_create(mv64360_dma_class, NULL, MKDEV(major_dev, 0), NULL, 
				"mv64360_dma_0");
        class_device_create(mv64360_dma_class, NULL, MKDEV(major_dev, 1), NULL, 
				"mv64360_dma_1");
        class_device_create(mv64360_dma_class, NULL, MKDEV(major_dev, 2), NULL, 
				"mv64360_dma_2");
        class_device_create(mv64360_dma_class, NULL, MKDEV(major_dev, 3), NULL, 
				"mv64360_dma_3");

	return 0;

}				/* mv64360_dma_init_module */

static void __exit mv64360_dma_cleanup_module(void)
{
	int chan;

	DEBUG(0, "exiting\n");

	free_irq(IDMA_ERROR_INT, NULL);
	for (chan = 0; chan < IDMA_CHANNEL_COUNT; chan++) {
		free_irq(IDMA_CHAN0_COMPLETION_INT + chan,
			 &mv64360_dma_cntl[chan]);
	}

	unregister_chrdev(major_dev, "mv64360_dma");
	iounmap((void *)mv64360_base);

	class_device_destroy(mv64360_dma_class, MKDEV(major_dev, 0));

	class_device_destroy(mv64360_dma_class, MKDEV(major_dev, 1));

	class_device_destroy(mv64360_dma_class, MKDEV(major_dev, 2));

	class_device_destroy(mv64360_dma_class, MKDEV(major_dev, 3));

}				/* mv64360_dma_cleanup_module */

module_init(mv64360_dma_init_module);
module_exit(mv64360_dma_cleanup_module);

MODULE_AUTHOR("Ajit Prem");
MODULE_DESCRIPTION("Driver for the Marvell MV64360 IDMA controller");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
