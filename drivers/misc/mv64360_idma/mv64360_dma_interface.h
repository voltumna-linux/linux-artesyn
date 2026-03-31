#ifndef __MV64360_DMA_INTERFACE
#define __MV64360_DMA_INTERFACE

#define	FIXED_SOURCE		0x00000001
#define FIXED_DESTINATION	0x00000002

/*
 * The following structure defines the basic DMA transfer parameters.
 */

struct dma_descriptor_t {
	/* Number of bytes to transfer */
	unsigned int byte_count;

	/* starting addr of source data */
	unsigned int source_addr;

	/* starting addr of destination */
	unsigned int dest_addr;
};

/*
 * The following structure defines user controlled attributes for a
 * given DMA transfer. 
 */

struct dma_attributes_t {
	/* Specify whether source or destination is fixed */
	int flags;

	/* Do not increment the source addr */
	int hold_source_addr;

	/* Do not increment the dest addr */
	int hold_dest_addr;
};

/*
 * The following structure defines DMA status information for the last
 * successful DMA transfer or the last error for a particular channel.
 * The dma_error_code and dma_error_addr fields are only meaningful if
 * dma_complete is non-zero. No error code (0) with a non-zero dma_complete 
 * status indicates DMA is still in progress.
 */

struct dma_status_t {
	/* O when DMA completes successfully */
	int dma_complete;

	/* Error code when dma_complete is not 0 */
	unsigned int dma_error_code;

	/* Address causing the error code */
	unsigned int dma_error_addr;

	/* Current address of source data */
	unsigned int cur_source_addr;

	/* Current address of destination */
	unsigned int cur_dest_addr;

	/* Address of next descriptor */
	unsigned int cur_next_desc;

	/* Current channel control settings */
	unsigned int chan_cntl_low;
};

struct dma_setup_t {
	struct dma_descriptor_t dma_descriptor;
	struct dma_attributes_t dma_attributes;
};

struct dma_arbiter_setup_t {
	unsigned int values[16];
};

#define MV64360_DMA_SETUP		_IOW('w', 1, struct dma_setup_t)
#define MV64360_DMA_START		_IO('w', 2)
#define MV64360_DMA_ABORT		_IO('w', 3)
#define MV64360_DMA_PAUSE		_IO('w', 4)
#define MV64360_DMA_RESUME		_IO('w', 5)
#define MV64360_DMA_STATUS		_IOR('w', 6, struct dma_status_t)
#define MV64360_DMA_ARBITER_SETUP	_IOW('w', 7, struct dma_arbiter_setup_t)

#endif				/* __MV64360_DMA_INTERFACE */
