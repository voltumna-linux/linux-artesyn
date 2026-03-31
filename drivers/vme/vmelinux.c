/*
 * vmelinux.c
 *
 * Provides the device node interface from the user space programs to 
 * the VME bridge.  Primary purpose is to convert user space
 * data to kernel and back.
 *                                  
 * This driver supports both the Tempe, and the Universe/Universe II chips.
 * 
 * Author: Tom Armistead, Ajit Prem
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
#include <linux/moduleparam.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/pagemap.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/mv643xx.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/mv64x60.h>
#include "vmedrv.h"

static char Version[] = "4.0 5/23/2008";

static int vme_open(struct inode *, struct file *);
static int vme_release(struct inode *, struct file *);
static ssize_t vme_read(struct file *, char *, size_t, loff_t *);
static ssize_t vme_write(struct file *, const char *, size_t, loff_t *);
static unsigned int vme_poll(struct file *, poll_table *);
static int vme_ioctl(struct inode *, struct file *, unsigned int,
		     unsigned long);
static int vme_mmap(struct file *file, struct vm_area_struct *vma);
static int vme_procinfo(char *, char **, off_t, int, int *, void *);
static struct class *vme_class;

void vmemod_setup_options(char *cmd_line);

extern unsigned tb_ticks_per_jiffy;

static struct proc_dir_entry *vme_procdir;

static struct file_operations vme_fops = {
	.owner = THIS_MODULE,
	.read = vme_read,
	.write = vme_write,
	.poll = vme_poll,
	.ioctl = vme_ioctl,
	.mmap = vme_mmap,
	.open = vme_open,
	.release = vme_release
};

#define VME_MAJOR	221
#define MAX_MINOR	0xFF

#define DMATYPE_SINGLE  1
#define DMATYPE_LLIST   2

#define INTERBOARD_BUFFER_SIZE 			512 * 1024
#define DEFAULT_INBOUND_WINDOW_BUFFER_SIZE	4 * 1024 * 1024

static unsigned long opened[MAX_MINOR + 1];
struct semaphore devinuse[MAX_MINOR + 1];

// Global VME controller information
extern int vmechip_irq;		// PCI irq
extern int vmechip_devid;	// PCI devID of VME bridge
extern int vmeparent_devid;	// PCI devID of VME bridge parent
extern int vmechip_revision;	// PCI revision
extern void __iomem *vmechip_baseaddr;	// virtual address of chip registers
extern int vme_slotnum;		// VME slot num (-1 = unknown)
extern int vme_syscon;		// VME sys controller (-1 = unknown)

#ifdef CONFIG_VME_BRIDGE_BOOTMEM
extern void *vme_driver_bootmem;
extern unsigned int vme_bootmem_size;
#endif

char *in_image_ba[0x8];		// Virtual Address
dma_addr_t in_image_pa[0x8];	// Physical Address
int in_image_size[0x8];		// Size
int in_image_mapped[0x8];
static int out_image_mapped[0x8];
static struct semaphore in_image_sem[0x8];
static struct semaphore out_image_sem[0x8];

struct resource out_resource[0x8];
extern unsigned int out_image_va[8];	// Base virtual address
extern unsigned int out_image_valid[8];	// validity

/*
 * External functions
 */
extern void uni_shutdown(void);
extern void tsi148_shutdown(void);

extern int vme_init(void *);
extern int vme_bus_error_chk(int);
extern int vme_get_slot_num(void);
extern int vme_get_out_bound(vmeOutWindowCfg_t *);
extern int vme_set_out_bound(vmeOutWindowCfg_t *);
extern int vme_do_dma(vmeDmaPacket_t *);
extern int vme_get_slot_info(vmeInfoCfg_t *);
extern int vme_get_requestor(vmeRequesterCfg_t *);
extern int vme_set_requestor(vmeRequesterCfg_t *);
extern int vme_get_arbiter(vmeArbiterCfg_t *);
extern int vme_set_arbiter(vmeArbiterCfg_t *);
extern int vme_generate_irq(virqInfo_t *);
extern int vme_get_irq_status(virqInfo_t *);
extern int vme_clr_irq_status(virqInfo_t *);
extern int vme_get_in_bound(vmeInWindowCfg_t *);
extern int vme_set_in_bound(vmeInWindowCfg_t *);
extern int vme_do_rmw(vmeRmwCfg_t *);
extern int vme_setup_lm(vmeLmCfg_t *);
extern int vme_wait_lm(vmeLmCfg_t *);

extern void vme_sync_data(void);
extern void vme_flush_range(unsigned long, unsigned long);

extern int uni_procinfo(char *);
extern int tsi148_procinfo(char *);

extern unsigned long vme_pci_bus_mem_base_phys(struct pci_bus *bus);

#define	DEV_VALID	1
#define	DEV_EXCLUSIVE	2
#define	DEV_RW		4

static char vme_minor_dev_flags[] = {
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_m0 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_m1 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_m2 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_m3 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_m4 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_m5 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_m6 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_m7 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_s0 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_s1 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_s2 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_s3 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_s4 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_s5 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_s6 */
	DEV_VALID | DEV_EXCLUSIVE | DEV_RW,	/* /dev/vme_s7 */
	DEV_VALID,		/* /dev/vme_dma0 */
	DEV_VALID,		/* /dev/vme_dma1 */
	0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	DEV_VALID,		/* /dev/vme_ctl */
	DEV_VALID | DEV_RW,	/* /dev/vme_regs */
	DEV_VALID | DEV_EXCLUSIVE,	/* /dev/vme_rmw0 */
	DEV_VALID | DEV_EXCLUSIVE,	/* /dev/vme_lm0 */
	0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	DEV_VALID | DEV_RW,	/* /dev/vme_slot0 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot1 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot2 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot3 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot4 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot5 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot6 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot7 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot8 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot9 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot10 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot11 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot12 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot13 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot14 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot15 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot16 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot17 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot18 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot19 */
	DEV_VALID | DEV_RW,	/* /dev/vme_slot20 */
	DEV_VALID | DEV_RW	/* /dev/vme_slot21 */
};

static unsigned int reads;
static unsigned int writes;
static unsigned int ioctls;

/* Shared global */
wait_queue_head_t dma_queue[2];
wait_queue_head_t lm_queue;
wait_queue_head_t mbox_queue;
wait_queue_head_t vmeint_queue[8];
struct vmeSharedData *vmechip_interboard_data;	// vaddress of board data
dma_addr_t vmechip_interboard_datap;	// paddress of board data
struct resource *vmepcimem;
int tb_speed;
struct semaphore virq_inuse;
struct pci_dev *vme_pci_dev;

MODULE_PARM_DESC(vmechip_irq, "VME chip irq in the range 1-255");
module_param(vmechip_irq, int, 0);
MODULE_PARM_DESC(vme_slotnum, "Slot Number in the range 1-21");
module_param(vme_slotnum, int, 0);
MODULE_LICENSE("GPL");

static int vme_minor_type(int minor)
{
	if (minor > sizeof(vme_minor_dev_flags)) {
		return (0);
	}
	return (vme_minor_dev_flags[minor]);
}

/*
 * vme_alloc_buffers()
 *
 * Obtain a buffer for the data structure that will be 
 * advertized through CS/CSR space. The CS/CSR decoder only
 * allows remapping on 512K boundaries so this buffer must be
 * aligned on a 512K byte boundary. 
 *
 */

static int vme_alloc_buffers(void)
{
	if (vmechip_interboard_data == NULL) {
		vmechip_interboard_data = (struct vmeSharedData *)
		    dma_alloc_coherent((struct device *)&vme_pci_dev->dev,
				       INTERBOARD_BUFFER_SIZE,
				       (dma_addr_t *)&vmechip_interboard_datap,
					GFP_DMA);
		if (!vmechip_interboard_data) {
			printk("tsi148: No memory for interboard data\n");
			return -ENOMEM;
		}
		if (((int)vmechip_interboard_data & 0x7FFFF) != 0) {
			printk
			    ("tsi148: Unable to get DMA buffer with required alignment\n");
			return -1;
		}
	}
#ifdef CONFIG_VME_BRIDGE_BOOTMEM
	{
		struct page *page, *pend;

		in_image_size[7] = vme_bootmem_size;
		if (vme_driver_bootmem != NULL) {
			in_image_ba[7] = (char *) vme_driver_bootmem;
			in_image_pa[7] = __pa(in_image_ba[7]);
			/* now mark the pages as reserved; otherwise */
			/* remap_pfn_range doesn't do what we want */
			pend = virt_to_page(in_image_ba[7] + in_image_size[7] - 1);
			for (page = virt_to_page(in_image_ba[7]); 
				page <= pend; page++)
				SetPageReserved(page);
		}
	}
#endif
	return (0);
}

//----------------------------------------------------------------------------
//  vme_free_buffers()
//   Free all buffers allocated by this driver.
//----------------------------------------------------------------------------
static void vme_free_buffers(void)
{
	int i;
	struct page *page, *pend;

	/* 
	 * clear valid flags of interboard data and free it
	 */
	if (vmechip_interboard_data != NULL) {
		strcpy(vmechip_interboard_data->validity1, "OFF");
		strcpy(vmechip_interboard_data->validity2, "OFF");
		dma_free_coherent(&vme_pci_dev->dev,
				  INTERBOARD_BUFFER_SIZE,
				  vmechip_interboard_data,
				  vmechip_interboard_datap);
	}
	// free inbound buffers
	for (i = 0; i < 8; i++) {
		if (in_image_ba[i] != NULL) {
			/* undo marking the pages as reserved */
			pend = virt_to_page(in_image_ba[i] + in_image_size[i] - 1);
			for (page = virt_to_page(in_image_ba[i]); page <= pend; page++)
				ClearPageReserved(page);

			if (i == 7) {
#ifndef CONFIG_VME_BRIDGE_BOOTMEM
				dma_free_coherent(&vme_pci_dev->dev,
					  in_image_size[i],
					  in_image_ba[i], in_image_pa[i]);
#endif
				in_image_mapped[i] = 0;
			} else {
				dma_free_coherent(&vme_pci_dev->dev,
					  in_image_size[i],
					  in_image_ba[i], in_image_pa[i]);
				in_image_mapped[i] = 0;
			}
		}
	}
}

//----------------------------------------------------------------------------
//  vme_init_test_data()
//   Initialize the test area of the interboard data structure 
//   with the expected data patterns.
//----------------------------------------------------------------------------
static void vme_init_test_data(int testdata[])
{
	int i;

	// walking 1 & walking 0
	for (i = 0; i < 32; i++) {
		testdata[i * 2] = 1 << i;
		testdata[i * 2 + 1] = ~(1 << i);
	}

	// all 0, all 1's, 55's and AA's
	testdata[64] = 0x0;
	testdata[65] = 0xFFFFFFFF;
	testdata[66] = 0x55555555;
	testdata[67] = 0xAAAAAAAA;

	// Incrementing data.
	for (i = 68; i < 1024; i++) {
		testdata[i] = i;
	}
}

//----------------------------------------------------------------------------
//  vme_init_interboard_data()
//    Initialize the interboard data structure.
//----------------------------------------------------------------------------
static void vme_init_interboard_data(struct vmeSharedData *ptr)
{
	if (ptr == NULL)
		return;

	memset(ptr, 0, sizeof(struct vmeSharedData));
	vme_init_test_data(&ptr->readTestPatterns[0]);

	ptr->structureRev = 1;
	strcpy(&ptr->osname[0], "linux");

	ptr->driverRev = 1;

#ifdef CONFIG_MVME7100
	strcpy(&ptr->boardString[0], "MVME7100");
#endif
#ifdef CONFIG_MVME3100
	strcpy(&ptr->boardString[0], "MVME3100");
#endif
#ifdef CONFIG_MVME6100
	strcpy(&ptr->boardString[0], "MVME6100");
#endif
#ifdef CONFIG_MVME5500
	strcpy(&ptr->boardString[0], "MVME5500");
#endif
#ifdef CONFIG_MVME5100
	strcpy(&ptr->boardString[0], "MVME5100");
#endif
	ptr->vmeControllerType = vmechip_devid;
	ptr->vmeControllerRev = vmechip_revision;

	/* Set valid strings "VME", "RDY" */
	strcpy(ptr->validity1, "VME");
	strcpy(ptr->validity2, "RDY");
	vme_flush_range((unsigned long)ptr, (unsigned long)ptr + sizeof(struct vmeSharedData));
}

/*
 *  vme_procinfo()
 *     /proc interface into this driver
 */

static int vme_procinfo(char *buf, char **start, off_t fpos, int length,
			int *eof, void *data)
{
	char *p;

	p = buf;
	p += sprintf(p, "VME driver %s\n\n", Version);

	p += sprintf(p, "vmechip baseaddr = %08X\n", (int)vmechip_baseaddr);
	p += sprintf(p, "vmechip device   = %08X\n", (int)vmechip_devid);
	p += sprintf(p, "vmechip revision = %08X\n", (int)vmechip_revision);
	p += sprintf(p, "vme slot number  = %2d\n", (int)vme_slotnum);
	if (vme_syscon)
		p += sprintf(p, "vme system controller = TRUE\n");
	else
		p += sprintf(p, "vme system controller = FALSE\n");

	if (vmechip_devid == PCI_DEVICE_ID_TUNDRA_TSI148) {
		p += tsi148_procinfo(p);
	}

	if (vmechip_devid == PCI_DEVICE_ID_TUNDRA_CA91C042) {
		p += uni_procinfo(p);
	}

	p += sprintf(p, "\n");
	p += sprintf(p, "statistics:  reads = %i  writes = %i  ioctls = %i\n\n",
		     reads, writes, ioctls);
	p += sprintf(p, "\n");

	*eof = 1;
	return p - buf;
}

/*
  register_proc()
*/
static void register_proc(void)
{
	vme_procdir = create_proc_entry("vmeinfo", S_IFREG | S_IRUGO, 0);
	vme_procdir->read_proc = vme_procinfo;
}

/*
  unregister_proc()
*/
static void unregister_proc(void)
{
	remove_proc_entry("vmeinfo", 0);
}

//----------------------------------------------------------------------------
//  vme_chip_reg_read
//    Read a VME chip register.
//
//    Note that Tempe swizzles registers at offsets > 0x100 on its own.
//----------------------------------------------------------------------------
unsigned int vme_chip_reg_read(unsigned int *ptr)
{
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (readl(ptr));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		if (((char *)ptr) - ((char *)vmechip_baseaddr) > 0x100) {
			return (*ptr);
		}
		return (readl(ptr));
		break;
	}
	return (0);
}

/*
  vme_chip_reg_write
    Write a VME chip register.

    Note that Tempe swizzles registers at offsets > 0x100 on its own.
*/
void vme_chip_reg_write(unsigned int *ptr, unsigned int value)
{
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		writel(value, ptr);
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		if (((char *)ptr) - ((char *)vmechip_baseaddr) > 0x100) {
			*ptr = value;
		} else {
			writel(value, ptr);
		}
		break;
	}
}

/*
 Function   : vme_ioctl_free_dma
 Description: Free the pages associated with the linked list of DMA packets.
*/
static int vme_ioctl_free_dma(vmeDmaPacket_t *start_pkt)
{
        vmeDmaPacket_t *current_pkt;
        vmeDmaPacket_t *previous_pkt;
        vmeDmaPacket_t *next_pkt;

        /* Free all pages associated with the packets. */
        current_pkt = start_pkt;
        previous_pkt = current_pkt;
        while (current_pkt != 0) {
                next_pkt = current_pkt->pNextPacket;
                if (current_pkt + 1 != next_pkt) {
                        free_pages((int)previous_pkt, 0);
                        previous_pkt = next_pkt;
                }
                current_pkt = next_pkt;
        }
        return (0);
}


/*
 Function   : vme_ioctl_setup_dma
 Description:
    Read descriptors from user space and create a linked list
    of DMA packets.
*/
static vmeDmaPacket_t *vme_ioctl_setup_dma(vmeDmaPacket_t *vme_dma,
                                    int *return_status)
{
        vmeDmaPacket_t *vme_current;
        int max_per_page;
        int current_pktcount;
        vmeDmaPacket_t *start_pkt;
        vmeDmaPacket_t *current_pkt;

        max_per_page = PAGE_SIZE / sizeof(vmeDmaPacket_t) - 1;
        start_pkt = (vmeDmaPacket_t *) __get_free_pages(GFP_KERNEL, 0);
        if (start_pkt == 0) {
                *return_status = -ENOMEM;
                return (0);
        }
        /* First allocate pages for packets and create linked list */
        vme_current = vme_dma;
        current_pkt = start_pkt;
        current_pktcount = 0;

        while (vme_current != 0) {
                if (copy_from_user
                    (current_pkt, vme_current, sizeof(vmeDmaPacket_t))) {
                        current_pkt->pNextPacket = 0;
                        vme_ioctl_free_dma(start_pkt);
                        *return_status = -EFAULT;
                        return (0);
                }
                if ((current_pkt->srcBus == VME_DMA_USER) ||
                    (current_pkt->dstBus == VME_DMA_USER) || 
                    (current_pkt->srcBus == VME_DMA_KERNEL) ||
                    (current_pkt->dstBus == VME_DMA_KERNEL)) {
                                current_pkt->pNextPacket = 0;
                                vme_ioctl_free_dma(start_pkt);
                                *return_status = -EINVAL;
                                return (0);
                }

                if (current_pkt->pNextPacket == NULL) {
                        break;
                }
                vme_current = current_pkt->pNextPacket;

                current_pkt->pNextPacket = current_pkt + 1;
                current_pktcount++;
                if (current_pktcount >= max_per_page) {
                        current_pkt->pNextPacket =
                            (vmeDmaPacket_t *) __get_free_pages(GFP_KERNEL, 0);
                        current_pktcount = 0;
                }
                current_pkt = current_pkt->pNextPacket;
        }

        /* Return to packets list */
        *return_status = 0;
        return (start_pkt);
}


//-----------------------------------------------------------------------------
// Function   : vme_ioctl_dma()
// Inputs     : 
//   User pointer to DMA packet (possibly chained)
//   DMA channel number.
// Outputs    : returns 0 on success, error code on failure.
// Description: 
//   Copy DMA chain into kernel space.
//   Verify validity of chain.
//   Wait, if needed, for DMA channel to be available.
//   Do the DMA operation.
//   Free the DMA channel.
//   Copy the DMA packet with status back to user space.
//   Free resources allocated by this operation.
// Remarks    : 
//    Note, due to complexity, DMA to/from a user space buffer is dealt with 
//    via a separate routine.
// History    : 
//-----------------------------------------------------------------------------
static int vme_ioctl_dma(vmeDmaPacket_t *vme_dma, int channel)
{

	vmeDmaPacket_t *start_pkt;
	int status = 0;

	// Read the (possibly linked) descriptors from the user
	start_pkt = vme_ioctl_setup_dma(vme_dma, &status);
	if (status < 0) {
		return (status);
	}

	start_pkt->channel_number = channel;
	if (start_pkt->byteCount <= 0) {
		vme_ioctl_free_dma(start_pkt);
		return (-EINVAL);
	}

	// Wait for DMA channel to be available.
	if (down_interruptible(&devinuse[channel + VME_MINOR_DMA])) {
		vme_ioctl_free_dma(start_pkt);
		return (-ERESTARTSYS);
	}

	// Do the DMA.
	status = vme_do_dma(start_pkt);

	// Free the DMA channel.
	up(&devinuse[channel + VME_MINOR_DMA]);

	// Copy the result back to the user.
	if (copy_to_user(vme_dma, start_pkt, sizeof(vmeDmaPacket_t))) {
		vme_ioctl_free_dma(start_pkt);
		return (-EFAULT);
	}
	// Free the linked list.
	vme_ioctl_free_dma(start_pkt);
	return (status);
}

//----------------------------------------------------------------------------
//
//  vme_poll()
//  Place holder - this driver function not implemented.
//
//----------------------------------------------------------------------------
static unsigned int vme_poll(struct file *file, poll_table * wait)
{
	return 0;
}

static int vme_mmap(struct file *file, struct vm_area_struct *vma)
{
	int minor;
	int ret = 0;

	if ((!file) || (!vma))
		return -ENXIO;
	minor = MINOR(file->f_dentry->d_inode->i_rdev);

	if ((minor < 0) || (minor > 15))
		return -EINVAL;

	if (minor > 7) {
		/* Inbound windows */
		minor -= 8;

		if (vma->vm_pgoff)
			return -EINVAL;	/* want no offset */

		/*
		 * Lock a read or write against an mmap.
		 */
		down(&in_image_sem[minor]);
		ret = -EAGAIN;
		if (remap_pfn_range(vma,
				    vma->vm_start,
				    in_image_pa[minor] >> PAGE_SHIFT,
				    vma->vm_end - vma->vm_start,
				    vma->vm_page_prot))
			goto mmap_in;
		in_image_mapped[minor] = 1;
		ret = 0;
mmap_in:
		up(&in_image_sem[minor]);
		return ret;
	} else {
		unsigned long vsize = vma->vm_end - vma->vm_start;
		unsigned long psize;

		/* Outbound windows */
		if (out_image_va[minor] == 0)
			return (-EINVAL);

		psize = out_resource[minor].end - out_resource[minor].start + 1;
		psize -= vma->vm_pgoff * PAGE_SIZE;

		if (vsize > psize)
			return -EINVAL;

		/*
		 * Lock a read or write against an mmap.
		 */
		down(&out_image_sem[minor]);
		ret = -EAGAIN;
		vma->vm_page_prot |= _PAGE_NO_CACHE | _PAGE_GUARDED;
		if (io_remap_pfn_range(vma,
			vma->vm_start,
			((out_resource[minor].start -
				vme_pci_bus_mem_base_phys(vme_pci_dev->bus)) >> 
					PAGE_SHIFT) + vma->vm_pgoff, 
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot))
			goto mmap_out;
		out_image_mapped[minor] = 1;
		ret = 0;
mmap_out:
		up(&out_image_sem[minor]);
		return ret;
	}
}

/*
 Function   : vme_open()
 Inputs     : standard Linux device driver open arguments.
 Outputs    : returns 0 on success, error code on failure.
 Description: 
   Verify device is valid,
   If device is exclusive use and already open, return -EBUSY
   Increment device use count.
*/
static int vme_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	int minor_flags;

	/* Check for valid minor num */
	minor_flags = vme_minor_type(minor);
	if (!minor_flags)
		return -ENODEV;

	/* Restrict exclusive use devices to one open */
	if ((minor_flags & DEV_EXCLUSIVE) &&
	    test_and_set_bit(0, &opened[minor]))
		return -EBUSY;
	return (0);
}

/*
 Function   : vme_release()
 Inputs     : standard Linux device driver release arguments.
 Outputs    : returns 0.
 Description: 
    Decrement use count and return.
*/
static int vme_release(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	int minor_flags;

	minor_flags = vme_minor_type(minor);
	if ((minor & VME_MINOR_TYPE_MASK) == VME_MINOR_OUT) {
		int decoder_num;

		decoder_num = minor & 0xF;	/* Get outbound window num */

		if (decoder_num < 8) 
			out_image_mapped[decoder_num] = 0;
		 else 
			in_image_mapped[decoder_num - 8] = 0;
	}
	clear_bit(0, &opened[minor]);

	return 0;
}

//-----------------------------------------------------------------------------
// Function   : vme_read()
// Inputs     : standard linux device driver read() arguments.
// Outputs    : count of bytes read or error code.
// Description: 
//    Sanity check inputs,
//    If register read,  write user buffer from chip registers
//    If VME window read, find the outbound window and read data
//    through the window.  Return number of bytes successfully read.
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static ssize_t vme_read(struct file *file, char *buf, size_t count,
			loff_t * ppos)
{
	int x = 0;
	int decoder_num;
	int okcount = 0;
	char *image_p;
	unsigned int v, numt, remain;
	char *temp = buf;
	unsigned int minor = iminor(file->f_dentry->d_inode);
	unsigned int foffset;

	unsigned char vc;
	unsigned short vs;
	unsigned int vl;
	int minor_flags;
	int image_size;

	minor_flags = vme_minor_type(minor);
	if (!(minor_flags & DEV_RW)) {
		return (-EINVAL);
	}

	if (*ppos > 0xFFFFFFFF) {
		return (-EINVAL);
	}
	foffset = *ppos;

	if (minor == VME_MINOR_REGS) {
		if ((count != 4) || (foffset & (4 - 1))) {	/* word access only */
			return (-EINVAL);
		}
		if (foffset >= 0x1000) {	/* Truncate at end of reg image */
			return (0);
		}
		v = vme_chip_reg_read((unsigned int *)(vmechip_baseaddr +
						       foffset));
		__copy_to_user(temp, &v, 4);
	}

	if ((minor & VME_MINOR_TYPE_MASK) == VME_MINOR_OUT) {

		decoder_num = minor & 0xF;	/* Get outbound window num */

		if (decoder_num < 8) {
			if (out_image_va[decoder_num] == 0) {
				return (-EINVAL);
			}
			if (out_image_mapped[decoder_num])
				return -ENXIO;
			image_size =
			    out_resource[decoder_num].end -
			    out_resource[decoder_num].start + 1;
			if (foffset >= image_size) {
				return (0);
			}
			down(&out_image_sem[decoder_num]);
			if ((foffset + count) > image_size) {
				count = image_size - foffset;
			}
			reads++;
			image_p = (char *)(out_image_va[decoder_num] + foffset);

			// Calc the number of longs we need
			numt = count / 4;
			remain = count % 4;
			for (x = 0; x < numt; x++) {
				vl = *(int *)(image_p);

				// Lets Check for a Bus Error
				if (vme_bus_error_chk(1)) {
					up(&out_image_sem[decoder_num]);
					return (okcount);
				}
				okcount += 4;

				__copy_to_user(temp, &vl, 4);
				image_p += 4;
				temp += 4;
			}

			// Calc the number of Words we need
			numt = remain / 2;
			remain = remain % 2;
			for (x = 0; x < numt; x++) {
				vs = *(short *)(image_p);

				// Lets Check for a Bus Error
				if (vme_bus_error_chk(1)) {
					up(&out_image_sem[decoder_num]);
					return (okcount);
				}
				okcount += 2;

				__copy_to_user(temp, &vs, 2);
				image_p += 2;
				temp += 2;
			}

			for (x = 0; x < remain; x++) {
				vc = readb(image_p);

				// Lets Check for a Bus Error
				if (vme_bus_error_chk(1)) {
					up(&out_image_sem[decoder_num]);
					return (okcount);
				}
				okcount++;

				__copy_to_user(temp, &vc, 1);
				image_p += 1;
				temp += 1;
			}
			up(&out_image_sem[decoder_num]);
		} else {	/* inbound windows */
			decoder_num -= 8;
			if (in_image_ba[decoder_num] == 0)
				return -EINVAL;
			if (in_image_mapped[decoder_num])
				return -ENXIO;
			image_size = in_image_size[decoder_num];
			if (foffset >= image_size) {
				return (0);
			}
			down(&in_image_sem[decoder_num]);
			if ((foffset + count) > image_size) {
				count = image_size - foffset;
			}
			reads++;
			image_p = (char *)(in_image_ba[decoder_num] + foffset);

			// Calc the number of longs we need
			numt = count / 4;
			remain = count % 4;
			for (x = 0; x < numt; x++) {
				vl = *(int *)(image_p);
				__copy_to_user(temp, &vl, 4);
				image_p += 4;
				temp += 4;
			}

			// Calc the number of Words we need
			numt = remain / 2;
			remain = remain % 2;
			for (x = 0; x < numt; x++) {
				vs = *(short *)(image_p);
				__copy_to_user(temp, &vs, 2);
				image_p += 2;
				temp += 2;
			}

			for (x = 0; x < remain; x++) {
				vc = *(char *)(image_p);
				__copy_to_user(temp, &vc, 1);
				image_p += 1;
				temp += 1;
			}
			up(&in_image_sem[decoder_num]);
		}
	}

	if (((minor & VME_MINOR_TYPE_MASK) == VME_MINOR_SLOTS1) ||
	    ((minor & VME_MINOR_TYPE_MASK) == VME_MINOR_SLOTS2)) {
		return (-EINVAL);	/* Not implemented yet */
	}
	*ppos += count;
	return (count);
}

//-----------------------------------------------------------------------------
// Function   : vme_write()
// Inputs     : standard linux device driver write() arguments.
// Outputs    : count of bytes written or error code.
// Description: 
//    Sanity check inputs,
//    If register write,  write VME chip registers with the user data
//    If VME window write, find the outbound window and write user data
//    through the window.  Return number of bytes successfully written.
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static ssize_t vme_write(struct file *file, const char *buf, size_t count,
			 loff_t * ppos)
{
	int x;
	int decoder_num;
	char *image_p;
	int okcount = 0;
	unsigned int numt, remain;
	char *temp = (char *)buf;
	unsigned int minor = iminor(file->f_dentry->d_inode);
	unsigned int foffset;

	unsigned char vc;
	unsigned short vs;
	unsigned int vl;
	int minor_flags;
	int image_size;

	minor_flags = vme_minor_type(minor);
	if (!(minor_flags & DEV_RW)) {
		return (-EINVAL);
	}

	if (*ppos > 0xFFFFFFFF) {
		return (-EINVAL);
	}
	foffset = *ppos;

	writes++;
	if (minor == VME_MINOR_REGS) {
		if ((count != 4) || (foffset & (4 - 1))) {	/* word access only */
			return (-EINVAL);
		}
		if (foffset >= 0x1000) {	/* Truncate at end of reg image */
			return (0);
		}
		__copy_from_user(&vl, temp, 4);
		vme_chip_reg_write((unsigned int *)(vmechip_baseaddr + foffset),
				   vl);
	}

	if ((minor & VME_MINOR_TYPE_MASK) == VME_MINOR_OUT) {

		decoder_num = minor & 0xF;	/* Get outbound window num */
		if (decoder_num < 8) {
			if (out_image_va[decoder_num] == 0) {
				return (-EINVAL);
			}
			if (out_image_mapped[decoder_num])
				return -ENXIO;

			image_size =
			    out_resource[decoder_num].end -
			    out_resource[decoder_num].start;

			if (foffset >= image_size) {
				return (0);
			}
			down(&out_image_sem[decoder_num]);
			if ((foffset + count) > image_size) {
				count = image_size - foffset;
			}
			// Calc the number of longs we need
			numt = count / 4;
			remain = count % 4;
			image_p = (char *)(out_image_va[decoder_num] + foffset);

			for (x = 0; x < numt; x++) {
				__copy_from_user(&vl, temp, 4);
				*(int *)image_p = vl;

				// Lets Check for a Bus Error
				if (vme_bus_error_chk(1)) {
					up(&out_image_sem[decoder_num]);
					return (okcount);
				} else
					okcount += 4;

				image_p += 4;
				temp += 4;
			}

			// Calc the number of Words we need
			numt = remain / 2;
			remain = remain % 2;

			for (x = 0; x < numt; x++) {
				__copy_from_user(&vs, temp, 2);
				*(short *)image_p = vs;

				// Lets Check for a Bus Error
				if (vme_bus_error_chk(1)) {
					up(&out_image_sem[decoder_num]);
					return (okcount);
				} else
					okcount += 2;

				image_p += 2;
				temp += 2;
			}

			for (x = 0; x < remain; x++) {
				__copy_from_user(&vc, temp, 1);
				writeb(vc, image_p);

				// Lets Check for a Bus Error
				if (vme_bus_error_chk(1)) {
					up(&out_image_sem[decoder_num]);
					return (okcount);
				} else
					okcount += 2;

				image_p += 1;
				temp += 1;
			}
			up(&out_image_sem[decoder_num]);
		} else {	/* inbound windows */
			decoder_num -= 8;
			if (in_image_ba[decoder_num] == 0)
				return -EINVAL;
			if (in_image_mapped[decoder_num])
				return -ENXIO;

			image_size = in_image_size[decoder_num];

			if (foffset >= image_size) {
				return (0);
			}
			down(&in_image_sem[decoder_num]);
			if ((foffset + count) > image_size) {
				count = image_size - foffset;
			}
			// Calc the number of longs we need
			numt = count / 4;
			remain = count % 4;
			image_p = (char *)(in_image_ba[decoder_num] + foffset);

			for (x = 0; x < numt; x++) {
				__copy_from_user(&vl, temp, 4);
				*(int *)image_p = vl;
				image_p += 4;
				temp += 4;
			}

			// Calc the number of Words we need
			numt = remain / 2;
			remain = remain % 2;

			for (x = 0; x < numt; x++) {
				__copy_from_user(&vs, temp, 2);
				*(short *)image_p = vs;
				image_p += 2;
				temp += 2;
			}

			for (x = 0; x < remain; x++) {
				__copy_from_user(&vc, temp, 1);
				*(char *)image_p = vc;
				image_p += 1;
				temp += 1;
			}
			up(&in_image_sem[decoder_num]);
		}
	}

	if (((minor & VME_MINOR_TYPE_MASK) == VME_MINOR_SLOTS1) ||
	    ((minor & VME_MINOR_TYPE_MASK) == VME_MINOR_SLOTS2)) {
		return (-EINVAL);	/* Not implemented yet */
	}

	*ppos += count;
	return (count);
}

//-----------------------------------------------------------------------------
// Function   : vme_ioctl()
// Inputs     : 
//    cmd -> the IOCTL command
//    arg -> pointer (if any) to the user ioctl buffer 
// Outputs    : 0 for sucess or errorcode for failure.
// Description: 
//    Copy in, if needed, the user input buffer pointed to by arg.
//    Call vme driver to perform actual ioctl
//    Copy out, if needed, the user output buffer pointed to be arg.
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static int vme_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     unsigned long arg)
{
	unsigned int minor = iminor(inode);
	vmeOutWindowCfg_t vme_out;
	vmeInfoCfg_t vme_info;
	vmeRequesterCfg_t vme_req;
	vmeArbiterCfg_t vme_arb;
	vmeInWindowCfg_t vme_in;
	virqInfo_t vme_irq;
	vmeRmwCfg_t vme_rmw;
	vmeLmCfg_t vme_lm;
	int status;

	ioctls++;

	switch (minor & VME_MINOR_TYPE_MASK) {
	case VME_MINOR_OUT:
		switch (cmd) {
		case VME_IOCTL_GET_OUTBOUND:
			if (copy_from_user
			    (&vme_out, (void *)arg, sizeof(vme_out)))
				return -EFAULT;
			status = vme_get_out_bound(&vme_out);
			if (status != 0)
				return (status);
			if (copy_to_user
			    ((void *)arg, &vme_out, sizeof(vme_out)))
				return -EFAULT;
			break;
		case VME_IOCTL_SET_OUTBOUND:
			if (copy_from_user
			    (&vme_out, (void *)arg, sizeof(vme_out)))
				return -EFAULT;
			if ((vme_out.windowNbr < 0) || (vme_out.windowNbr > 7))
				return -EINVAL;
			if (out_image_mapped[vme_out.windowNbr])
				return -ENXIO;

			status = vme_set_out_bound(&vme_out);
			if (status != 0)
				return (status);
			break;
		default:
			return -EINVAL;
		}
		break;

	case VME_MINOR_DMA:
		switch (cmd) {
		case VME_IOCTL_PAUSE_DMA:
		case VME_IOCTL_CONTINUE_DMA:
		case VME_IOCTL_ABORT_DMA:
		case VME_IOCTL_WAIT_DMA:
			return (-EINVAL);	/* Not supported for now. */
			break;
		case VME_IOCTL_START_DMA:
			status =
			    vme_ioctl_dma((vmeDmaPacket_t *) arg,
					  minor - VME_MINOR_DMA);
			if (status != 0)
				return (status);
			break;
		default:
			return (-EINVAL);
		}
		break;

	case VME_MINOR_MISC:
		switch (minor) {
		case VME_MINOR_CTL:
			switch (cmd) {
			case VME_IOCTL_GET_SLOT_VME_INFO:
				if (copy_from_user
				    (&vme_info, (void *)arg, sizeof(vme_info)))
					return (-EFAULT);
				status = vme_get_slot_info(&vme_info);
				if (status != 0)
					return (status);
				if (copy_to_user
				    ((void *)arg, &vme_info, sizeof(vme_info)))
					return (-EFAULT);
				break;
			case VME_IOCTL_SET_REQUESTOR:
				if (copy_from_user
				    (&vme_req, (void *)arg, sizeof(vme_req)))
					return (-EFAULT);
				status = vme_set_requestor(&vme_req);
				if (status != 0)
					return (status);
				break;
			case VME_IOCTL_GET_REQUESTOR:
				if (copy_from_user
				    (&vme_req, (void *)arg, sizeof(vme_req)))
					return (-EFAULT);
				status = vme_get_requestor(&vme_req);
				if (status != 0)
					return (status);
				if (copy_to_user
				    ((void *)arg, &vme_req, sizeof(vme_req)))
					return (-EFAULT);
				break;
			case VME_IOCTL_SET_CONTROLLER:
				if (copy_from_user
				    (&vme_arb, (void *)arg, sizeof(vme_arb)))
					return (-EFAULT);
				status = vme_set_arbiter(&vme_arb);
				if (status != 0)
					return (status);
				if (copy_to_user
				    ((void *)arg, &vme_arb, sizeof(vme_arb)))
					return (-EFAULT);
				break;
			case VME_IOCTL_GET_CONTROLLER:
				status = vme_get_arbiter(&vme_arb);
				if (status != 0)
					return (status);
				if (copy_to_user
				    ((void *)arg, &vme_arb, sizeof(vme_arb)))
					return (-EFAULT);
				break;
				/* For legacy purposes, VME_IOCTL_GET_INBOUND 
				 * and VME_IOCTL_SET_INBOUND are done here
				 */
			case VME_IOCTL_SET_INBOUND:
				if (copy_from_user
				    (&vme_in, (void *)arg, sizeof(vme_in)))
					return -EFAULT;
				if ((vme_in.windowNbr < 0) ||
					 (vme_in.windowNbr > 7))
					return -EINVAL;
				if (in_image_mapped[vme_in.windowNbr])
					return -ENXIO;
				status = vme_set_in_bound(&vme_in);
				if (status != 0)
					return (status);
				break;
			case VME_IOCTL_GET_INBOUND:
				if (copy_from_user
				    (&vme_in, (void *)arg, sizeof(vme_in)))
					return -EFAULT;
				status = vme_get_in_bound(&vme_in);
				if (status != 0)
					return (status);
				if (copy_to_user
				    ((void *)arg, &vme_in, sizeof(vme_in)))
					return (-EFAULT);
				break;
			case VME_IOCTL_GENERATE_IRQ:

				if (copy_from_user
				    (&vme_irq, (void *)arg, sizeof(vme_irq)))
					return (-EFAULT);

				// Obtain access to VIRQ generator.
				if (down_interruptible(&virq_inuse))
					return (-ERESTARTSYS);
				status = vme_generate_irq(&vme_irq);
				up(&virq_inuse);	// Release VIRQ generator.

				if (status != 0)
					return (status);
				if (copy_to_user
				    ((void *)arg, &vme_irq, sizeof(vme_irq)))
					return (-EFAULT);
				break;

			case VME_IOCTL_GET_IRQ_STATUS:
				if (copy_from_user
				    (&vme_irq, (void *)arg, sizeof(vme_irq)))
					return (-EFAULT);
				status = vme_get_irq_status(&vme_irq);
				if (status != 0)
					return (status);
				if (copy_to_user
				    ((void *)arg, &vme_irq, sizeof(vme_irq)))
					return (-EFAULT);
				break;
			case VME_IOCTL_CLR_IRQ_STATUS:
				if (copy_from_user
				    (&vme_irq, (void *)arg, sizeof(vme_irq)))
					return (-EFAULT);
				status = vme_clr_irq_status(&vme_irq);
				if (status != 0)
					return (status);
				break;
			default:
				return (-EINVAL);
			}
			break;
		case VME_MINOR_REGS:
			return (-EINVAL);
		case VME_MINOR_RMW:
			if (copy_from_user
			    (&vme_rmw, (void *)arg, sizeof(vme_rmw)))
				return (-EFAULT);
			switch (cmd) {
			case VME_IOCTL_DO_RMW:
				status = vme_do_rmw(&vme_rmw);
				if (status != 0)
					return (status);
				break;
			default:
				return (-EINVAL);
			}
			if (copy_to_user
			    ((void *)arg, &vme_rmw, sizeof(vme_rmw)))
				return (-EFAULT);
			break;

		case VME_MINOR_LM:
			if (copy_from_user
			    (&vme_lm, (void *)arg, sizeof(vme_lm)))
				return (-EFAULT);
			switch (cmd) {
			case VME_IOCTL_SETUP_LM:
				status = vme_setup_lm(&vme_lm);
				if (status != 0)
					return (status);
				break;
			case VME_IOCTL_WAIT_LM:
				status = vme_wait_lm(&vme_lm);
				if (status != 0)
					return (status);
				break;
			default:
				return (-EINVAL);
			}
			if (copy_to_user((void *)arg, &vme_lm, sizeof(vme_lm)))
				return (-EFAULT);
			break;

		default:
			return (-EINVAL);
		}
		break;

#if	0			// interboard data channels not implemented yet
	case VME_MINOR_SLOTS1:
	case VME_MINOR_SLOTS3:
		break;
#endif
	default:
		return (-EINVAL);
	}			// masked minor
	return (status);
}

//-----------------------------------------------------------------------------
// Function   : find_vme_bridge()
// Inputs     : void
// Outputs    : failed flag.
// Description: 
//    Search for a supported VME bridge chip.  
//    Initialize VME config registers.
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static struct pci_dev *find_vme_bridge(void)
{
	struct pci_dev *vme_pci_dev = NULL;

	if ((vme_pci_dev = pci_find_device(PCI_VENDOR_ID_TUNDRA,
					   PCI_DEVICE_ID_TUNDRA_CA91C042,
					   vme_pci_dev))) {
		vmechip_devid = PCI_DEVICE_ID_TUNDRA_CA91C042;
	}
	if (vme_pci_dev == NULL) {
		if ((vme_pci_dev = pci_find_device(PCI_VENDOR_ID_TUNDRA,
						   PCI_DEVICE_ID_TUNDRA_TSI148,
						   vme_pci_dev))) {
			vmechip_devid = PCI_DEVICE_ID_TUNDRA_TSI148;
		}
	}
	if (vme_pci_dev == NULL) {
		printk(KERN_ERR "vmemod: VME bridge not found on PCI Bus.\n");
		return (vme_pci_dev);
	}
	// read revision.
	pci_read_config_dword(vme_pci_dev, PCI_CLASS_REVISION,
			      &vmechip_revision);
	printk(KERN_DEBUG "vmemod: PCI_CLASS_REVISION = %08x\n",
	       vmechip_revision);
	vmechip_revision &= 0xFF;

	// Unless user has already specified it, 
	// determine the VMEchip IRQ number.
	if (vmechip_irq == 0) {
		vmechip_irq = vme_pci_dev->irq;
		printk(KERN_DEBUG "vme: irq  = %08x\n", vmechip_irq);
		vmechip_irq &= 0x000000FF;	// Only a byte in size
		if (vmechip_irq == 0)
			vmechip_irq = 0x00000050;	// Only a byte in size
	}
	if ((vmechip_irq == 0) || (vmechip_irq == 0xFF)) {
		printk(KERN_ERR "vmemod: Bad VME IRQ number: %02x\n",
		       vmechip_irq);
		return (NULL);
	}

	return (vme_pci_dev);
}

//-----------------------------------------------------------------------------
// Function   : map_in_vme_bridge()
// Inputs     : void
// Outputs    : failed flag.
// Description: 
//    Map VME chip registers in.  Perform basic sanity check of chip ID.
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static int map_in_vme_bridge(struct pci_dev *vme_pci_dev)
{
	unsigned int temp, ba;

	// Setup VME Bridge Register Space
	// This is a 4k wide memory area that need to be mapped into the kernel
	// virtual memory space so we can access it.
	ba = pci_resource_start(vme_pci_dev, 0);
	vmechip_baseaddr = ioremap(ba, 4096);
	if (!vmechip_baseaddr) {
		printk("ioremap failed to map VMEchip to Kernel Space.\n");
		return 1;
	}
	// Check to see if the Mapping Worked out
	temp = readl(vmechip_baseaddr) & 0x0000FFFF;
	if (temp != PCI_VENDOR_ID_TUNDRA) {
		printk("VME Chip Failed to Return PCI_ID in Memory Map.\n");
		iounmap(vmechip_baseaddr);
		return 1;
	}
	return (0);
}

//-----------------------------------------------------------------------------
// Function   : vme_driver_setup_windows()
// Inputs     : void
// Outputs    : void
// Description: 
//    Creates initial inbound & outbound VME windows.
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static void vme_driver_setup_windows(void)
{
	vmeOutWindowCfg_t vme_driver_window;

	// Setup the outbound CRCSR window for use by the driver.
	memset(&vme_driver_window, 0, sizeof(vme_driver_window));
	vme_driver_window.windowNbr = 7;
	vme_driver_window.windowEnable = 1;
	vme_driver_window.windowSizeL = (512 * 1024) * 32;
	vme_driver_window.xlatedAddrL = 0;
	vme_driver_window.wrPostEnable = 1;
	vme_driver_window.addrSpace = VME_CRCSR;
	vme_driver_window.maxDataWidth = VME_D32;
	vme_driver_window.xferProtocol = 0;
	vme_driver_window.dataAccessType = VME_DATA;
	vme_driver_window.userAccessType = VME_SUPER;
	vme_set_out_bound(&vme_driver_window);

#if 0
	/* Setup the inbound windows */
	if (vme_slotnum > 0) {
        	vmeInWindowCfg_t vme_in_window;
        	unsigned int x;

		memset(&vme_in_window, 0, sizeof(vme_in_window));
		if (in_image_size[x] != 0) {
			memset(in_image_ba[x], 0, in_image_size[x]);
			vme_flush_range(in_image_ba[x], in_image_ba[x] + in_image_size[x]);
			vme_in_window.windowNbr = x;
			vme_in_window.windowEnable = 1;
			vme_in_window.windowSizeL = in_image_size[x];
			vme_in_window.pciAddrL = (unsigned int)in_image_pa[x];
			vme_in_window.vmeAddrL =
			    0x80000000 + (vme_slotnum * 0x02000000) + (0x400000 * x);
			vme_in_window.wrPostEnable = 1;
			vme_in_window.addrSpace = VME_A32;
			vme_in_window.xferProtocol =
				    VME_SCT | VME_BLT | VME_MBLT | VME_2eVME | VME_2eSST;
			/*  2eSST Transfer Rate */
			vme_in_window.xferRate2esst = VME_SST320;
			vme_in_window.dataAccessType = VME_DATA | VME_PROG;
			vme_in_window.userAccessType = VME_SUPER | VME_USER;
			vme_set_in_bound(&vme_in_window);

		}
	}
#endif
}

void __init vmemod_setup_options(char *cmdline)
{
	char *options;

	/* 
	 * Look for vme=option on command line 
	 * Accept something like: 
	 *      vme=vme_slotnum=<num>
	 *      vme=vme_slotnum=<num>,vmechip_irq=<num>
	 */

	options = strstr(cmdline, "vme=");
	if (!options)
		return;
	options = strchr(options, '=') + 1;
	if (options && !strncmp(options, "vme_slotnum=", 12)) {
		vme_slotnum = simple_strtoul(options + 12, NULL, 0);
		options = strchr(options, ',');
		if (!options)
			return;
		if (!strncmp(options + 1, "vmechip_irq=", 12)) {
			vmechip_irq = simple_strtoul(options + 13, NULL, 0);
		}
	} else if (options && !strncmp(options, "vmechip_irq=", 12)) {
		vmechip_irq = simple_strtoul(options + 12, NULL, 0);
		options = strchr(options, ',');
		if (!options)
			return;
		if (!strncmp(options + 1, "vme_slotnum=", 12)) {
			vme_slotnum = simple_strtoul(options + 13, NULL, 0);
		}
	}
	return;
}

//-----------------------------------------------------------------------------
// Function   : cleanup_module
// Inputs     : void
// Outputs    : void
// Description: 
//    Initialize VME chip to quiescent state.
//    Free driver resources.
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static void __exit vme_bridge_exit(void)
{
	int x;

	// Quiesce the hardware.
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		uni_shutdown();
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		tsi148_shutdown();
		break;
	}

	// Free  & unmap resources
	free_irq(vmechip_irq, vmechip_baseaddr);
	iounmap(vmechip_baseaddr);
	vme_free_buffers();

	for (x = 0; x < 8; x++) {
		if (out_image_va[x] != 0) {
			iounmap((void *)out_image_va[x]);
			out_image_va[x] = 0;
			out_image_mapped[x] = 0;
		}
		if (out_resource[x].end - out_resource[x].start) {
			iounmap((void *)out_image_va[x]);
			if (out_resource[x].name != NULL)
				kfree(out_resource[x].name);
			release_resource(&out_resource[x]);
			memset(&out_resource[x], 0, sizeof(struct resource));
		}
	}

	unregister_proc();
	unregister_chrdev(VME_MAJOR, "vme");
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 0));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 1));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 2));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 3));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 4));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 5));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 6));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 7));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 8));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 9));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 10));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 11));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 12));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 13));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 14));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 15));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 16));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 17));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 32));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 33));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 34));
	class_device_destroy(vme_class, MKDEV(VME_MAJOR, 35));
	class_destroy(vme_class);
}

//-----------------------------------------------------------------------------
// Function   : init_module()
// Inputs     : void
// Outputs    : module initialization fail flag
// Description: 
//  top level module initialization.
//  find VME bridge
//  initialize VME bridge chip.
//  Sanity check bridge chip.
//  allocate and initialize driver resources.
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------

static int __init vme_bridge_init(void)
{
	int x;
	unsigned int temp;
	struct resource pcimemres;
#ifdef CONFIG_MVME5500
	struct pci_dev *vme_ptp_pci_dev = NULL;
#endif
#ifdef CONFIG_MVME6100
	void __iomem *mv64360;
	u32  pci_acc_cntl_reg;
#endif

	printk("Tundra TSI148/CA91C042 PCI-VME Bridge Driver %s\n", Version);

	// find VME bridge
	vme_pci_dev = find_vme_bridge();
	if (vme_pci_dev == NULL) {
		return -ENODEV;
	}
#ifdef CONFIG_MVME6100
#undef	DISABLE_MV64360_SNOOP_WORKAROUND 
#ifdef	DISABLE_MV64360_SNOOP_WORKAROUND
	// This is ugly but needed for now.  Disable snoop at 64360.
	if (vmechip_devid == PCI_DEVICE_ID_TUNDRA_TSI148) {
		mv64360 = ioremap(CONFIG_MV64X60_NEW_BASE, 0x100000);
		if (!mv64360) {
			printk("ioremap failed to map MV64360 regs.\n");
			return -ENOMEM;
		}
		pci_acc_cntl_reg = readl(mv64360 + 
			MV64x60_PCI0_ACC_CNTL_0_BASE_LO);
	        if (pci_acc_cntl_reg & 0x00000001) {	
			pci_acc_cntl_reg = MV64360_PCI_ACC_CNTL_SNOOP_NONE |
			MV64360_PCI_ACC_CNTL_SWAP_NONE |
			MV64360_PCI_ACC_CNTL_MBURST_128_BYTES |
			MV64360_PCI_ACC_CNTL_RDSIZE_256_BYTES | 1;
			writel(pci_acc_cntl_reg,
				mv64360 + MV64x60_PCI0_ACC_CNTL_0_BASE_LO);
		}
		pci_acc_cntl_reg = readl(mv64360 + 
			MV64x60_PCI0_ACC_CNTL_1_BASE_LO);
	        if (pci_acc_cntl_reg & 0x00000001) {	
			pci_acc_cntl_reg = MV64360_PCI_ACC_CNTL_SNOOP_NONE |
			MV64360_PCI_ACC_CNTL_SWAP_NONE |
			MV64360_PCI_ACC_CNTL_MBURST_128_BYTES |
			MV64360_PCI_ACC_CNTL_RDSIZE_256_BYTES | 1;
			writel(pci_acc_cntl_reg,
				mv64360 + MV64x60_PCI0_ACC_CNTL_1_BASE_LO);
		}
		pci_acc_cntl_reg = readl(mv64360 + 
			MV64x60_PCI0_ACC_CNTL_2_BASE_LO);
	        if (pci_acc_cntl_reg & 0x00000001) {	
			pci_acc_cntl_reg = MV64360_PCI_ACC_CNTL_SNOOP_NONE |
			MV64360_PCI_ACC_CNTL_SWAP_NONE |
			MV64360_PCI_ACC_CNTL_MBURST_128_BYTES |
			MV64360_PCI_ACC_CNTL_RDSIZE_256_BYTES | 1;
			writel(pci_acc_cntl_reg,
				mv64360 + MV64x60_PCI0_ACC_CNTL_2_BASE_LO);
		}
		pci_acc_cntl_reg = readl(mv64360 + 
			MV64x60_PCI0_ACC_CNTL_3_BASE_LO);
	        if (pci_acc_cntl_reg & 0x00000001) {	
			pci_acc_cntl_reg = MV64360_PCI_ACC_CNTL_SNOOP_NONE |
			MV64360_PCI_ACC_CNTL_SWAP_NONE |
			MV64360_PCI_ACC_CNTL_MBURST_128_BYTES |
			MV64360_PCI_ACC_CNTL_RDSIZE_256_BYTES | 1;
			writel(pci_acc_cntl_reg,
				mv64360 + MV64x60_PCI0_ACC_CNTL_3_BASE_LO);
		}
		vme_sync_data();
		iounmap(mv64360);
	}
#endif
#endif

	// Get parent PCI resource & verify enough space is available.
	memset(&pcimemres, 0, sizeof(pcimemres));
	pcimemres.flags = IORESOURCE_MEM;
	vmepcimem = pci_find_parent_resource(vme_pci_dev, &pcimemres);
	if (vmepcimem == 0) {
		printk(KERN_ERR
		       "vmemod: cannot get VME parent device PCI resource\n");
		return -ENODEV;
	}
#ifdef CONFIG_MVME5500
#define	VME_BEHIND_BRIDGE_WORKAROUND 0
#ifdef	VME_BEHIND_BRIDGE_WORKAROUND
	// If the VME bridge is behind a PCI bridge, only a meg of PCI MEM space
	// has been allocated.  Bump it up.  This is ugly too but is needed
	// esp on the MVME5500.
	if ((vmepcimem->end - vmepcimem->start + 1) == 0x100000) {
		// find bridge VME is under info
		vme_ptp_pci_dev = vme_pci_dev->bus->self;
		pci_write_config_word(vme_ptp_pci_dev, PCI_MEMORY_BASE, 0x8000);
		vmepcimem->start = 0x80000000;
	}
#endif
#endif

#ifndef CONFIG_MVME7100
	if ((vmepcimem->end - vmepcimem->start) < 0x2000000) {
		printk("Not enough PCI memory space available %08x\n",
		       (unsigned int)(vmepcimem->end - vmepcimem->start));
		return -ENOMEM;
	}
#endif
	// Map in VME Bridge registers.
	if (map_in_vme_bridge(vme_pci_dev)) {
		return -ENODEV;
	}
	// Unless, user has already specified it, 
	// determine the VME slot that we are in.
	if (vme_slotnum == -1) {
		vme_slotnum = vme_get_slot_num();
	}
	if (vme_slotnum <= 0) {
		printk(KERN_ERR "Bad VME slot #%02d\n", vme_slotnum);
		iounmap(vmechip_baseaddr);
		return -ENODEV;
	}
	// Initialize wait queues & mutual exclusion flags
	init_waitqueue_head(&dma_queue[0]);
	init_waitqueue_head(&dma_queue[1]);
	init_waitqueue_head(&lm_queue);
	init_waitqueue_head(&mbox_queue);
	for (x = 0; x < 8; x++) {
		init_waitqueue_head(&vmeint_queue[x]);
	}
	sema_init(&virq_inuse, 1);
	for (x = 0; x < MAX_MINOR + 1; x++) {
		sema_init(&devinuse[x], 1);
	}
	for (x = 0; x < 8; x++) {
		init_MUTEX(&in_image_sem[x]);
	}
	for (x = 0; x < 8; x++) {
		init_MUTEX(&out_image_sem[x]);
	}

	// allocate buffers needed by the driver.
	if (vme_alloc_buffers() != 0)
		return -ENODEV;

	// Initialize the interboard data structure.
	if (vmechip_interboard_data != NULL) {
		vme_init_interboard_data(vmechip_interboard_data);
	}
	// Display VME information  
	printk(KERN_DEBUG
	       "Vendor = %04X  Device = %04X  Revision = %02X\n",
	       vme_pci_dev->vendor, vme_pci_dev->device, vmechip_revision);
	printk(KERN_DEBUG "Class = %08X\n", vme_pci_dev->class);

	pci_read_config_dword(vme_pci_dev, PCI_CACHE_LINE_SIZE, &temp);
	printk(KERN_DEBUG "Misc0 = %08X\n", temp);
	printk(KERN_DEBUG "Irq = %04X\n", vmechip_irq);
	if (vme_slotnum == -1) {
		printk("VME slot number: unknown\n");
	} else {
		printk("VME slot number: %d\n", vme_slotnum);
	}

	// Initialize chip registers and register module.
	if (!vme_init(vmechip_interboard_data)) {
		printk(KERN_ERR "vmemod: VME Chip Initialization failed.\n");
		iounmap(vmechip_baseaddr);
		vme_free_buffers();
		return -ENODEV;
	}
	if (register_chrdev(VME_MAJOR, "vme", &vme_fops)) {
		printk(KERN_ERR "vmemod: Unable to get major number.\n");
		iounmap(vmechip_baseaddr);
		vme_free_buffers();
		return -ENODEV;
	}
	vme_class = class_create(THIS_MODULE, "vme");
	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 0), NULL, "vme_m0");
	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 1), NULL, "vme_m1");
	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 2), NULL, "vme_m2");
	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 3), NULL, "vme_m3");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 4), NULL, "vme_m4");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 5), NULL, "vme_m5");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 6), NULL, "vme_m6");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 7), NULL, "vme_m7");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 8), NULL, "vme_s0");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 9), NULL, "vme_s1");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 10), NULL, "vme_s2");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 11), NULL, "vme_s3");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 12), NULL, "vme_s4");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 13), NULL, "vme_s5");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 14), NULL, "vme_s6");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 15), NULL, "vme_s7");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 16), NULL, "vme_dma0");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 17), NULL, "vme_dma1");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 32), NULL, "vme_ctl");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 33), NULL, "vme_regs");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 34), NULL, "vme_rmw0");

	class_device_create(vme_class, NULL, MKDEV(VME_MAJOR, 35), NULL, "vme_lm0");

	if (vme_syscon == 0) {
		printk("Board is not the VME system controller\n");
	}
	if (vme_syscon == 1) {
		printk("Board is the VME system controller\n");
	}

	register_proc();

	// Get time base speed.  Needed to support DMA throughput measurements
	tb_speed = tb_ticks_per_jiffy * HZ;

	// Open up the default VME windows.
	vme_driver_setup_windows();

	return 0;
}

module_init(vme_bridge_init)
module_exit(vme_bridge_exit)
