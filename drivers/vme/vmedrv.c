/*
 * vmedrv.c
 *
 * Provided generic (non bridge specific) entry-points. The entry-points
 * call CA91C042 or TSI148 specific routines and return the result. 
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
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/poll.h>

#include "vmedrv.h"
#include "ca91c042.h"

extern struct resource *vmepcimem;
extern struct pci_dev *vme_pci_dev;

/* Global VME controller information */
int vmechip_irq;		// PCI irq
int vmechip_irq_overhead_ticks;	// Interrupt overhead
int vmechip_devid;		// PCI devID
int vmeparent_devid;		// VME bridge parent ID
int vmechip_revision;		// PCI revision
int vmechip_bus;		// PCI bus number.
void __iomem *vmechip_baseaddr;		// virtual address of chip registers
int vme_slotnum = -1;		// VME slot num (-1 = unknown)
int vme_syscon = -1;		// VME sys controller (-1 = unknown)

/* address of board data */
extern struct vmeSharedData *vmechip_interboard_data;

extern struct resource out_resource[VME_MAX_WINDOWS];
unsigned int out_image_va[VME_MAX_WINDOWS];	// Base virtual address

/*
 * External functions 
 */
extern int uni_init(void *);
extern void uni_shutdown(void);
extern int uni_set_arbiter(vmeArbiterCfg_t * vmeArb);
extern int uni_get_arbiter(vmeArbiterCfg_t * vmeArb);
extern int uni_set_requestor(vmeRequesterCfg_t * vmeReq);
extern int uni_get_requestor(vmeRequesterCfg_t * vmeReq);
extern int uni_set_in_bound(vmeInWindowCfg_t * vmeIn);
extern int uni_get_in_bound(vmeInWindowCfg_t * vmeIn);
extern int uni_set_out_bound(vmeOutWindowCfg_t * vmeOut);
extern int uni_get_out_bound(vmeOutWindowCfg_t * vmeOut);
extern int uni_setup_lm(vmeLmCfg_t * vmeLm);
extern int uni_wait_lm(vmeLmCfg_t * vmeLm);
extern int uni_do_rmw(vmeRmwCfg_t * vmeRmw);
extern int uni_do_dma(vmeDmaPacket_t * vmeDma);
extern int uni_generate_irq(virqInfo_t *);
extern int uni_bus_error_chk(int);

extern int tempe_init(void *);
extern void tempe_shutdown(void);
extern int tempe_set_arbiter(vmeArbiterCfg_t * vmeArb);
extern int tempe_get_arbiter(vmeArbiterCfg_t * vmeArb);
extern int tempe_set_requestor(vmeRequesterCfg_t * vmeReq);
extern int tempe_get_requestor(vmeRequesterCfg_t * vmeReq);
extern int tempe_set_in_bound(vmeInWindowCfg_t * vmeIn);
extern int tempe_get_in_bound(vmeInWindowCfg_t * vmeIn);
extern int tempe_set_out_bound(vmeOutWindowCfg_t * vmeOut);
extern int tempe_get_out_bound(vmeOutWindowCfg_t * vmeOut);
extern int tempe_setup_lm(vmeLmCfg_t * vmeLm);
extern int tempe_wait_lm(vmeLmCfg_t * vmeLm);
extern int tempe_do_rmw(vmeRmwCfg_t * vmeRmw);
extern int tempe_do_dma(vmeDmaPacket_t * vmeDma);
extern int tempe_generate_irq(virqInfo_t *);
extern int tempe_bus_error_chk(int);

extern wait_queue_head_t vmeint_queue[];

int vme_irqlog[8][0x100];

//----------------------------------------------------------------------------
//  Some kernels don't supply pci_bus_mem_base_phys()... this subroutine
//  provides that functionality.
//----------------------------------------------------------------------------

unsigned long vme_pci_bus_mem_base_phys(struct pci_bus *bus)
{
	struct pci_controller *hose;	
	
	hose = bus->sysdata;
	if (!hose)
		return 0;
	return hose->pci_mem_offset;
}

//----------------------------------------------------------------------------
//  vme_sync_data()
//    Ensure completion of all previously issued loads/stores
//----------------------------------------------------------------------------
void vme_sync_data(void)
{
	asm("sync");		// PPC specific.
	asm("eieio");
}

//----------------------------------------------------------------------------
//  vme_flush_line(void *ptr)
//    Flush & invalidate a cache line 
//----------------------------------------------------------------------------
extern void flush_dcache_range(unsigned long, unsigned long );

#ifdef CONFIG_MVME6100
void vme_flush_line(unsigned long start)
{
	flush_dcache_range(start, start + L1_CACHE_BYTES);
}
#else
void vme_flush_line(unsigned long start) { }
#endif

//----------------------------------------------------------------------------
//  vme_flush_range()
//    Flush & invalidate a range of memory
//----------------------------------------------------------------------------

#ifdef CONFIG_MVME6100
void vme_flush_range(unsigned long start, unsigned long end)
{
	flush_dcache_range(start, end);
}
#else
void vme_flush_range(unsigned long start, unsigned long end) {}
#endif

//----------------------------------------------------------------------------
//  vme_bus_error_chk()
//     Return 1 if VME bus error occured, 0 otherwise.
//     Optionally clear VME bus error status.
//----------------------------------------------------------------------------
int vme_bus_error_chk(int clrflag)
{
	vme_sync_data();
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_bus_error_chk(clrflag));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_bus_error_chk(clrflag));
		break;
	}
	return (0);
}

//----------------------------------------------------------------------------
//  vme_get_slot_num
//    Determine the slot number that we are in.  Uses CR/CSR base 
//    address reg if it contains a valid value.  If not, obtains
//    slot number from board level register.
//----------------------------------------------------------------------------
int vme_get_slot_num(void)
{
	int slotnum = 0;

	// get slot num from VME chip reg if it is set.
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		slotnum = readl(vmechip_baseaddr + VCSR_BS);
		slotnum = (slotnum >> 27) & 0x1F;
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		slotnum = *(int *)(vmechip_baseaddr + 0x23C);
		slotnum = slotnum & 0x1F;
		return (slotnum);
		break;
	}

	// Following is board specific!!

#ifdef	MCG_MVME2600
	if (slotnum == 0)
		slotnum = ~inb(0x1006) & 0x1F;	// Genesis2 slot register
#endif
#ifdef	MCG_MVME2400
	if (slotnum == 0)
		slotnum = ~inb(0x1006) & 0x1F;	// Genesis2 slot register
#endif
#ifdef	MCG_MVME2300
	if (slotnum == 0)
		slotnum = ~inb(0x1006) & 0x1F;	// Genesis2 slot register
#endif

	return (slotnum);
}

//----------------------------------------------------------------------------
//  vme_get_out_bound
//    Return attributes of an outbound window.
//----------------------------------------------------------------------------
int vme_get_out_bound(vmeOutWindowCfg_t * vmeOut)
{
	int window_number;

	window_number = vmeOut->windowNbr;
	memset(vmeOut, 0, sizeof(vmeOutWindowCfg_t));
	vmeOut->windowNbr = window_number;
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_get_out_bound(vmeOut));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_get_out_bound(vmeOut));
		break;
	}
	return (0);
}

//----------------------------------------------------------------------------
//  vme_set_out_bound
//    Set attributes of an outbound window.
//----------------------------------------------------------------------------
int vme_set_out_bound(vmeOutWindowCfg_t * vmeOut)
{
	int window_number = vmeOut->windowNbr;
	unsigned int existing_size;
	char *res_name = NULL;

        if (vmeOut->windowNbr > 7) 
                return (-EINVAL);

	if (vmepcimem == 0)
		return (-ENOMEM);

	// Allocate and map PCI memory space for this window.
	existing_size =
	    out_resource[window_number].end - out_resource[window_number].start;

	if ((vmeOut->windowEnable == 0) || 
			(existing_size != vmeOut->windowSizeL)) {
		if (existing_size != 0) {
			if (out_image_va[window_number] != 0) 
				iounmap((char *)out_image_va[window_number]);
			if (out_resource[window_number].name != NULL)
				kfree(out_resource[window_number].name);
			release_resource(&out_resource[window_number]);
			memset(&out_resource[window_number], 0, sizeof(struct resource));
		}
	}

	if ((vmeOut->windowEnable) && 
			(existing_size != vmeOut->windowSizeL)) {

		if (vmeOut->windowSizeL == 0)
			return (-EINVAL);

		res_name = kmalloc(32, GFP_KERNEL);
		if (!res_name)
			printk(KERN_ERR "vmedrv: kmalloc() failed!\n");
		else
			sprintf(res_name, "VME Window %01x", window_number);
		out_resource[window_number].name = (char *)res_name;
		out_resource[window_number].start = 0;
		out_resource[window_number].end = vmeOut->windowSizeL;
#ifdef CONFIG_MVME7100
		out_resource[window_number].flags = IORESOURCE_MEM | IORESOURCE_PREFETCH;
#else
		out_resource[window_number].flags = IORESOURCE_MEM;
#endif

		if (pci_bus_alloc_resource(vme_pci_dev->bus,
				   &out_resource[window_number],
				   vmeOut->windowSizeL,
				   vmeOut->windowSizeL,
				   PCIBIOS_MIN_MEM, 
#ifdef CONFIG_MVME7100
				   IORESOURCE_PREFETCH,
#else
				   0, 
#endif
				   NULL, 
			           NULL)) {
			printk(KERN_ERR
			       "vmedrv: Failed to allocate mem resource for window %d size 0x%x start 0x%x\n",
			       window_number, vmeOut->windowSizeL,
			       out_resource[window_number].start);
			if (out_resource[window_number].name != NULL)
				kfree(out_resource[window_number].name);
			memset(&out_resource[window_number], 0, sizeof(struct resource));
			return (-ENOMEM);
		}

		out_image_va[window_number] = (int)ioremap(out_resource[window_number].start,
				 vmeOut->windowSizeL);
		if (out_image_va[window_number] == 0) {
			printk(KERN_ERR
		       	"vmedrv: No memory for outbound window\n");
			if (out_resource[window_number].name != NULL)
				kfree(out_resource[window_number].name);
			release_resource(&out_resource[window_number]);
			memset(&out_resource[window_number], 0, sizeof(struct resource));
			return (-ENOMEM);
		}
		vmeOut->pciBusAddrL = out_resource[window_number].start
	    		- vme_pci_bus_mem_base_phys(vme_pci_dev->bus);
	}

	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_set_out_bound(vmeOut));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_set_out_bound(vmeOut));
		break;
	}
	return (-ENODEV);
}

/*
 *  vme_get_in_bound
 *    Return attributes of an inbound window.
 */

int vme_get_in_bound(vmeInWindowCfg_t * vmeIn)
{
	int window_number;

	window_number = vmeIn->windowNbr;
	memset(vmeIn, 0, sizeof(vmeInWindowCfg_t));
	vmeIn->windowNbr = window_number;
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_get_in_bound(vmeIn));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_get_in_bound(vmeIn));
		break;
	}
	return (-ENODEV);
}

//----------------------------------------------------------------------------
//  vme_set_in_bound
//    Set attributes of an inbound window.
//----------------------------------------------------------------------------
int vme_set_in_bound(vmeInWindowCfg_t * vmeIn)
{
	int status = -ENODEV;

	// Mask off VME address bits per address space.
	switch (vmeIn->addrSpace) {
	case VME_A16:
		vmeIn->vmeAddrU = 0;
		vmeIn->vmeAddrL &= 0xFFFF;
		break;
	case VME_A24:
	case VME_CRCSR:
		vmeIn->vmeAddrU = 0;
		vmeIn->vmeAddrL &= 0xFFFFFF;
		break;
	case VME_A32:
		vmeIn->vmeAddrU = 0;
		break;
	case VME_A64:
	case VME_USER1:
	case VME_USER2:
	case VME_USER3:
	case VME_USER4:
		break;
	}

	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		status = uni_set_in_bound(vmeIn);
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		status = tempe_set_in_bound(vmeIn);
		break;
	}

	if (!status) {
		vmechip_interboard_data->inBoundVmeSize[vmeIn->windowNbr] =
		    vmeIn->windowSizeL;
		vme_flush_line((unsigned long)(&vmechip_interboard_data->
			       inBoundVmeSize[vmeIn->windowNbr]));

		vmechip_interboard_data->inBoundVmeAddrHi[vmeIn->windowNbr] =
		    vmeIn->vmeAddrU;
		vme_flush_line((unsigned long)(&vmechip_interboard_data->
			       inBoundVmeAddrHi[vmeIn->windowNbr]));

		vmechip_interboard_data->inBoundVmeAddrLo[vmeIn->windowNbr] =
		    vmeIn->vmeAddrL;
		vme_flush_line((unsigned long)(&vmechip_interboard_data->
			       inBoundVmeAddrLo[vmeIn->windowNbr]));

		vmechip_interboard_data->inBoundVmeAM[vmeIn->windowNbr] =
		    vmeIn->addrSpace;
		vme_flush_line((unsigned long)(&vmechip_interboard_data->
			       inBoundVmeAM[vmeIn->windowNbr]));
	}

	return (status);
}

//----------------------------------------------------------------------------
//  vme_do_dma()
//     Perform the requested DMA operation.
//     Caller must assure exclusive access to the DMA channel.  
//----------------------------------------------------------------------------
int vme_do_dma(vmeDmaPacket_t * vmeDma)
{
	int retval = -ENODEV;

	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		retval = uni_do_dma(vmeDma);
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		retval = tempe_do_dma(vmeDma);
		break;
	}

	return (retval);
}

//----------------------------------------------------------------------------
//  vme_get_slot_info()
//     Obtain information about the board (if any) that is present in
//     a specified VME bus slot.   
//
//     (Note VME slot numbers start at 1, not 0).
//     Depends on board mapping in CS/CSR space.  Boards which do not 
//     support or are not configured to respond in CS/CSR space will 
//     not be detected by this routine.
//----------------------------------------------------------------------------
int vme_get_slot_info(vmeInfoCfg_t * vmeInfo)
{
	int slot_of_interest;
	unsigned int temp;
	int i;

	struct vmeSharedData *remote_data;
	unsigned int *remote_csr;

	slot_of_interest = vmeInfo->vmeSlotNum;
	memset(vmeInfo, 0, sizeof(vmeInfoCfg_t));

	if (slot_of_interest > 21) {
		return (-EINVAL);
	}
	// Fill in information for our own board.
	if ((slot_of_interest == 0) || (slot_of_interest == vme_slotnum)) {
		vmeInfo->vmeSlotNum = vme_slotnum;
		vmeInfo->boardResponded = 1;
		vmeInfo->sysConFlag = vme_syscon;
		vmeInfo->vmeControllerID =
		    PCI_VENDOR_ID_TUNDRA | (vmechip_devid << 16);
		vmeInfo->vmeControllerRev = vmechip_revision;
		strcpy(vmeInfo->osName, "Linux");
		vmeInfo->vmeDriverRev = VMEDRV_REV;
		return (0);
	}
	// Fill in information for another board in the chassis.
	vmeInfo->vmeSlotNum = slot_of_interest;
	if (out_image_va[7] == 0) {
		return (-ENODEV);
	}
	// See if something responds at that slot.
	remote_data = (struct vmeSharedData *)
	    (vmeInfo->vmeSlotNum * 512 * 1024 + out_image_va[7]);

	remote_csr = (unsigned int *)remote_data;
	temp = remote_csr[0x7FFFC / 4];
	if (vme_bus_error_chk(1)) {
		return (0);	// no response.
	}
	if (temp != (slot_of_interest << 3)) {
		return (0);	// non-sensical response.
	}
	vmeInfo->boardResponded = 1;
	vmeInfo->vmeControllerID = readl(&remote_csr[0x7F000 / 4]);
	vmeInfo->vmeControllerRev = readl(&remote_csr[0x7F008 / 4]) & 0xFF;

	// If there is no valid driver data structure there, 
	// nothing left to do

	if (strcmp("VME", remote_data->validity1) ||
	    strcmp("RDY", remote_data->validity2)) {
		return (0);
	}
	// Copy information from struct 
	vmeInfo->vmeSharedDataValid = 1;
	vmeInfo->vmeDriverRev = remote_data->driverRev;
	strncpy(vmeInfo->osName, remote_data->osname, 8);
	for (i = 0; i < 8; i++) {
		vmeInfo->vmeAddrHi[i] = remote_data->inBoundVmeAddrHi[i];
		vmeInfo->vmeAddrLo[i] = remote_data->inBoundVmeAddrLo[i];
		vmeInfo->vmeAm[i] = remote_data->inBoundVmeAM[i];
		vmeInfo->vmeSize[i] = remote_data->inBoundVmeSize[i];
	}

	return (0);
}

//----------------------------------------------------------------------------
//  vme_get_requestor
//    Return current VME bus requestor attributes.
//----------------------------------------------------------------------------
int vme_get_requestor(vmeRequesterCfg_t * vmeReq)
{
	memset(vmeReq, 0, sizeof(vmeRequesterCfg_t));
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_get_requestor(vmeReq));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_get_requestor(vmeReq));
		break;
	}
	return (-ENODEV);
}

//----------------------------------------------------------------------------
//  vme_set_requestor
//    Set VME bus requestor attributes.
//----------------------------------------------------------------------------
int vme_set_requestor(vmeRequesterCfg_t * vmeReq)
{
	if (vmeReq->requestLevel > 3) {
		return (-EINVAL);
	}

	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_set_requestor(vmeReq));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_set_requestor(vmeReq));
		break;
	}
	return (-ENODEV);
}

//----------------------------------------------------------------------------
//  vme_get_arbiter
//    Return VME bus arbiter attributes.
//----------------------------------------------------------------------------
int vme_get_arbiter(vmeArbiterCfg_t * vmeArb)
{
	// Only valid for system controller.
	if (vme_syscon == 0) {
		return (-EINVAL);
	}
	memset(vmeArb, 0, sizeof(vmeArbiterCfg_t));
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_get_arbiter(vmeArb));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_get_arbiter(vmeArb));
		break;
	}
	return (-ENODEV);
}

//----------------------------------------------------------------------------
//  vme_set_arbiter
//    Set VME bus arbiter attributes.
//----------------------------------------------------------------------------
int vme_set_arbiter(vmeArbiterCfg_t * vmeArb)
{
	// Only valid for system controller.
	if (vme_syscon == 0) {
		return (-EINVAL);
	}
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_set_arbiter(vmeArb));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_set_arbiter(vmeArb));
		break;
	}
	return (-ENODEV);
}

//----------------------------------------------------------------------------
//  vme_generate_irq
//    Generate a VME bus interrupt at the specified level & vector.
//
//  Caller must assure exclusive access to the VIRQ generator.
//----------------------------------------------------------------------------
int vme_generate_irq(virqInfo_t * vmeIrq)
{
	int status = -ENODEV;

	// Only valid for non system controller.
	if (vme_syscon != 0) {
		return (-EINVAL);
	}
	if ((vmeIrq->level < 1) || (vmeIrq->level > 7)) {
		return (-EINVAL);
	}
	if ((vmeIrq->vector < 0) || (vmeIrq->vector > 0xFF)) {
		return (-EINVAL);
	}

	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		status = uni_generate_irq(vmeIrq);
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		status = tempe_generate_irq(vmeIrq);
		break;
	}

	return (status);
}

//----------------------------------------------------------------------------
//  vme_get_irq_status()
//    Wait for a specific or any VME bus interrupt to occur.
//----------------------------------------------------------------------------
int vme_get_irq_status(virqInfo_t * vmeIrq)
{
	int i, j;

	// Only valid for system controller.
	if (vme_syscon == 0) {
		return (-EINVAL);
	}
	if ((vmeIrq->level >= 8) || (vmeIrq->vector > 0xFF)) {
		return (-EINVAL);
	}
	if ((vmeIrq->level == 0) && (vmeIrq->vector != 0)) {
		return (-EINVAL);
	}

	vmeIrq->timeOutFlag = 0;

	if (vmeIrq->level == 0) {
		if (interruptible_sleep_on_timeout(&vmeint_queue[0],
			vmeIrq->waitTime) != 0) {
			for (i = 1; i < 8; i++) {
				for (j = 0; j <= 0xFF; j++) {
					if (vme_irqlog[i][j] != 0) {
						vmeIrq->level = i;
						vmeIrq->vector = j;
						return (0);
					}
				}
			}
		} else 
			vmeIrq->timeOutFlag = 1;
	} else {
		if (wait_event_interruptible(vmeint_queue[vmeIrq->level],
			(vme_irqlog[vmeIrq->level][vmeIrq->vector] != 0)) == 0) {
				vmeIrq->timeOutFlag = 1;
		}
	}
 
	return (0);
}

//----------------------------------------------------------------------------
//  vme_clr_irq_status()
//    Clear irq log of a specific or all VME interrupts.
//----------------------------------------------------------------------------
int vme_clr_irq_status(virqInfo_t * vmeIrq)
{
	int i, j;

	// Only valid for system controller.
	if (vme_syscon == 0) {
		return (-EINVAL);
	}
	// Sanity check input
	if ((vmeIrq->level >= 8) || (vmeIrq->vector > 0xFF)) {
		return (-EINVAL);
	}
	if ((vmeIrq->level == 0) && (vmeIrq->vector != 0)) {
		return (-EINVAL);
	}

	if ((vmeIrq->level == 0) && (vmeIrq->vector == 0)) {
		// Clear all irqs.
		for (i = 1; i < 8; i++) {
			for (j = 0; j <= 0xFF; j++) {
				vme_irqlog[i][j] = 0;
			}
		}
	}
	// Clear a single irq.
	vme_irqlog[vmeIrq->level][vmeIrq->vector] = 0;
	return (0);
}

//----------------------------------------------------------------------------
//  vme_do_rmw()
//    Perform a RMW operation on the VME bus.
//----------------------------------------------------------------------------
int vme_do_rmw(vmeRmwCfg_t * vmeRmw)
{
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_do_rmw(vmeRmw));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_do_rmw(vmeRmw));
		break;
	}
	return (-ENODEV);
}

//----------------------------------------------------------------------------
//  vme_setup_lm()
//    Setup the VME bus location monitor to detect the specified access.
//----------------------------------------------------------------------------
int vme_setup_lm(vmeLmCfg_t * vmeLm)
{
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_setup_lm(vmeLm));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_setup_lm(vmeLm));
		break;
	}
	return (-ENODEV);
}

//----------------------------------------------------------------------------
//  vme_wait_lm()
//    Wait for the location monitor to trigger.
//----------------------------------------------------------------------------
int vme_wait_lm(vmeLmCfg_t * vmeLm)
{
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_wait_lm(vmeLm));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_wait_lm(vmeLm));
		break;
	}
	return (-ENODEV);
}

//----------------------------------------------------------------------------
//  vme_init()
//    Initialize the VME bridge.
//----------------------------------------------------------------------------
int vme_init(void *driverdata)
{
	switch (vmechip_devid) {
	case PCI_DEVICE_ID_TUNDRA_CA91C042:
		return (uni_init(driverdata));
		break;
	case PCI_DEVICE_ID_TUNDRA_TSI148:
		return (tempe_init(driverdata));
		break;
	}
	return (0);
}
