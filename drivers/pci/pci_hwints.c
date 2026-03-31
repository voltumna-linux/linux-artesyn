/*
**    PCI PIRQ Hardware Interrupt Register Update
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_res.h>

static unsigned int pci_localpirqs[4] =
{
	CONFIG_PCI_LOCAL_PIRQA,
	CONFIG_PCI_LOCAL_PIRQB,
	CONFIG_PCI_LOCAL_PIRQC,
	CONFIG_PCI_LOCAL_PIRQD
};

#ifdef CONFIG_PCI_MULTI_DOMAIN

static unsigned int pci_remotepirqs[4] =
{
	CONFIG_PCI_REMOTE_PIRQA,
	CONFIG_PCI_REMOTE_PIRQB,
	CONFIG_PCI_REMOTE_PIRQC,
	CONFIG_PCI_REMOTE_PIRQD
};

static unsigned int pci_remotedev = CONFIG_PCI_REMOTE_DEV;
#endif

unsigned int IrqTableUpdated = 0;

#ifdef CONFIG_PCI_PIRQ_UPDATE
/*
**    Mapping from PCI vendor/device ID pairs to PIRQ
**    hardware interrupt register update function types
**    and arguments
*/

typedef int (*pci_hwints_handler) (struct pci_dev *);

struct pci_hwints_info {
	unsigned short vendor;	/* Vendor ID */
	unsigned short device;	/* Device ID */
	pci_hwints_handler handler;	/* Function/Handler */
};

/*
**
**    set_82371 - Intel 82371 ISA Bridge
**
**        This function will write the IRQ value to the
**        i82371's ISA PIRQ register.
*/

static int 
set_82371(struct pci_dev *pdev)
{
	unsigned int intnum;	/* PIRQ Interrupt Number */
	u8 offset;		/* PIRQ Register Offset */
	u8 irq;			/* Interrupt Pin/Line */

	for (intnum = 0; intnum < 4; ++intnum) {
		/*
		**    First, read the current IRQ value from the
		**    i82371's PIRQ PCI configuration register.
		**    Check to see if it has been initialized by
		**    the BIOS or not. If it has, then save the
		**    PIRQ value in the PIRQ array.
		*/

		offset = 0x60 + intnum;
		pci_read_config_byte(pdev, offset, &irq);
		if (irq & 0x80) {
			/*
			**    This PIRQ PCI configuration register has
			**    not been initialized. Hence, write the
			**    IRQ value to the i82371's PIRQ PCI
			**    configuration register.
			*/

			irq = pci_localpirqs[intnum];
			pci_write_config_byte(pdev, offset, irq);
		} else {
			pci_localpirqs[intnum] = irq;
#ifdef CONFIG_PCI_MULTI_DOMAIN
			pci_remotepirqs[intnum] = irq;
#endif
		}
	}
	IrqTableUpdated = 1;
	return (PCI_OK);
}

/*
**    Table of devices and handlers.
*/

static struct pci_hwints_info pci_hwints_list[] =
{
	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82371SB_0, set_82371},
	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82371AB_0, set_82371},
	{0xffff, 0xffff, NULL},
};

/*
**
**    pci_hwints_update_pirqs - Update PIRQ hardware interrupt register
**
**        This function will call a function/handler to write
**        the given IRQ value to the PIRQ hardware interrupt
**        register for the selected devices.
*/

int 
pci_hwints_update_pirqs(void)
{
	struct pci_dev *pdev;	/* Device Structure Pointer */
	struct pci_hwints_info *pinfo;	/* Hardware Info Structure Pointer */

	if (IrqTableUpdated)
		return (PCI_OK);
	/*
	**    Go through each vendor/device in the list
	**    of hardware interrupt handlers.
	*/

	for (pinfo = pci_hwints_list; pinfo->vendor != 0xffff; ++pinfo) {
		/*
		**    Compare the vendor and device IDs. If the same, then
		**    call the hardware interrupt update handler.
		*/

		pdev = pci_find_device(pinfo->vendor, pinfo->device, (struct pci_dev *) NULL);
		if (pdev != (struct pci_dev *) NULL) {
			if (pinfo->handler(pdev) != PCI_OK) {
				return (PCI_ERR);
			} else {
				break;
			}
		}
	}
	return (PCI_OK);
}
#endif

/*
**
**    pci_hwints_getirq - Return IRQ value
**
**        This function will return the IRQ value for the
**        given device. It will also update the Interrupt
**        Line register of the device with the value.
*/

void 
pci_hwints_getirq(struct pci_dev *pdev)
{
	struct pci_bus *pbus;	/* Parent Bus Structure Pointer */
	unsigned int devnum;	/* Accumulated Device Number */
	unsigned int irq;	/* IRQ Value */

	devnum = PCI_SLOT(pdev->devfn);
	pbus = pdev->bus;
#ifdef CONFIG_PCI_PIRQ_UPDATE
	if (!IrqTableUpdated) {
		pci_hwints_update_pirqs();
		IrqTableUpdated = 1;
	}

#endif

	/*
	**    Check to see if this could possibly be a
	**    peer Host Bridge device.
	*/

	if (pbus->parent != (struct pci_bus *) NULL) {
		/*
		**    The IRQ line number will be generated after
		**    taking into account all the PCI-2-PCI bridge
		**    devices between the device and the Host Bridge.
		*/

		while ((pbus->parent)->primary != (pbus->parent)->secondary) {
			devnum += PCI_SLOT((pbus->self)->devfn);
			pbus = pbus->parent;
		}
	}
	devnum &= 0x03;
	irq = pci_localpirqs[devnum];

#ifdef CONFIG_PCI_MULTI_DOMAIN
	/*
	**    Check to see if this could possibly be a
	**    peer Host Bridge device.
	*/
	pbus = pdev->bus;
	if (pbus->parent != (struct pci_bus *) NULL) {
		/*
		**    Determine if the device is located in the
		**    remote domain or not. We must find the
		**    domain's bridge device located on bus 0.
		*/
		while (pbus->primary != 0)
			pbus = pbus->parent;
		/*
		**    Check the device/function of domain's bridge
		**    device against the remote device/function.
		**    If the same, then the device is located in
		**    the remote domain. Thus, get the PCI remote
		**    domain IRQ value.
		*/
		if ((pbus->self)->devfn == pci_remotedev) {
			irq = pci_remotepirqs[devnum];
		}
	}
#endif
	pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, (u8) irq);
	pdev->irq = irq;
}
