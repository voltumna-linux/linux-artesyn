/*
**    PCI Bus Probe/Scan
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_res.h>
#include "pci.h"

#undef DEBUG

#ifdef DEBUG
#define DBG(x...) printk(x)
#else				/*  */
#define DBG(x...)
#endif				/*  */
extern unsigned char pci_processor;
extern int pci_find_level(struct pci_bus *);
int pci_is_booting;
int
pci_is_device_configured(struct pci_dev *pdev)
{
	u16 command;
	u32 class;
	int ret;
	struct pci_bus *pbus;

#ifndef CONFIG_PCI_PRESERVE_CONFIGURATION
	return FALSE;

#endif				/*  */
	if (pci_is_booting == TRUE)
		return FALSE;
	ret = TRUE;
	pbus = pdev->bus;

#ifdef CONFIG_PCI_AGP_BRIDGE
	if (pci_agp_bridge(pbus, pdev->devfn) == TRUE)
		return FALSE;
	if (pbus->parent != NULL) {
		if (pbus->self != NULL) {
			if (pci_agp_bridge(pbus->parent, pbus->self->devfn) ==
			    TRUE)
				return FALSE;
		}
	}
#endif		/* CONFIG_PCI_AGP_BRIDGE */

#ifdef CONFIG_PCI_EXT_BRIDGE
	{
		int bus_level;
		bus_level = pci_find_level(pbus);
		pci_read_config_dword(pdev, PCI_CLASS_REVISION, &class);
		if ((bus_level == 1)
		    && ((class >> 16) == PCI_CLASS_BRIDGE_PCI)
		    && (pdev->devfn == CONFIG_PCI_EXT_BRIDGE_1
			|| pdev->devfn == CONFIG_PCI_EXT_BRIDGE_2)) {
			ret = FALSE;
		}
	}

#endif	
	if (pbus && pbus->number == 0) {
		pci_read_config_dword(pdev, PCI_CLASS_REVISION, &class);
		if ((class >> 16) == PCI_CLASS_BRIDGE_PCI)
			ret = FALSE;
	} else {
		pci_read_config_word(pdev, PCI_COMMAND, &command);
		if (!(command & PCI_COMMAND_MEMORY)
		    && !(command & PCI_COMMAND_IO))
			ret = FALSE;
	}
	return ret;
}

/*
**    pci_find_host - Find Host Bridge Device on Bus
**
**        This function will probe the given PCI bus for
**        a Host Bridge Device.
*/
int
pci_find_host(struct pci_dev *temp)
{
	unsigned short class;	/* Class Code */
	int func = 0;
	int is_multi = 0;
	u8 hdr_type;
	int ret_code;		/* Function Return Code */
	ret_code = PCI_ERR;

	pr_debug("pci_find_host: dev 0x%p\n", temp);
	for (func = 0; func < 8; func++, temp->devfn++) {
		if (func && !is_multi)	/* not a multi-function device */
			continue;
		if (pci_read_config_byte(temp, PCI_HEADER_TYPE, &hdr_type))
			continue;
		if (!func) {
			is_multi = hdr_type & 0x80;
		}
		pci_read_config_word(temp, PCI_CLASS_DEVICE, &class);
		if (class == PCI_CLASS_BRIDGE_HOST) {
			ret_code = PCI_OK;
			break;
		}
	}
	return (ret_code);
}

/*
**    pci_kern_err - Display PCI Kernel Error Message
**
**        This function will format and display a
**        PCI kernel error message.
*/
void
pci_kern_err(char *func, char *err_msg)
{
	if (func != (char *) NULL)
		printk(KERN_ERR "PCI: %s() - %s\n", func, err_msg);
	else
		printk(KERN_ERR "PCI: %s\n", err_msg);
}

/*
**    pci_valid_devid - Read and Validate the device ID
**
**        This function will read the device ID at the given
**        bus, device, and function number. It will also
**        validate it.
*/
int
pci_valid_devid(struct pci_dev *pdev)
{
	u16 devid;

	/*
	 **    NOTE: Some broken boards return 0  or ~0 if a
	 **          slot is empty.
	 */
	pci_read_config_word(pdev, PCI_DEVICE_ID, &devid);
	if (devid == 0xffff || devid == 0x0000)
		return FALSE;
	else
		return TRUE;
}


/*
**    pci_multi_func - Check for Multi-Function Device
**
**        This function will read the header type at the given
**        bus, device, and function number. It will check it
**        for a multi-function device or not.
*/
int
pci_multi_func(struct pci_dev *dev)
{
	unsigned char hdr_type;	/* Device Header Type */
	int ret_code;

	ret_code = TRUE;
	if (PCI_FUNC(dev->devfn) == 0) {
		pci_read_config_byte(dev, PCI_HEADER_TYPE, &hdr_type);
		if ((hdr_type & 0x80) == 0)
			ret_code = FALSE;
	}
	return (ret_code);
}

/*
**    pci_probe_dev - Probe PCI Bus For Devices
**
**        This function will probe the given bus for all
**        devices connected to it. It will add the found
**        devices to the global device list and to the
**        bus structure devices list.
*/

static int
pci_probe_dev(struct pci_bus *pbus)
{
	struct pci_dev *pdev;	/* PCI Device Pointer */
	struct pci_dev *adev;	/* PCI Device Pointer */
	struct pci_dev *bdev;	/* PCI Device Pointer */
	struct pci_bus *child;	/* PCI Bus Child Pointer */
	struct pci_dev *dev0;
	struct list_head *root_lh;
	struct list_head *lh;
	unsigned int dev;	/* Device Number */
	unsigned int func;	/* Function Number */
	unsigned int devfn;	/* Device/Function Number */
	unsigned int buses;	/* Buses Config. Register */
	unsigned char slat_timer;	/* Secondary Latency Timer */
	int configured;		/* Configuration Status */
	int ret_code;		/* Function Return Code */

	pr_debug("pci_probe_dev: bus 0x%p\n", pbus);
	ret_code = PCI_OK;
	adev = NULL;
	bdev = NULL;
	dev0 = kzalloc(sizeof (*dev0), GFP_KERNEL);
	if (!dev0)
		return -ENOMEM;
	dev0->bus = pbus;
	dev0->sysdata = pbus->sysdata;
	for (dev = 0; (ret_code == PCI_OK) && (dev <= MAX_PCI_DEV); dev++) {
		for (func = 0; (ret_code == PCI_OK) && (func <= MAX_PCI_FUNC);
		     ++func) {
			devfn = PCI_DEVFN(dev, func);
			dev0->devfn = devfn;
			if (pci_valid_devid(dev0) == TRUE) {
				ret_code = pci_add_dev(pbus, devfn, &pdev);
				if (ret_code == PCI_OK) {
					configured =
					    pci_is_device_configured(pdev);
					if (!configured) {
						if ((pdev->class >> 8) ==
						    PCI_CLASS_BRIDGE_PCI) {
							pci_read_config_byte
							    (pdev,
							     PCI_SEC_LATENCY_TIMER,
							     &slat_timer);
							buses = slat_timer;
							buses =
							    (buses << 8) | 0;
							buses =
							    (buses << 8) | 0;
							buses =
							    (buses << 8) | 0;
							pci_write_config_dword
							    (pdev,
							     PCI_PRIMARY_BUS,
							     buses);
						}
					}
					if (pci_multi_func(pdev) == FALSE)
						break;
				}
			}
		}
	}

#ifdef __powerpc__
	if (pbus->number == 0)
		pcibios_fixup_bus(pbus);
#else	
	pcibios_fixup_bus(pbus);
#endif

	/*
	 **    Look behind all PCI-to-PCI bridges on this bus.
	 */
	root_lh = &pbus->devices;
	lh = root_lh->next;
	while ((ret_code == PCI_OK) && (lh != root_lh)) {
		pdev = list_entry(lh, struct pci_dev, bus_list);

		/*
		 **    If it's a bridge, scan the bus behind it.
		 */
		if ((pdev->class >> 8) == PCI_CLASS_BRIDGE_PCI) {

			/*
			 **    Allocate a child pci_bus structure,
			 **    initialize it, and insert it into the
			 **    tree of buses. Then scan all the child's
			 **    subordinate buses.
			 */
#ifdef CONFIG_PCI_MULTI_DOMAIN
#ifndef __powerpc__
			if ((pbus->number == 0)
			    && (pci_processor == DOMAIN_B)
			    && (pdev->devfn == CONFIG_PCI_REMOTE_DEV)) {

#else	
			if ((pbus->number == 0)
			    && (pci_processor == DOMAIN_B)
			    && (pdev->devfn == CONFIG_PCI_LOCAL_DEV)) {

#endif
				bdev = pdev;
				lh = lh->next;
				continue;
			}
#ifdef CONFIG_PCI_HSC_BRIDGE
#ifdef __powerpc__
			if ((pbus->number == 0)
			    && (pci_processor == DOMAIN_A)
			    && (pdev->devfn == CONFIG_PCI_LOCAL_DEV)) {
				adev = pdev;
				lh = lh->next;
				continue;
			}
			if ((pbus->number == 0)
			    && (pci_processor == DOMAIN_A)
			    && (pdev->devfn == CONFIG_PCI_REMOTE_DEV)) {
				bdev = pdev;
				lh = lh->next;
				continue;
			}
#endif				/* __powerpc__ */
#endif				/* CONFIG_PCI_HSC_BRIDGE */
#endif				/* CONFIG_PCI_MULTI_DOMAIN */
			ret_code = pci_add_bus(pbus, pdev, &child);
			if (ret_code == PCI_OK)
				ret_code = pci_probe_bus(child, pdev->devfn);
		}
		lh = lh->next;
	}

#ifdef CONFIG_PCI_MULTI_DOMAIN
#ifdef CONFIG_PCI_HSC_BRIDGE
#ifdef __powerpc__
	if ((pbus->number == 0) && (pci_processor == DOMAIN_A) &&
	    (adev != (struct pci_dev *) NULL) &&
	    (adev->devfn == CONFIG_PCI_LOCAL_DEV)) {
		ret_code = pci_add_bus(pbus, adev, &child);
		if (ret_code == PCI_OK)
			ret_code = pci_probe_bus(child, adev->devfn);
	}
	if ((pbus->number == 0) && (pci_processor == DOMAIN_A) &&
	    (bdev != (struct pci_dev *) NULL) &&
	    (bdev->devfn == CONFIG_PCI_REMOTE_DEV)) {
		ret_code = pci_add_bus(pbus, bdev, &child);
		if (ret_code == PCI_OK)
			ret_code = pci_probe_bus(child, bdev->devfn);
	}
#endif		
#endif	
#ifndef __powerpc__
	if ((pbus->number == 0) && (pci_processor == DOMAIN_B) &&
	    (bdev != (struct pci_dev *) NULL) &&
	    (bdev->devfn == CONFIG_PCI_REMOTE_DEV)) {

#else
	if ((pbus->number == 0) && (pci_processor == DOMAIN_B) &&
	    (bdev != (struct pci_dev *) NULL) &&
	    (bdev->devfn == CONFIG_PCI_LOCAL_DEV)) {

#endif	
		ret_code = pci_add_bus(pbus, bdev, &child);
		if (ret_code == PCI_OK)
			ret_code = pci_probe_bus(child, bdev->devfn);
	}
#endif		
	kfree(dev0);
	return (ret_code);
}

/*
**    pci_probe_bus - Probe PCI Bus
**
**        This function gets called recursively. However, for
**        the first time it gets called from pci_init(). The
**        argument to this function has to be bus node.
*/
int
pci_probe_bus(struct pci_bus *pbus, unsigned int devfn)
{
	struct pci_bus *pbus_par;	/* Parent Bus Pointer */
	unsigned int buses;	/* Buses Config. Register */
	unsigned short cmd;	/* Command Register */
	unsigned short class;	/* Class Code */
	unsigned char slat_timer;	/* Secondary Latency Timer */
	int configured;		/* Configuration Status */
	int ret_code;		/* Function Return Code */
	struct pci_dev *dev0;
	ret_code = PCI_OK;

	pr_debug("pci_probe_bus: bus 0x%p devfn 0x%x\n", pbus, devfn);
	if ((pbus->subordinate == 0) || (pbus->subordinate == MAX_PCI_BUS)) {
		ret_code = pci_alloc_busnum(pbus, devfn);
		dev0 = kzalloc(sizeof (*dev0), GFP_KERNEL);
		if (!dev0)
			return -ENOMEM;
		dev0->devfn = devfn;
		if (ret_code == PCI_OK) {
			if (pbus->parent == (struct pci_bus *) NULL)
				configured = FALSE;
			else {
				dev0->sysdata = pbus->parent->sysdata;
				dev0->bus = pbus->parent;
				dev0->devfn = devfn;
				configured = pci_is_device_configured(dev0);
			}
			if ((pbus_par =
			     pbus->parent) == (struct pci_bus *) NULL)
				pbus_par = pbus;
				dev0->sysdata = pbus_par->sysdata;
				dev0->bus = pbus_par;
				dev0->devfn = devfn;
			if (!configured) {
				slat_timer = 0;
				pci_read_config_word(dev0, PCI_CLASS_DEVICE,
						     &class);
				if (class == PCI_CLASS_BRIDGE_PCI) {
					pci_read_config_word(dev0,
							     PCI_BRIDGE_CONTROL,
							     &cmd);
					cmd &= ~PCI_BRIDGE_CTL_BUS_RESET;
					pci_write_config_word(dev0,
							      PCI_BRIDGE_CONTROL,
							      cmd);
					pci_read_config_word(dev0,
							     PCI_COMMAND, &cmd);
					cmd &= ~0x0007;
					pci_write_config_word(dev0,
							      PCI_COMMAND, cmd);
					pci_read_config_byte(dev0,
							     PCI_SEC_LATENCY_TIMER,
							     &slat_timer);
					if (slat_timer < 16)
						slat_timer = 64;
				}
				buses = slat_timer;
				buses = (buses << 8) | pbus->subordinate;
				buses = (buses << 8) | pbus->secondary;
				buses = (buses << 8) | pbus->primary;
				pci_write_config_dword(dev0, PCI_PRIMARY_BUS,
						       buses);
			}
			pci_write_config_word(dev0, PCI_STATUS, 0xffff);
		}
		kfree(dev0);
	}

	if (ret_code == PCI_OK) {
		ret_code = pci_probe_dev(pbus);
                if (pbus->number != 0) {
                        pbus->class_dev.class = &pcibus_class;
                                                                                
                        sprintf(pbus->class_dev.class_id, "%04x:%02x",
                                pci_domain_nr(pbus), pbus->secondary);
                        ret_code = class_device_register(&pbus->class_dev);
                        if (ret_code) 
                                goto error_register;
                        ret_code = class_device_create_file(&pbus->class_dev,
                                &class_device_attr_cpuaffinity);
                        if (ret_code) 
                                goto error_file_create;
			ret_code = sysfs_create_link(
					&pbus->class_dev.kobj,
					&pbus->bridge->kobj,
					"bridge");
                        if (ret_code) 
                                goto sys_create_link_err;
			
		}
	}
	return (ret_code);
sys_create_link_err:
        class_device_remove_file(&pbus->class_dev, &class_device_attr_cpuaffinity);
error_file_create:
	class_device_unregister(&pbus->class_dev);
error_register:
	return (ret_code);

}

/*
**    pci_services_scan_bus - Scan PCI Bus
**
**        This function gets called recursively. However, for
**        the first time it gets called from pci_init(). The
**        argument to this function has to be bus node.
*/
unsigned int __init
pci_services_scan_bus(struct pci_bus *pbus)
{
	unsigned int devfn;	/* Host Device/Function Number */
	unsigned short class;
	unsigned short cmd;
	struct pci_dev *dev0;
	int ret_code;		/* Function Return Code */

	ret_code = PCI_OK;
	dev0 = kzalloc(sizeof (*dev0), GFP_KERNEL);
	if (!dev0)
		return -ENOMEM;
	dev0->bus = pbus;
	dev0->sysdata = pbus->sysdata;
	pci_is_booting = TRUE;
	for (devfn = 0; devfn < 0x100; devfn += 1) {
		dev0->devfn = devfn;
		pci_read_config_word(dev0, PCI_CLASS_DEVICE, &class);
		if (class == PCI_CLASS_BRIDGE_PCI) {
			pci_read_config_word(dev0, PCI_COMMAND, &cmd);
			cmd &= ~0x0007;
			pci_write_config_word(dev0, PCI_COMMAND, cmd);
		}
	}
	for (devfn = 0; devfn < 0x100; devfn += 8) {
		dev0->devfn = devfn;
		ret_code = pci_find_host(dev0);
		if (ret_code == PCI_OK)
			break;
	}

#if (CONFIG_PCI_BUSLVL0_NODES > 1)
	if (ret_code == PCI_OK) {
		if (pbus->number == 0) {

			/*
			 **    It is our first time through. Thus, we must
			 **    reassign bus numbers to all Host Bridge
			 **    devices before probing.
			 */
			ret_code = pci_alloc_hosts(pbus);
		}
	}
#endif				/*  */
	if (ret_code == PCI_OK) {
		if (pci_probe_bus(pbus, devfn) == PCI_OK) {

			/*
			 **    Next, set the Host Bridge node's
			 **    device structure.
			 */
			pbus->self = pci_find_slot(pbus->number, devfn);
			pci_conf_bus(pbus, TRUE);
		}
	}
	kfree(dev0);
	pci_is_booting = FALSE;
	return (pbus->subordinate);
}

struct pci_bus *__init
pci_pks_scan_bus(int bus, struct pci_ops *ops, void *sysdata)
{
	int error;
	struct pci_bus *b;
	struct device *dev;

	b = kzalloc(sizeof (*b), GFP_KERNEL);
	if (!b) 
		return NULL;
		
	dev = kmalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev){
		kfree(b);
		return NULL;
	}
                                                                                
	INIT_LIST_HEAD(&b->node);
	INIT_LIST_HEAD(&b->children);
	INIT_LIST_HEAD(&b->devices);

	b->sysdata = sysdata;
	b->ops = ops;

	if (pci_find_bus(pci_domain_nr(b), bus)) {
		/* If we already got to this bus through a different bridge, ignore it */
		pr_debug("PCI: Bus %04x:%02x already known\n", pci_domain_nr(b), bus);
		goto err_out;
	}

	down_write(&pci_bus_sem);
	list_add_tail(&b->node, &pci_root_buses);
	up_write(&pci_bus_sem);

	memset(dev, 0, sizeof(*dev));
	dev->parent = NULL;
	dev->release = pci_release_bus_bridge_dev;
	sprintf(dev->bus_id, "pci%04x:%02x", pci_domain_nr(b), bus);
	error = device_register(dev);
	if (error)
               	goto dev_reg_err;
	b->bridge = get_device(dev);
                                                                               
	b->class_dev.class = &pcibus_class;
	sprintf(b->class_dev.class_id, "%04x:%02x", pci_domain_nr(b), bus);
	error = class_device_register(&b->class_dev);
	if (error) {
		goto class_dev_reg_err;
	}
	error = class_device_create_file(&b->class_dev, &class_device_attr_cpuaffinity);
	if (error) {
		goto class_dev_create_file_err;
	}
                                                                                
	error = sysfs_create_link(&b->class_dev.kobj, &b->bridge->kobj, "bridge");
	if (error) {
		goto sys_create_link_err;
	}
		
	b->resource[0] = &ioport_resource;
	b->resource[1] = &iomem_resource;
	b->number = bus;
	b->secondary = bus;
	if (pci_alloc_businit(b) != PCI_OK)
		return NULL;
	b->subordinate = pci_services_scan_bus(b);

	return b;
sys_create_link_err:
	class_device_remove_file(&b->class_dev, &class_device_attr_cpuaffinity);
class_dev_create_file_err:
	class_device_unregister(&b->class_dev);
class_dev_reg_err:
	device_unregister(dev);
dev_reg_err:
	down_write(&pci_bus_sem);
	list_del(&b->node);
	up_write(&pci_bus_sem);
err_out:
	kfree(dev);
	kfree(b);
        return NULL;
}
