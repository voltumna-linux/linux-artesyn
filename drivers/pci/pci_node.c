/*
**    PCI Bus/Device Node Management
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>
#include <linux/pci.h>
#include <linux/pci_res.h>

#include <asm/errno.h>
#include <asm/system.h>

#include "pci.h"

extern struct bus_type *get_bus(struct bus_type * bus);
extern void put_bus(struct bus_type * bus);

extern int pci_is_booting;
extern void pci_destroy_dev(struct pci_dev *dev);

#if 0
static void
pci_free_resources(struct pci_dev *dev)
{
        int i;
                                                                                
        for (i = 0; i < PCI_NUM_RESOURCES; i++) {
                struct resource *res = dev->resource + i;
                if (res->parent)
                        release_resource(res);
        }
}
#endif

/*
**    pci_add_dev - Add Device Structure
**
**        This function will allocate a device structure
**        and fill it with the appropriate device information,
**        If the device structure cannot be allocated or
**        if the device header is incorrect, a NULL pointer
**        will be returned.
*/

int 
pci_add_dev(struct pci_bus *pbus, unsigned int devfn, struct pci_dev **pdev)
{
	int ret_code;		/* Function Return Code */
	u32 devid;		/* Device/Vendor ID */
	u8 hdr_type;		/* Configuration Header Type */

	pr_debug("pci_add_dev: bus 0x%p devfn 0x%x\n", pbus, devfn);
	ret_code = PCI_OK;
	*pdev = (struct pci_dev *) kzalloc(sizeof (struct pci_dev), GFP_ATOMIC);
	if (*pdev != (struct pci_dev *) NULL) {
		(*pdev)->bus = pbus;
		(*pdev)->sysdata = pbus->sysdata;
		(*pdev)->dev.parent = pbus->bridge;
		(*pdev)->dev.bus = &pci_bus_type;
		(*pdev)->devfn = devfn;
		pci_read_config_dword(*pdev, PCI_VENDOR_ID, &devid);
		(*pdev)->vendor = devid & 0xffff;
		(*pdev)->device = (devid >> 16) & 0xffff;
		pci_read_config_byte(*pdev, PCI_HEADER_TYPE, &hdr_type);
		(*pdev)->hdr_type = hdr_type & 0x7f;
		(*pdev)->multifunction = !!(hdr_type & 0x80);
		(*pdev)->cfg_size = pci_cfg_space_size(*pdev);
		(*pdev)->error_state = pci_channel_io_normal;
		(*pdev)->dma_mask = 0xffffffff;

		ret_code = pci_setup_device(*pdev);
		if (ret_code == PCI_OK) {
			pci_device_add(*pdev, pbus);
			ret_code = pci_bus_add_device(*pdev);
		} else {
			kfree(*pdev);
			*pdev = NULL;
			return ret_code;
		}
	}
	return (ret_code);
}

/*
**    pci_add_bus - Add Bus Structure
**
**        This function will allocate a bus structure for
**        a child bus and fill it with the appropriate
**        bus node information. If the child bus structure
**        cannot be allocated, a NULL pointer will be
**        returned.
*/

int 
pci_add_bus(struct pci_bus *pbus, struct pci_dev *pdev,
	    struct pci_bus **child)
{
	int i;
	int ret_code;		/* Function Return Code */

	pr_debug("pci_add_bus: bus 0x%p dev 0x%p\n", pbus, pdev);
	ret_code = PCI_OK;
	*child = (struct pci_bus *) kzalloc(sizeof (struct pci_bus), GFP_KERNEL);
	if (*child != (struct pci_bus *) NULL) {
		INIT_LIST_HEAD(&((*child)->node));
		INIT_LIST_HEAD(&((*child)->children));
		INIT_LIST_HEAD(&((*child)->devices));

		down_write(&pci_bus_sem);
		list_add_tail(&((*child)->node), &pbus->children);
		up_write(&pci_bus_sem);

		(*child)->self = pdev;
		pdev->subordinate = (*child);
		(*child)->parent = pbus;
		(*child)->ops = pbus->ops;
		(*child)->sysdata = pbus->sysdata;
		(*child)->bus_flags = pbus->bus_flags;
		(*child)->bridge = get_device(&pdev->dev);

		/* Set up default resource pointers */
		for (i = 0; i < 4; i++) {
			(*child)->resource[i] = &pdev->resource[PCI_BRIDGE_RESOURCES + i];
			(*child)->resource[i]->name = (*child)->name;
		}
		sprintf((*child)->name, "PCI Bus #%02x", (*child)->number);

	}
	else
		ret_code = -ENOMEM;

	return (ret_code);
}

/*
**    pci_add_node - Add Node Structure
**
**        This function will allocate a structure for the
**        node, i.e. either bus or device structure, and
**        all resources for that node. It will be added to
**        the various lists as well.
*/

int 
pci_add_node(unsigned int number, unsigned int devfn)
{
	struct pci_bus *pbus;	/* PCI Parent Bus Pointer */
	struct pci_bus *child;	/* PCI Child Bus Pointer */
	struct pci_dev *pdev;	/* PCI Device Pointer */
	unsigned short class;	/* Class Code */
	int ret_code;		/* Function Return Code */
	struct list_head *lh;
	struct list_head *lh_root;
	struct pci_dev *dev0;

	pr_debug("pci_add_node: bus 0x%x devfn 0x%x\n", number, devfn);
	ret_code = PCI_OK;
	child = (struct pci_bus *) NULL;
	pdev = (struct pci_dev *) NULL;

	if (pci_get_bus_and_slot(number, devfn) == (struct pci_dev *) NULL) {
		pbus = list_entry(pci_root_buses.next, struct pci_bus, node);
		lh_root = &pbus->children;
		lh = lh_root;
		while ((pbus->secondary != number) && (lh->next != lh_root)) {
			pbus = list_entry(lh->next, struct pci_bus, node);
			if ((number >= pbus->secondary) && (number <= pbus->subordinate)) {
				lh_root = &pbus->children;
				lh = lh_root;
			} else {
				lh = lh->next;
			}
		}

		if (pbus->secondary == number) {
        		dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
        		if (!dev0)
                		return -ENOMEM;
        		dev0->bus = pbus;                  
			dev0->sysdata = pbus->sysdata;
			dev0->devfn = devfn;
			if (pci_valid_devid(dev0) == TRUE) {
				pci_read_config_word(dev0, PCI_CLASS_DEVICE, &class);
				kfree(dev0);
				if (class == PCI_CLASS_BRIDGE_HOST) {
					if (number != 0) {
						ret_code = pci_add_bus((struct pci_bus *) NULL, (struct pci_dev *) NULL, &pbus);

						if (ret_code == PCI_OK) {
							pbus->number = number;
							pbus->primary = number;
							pbus->secondary = number;
							pbus->subordinate = 0xff;
							ret_code = pci_probe_bus(pbus, devfn);

							if (ret_code == PCI_OK) {
								pbus->self = pci_get_bus_and_slot(number, devfn);
								ret_code = pci_conf_bus(pbus, TRUE);
							}
							if (ret_code != PCI_OK) {
								pci_del_bus(pbus);
							}
						}
					} else {
						printk(KERN_ERR "pci_add_node: Cannot add primary Host Bridge");
						ret_code = -EPERM;
					}
				} else {
					ret_code = pci_add_dev(pbus, devfn, &pdev);
					if (ret_code == PCI_OK) {
						if ((pdev->class >> 8) == PCI_CLASS_BRIDGE_PCI) {
							ret_code = pci_add_bus(pbus, pdev, &child);
							if (ret_code == PCI_OK) {
								ret_code = pci_probe_bus(child, devfn);

#ifdef CONFIG_PCI_AGP_BRIDGE
								if (ret_code == PCI_OK) {
									if (pci_agp_bridge (pdev->bus, devfn) == TRUE) {
										printk(KERN_ERR "pci_add_node: Cannot add AGP bridge at %02x:%02x",
											number, devfn);
										ret_code = -EPERM;
									}
								}
#endif
								if (ret_code == PCI_OK)
									ret_code = pci_conf_bus(child, TRUE);
							}
						} else {
							child = (struct pci_bus *) NULL;
							if (number != 0) {
								ret_code = pci_conf_dev(pdev, TRUE);
							} else {
								printk(KERN_ERR "pci_add_node: Cannot add bus 0 device");
								ret_code = -EPERM;
							}
						}
					}
				}

				if (ret_code != PCI_OK) {
					if (child != (struct pci_bus *) NULL) {
						pci_del_bus(child);
					} else if (pdev != (struct pci_dev *) NULL)
						pci_del_dev(pdev);
				}
			} else {
				printk(KERN_INFO "PCI: pci_add_node: invalid device ID  %02x:%02x\n",
				       number, devfn);
				ret_code = -ENODEV;
			}
		} else {
			printk(KERN_ERR "pci_add_node: Couldn't find parent bus structure for %02x:%02x",
				number, devfn);
			ret_code = -ENOENT;
		}
	} else {
		ret_code = -EEXIST;
	}
	return (ret_code);
}

/*
**    pci_del_dev - Delete Device Structure
**
**        This function will delete a device structure from
**        the list of devices for the bus.. The device structure
**        memory will be freed as well.
*/

int 
pci_del_dev(struct pci_dev *pdev)
{
	unsigned int num_bars;	/* Maximum Number Of BARs */
	unsigned int bar;	/* Base Address Register */
	int ret_code;		/* Function Return Code */
	int alt_ret_code;	/* Alternate Return Code */

	pr_debug("pci_del_dev: dev 0x%p\n", pdev);
	ret_code = PCI_OK;

	if ((alt_ret_code = pci_get_bars(pdev, &num_bars)) != PCI_OK)
		ret_code = alt_ret_code;

	/*
	**    Next, free the device's resources.
	*/

	for (bar = 0; bar < num_bars; ++bar)
		if ( (alt_ret_code = pci_free_devres(pdev, &(pdev->resources[bar]))) != PCI_OK)
			if (ret_code == PCI_OK)
				ret_code = alt_ret_code;

	if (pdev->driver) {
		if (pdev->driver->remove)
			pdev->driver->remove(pdev);
		pdev->driver = NULL;
	}
	pci_destroy_dev(pdev);
	return (ret_code);
}

/*
**    pci_del_bus - Delete Bus Structure
**
**        This function will delete a bus structure and
**        all the child bus/device structures associated
**        with it. All resources will be freed.
*/

int 
pci_del_bus(struct pci_bus *pbus)
{
	struct pci_bus *pbus_del;	/* PCI Bus Deleted Child Pointer */
	struct pci_dev *pdev_del;	/* PCI Deleted Device Pointer */
	struct list_head *root_lh;
	struct list_head *lh;
	int ret_code;		/* Function Return Code */
	int alt_ret_code;	/* Alternate Return Code */

	pr_debug("pci_del_bus: bus 0x%p\n", pbus);
	ret_code = PCI_OK;

	/*
	**    Recursively delete each child bus structure.
	*/
	root_lh = &pbus->children;
	lh = root_lh->next;
	while (lh != root_lh) {
		pbus_del = list_entry(lh, struct pci_bus, node);
		lh = lh->next;
		if ((alt_ret_code = pci_del_bus(pbus_del)) != PCI_OK)
			if (ret_code == PCI_OK)
				ret_code = alt_ret_code;
	}
	root_lh = &pbus->devices;
	lh = root_lh->next;

	while (lh != root_lh) {
		pdev_del = list_entry(lh, struct pci_dev, bus_list);
		lh = lh->next;
		if ((alt_ret_code = pci_del_dev(pdev_del)) != PCI_OK)
			if (ret_code == PCI_OK)
				ret_code = alt_ret_code;
	}

	if ((alt_ret_code = pci_free_busres(pbus)) != PCI_OK)
		if (ret_code == PCI_OK)
			ret_code = alt_ret_code;

	if ((alt_ret_code = pci_free_busnum(pbus)) != PCI_OK)
		if (ret_code == PCI_OK)
			ret_code = alt_ret_code;

	/* Delete bus node from parent's children list */
	pci_remove_bus(pbus);

	if (pbus->self != (struct pci_dev *) NULL)
		pbus->self->subordinate = NULL;
		if ((alt_ret_code = pci_del_dev(pbus->self)) != PCI_OK)
			if (ret_code == PCI_OK)
				ret_code = alt_ret_code;

	return (ret_code);
}

/*
**    pci_del_node - Delete Node Structure
**
**        This function will free all resources associated
**        with the node, i.e. either bus or device structure,
**        for the given bus and device/function numbers. It
**        will then remove the device structure from the bus'
**        list of devices as well as from the global list of devices.
 */

int 
pci_del_node(unsigned int number, unsigned int devfn)
{
	struct pci_bus *pbus = NULL;	/* PCI Bus Pointer */
	struct pci_dev *pdev;	/* PCI Device Pointer */
	struct list_head *root_lh, *lh;
	unsigned int class;	/* Class Code */
	int ret_code;		/* Function Return Code */

	/*
	**    First, set the function name for possible error
	**    messages. Then, initialize the function return
	**    code and find the device structure in the devices'
	**    list.
	*/

	pr_debug("pci_del_node: bus 0x%x devfn 0x%x\n", number, devfn);
	ret_code = PCI_OK;
	pdev = pci_get_bus_and_slot(number, devfn);
	if (pdev != (struct pci_dev *) NULL) {
		class = pdev->class >> 8;
		switch (class) {
		case PCI_CLASS_BRIDGE_HOST:
			{
				if ((pdev->bus)->number == 0) {
					printk(KERN_ERR "pci_del_node: Cannot delete primary Host Bridge");
					ret_code = -EPERM;
					break;
				}
			}
		case PCI_CLASS_BRIDGE_PCI:
			{
				if (class == PCI_CLASS_BRIDGE_HOST) {
					root_lh = &pci_root_buses;
					lh = root_lh->next;
				} else {
					root_lh = &((pdev->bus)->children);
					lh = root_lh->next;
				}

				while (lh != root_lh) {
					pbus = list_entry(lh, struct pci_bus, node);
					if (pbus->self != pdev)
						lh = lh->next;
					else
						break;
				}

				if (pbus->self == pdev) {
#ifdef CONFIG_PCI_AGP_BRIDGE
					if (pci_agp_bridge(pdev->bus, devfn) == TRUE) {
						printk(KERN_ERR "pci_del_node: Cannot delete AGP bridge at %02x:%02x", number, devfn);
						ret_code = -EPERM;
					}
#endif
					if (ret_code == PCI_OK)
						ret_code = pci_del_bus(pbus);
				} else {
					printk(KERN_ERR "Couldn't find bus structure for %02x:%02x", number, devfn);
					ret_code = -ENOENT;
				}
				break;
			}

		default:
			{
				if ((pdev->bus)->number != 0) {
					ret_code = pci_del_dev(pdev);
				} else {
					printk(KERN_ERR "pci_del_node: Cannot delete a bus 0 device");
					ret_code = -EPERM;
				}
			}
		}
	} else {
		printk(KERN_ERR "pci_del_node: Couldn't find device structure for %02x:%02x", number, devfn);
		ret_code = -ENODEV;
	}
	return (ret_code);
}

struct pci_dev *
pcisvrs_find_slot(unsigned int bus, unsigned int devfn)
{
	struct pci_dev *dev = NULL;

	for_each_pci_dev(dev) {
		if (dev->bus->number == bus && dev->devfn == devfn)
			return dev;
	}
	return NULL;
}

/*
**    pci_swap_node - Swap Node Structure
**
**        This function will swap all resources associated
**        with the nodes, i.e. either bus or device structure,
**        for the given bus and device/function numbers.
*/

int 
pci_swap_node(unsigned int number, unsigned int devfn,
	      unsigned int secdevfn)
{
	struct pci_bus *pbus = NULL;	/* PCI Bus Pointer */
	struct pci_dev *pdev;	/* PCI Device Pointer */
	unsigned int class;	/* Class Code */
	unsigned int buses;	/* Buses Config. Register */
	unsigned short cmd;	/* Command Register */
	unsigned char slat_timer;	/* Secondary Latency Timer */
	int ret_code;		/* Function Return Code */
	struct pci_dev *dev0;
	struct list_head *lh;
	struct list_head *lh_root;
	struct bus_type *bus;


	pr_debug("pci_swap_node: bus 0x%x devfn 0x%x secdevfn 0x%x\n", number, devfn, secdevfn);
	ret_code = PCI_OK;
	pbus = list_entry(pci_root_buses.next, struct pci_bus, node);
	lh_root = &pbus->children;
	lh = lh_root;
	while ((pbus->secondary != number) && (lh->next != lh_root)) {
		pbus = list_entry(lh->next, struct pci_bus, node);
		if ((number >= pbus->secondary) && (number <= pbus->subordinate)) {
			lh_root = &pbus->children;
			lh = lh_root;
		} else {
			lh = lh->next;
		}
	}
	if (pbus->secondary == number) {
       		dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
       		if (!dev0)
			return -ENOMEM;
		dev0->bus = pbus;
		dev0->sysdata = pbus->sysdata;
		pdev = pcisvrs_find_slot(number, devfn);
		if (pdev != (struct pci_dev *) NULL) {
			if (pci_get_bus_and_slot(number, secdevfn) == (struct pci_dev *) NULL) {
				dev0->devfn = devfn;
				if (pci_valid_devid(dev0) != TRUE) {
					dev0->devfn = secdevfn;
					if (pci_valid_devid(dev0) == TRUE) {
						pci_read_config_dword(dev0, PCI_CLASS_REVISION, &class);
						if ((class >> 16) != (pdev->class >> 8)) {
							printk(KERN_ERR "pci_swap_node: Invalid class for %02x:%02x", number, secdevfn);
							ret_code = -ENODEV;
						}
					} else {
						printk(KERN_ERR "pci_swap_node: Invalid device ID for %02x:%02x", number, secdevfn);
						ret_code = -ENODEV;
					}
				} else {
					printk(KERN_ERR "pci_swap_node: Expected invalid device ID for %02x:%02x", number, devfn);
					ret_code = -EEXIST;
				}
			} else {
				printk(KERN_ERR "pci_swap_node: Secondary device structure exists for %02x:%02x", number, secdevfn);
				ret_code = -EEXIST;
			}
		} else {
			printk(KERN_ERR "pci_swap_node: Couldn't find device structure for %02x:%02x", number, devfn);
			ret_code = -ENODEV;
		}

		if (ret_code == PCI_OK) {
			class = pdev->class >> 8;
			switch (class) {
			case PCI_CLASS_BRIDGE_HOST:
				{
					if ((pdev->bus)->number == 0) {
						printk(KERN_ERR "pci_swap_node: Cannot swap primary Host Bridge");
						ret_code = -EPERM;
						break;
					}
				}

			case PCI_CLASS_BRIDGE_PCI:
				{
					if (class == PCI_CLASS_BRIDGE_HOST) {
						lh_root = &pci_root_buses;
						lh = lh_root->next;
					} else {
						lh_root = &((pdev->bus)->children);
						lh = lh_root->next;
					}

					while (lh != lh_root) {
						pbus = list_entry(lh, struct pci_bus, node);
						if (pbus->self != pdev)
							lh = lh->next;
						else
							break;
					}

					if (pbus->self == pdev) {
#ifdef CONFIG_PCI_AGP_BRIDGE
						if (pci_agp_bridge(pdev->bus, devfn) == TRUE) {
							printk(KERN_ERR "pci_swap_node: Cannot swap AGP bridge at %02x:%02x", number, devfn);
							ret_code = -EPERM;
						}
#endif
						ret_code = pci_proc_detach_device(pdev);
						if (ret_code == PCI_OK) {
							char new_name[20];

							pdev->devfn = secdevfn;
							dev0->devfn = secdevfn;
							sprintf(new_name, "%04x:%02x:%02x.%d", 
								pci_domain_nr(pdev->bus),
                				pdev->bus->number, 
								PCI_SLOT(pdev->devfn),
								PCI_FUNC(pdev->devfn));

							pci_read_config_word(dev0, PCI_COMMAND, &cmd);
							cmd &= ~0x0007;
							pci_write_config_word(dev0, PCI_COMMAND, cmd);
							pci_read_config_byte(dev0, PCI_SEC_LATENCY_TIMER, &slat_timer);
							if (slat_timer < 16)
								slat_timer = 64;

							buses = slat_timer;
							buses = (buses << 8) | pbus->subordinate;
							buses = (buses << 8) | pbus->secondary;
							buses = (buses << 8) | pbus->primary;

							pci_write_config_dword (dev0, PCI_PRIMARY_BUS, buses);
							pci_write_config_word(dev0, PCI_STATUS, 0xffff);

							ret_code = pci_conf_bus(pbus, FALSE);

							bus = get_bus(pdev->dev.bus);
							sysfs_remove_link(&bus->devices.kobj, pdev->dev.bus_id);

							device_rename(&pdev->dev, new_name);

							ret_code = sysfs_create_link(&bus->devices.kobj,
											&(pdev->dev.kobj), pdev->dev.bus_id);
							if (ret_code) {
								printk(KERN_ERR "pci_swap_node: couldn't create sysfs link\n");
								put_bus(pdev->dev.bus);
							}
							
						}
					} else {
						printk(KERN_ERR "pci_swap_node: Couldn't find bus structure for %02x:%02x", number, devfn);
						ret_code = -ENOENT;
					}
					break;
				}

			default:
				{
					if ((pdev->bus)->number != 0) {
						ret_code = pci_proc_detach_device(pdev);
						if (ret_code == PCI_OK) {
							char new_name[20];

							pdev->devfn = secdevfn;
							dev0->devfn = secdevfn;
							sprintf(new_name, "%04x:%02x:%02x.%d", 
								pci_domain_nr(pdev->bus),
                				pdev->bus->number, 
								PCI_SLOT(pdev->devfn),
								PCI_FUNC(pdev->devfn));

							pci_write_config_word(dev0, PCI_STATUS, 0xffff);
							ret_code = pci_conf_dev(pdev, FALSE);

							bus = get_bus(pdev->dev.bus);
							sysfs_remove_link(&bus->devices.kobj, pdev->dev.bus_id);
							device_rename(&pdev->dev, new_name);
							ret_code = sysfs_create_link(&bus->devices.kobj,
											&(pdev->dev.kobj), pdev->dev.bus_id);
							if (ret_code) {
								printk(KERN_ERR "pci_swap_node: couldn't create sysfs link\n");
								put_bus(pdev->dev.bus);
							}
							
						}
					} else {
						printk(KERN_ERR "pci_swap_node: Cannot swap a bus 0 device");
						ret_code = -EPERM;
					}
				}
			}

			if (ret_code == PCI_OK) {
				if (pci_proc_attach_device(pdev) != 0) {
					printk(KERN_ERR "pci_swap_node: %02x:%02x [%04x/%04x/%06x] Cannot attach proc interface",
						number, secdevfn, pdev->vendor,
						pdev->device, class);
				}
			}
		} else {
			printk(KERN_ERR "pci_swap_node: Couldn't find parent bus structure for %02x:%02x",
				number, devfn);
			ret_code = -ENOENT;
		}
		kfree(dev0);
	}
	return (ret_code);
}

EXPORT_SYMBOL(pci_add_node);
EXPORT_SYMBOL(pci_del_node);
