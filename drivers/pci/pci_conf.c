/*
**    PCI Bus/Device Node Configuration
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_res.h>

#include <asm/errno.h>

extern unsigned long pci_bus_io_size[MAX_PCI_LEVELS];
extern unsigned long pci_bus_mem_size[MAX_PCI_LEVELS];
extern int pci_find_level(struct pci_bus *);
extern unsigned int pci_calc_resource_flags(unsigned int);

/**
 * pci_find_class - begin or continue searching for a PCI device by class
 * @class: search for a PCI device with this class designation
 * @from: Previous PCI device found in search, or %NULL for new search.
 *
 * Iterates through the list of known PCI devices.  If a PCI device is
 * found with a matching @class, a pointer to its device structure is
 * returned.  Otherwise, %NULL is returned.
 * A new search is initiated by passing %NULL to the @from argument.
 * Otherwise if @from is not %NULL, searches continue from next device
 * on the global list.
 */

static struct pci_dev *
pci_find_class(unsigned int class, const struct pci_dev *from)
{
        struct list_head *n = from ? from->global_list.next : pci_devices.next;
                                                                                
        while (n != &pci_devices) {
                struct pci_dev *dev = pci_dev_g(n);
                if (dev->class == class)
                        return dev;
                n = n->next;
        }
        return NULL;
}


/*
**
**    pci_conf_int_line - Configure Interrupt Line Register
**
**        This function will determine the correct value
**        for the device's Interrupt Line and set it.
*/

static void pci_conf_int_line(struct pci_dev *pdev)
{
	u8 irq;			/* Interrupt Pin/Line */

	pr_debug("pci_conf_int_line: dev 0x%p\n", pdev);
	pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &irq);
	if (irq != PCI_INT_NONE) {
#ifndef CONFIG_PCI_BUS_0_DEVICES
		if ((pdev->bus)->number == 0) {
			if ((pdev->class >> 8) != PCI_CLASS_BRIDGE_PCI) {
			/*
			**    Bus 0 device resources are configured by
			**    the BIOS and are static. Hence, just read
			**    the Interrupt Line register and use it.
			*/
				pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &irq);
				pdev->irq = irq;
				return;
			}
		}
#endif
		pci_hwints_getirq(pdev);
	}
}


/*
**
**    pci_conf_cache - Configure Cache Line Size Register
**
**        This function will write the Cache Line Size
**        register with the configuration value.
*/

static void pci_conf_cache(struct pci_dev *pdev)
{

	pr_debug("pci_conf_cache: dev 0x%p\n", pdev);
#ifndef CONFIG_PCI_BUS_0_DEVICES
	/*
	**    Check for a non-bridge Bus 0 device. If so,
	**    just return because resources are configured
	**    by the BIOS and are static.
	*/

	if ((pdev->bus)->number == 0)
		if ((pdev->class >> 8) != PCI_CLASS_BRIDGE_PCI)
			return;
#endif
	pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, CONFIG_PCI_CACHE_LINE_SIZE);
}


/*
**
**    pci_conf_latency - Configure Latency Timer Register
**
**        This function will write the latency timer
**        register with the configuratino value.
*/

static void pci_conf_latency(struct pci_dev *pdev)
{

	pr_debug("pci_conf_latency: dev 0x%p\n", pdev);
#ifndef CONFIG_PCI_BUS_0_DEVICES
	/*
	**    Check for a non-bridge Bus 0 device. If so,
	**    just return because resources are configured
	**    by the BIOS and are static.
	*/
	if ((pdev->bus)->number == 0)
		if ((pdev->class >> 8) != PCI_CLASS_BRIDGE_PCI)
			return;
#endif
	pci_write_config_byte(pdev, PCI_LATENCY_TIMER, CONFIG_PCI_LATENCY_TIMER);
}


/*
**    pci_conf_cmd - Configure Command Register
**
**        This function will write the Command register
**        for the device with the configured bits.
*/

static void pci_conf_cmd(struct pci_dev *pdev, u16 cmd)
{
	unsigned int number;	/* Bus Number */
	unsigned int dev;	/* Device Number */
	unsigned int func;	/* Function Number */
	unsigned int devfn;	/* Device/Function Number */
	unsigned short status;	/* Status Register */
	unsigned short class;	/* Class Code */
	u16 tmpcmd;		/* Temporary Command Register */
	struct pci_dev *dev0;

	pr_debug("pci_conf_cmd: dev 0x%p cmd 0x%x\n", pdev, cmd);
	pci_read_config_word(pdev, PCI_COMMAND, &tmpcmd);
	class = pdev->class >> 8;
	if ((class == PCI_CLASS_BRIDGE_HOST) || (class == PCI_CLASS_BRIDGE_PCI))
		cmd |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY);

	cmd |= PCI_COMMAND_MASTER;

	/*
	**    Set the Fast Back-to-Back Enable bit.
	**    Then, search each device on this bus,
	**    i.e. device 0 to 31.
	*/

	cmd |= PCI_COMMAND_FAST_BACK;
	number = (pdev->bus)->number;
	dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
	if (!dev0) {
		printk("PCI: ENOMEM in pci_conf_cmd\n");
		return;
	}
	dev0->bus = pdev->bus;
	dev0->sysdata = pdev->bus->sysdata;

	for (dev = 0; dev <= MAX_PCI_DEV; ++dev) {
		for (func = 0; func <= MAX_PCI_FUNC; ++func) {
			devfn = PCI_DEVFN(dev, func);
			dev0->devfn = devfn;
			if (pci_valid_devid(dev0) == TRUE) {
				/*
				**    Read the Status register and check the
				**    Fast Back-to-Back Capable bit. If not
				**    set, then clear the bit in the Command
				**    register.
				*/
				pci_read_config_word(dev0, PCI_STATUS, &status);
				if ((status & PCI_STATUS_FAST_BACK) == 0)
					cmd &= ~PCI_COMMAND_FAST_BACK;
				if (pci_multi_func(dev0) == FALSE)
					break;
			}
		}
	}
#ifdef CONFIG_PCI_COMMAND_SPECIAL
	cmd |= PCI_COMMAND_SPECIAL;
#endif
#ifdef CONFIG_PCI_COMMAND_INVALIDATE
	cmd |= PCI_COMMAND_INVALIDATE;
#endif
#ifdef CONFIG_PCI_COMMAND_PARITY
	cmd |= PCI_COMMAND_PARITY;
#endif
#ifdef CONFIG_PCI_COMMAND_WAIT
	cmd |= PCI_COMMAND_WAIT;
#endif
#ifdef CONFIG_PCI_COMMAND_SERR
	cmd |= PCI_COMMAND_SERR;
#endif
	cmd |= tmpcmd;
	pci_write_config_word(pdev, PCI_COMMAND, cmd);
	
	kfree(dev0);
	return;
}


/*
**    pci_conf_bridge_ctl - Configure Bridge Control Register
**
**        This function will write the Bridge Control
**        register for the bridge device with the
**        configured bits.
*/

static void pci_conf_bridge_ctl(struct pci_dev *pdev)
{
	unsigned int number;	/* Bus Number */
	unsigned int dev;	/* Device Number */
	unsigned int func;	/* Function Number */
	unsigned int devfn;	/* Device/Function Number */
	unsigned short status;	/* Status Register */
	u16 brdgctl;		/* Bridge Control Register */
	struct pci_dev *dev0;

	pr_debug("pci_conf_bridge_ctl: dev 0x%p\n", pdev);
	pci_read_config_word(pdev, PCI_BRIDGE_CONTROL, &brdgctl);

	brdgctl |= PCI_BRIDGE_CTL_FAST_BACK;
	number = (pdev->bus)->secondary;
	dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
	if (!dev0) {
		printk("PCI: ENOMEM in pci_conf_bridge_ctl\n");
		return ;
	}
	dev0->bus = pdev->bus;
	dev0->sysdata = pdev->bus->sysdata;

	for (dev = 0; dev <= MAX_PCI_DEV; ++dev) {
		for (func = 0; func <= MAX_PCI_FUNC; ++func) {
			devfn = PCI_DEVFN(dev, func);
			dev0->devfn = devfn;
			if (pci_valid_devid(dev0) == TRUE) {
				pci_read_config_word(dev0, PCI_STATUS, &status);
				if ((status & PCI_STATUS_FAST_BACK) == 0)
					brdgctl &= ~PCI_BRIDGE_CTL_FAST_BACK;
				if (pci_multi_func(dev0) == FALSE)
					break;
			}
		}
	}

#ifdef CONFIG_PCI_BRIDGE_CTL_PARITY
	brdgctl |= PCI_BRIDGE_CTL_PARITY;
#endif

#ifdef CONFIG_PCI_BRIDGE_CTL_SERR
	brdgctl |= PCI_BRIDGE_CTL_SERR;
#endif

#ifdef CONFIG_PCI_BRIDGE_CTL_ISA_MODE
	brdgctl |= PCI_BRIDGE_CTL_NO_ISA;
#endif

#ifdef CONFIG_PCI_BRIDGE_CTL_VGA
	brdgctl |= PCI_BRIDGE_CTL_VGA;
#endif

#ifdef CONFIG_PCI_BRIDGE_CTL_MASTER_ABORT
	brdgctl |= PCI_BRIDGE_CTL_MASTER_ABORT;
#endif

	pci_write_config_word(pdev, PCI_BRIDGE_CONTROL, brdgctl);
	kfree(dev0);
	return;
}


/*
**
**    pci_conf_bar - Determine Base Address Register Requirements
**
**        This function will determine the memory or I/O space
**        requirements for the given base address register.
**        The PCI specification defines that after 0xffffffff
**        is written to a base address register, the value
**        read back will indicate the size requirements of the
**        base address register. The highest bit set in register
**        can be converted to the address space size. 
*/

static unsigned int
pci_conf_bar(struct pci_dev *pdev, unsigned int bar, unsigned int *size)
{
	struct pci_bus *pbus;	/* Device's Bus Structure Pointer */
	unsigned int type;	/* Resource Type */
	u32 base_reg;		/* Base Address Register */
	u32 saved_base;		/* Saved Base Address Register */

	pr_debug("pci_conf_bar: dev 0x%p bar %d\n", pdev, bar);
	/*
	**    Write all F's to the base address registers so we
	**    can later read them back and determine what type
	**    of space the device needs and how much space it
	**    needs.
	*/

	bar = PCI_BASE_ADDRESS_0 + (bar << 2);
	pci_read_config_dword(pdev, bar, &saved_base);
	pci_write_config_dword(pdev, bar, 0xffffffff);
	pci_read_config_dword(pdev, bar, &base_reg);
	pci_write_config_dword(pdev, bar, saved_base);

	/*
	**    Set the resource type and clear the size. Then
	**    check the base register value. If it is zero,
	**    then the base register isn't implemented. If
	**    it is 0xffffffff, then it may be broken.
	*/

	type = PCI_RES_TYPE_NOTUSED;
	*size = 0;
	if ((base_reg != 0) && (base_reg != 0xffffffff)) {
		/*
		**    Get the parent bus structure pointer. Then
		**    see if the base register is an I/O type.
		*/
		pbus = pdev->bus;
		if ((base_reg & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO) {
			/*
			**    It is an I/O base register. Set the
			**    resource type if we have space.
			*/

			if (pbus->resources[PCI_RES_INDX_IO].size != 0)
				type = PCI_RES_TYPE_IO;
			/*
			**    Adjust the base address register content
			**    for the I/O bits and initialize the size.
			*/
			base_reg >>= 2;
			*size = 4;
		} else {
			/*
			**    It is a memory base register. Set the
			**    default resource type to anywhere in
			**    memory.
			*/

			type = PCI_RES_TYPE_MEM;
			/*    
			   **    Is it prefetchable memory?
			 */
			if ((base_reg & PCI_BASE_ADDRESS_MEM_PREFETCH) != 0) {
				/*
				   **    Prefetchable memory. See if we can locate
				   **    in prefetchable memory space.
				 */
				if (pbus->resources[PCI_RES_INDX_MEMPF].  size != 0)
					type = PCI_RES_TYPE_MEMPF;
			}

			/*
			   **    Locate below 1MB?
			 */

			if ((base_reg & PCI_BASE_ADDRESS_MEM_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_1M) {
				/*
				   **    Only bus 0 devices can be mapped below 1MB.
				   **    See if this device is a bus 0 device and if
				   **    we have 20-bit space.
				 */
				if (pbus->number == 0)
					if (pbus->resources[PCI_RES_INDX_MEM20].size != 0)
						type = PCI_RES_TYPE_MEM20;
			}

			/*
			   **    Adjust the base address register content
			   **    for the address memory bits and initialize
			   **    the size.
			 */
			base_reg >>= 4;
			*size = 16;
		}

		if (type != PCI_RES_TYPE_NOTUSED) {
			/*
			   **    Shift the base register right till a
			   **    0 bit is seen.
			 */
			while ((base_reg != 0) && ((base_reg & 1) == 0)) {
				/*
				   **    Multiply size by 2 and look for
				   **    another bit in the base address
				   **    register.
				 */
				*size <<= 1;
				base_reg >>= 1;
			}
		}
	}
	return (type);
}


/*
**    pci_get_bars - Get Number Of BARs
**
**        This function will return the number of Base
**        Address registers for the device.
*/

int pci_get_bars(struct pci_dev *pdev, int *num_bars)
{
	int ret_code;		/* Function Return Code */

	ret_code = PCI_OK;

	switch (pdev->hdr_type & 0x7f) {

	case PCI_HEADER_TYPE_NORMAL:
		{
			*num_bars = 6;
			break;
		}

	case PCI_HEADER_TYPE_BRIDGE:
		{
			*num_bars = 2;
			break;
		}

	case PCI_HEADER_TYPE_CARDBUS:
		{
			*num_bars = 1;
			break;
		}

	default:
		{
			*num_bars = 0;
			ret_code = -EFAULT;
		}
	}

	return (ret_code);
}


/*
**    pci_conf_dev - Configure Device
**
**        This function will configure memory and I/O space
**        for each function on the given device. 64-bit
**        memory and prefetch memory are not allocated at
**        this time.
*/

int pci_conf_dev(struct pci_dev *pdev, int res_flag)
{
	struct pci_res *pres;	/* Resource Structure Pointer */
	unsigned int num_bars;	/* Maximum Number Of BARs */
	unsigned int bar;	/* Base Address Register */
	unsigned int size;	/* Size Requirement */
	unsigned int type;	/* PCI Space Type */
	unsigned int barindx;	/* Base Address Reg. Index */
	u32 base_reg;		/* Base Address Register */
	u16 cmd;		/* Command Register */
	int configured;		/* Configuration Status */
	int ret_code;		/* Function Return Code */
	u8 device_io_bad = FALSE;
	u8 device_mem_bad = FALSE;
	int lvl;

	pr_debug("pci_conf_dev: dev 0x%p\n", pdev);
	configured = pci_is_device_configured(pdev);
	if (!configured) {
		pci_conf_cache(pdev);
		pci_conf_latency(pdev);
	}
	pci_conf_int_line(pdev);
	ret_code = pci_get_bars(pdev, &num_bars);

#ifdef CONFIG_PCI_EXT_BRIDGE
	pdev->IoNeeded = FALSE;
#endif	

	lvl = pci_find_level(pdev->bus);
	for (cmd = 0, bar = 0; (ret_code == PCI_OK) && (bar < num_bars); ++bar) {
		type = pci_conf_bar(pdev, bar, &size);
		if (type == 1 && size >= pci_bus_io_size[lvl]) {
			printk(KERN_INFO
			       "PCI: Bar %d requirements [0x%x] for device at bus 0x%x devfn 0x%x exceeds available PCI IO [0x%lx]\n",
			       bar, size, pdev->bus->number, pdev->devfn,
			       pci_bus_io_size[lvl]);
			device_io_bad = TRUE;
			cmd = cmd & !PCI_COMMAND_IO;
			continue;
		}

		if (type == 2 && size >= pci_bus_mem_size[lvl]) {
			printk(KERN_INFO
			       "PCI: Bar %d requirements [0x%x] for device at bus 0x%x devfn 0x%x exceeds available PCI memory [0x%lx]\n",
			       bar, size, pdev->bus->number, pdev->devfn,
			       pci_bus_mem_size[lvl]);
			device_mem_bad = TRUE;
			cmd = cmd & !PCI_COMMAND_MEMORY;
			continue;
		}
#ifdef CONFIG_PCI_EXT_BRIDGE
		if (type == 1)
			pdev->IoNeeded = TRUE;
#endif				/* CONFIG_PCI_EXT_BRIDGE */

		if ((size != 0) && (type != PCI_RES_TYPE_NOTUSED)) {
			/*
			   **    Get the current resource structure pointer.
			   **    Then, set the resource type, size, and
			   **    alignment. Then based upon the resource
			   **    type, set command space access enable bit.
			 */

			pres = &(pdev->resources[bar]);
			pres->type = type;
			pres->size = size;
			pres->align = size;

			if (!configured) {
				if (type == PCI_RES_TYPE_IO) {
					if (device_io_bad == FALSE)
						cmd |= PCI_COMMAND_IO;
				} else {
					if (device_mem_bad == FALSE)
						cmd |= PCI_COMMAND_MEMORY;
				}

			}
			ret_code = PCI_OK;
			if (res_flag == TRUE)
				ret_code = pci_alloc_devres(pdev, pres, bar);

			if (ret_code == PCI_OK) {
				struct resource *res;

				barindx = PCI_BASE_ADDRESS_0 + (bar << 2);
				base_reg = pres->baddr;

				if (!configured) {
					pci_write_config_dword(pdev, barindx, base_reg);
					pci_read_config_dword(pdev, barindx, &base_reg);
				} else {
					pci_read_config_dword(pdev, barindx, &base_reg);
				}

				res = &pdev->resource[bar];
				if ((base_reg & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_MEMORY) {
					res->start = base_reg & PCI_BASE_ADDRESS_MEM_MASK;
				} else {
					res->start = base_reg & PCI_BASE_ADDRESS_IO_MASK;
				}
				res->end =
				    res->start + (unsigned long) pres->size - 1;
				res->flags |= (base_reg & 0xf) | (pci_calc_resource_flags(base_reg));

				/*
				   **    Check for a 64-bit memory register.
				 */
				if ((base_reg & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_MEMORY) {
					if ( (base_reg & PCI_BASE_ADDRESS_MEM_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64) {
						/*
						   **    It is a 64-bit register. Thus, write the next
						   **    configuration Base Address Register with the
						   **    allocated upper address value. Also, read it
						   **    back and place the value in the base address
						   **    register array.
						 */

#if (BITS_PER_LONG == 64)
						base_reg = pres->baddr >> 32;
#else
						base_reg = 0;
						res->start = 0;
						res->flags = 0;
#endif
						barindx = PCI_BASE_ADDRESS_0 + (++bar << 2);
						if (!configured) {
							pci_write_config_dword (pdev, barindx, base_reg);
							pci_read_config_dword (pdev, barindx, &base_reg);
						} else {
							pci_read_config_dword (pdev, barindx, &base_reg);
						}

					}
				}

			}
		}
	}

	if (ret_code == PCI_OK) {
#ifdef __powerpc__
		pci_fixup_device(PCI_FIXUP_HEADER, pdev);
#endif
		if (!configured) {
			pci_conf_cmd(pdev, cmd);
			if ((pdev->class >> 8) == PCI_CLASS_BRIDGE_PCI)
				pci_conf_bridge_ctl(pdev);
		}
	}
	return (ret_code);
}


/*
**    pci_conf_bus - Configure Bus
**
**        This function will configure memory and I/O space
**        for the bus and its devices.
*/

int pci_conf_bus(struct pci_bus *pbus, int res_flag)
{
	struct pci_bus *child;	/* Child Bus Pointer */
	struct pci_dev *pdev;	/* Device Structure Pointer */
	struct pci_res *pres;	/* Pointer To Bus Resources */
	struct list_head *root_lh, *lh;
	unsigned int class;	/* Class Code */
	u32 memupper;		/* Base/Limit Upper 32 Bits */
	u16 membaddr;		/* Memory Base Address */
	u16 membase;		/* Memory Base register */
	u16 iobaddr;		/* I/O Base Address */
	u8 iobase;		/* I/O Base Register */
	int configured;		/* Configuration Status */
	int ret_code;		/* Function Return Code */
	struct pci_dev *dev0;

#ifdef CONFIG_PCI_EXT_BRIDGE
	int level;		/* Which level */
	unsigned int IoNeeded;	/* Whether IO is needed */
#endif

	pr_debug("pci_conf_bus: bus 0x%p\n", pbus);
	ret_code = PCI_OK;
	if (res_flag == TRUE)
		ret_code = pci_alloc_busres(pbus);

	if (ret_code == PCI_OK) {
		struct resource *res;

		pdev = pbus->self;

		if (pbus->parent != (struct pci_bus *) NULL) {
        		dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
			if (!dev0)
				return -ENOMEM;
			dev0->bus = pbus->parent;
			dev0->sysdata = pbus->sysdata;
			dev0->devfn = pdev->devfn;
			configured = pci_is_device_configured(dev0);
			kfree(dev0);
		} else
			configured = FALSE;

		if (!configured) {
			if ((pdev->class >> 8) == PCI_CLASS_BRIDGE_PCI) {
				int i;

				pci_write_config_byte(pdev, PCI_SEC_LATENCY_TIMER, CONFIG_PCI_SEC_LATENCY_TIMER);
				for (i = 0; i < 3; i++)
					pbus->resource[i] = &(pbus->self)->resource[PCI_BRIDGE_RESOURCES + i];

				res = pbus->resource[0];
				pci_write_config_byte(pdev, PCI_IO_BASE, 0xf0);
				pci_write_config_byte(pdev, PCI_IO_LIMIT, 0x00);
				pci_write_config_word(pdev, PCI_IO_BASE_UPPER16, 0xffff);
				pci_write_config_word(pdev, PCI_IO_LIMIT_UPPER16, 0x0000);
				pci_read_config_byte(pdev, PCI_IO_BASE, &iobase);
				iobase &= PCI_IO_RANGE_TYPE_MASK;

				/*
				   **    Next, set the I/O resource pointer and see
				   **    if we have any allocated I/O space for this
				   **    this node.
				 */

				pres = &(pbus->resources[PCI_RES_INDX_IO]);
				if (pres->size != 0) {
					u8 io_base_lo;

					/*
					   **    We have space. Is the I/O decoder correct?
					 */
					if ( (iobase == PCI_IO_RANGE_TYPE_16)
					    || (iobase == PCI_IO_RANGE_TYPE_32)) {

						iobaddr = pres->baddr >> 8;
						iobaddr &= 0xf0;
						res->start = pres->baddr;
						res->end = res->start + pres->size - 1;
						res->name = pbus->name;
						pci_read_config_byte(pdev, PCI_IO_BASE, &io_base_lo);
						res->flags = (io_base_lo & PCI_IO_RANGE_TYPE_MASK) | IORESOURCE_IO;
						pci_write_config_byte(pdev, PCI_IO_BASE, (u8) iobaddr);
						iobaddr = (pres->baddr + (pres->size - 1)) >> 8;
						iobaddr &= 0xf0;

						pci_write_config_byte(pdev, PCI_IO_LIMIT, (u8) iobaddr);

						if (iobase == PCI_IO_RANGE_TYPE_32) {
							/*
							   **    Set the I/O Base Upper 16-Bits
							   **    register.
							 */
							iobaddr = pres->baddr >> 16;
							iobaddr &= 0xffff;
							pci_write_config_word (pdev, PCI_IO_BASE_UPPER16, iobaddr);
							/*
							   **    Set the I/O Limit Upper 16-Bits
							   **    register.
							 */
							iobaddr = (pres->baddr + (pres->size - 1)) >> 16;
							iobaddr &= 0xffff;

							pci_write_config_word (pdev, PCI_IO_LIMIT_UPPER16, iobaddr);
						}
					}
				} else {
					pbus->resource[0] = pbus->parent->resource[0];
				}

				/*
				   **    Next, shut off PCI memory space. Then set
				   **    the memory resource pointer and see if we
				   **    have any allocated 32-bit memory for this
				   **    node.
				 */

				res = pbus->resource[1];
				pci_write_config_word(pdev, PCI_MEMORY_BASE, 0xffff);
				pci_write_config_word(pdev, PCI_MEMORY_LIMIT, 0x0000);
				pres = &(pbus->resources[PCI_RES_INDX_MEM]);
				if (pres->size != 0) {
					u16 mem_base_lo;

					membaddr = pres->baddr >> 16;
					membaddr &= 0xffff;
					res->start = pres->baddr;
					res->end = res->start + pres->size - 1;
					res->name = pbus->name;
					pci_read_config_word(pdev, PCI_MEMORY_BASE, &mem_base_lo);
					res->flags = (mem_base_lo & PCI_MEMORY_RANGE_TYPE_MASK) | IORESOURCE_MEM;

					pci_write_config_word(pdev, PCI_MEMORY_BASE, membaddr);

					/*
					   **    Set the Memory Limit register.
					 */

					membaddr = (pres->baddr + (pres->size - 1)) >> 16;
					membaddr &= 0xffff;
					pci_write_config_word(pdev, PCI_MEMORY_LIMIT, membaddr);
				} else {
					pbus->resource[1] = pbus->parent->resource[1];
				}

				res = pbus->resource[2];
				pci_write_config_word(pdev, PCI_PREF_MEMORY_BASE, 0xfff0);
				pci_write_config_word(pdev, PCI_PREF_MEMORY_LIMIT, 0x0000);
				pci_write_config_dword(pdev, PCI_PREF_BASE_UPPER32, 0xffffffff);
				pci_write_config_dword(pdev, PCI_PREF_LIMIT_UPPER32, 0x00000000);
				pci_read_config_word(pdev, PCI_PREF_MEMORY_BASE, &membase);
				membase &= PCI_PREF_RANGE_TYPE_MASK;

				/*
				   **    Next, set the prefetchable memory resource pointer
				   **    and see if we have any allocated prefetchable
				   **    memory for this node and if the memory decoder is
				   **    correct.
				 */
				pres = &(pbus->resources[PCI_RES_INDX_MEMPF]);
				if (pres->size != 0) {
					u16 mem_base_lo;
					/*
					   **    We have space. Is the memory decoder correct?
					 */
					if ( (membase == PCI_PREF_RANGE_TYPE_32) || (membase == PCI_PREF_RANGE_TYPE_64)) {
						/*
						   **    Set the Prefetchable Memory
						   **    Base register.
						 */

						membaddr = pres->baddr >> 16;
						membaddr &= 0xfff0;

						res->start = pres->baddr;
						res->end = res->start + pres->size - 1;
						res->name = pbus->name;
						pci_read_config_word(pdev, PCI_PREF_MEMORY_BASE, &mem_base_lo);
						res->flags = (mem_base_lo & PCI_MEMORY_RANGE_TYPE_MASK) | IORESOURCE_MEM |
						    IORESOURCE_PREFETCH;
						pci_write_config_word(pdev, PCI_PREF_MEMORY_BASE, membaddr);

						/*
						   **    Set the Prefetchable Memory
						   **    Limit register.
						 */

						membaddr = (pres->baddr + (pres->size - 1)) >> 16;
						membaddr &= 0xfff0;
						pci_write_config_word(pdev, PCI_PREF_MEMORY_LIMIT, membaddr);

						/*
						   **    Next, determine if we need to set the
						   **    Prefetchable Base/Limit Upper 32 Bits registers.
						 */

						if (membase == PCI_PREF_RANGE_TYPE_64)
						{
							/*
							   **    Set the Prefetchable Base Upper 32-Bits
							   **    register.
							 */

#if (BITS_PER_LONG == 64)
							memupper = pres->baddr >> 32;
#else
							memupper = 0;
#endif
							pci_write_config_dword (pdev, PCI_PREF_BASE_UPPER32, memupper);

							/*
							   **    Set the Prefetchable Limit Upper 32-Bits
							   **    register.
							 */

#if (BITS_PER_LONG == 64)
							memupper = (pres->baddr + (pres->size - 1)) >> 32;
#else
							memupper = 0;
#endif

							pci_write_config_dword (pdev, PCI_PREF_LIMIT_UPPER32, memupper);
						}
					}
				} else {
					pbus->resource[2] = pbus->parent->resource[2];
				}

				/*
				**    Finally, do the bridge device.
				*/
				ret_code = pci_conf_dev(pbus->self, res_flag);
			}

		} else
			ret_code = pci_conf_dev(pbus->self, res_flag);


		/*
		**    Next, configure each bridge and it's devices
		**    on the bus.
		*/

		root_lh = &pbus->children;
		lh = root_lh->next;
		while ((ret_code == PCI_OK) && (lh != root_lh)) {
			child = list_entry(lh, struct pci_bus, node);

			/*
			**    Allocate/assign memory and I/O space to
			**    each bridge and it's devices on this bus.
			*/

			ret_code = pci_conf_bus(child, res_flag);

			/*
			   **    Do the next child.
			 */
			lh = lh->next;
		}

		/*
		   **    Next, configure each non-bridge device on
		   **    the bus.
		 */

		root_lh = &pbus->devices;
		lh = root_lh->next;

#ifdef CONFIG_PCI_EXT_BRIDGE
		IoNeeded = FALSE;
#endif

		while ((ret_code == PCI_OK) && (lh != root_lh)) {
			pdev = list_entry(lh, struct pci_dev, bus_list);
			/*
			   **    Get the device's class. If it's either a
			   **    Host Bridge or a PCI-2-PCI bridge, then
			   **    don't configure the device.
			 */

			class = pdev->class >> 8;
			if (class != PCI_CLASS_BRIDGE_HOST)
				if (class != PCI_CLASS_BRIDGE_PCI) {
					ret_code = pci_conf_dev(pdev, res_flag);

#ifdef CONFIG_PCI_EXT_BRIDGE
					if (pdev->IoNeeded) {
						IoNeeded = TRUE;
					}
#endif
				}
			/*
			   **    Do the next sibling.
			 */
			lh = lh->next;

		}


#ifdef CONFIG_PCI_EXT_BRIDGE
		if (!configured && (&(pbus->devices) != pbus->devices.next)) {
			level = pci_find_level(pbus);
			if (((!IoNeeded) && (level == 3))
			    && ((pbus->self->class >> 8) == PCI_CLASS_BRIDGE_PCI)) {

				/*
				 * Prog. Null in IoLim reg for parent bridge
				 */

				pci_write_config_byte(pbus->self, PCI_IO_BASE, 0xf0);
				pci_write_config_byte(pbus->self, PCI_IO_LIMIT, 0x00);
			}
		}
#endif				/* CONFIG_PCI_EXT_BRIDGE */

	}
#ifdef __powerpc__
	pcibios_fixup_bus(pbus);
#endif

	return (ret_code);
}


/*
**
**    pci_conf_display - Configure VGA and GFX Display Controllers
**
**        This function is used to detect and set up all
**        VGA-compatible controllers and non-VGA, GFX,
**        controllers.
*/

void pci_conf_display(void)
{
	struct pci_bus *pbus;	/* Bus Structure Pointer */
	struct pci_dev *pdev;	/* Device Structure Pointer */
	struct pci_dev *pdev_vga;	/* VGA Device Structure Pointer */
	struct pci_dev *pdev_gfx;	/* GFX Device Structure Pointer */
	unsigned int number;	/* Bus Number */
	unsigned int dev;	/* Device Number */
	unsigned int func;	/* Function Number */
	unsigned short class;	/* Class Code */
	u16 cnfgreg;		/* Command/Bridge Control Register */

	/*
	   **    Identify the VGA display device to be utilized
	   **    during initialization. This is accomplished by
	   **    first scanning the standard expansion bus (i.e.,
	   **    ISA, EISA or Micro Channel). If the VGA display
	   **    device is found on the expansion bus, that display
	   **    is used during initialization and the display
	   **    configuration process has completed.
	 */

	pdev_vga = (struct pci_dev *) NULL;

	pdev = pci_find_class((PCI_CLASS_BRIDGE_ISA << 8), (struct pci_dev *) NULL);
	if (pdev == (struct pci_dev *) NULL)
		pdev = pci_find_class((PCI_CLASS_BRIDGE_EISA << 8), (struct pci_dev *) NULL);

	if (pdev == (struct pci_dev *) NULL)
		pdev = pci_find_class((PCI_CLASS_BRIDGE_MC << 8), (struct pci_dev *) NULL);

	if (pdev != (struct pci_dev *) NULL) {
		/*
		   **    An expansion bus bridge device has been found.
		   **    Save the bridge's bus and device numbers. Then,
		   **    loop through each function looking for a VGA
		   **    display device.
		 */

		number = (pdev->bus)->number;
		dev = PCI_SLOT(pdev->devfn);
		for (func = 0; func <= MAX_PCI_FUNC; ++func) {
			/*
			   **    Is there a device here?
			 */
			pdev = pci_get_bus_and_slot(number, PCI_DEVFN(dev, func));
			if (pdev != (struct pci_dev *) NULL) {
				/*
				   **    Read the Class register and check for
				   **    a VGA-compatible controller. If so,
				   **    then we are done here.
				 */
				if ((pdev->class >> 8) == PCI_CLASS_DISPLAY_VGA) return;
			}
		}
	}

	/*
	   **    Is a VGA device on the PCI bus?
	 */

	pdev_vga = pci_find_class((PCI_CLASS_DISPLAY_VGA << 8), (struct pci_dev *) NULL);

	if (pdev_vga != (struct pci_dev *) NULL) {
		/*
		   **    Set the I/O ENABLE and MEMORY ENABLE
		   **    bits in the device's command register
		   **    so it can respond to VGA accesses.
		 */
		pci_read_config_word(pdev_vga, PCI_COMMAND, &cnfgreg);
		cnfgreg |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY);
		pci_write_config_word(pdev_vga, PCI_COMMAND, cnfgreg);
		/*
		   **    Starting at the PCI bus number the VGA
		   **    display device is on, scan the PCI bus
		   **    hierarchy upstream (towards the Host Bridge).
		 */
		pbus = pdev_vga->bus;
		while (pbus->primary != pbus->secondary) {
			/*
			   **    In each PCI-to-PCI bridge detected, set
			   **    the VGA ENABLE bit in its bridge control
			   **    register.
			 */
			pci_read_config_word(pbus->self, PCI_BRIDGE_CONTROL, &cnfgreg);
			cnfgreg |= PCI_BRIDGE_CTL_VGA;
			pci_write_config_word(pbus->self, PCI_BRIDGE_CONTROL, cnfgreg);
			pbus = pbus->parent;
		}

		/*
		   **    Starting at the PCI bus number the VGA
		   **    display device is on, scan the PCI buses
		   **    downstream (all buses subordinate to
		   **    this bus) looking for GFXs.
		 */

		pdev_gfx = (struct pci_dev *) NULL;
		for (number = (pdev_vga->bus)->number; number <= (pdev_vga->bus)->subordinate; ++number) {
			/*
			   **    Device loop, from 0 to 31.
			 */
			for (dev = 0; dev <= MAX_PCI_DEV; ++dev) {
				/*
				   **    Function loop, from 0 to 7.
				 */
				for (func = 0; func <= MAX_PCI_FUNC; ++func) {
					/*
					   **    Is there a device here?
					 */

					pdev = pci_get_bus_and_slot(number, PCI_DEVFN(dev, func));
					if (pdev != (struct pci_dev *) NULL) {
						/*
						   **    Read the Class register and
						   **    check for a GFX controller.
						 */
						class = pdev->class >> 8;
						if (class != PCI_CLASS_DISPLAY_XGA)
							    if (class != PCI_CLASS_DISPLAY_OTHER)
								    pdev = (struct pci_dev *) NULL;
					}

					/*
					   **    Was a GFX controller found?
					 */

					if (pdev != (struct pci_dev *) NULL) {
						/*
						   **    Set the GFX device's I/O ENABLE
						   **    and VGA SNOOP ENABLE bits in the
						   **    device's command register.
						 */

						pdev_gfx = pdev;
						pci_read_config_word (pdev_gfx, PCI_COMMAND, &cnfgreg);
						cnfgreg |= (PCI_COMMAND_IO | PCI_COMMAND_VGA_PALETTE);
						pci_write_config_word (pdev_gfx, PCI_COMMAND, cnfgreg);
						/*
						   **    Scan back upstream from the bus
						   **    the GFX is on towards the bus the
						   **    VGA display device resides on.
						 */

						pbus = pdev_gfx->bus;
						while (pbus->number != (pdev_vga->bus)->number) {
							/*
							   **    At each PCI-to-PCI bridge
							   **    encountered, set the VGA
							   **    SNOOP ENABLE bit in the
							   **    bridge command register.
							 */

							pci_read_config_word (pbus->self, PCI_COMMAND, &cnfgreg);
							cnfgreg |= PCI_COMMAND_VGA_PALETTE;
							pci_write_config_word (pbus->self, PCI_COMMAND, cnfgreg);
							pbus = pbus->parent;
						}
					}
				}
			}
		}

		/*
		   **    Was a GFX controller found?
		 */

		if (pdev_gfx != (struct pci_dev *) NULL) {
			/*
			   **    A GFX controller was found. Set the
			   **    the VGA SNOOP ENABLE bit in the VGA
			   **    display device's command register.
			 */

			pci_read_config_word(pdev_vga, PCI_COMMAND, &cnfgreg);
			cnfgreg |= PCI_COMMAND_VGA_PALETTE;
			pci_write_config_word(pdev_vga, PCI_COMMAND, cnfgreg);
		} else {
			/*
			   **    A GFX controller was not found. Clear
			   **    the the VGA SNOOP ENABLE bit in the
			   **    VGA display device's command register.
			 */
			pci_read_config_word(pdev_vga, PCI_COMMAND, &cnfgreg);
			cnfgreg &= ~PCI_COMMAND_VGA_PALETTE;
			pci_write_config_word(pdev_vga, PCI_COMMAND, cnfgreg);
		}
		return;
	}

	/*
	   **    We have no VGA device on the PCI bus. Hence,
	   **    we need to reset the SNOOP ENABLE bits.
	 */

	while ((pdev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pdev)) != NULL) {
		/*
		   **    Read the Class register and check for
		   **    a GFX controller.
		 */

		class = pdev->class >> 8;
		if ((class == PCI_CLASS_DISPLAY_XGA)
		    || (class == PCI_CLASS_DISPLAY_OTHER)) {
			/*
			   **    It is a GFX controller. Hence, set
			   **    the the VGA SNOOP ENABLE bit in the
			   **    device's command register.
			 */
			pci_read_config_word(pdev, PCI_COMMAND, &cnfgreg);
			cnfgreg |= PCI_COMMAND_VGA_PALETTE;
			pci_write_config_word(pdev, PCI_COMMAND, cnfgreg);
		} else {
			/*
			   **    It is not a GFX controller. Hence, clear
			   **    the the VGA SNOOP ENABLE bit in the device's
			   **    command register.
			 */
			pci_read_config_word(pdev, PCI_COMMAND, &cnfgreg);
			cnfgreg &= ~PCI_COMMAND_VGA_PALETTE;
			pci_write_config_word(pdev, PCI_COMMAND, cnfgreg);
		}

		if (class == PCI_CLASS_BRIDGE_PCI) {
			/*
			   **    In each bridge detected, clear the VGA
			   **    ENABLE bit in its bridge control register.
			 */
			pci_read_config_word(pdev, PCI_BRIDGE_CONTROL, &cnfgreg);
			cnfgreg &= ~PCI_BRIDGE_CTL_VGA;
			pci_write_config_word(pdev, PCI_BRIDGE_CONTROL, cnfgreg);
		}
	}
}
