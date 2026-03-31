/*
**    PCI Bus Number/Device Resource Allocation
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_res.h>

#include <asm/errno.h>
#include <asm/system.h>
#include <asm/io.h>
#ifdef __powerpc__
#include <asm/pci-bridge.h>
#endif

unsigned char pci_processor;

#if (CONFIG_PCI_BUSLVL0_NODES > 1)
/*
**    PCI Host Bridge Structure
**
**        This structure is used by the pci_alloc_hosts()
**        functions only.
*/

struct pci_host {
	unsigned int devfn;	/* Host Device/Function Number */
	unsigned int number;	/* Host Current Bus Number */
	unsigned int primary;	/* Host New Primary Number */
	unsigned int secondary;	/* Host New Secondary Number */
	unsigned int subordinate;	/* Host New Subordinate Number */
};

#endif

/*
**    Array of maximum number of PCI bus nodes per
**    PCI bus level.
*/

static int pci_max_busnodes_perlevel[MAX_PCI_LEVELS] =
{
	CONFIG_PCI_BUSLVL0_NODES,	/* Bus Level 0 Nodes  */
	CONFIG_PCI_BUSLVL1_NODES,	/* Bus Level 1 Nodes */
	CONFIG_PCI_BUSLVL2_NODES,	/* Bus Level 2 Nodes */
	CONFIG_PCI_BUSLVL3_NODES	/* Bus Level 3 Nodes */
};

/*
   **    Array of PCI bus numbers at each level. The
   **    array will be filled in as follows, with H0
   **    denoting host node 0, H1 denoting host node 1,
   **    etc.:
   **
   **    BusH0,                Level 0
   **     BusH0, BusH0,...,    Level 1
   **      BusH0, BusH0,...,   Level 2
   **       BusH0, BusH0,...,  Level 3
   **
   **    BusH1,                Level 0
   **     BusH1, BusH1,...,    Level 1
   **      BusH1, BusH1,...,   Level 2
   **       BusH1, BusH1,...,  Level 3
   **
   **    BusH2,                Level 0
   **     BusH2, BusH2,...,    Level 1
   **      BusH2, BusH2,...,   Level 2
   **       BusH2, BusH2,...,  Level 3
   **
   **    For example, with two Bus Level 0 nodes, two Bus Level 1
   **    nodes, four Bus Level 2 nodes, and two Bus Level 3 nodes,
   **    (for a maximum node count of 54) the array would look
   **    like this:
   **
   **    {
   **        0,                Level 0 - Host 0
   **        1, 14,            Level 1
   **        2,  5,  8, 11,    Level 2
   **       15, 18, 21, 24,
   **        3,  4,            Level 3
   **        6,  7,
   **        9, 10,
   **       12, 13,
   **       16, 17,
   **       19, 20,
   **       22, 23,
   **       25, 26,
   **
   **       27,                Level 0 - Host 1
   **       28, 41,            Level 1
   **       29, 32, 35, 38,    Level 2
   **       42, 45, 48, 51, 
   **       30, 31,            Level 3
   **       33, 34,
   **       36, 37,
   **       39, 40,
   **       43, 44,
   **       46, 47,
   **       49, 50,
   **       52, 53
   **    }
 */

static int pci_busnum_perlevel[MAX_PCI_NODES];

/*
   **    Array of indexes into the pci_busnum_perlevel[]
   **    array to provide the starting index of bus
   **    children. A -1 value indicates there are no
   **    child bus numbers. The input to this array is
   **    a bus number.
   **
   **    For example, with two Bus Level 0 nodes, two Bus Level 1
   **    nodes, four Bus Level 2 nodes, and two Bus Level 3 nodes,
   **    (for a maximum node count of 54) the array would look
   **    like this:
   **
   **    {       1, *Bus   0    * - Host 0
   **            3, *Bus   1    *
   **           11, *Bus   2    *
   **       -1, -1, *Buses 3-4  *
   **           13, *Bus   5    *
   **       -1, -1, *Buses 6-7  *
   **           15, *Bus   8    *
   **       -1, -1, *Buses 9-10 *
   **           17, *Bus   11   *
   **       -1, -1, *Buses 12-13*
   **            7, *Bus   14   *
   **           19, *Bus   15   *
   **       -1, -1, *Buses 16-17*
   **           21, *Bus   18   *
   **       -1, -1, *Buses 19-20*
   **           23, *Bus   21   *
   **       -1, -1, *Buses 22-23*
   **           25, *Bus   24   *
   **       -1, -1, *Buses 25-26*
   **
   **           28, *Bus   27   * - Host 1
   **           30, *Bus   28   *
   **           38, *Bus   29   *
   **       -1, -1, *Buses 30-31*
   **           40, *Bus   32   *
   **       -1, -1, *Buses 33-34*
   **           42, *Bus   35   *
   **       -1, -1, *Buses 36-37*
   **           44, *Bus   38   *
   **       -1, -1, *Buses 39-40*
   **           34, *Bus   41   *
   **           46, *Bus   42   *
   **       -1, -1, *Buses 43-44*
   **           48, *Bus   45   *
   **       -1, -1, *Buses 46-47*
   **           50, *Bus   48   *
   **       -1, -1, *Buses 49-50*
   **           52, *Bus   51   *
   **       -1, -1  *Buses 52-53*
   **    }
 */

static int pci_busnum_indx[MAX_PCI_NODES];

/*
   **    Array of subordinate bus numbers for each bus.
   **    Leaf bus nodes just return their own bus number.
   **    The input to this array is a bus number.
   **
   **    For example, with two Bus Level 0 nodes, two Bus Level 1
   **    nodes, four Bus Level 2 nodes, and two Bus Level 3 nodes,
   **    (for a maximum node count of 54) the array would look
   **    like this:
   **
   **    {
   **        * HOST 0 *
   **        26, *Bus  0*    13, *Bus  1*     4, *Bus  2*     3, *Bus  3*
   **         4, *Bus  4*     7, *Bus  5*     6, *Bus  6*     7, *Bus  7*
   **        10, *Bus  8*     9, *Bus  9*    10, *Bus 10*    13, *Bus 11*
   **        12, *Bus 12*    13, *Bus 13*    26, *Bus 14*    17, *Bus 15*
   **        16, *Bus 16*    17, *Bus 17*    20, *Bus 18*    19, *Bus 19*
   **        20, *Bus 20*    23, *Bus 21*    22, *Bus 22*    23, *Bus 23*
   **        26, *Bus 24*    25, *Bus 25*    26, *Bus 26*
   **
   **        * HOST 1 *
   **        53, *Bus 27*    40, *Bus 28*    31, *Bus 29*    30, *Bus 30*
   **        31, *Bus 31*    34, *Bus 32*    33, *Bus 33*    34, *Bus 34*
   **        37, *Bus 35*    36, *Bus 36*    37, *Bus 37*    40, *Bus 38*
   **        39, *Bus 39*    40, *Bus 40*    53, *Bus 41*    44, *Bus 42*
   **        43, *Bus 43*    44, *Bus 44*    47, *Bus 45*    46, *Bus 46*
   **        47, *Bus 47*    50, *Bus 48*    49, *Bus 49*    50, *Bus 50*
   **        53, *Bus 51*    52, *Bus 52*    53  *Bus 53*
   **    }
 */

static int pci_sub_busnum[MAX_PCI_NODES];

/*
   **    Array to hold bus number allocation status,
   **    i.e. allocated (PCI_BUS_ALLOC) or not
   **    allocated (PCI_BUS_NOT_ALLOC).
 */

static unsigned char pci_busnum_alloc_status[MAX_PCI_NODES];

/*
   **    Array to hold amount (in bytes) of available PCI
   **    I/O space size per bus level.
 */

unsigned long pci_bus_io_size[MAX_PCI_LEVELS];

/*
   **    Array to hold amount (in bytes) of available PCI
   **    memory space size per bus level.
 */

unsigned long pci_bus_mem_size[MAX_PCI_LEVELS];

/*
   **    Array to hold amount (in bytes) of available PCI
   **    20-bit memory space size per bus level.
 */

static unsigned long pci_bus_mem20_size[MAX_PCI_LEVELS];

/*
   **    Array to hold amount (in bytes) of available PCI
   **    prefetchable memory space size per bus level.
 */

static unsigned long pci_bus_mempf_size[MAX_PCI_LEVELS];

/*
   **    Some common error messages.
 */

static char no_bus_struct[] = "no PCI-2-PCI bridge bus structure";

static char no_dev_struct[] = "no PCI-2-PCI bridge device structure";

#ifdef __powerpc__
#ifdef CONFIG_PCI_MCP750_BUS_0_DEVICES

#define FIND_FIRST_BIT(n)       ((n) - ((n) & ((n) - 1)))
#define BAR(n)                  (PCI_BASE_ADDRESS_0 + (4 * (n)))

unsigned int mem_offset = 0x100000;
unsigned int io_offset = 0x10000;
unsigned int PciMemoryStart;
unsigned int PciMemoryEnd = 0x7cefffff;

static void 
config_mcp750_bus_0_devices(void)
{
	u32 signature;
	u16 class;
	u8 devfn, hdr_type;
	u8 is_multi = 0;
	u32 base, size;
	int i, bar;

	/* skip host bridge and ISA bridge */
        /* Due to problems with ghost devices with the newer HSC, we have
           to stop at devfn 0xb0 instead of checking till devfn 0xff */

	for (devfn = 0x60; devfn <= 0xb0; ++devfn) {
		if (PCI_FUNC(devfn) && !is_multi)
			continue;
		if (early_read_config_byte(0, 0, devfn, PCI_HEADER_TYPE, &hdr_type))
			continue;
		if (!PCI_FUNC(devfn))
			is_multi = hdr_type & 0x80;
		if (early_read_config_dword(0, 0, devfn, PCI_VENDOR_ID, &signature)
		    || (signature == 0xffffffff)
		    || (signature == 0x00000000)
		    || (signature == 0x0000ffff)
		    || (signature == 0xffff0000)) {
			is_multi = 0;
			continue;
		}
		early_read_config_word(0, 0, devfn, PCI_CLASS_DEVICE, &class);
		class >>= 8;

		switch (hdr_type & 0x7f) {
		case PCI_HEADER_TYPE_NORMAL:
			if (class == PCI_BASE_CLASS_BRIDGE)
				break;
			for (i = 0; i < 6; i++) {
				bar = BAR(i);

				early_read_config_dword(0, 0, devfn, bar, &base);
				early_write_config_dword(0, 0, devfn, bar, 0xFFFFFFFF);
				early_read_config_dword(0, 0, devfn, bar, &size);

				size &= PCI_BASE_ADDRESS_MEM_MASK;
				size = FIND_FIRST_BIT(size);
				if (size == 0)
					continue;
				if ((base & PCI_BASE_ADDRESS_SPACE) == 0) {	/* memory */
					mem_offset =
					    PCI_ALIGN_ADDR(mem_offset, size);
                    			/* Workaround for HSC (W3451F04A, W3377F04A) bug  - ghost de
vices */
                    			if ((devfn == 0xb0) && ((mem_offset << 16) == 0))
                        			mem_offset +=size;
					early_write_config_dword(0, 0, devfn, bar, mem_offset);
					mem_offset += size;
				} else {
					io_offset = PCI_ALIGN_ADDR(io_offset, size);
					early_write_config_dword(0, 0, devfn, bar, io_offset);
					io_offset += size;
				}
			}
			break;
		case PCI_HEADER_TYPE_BRIDGE:
		case PCI_HEADER_TYPE_CARDBUS:
		default:
			break;
		}
	}
    PciMemoryStart = mem_offset;
}

#endif				/* CONFIG_PCI_MCP750_BUS_0_DEVICES */
#endif				/*  __powerpc__ */

/*
   **    pci_alloc_businit - PCI Bus Number Allocation Initialization
   **
   **        This is the PCI bus number allocation initialization
   **        function. It will fill in the pci_busnum_perlevel[],
   **        pci_busnum_indx[], and pci_sub_busnum[] arrays per the
   **          Bus Level 0 (CONFIG_PCI_BUSLVL0_NODES),
   **          Bus Level 1 (CONFIG_PCI_BUSLVL1_NODES),
   **          Bus Level 2 (CONFIG_PCI_BUSLVL2_NODES), and
   **          Bus Level 3 (CONFIG_PCI_BUSLVL3_NODES)
   **        node configuration parameters.
   **
   **        It will then set the entries in the bus number
   **        allocation array, pci_busnum_alloc_status[], to "not
   **        allocated" (PCI_BUS_NOT_ALLOC).
   **
   **        Finally, it will initialize the PCI I/O and memory
   **        space arrays (i.e. pci_bus_io_size[], pci_bus_mem_size[],
   **        pci_bus_mem20_size[], and pci_bus_mempf_size[]) for
   **        available space per node at each bus level.
 */

int __init 
pci_alloc_businit(struct pci_bus *pbus)
{
	unsigned int busnum;	/* Current Bus Number */

	unsigned int l0sub;	/* Bus Level 0 Subordinate Bus Number */
	unsigned int l0cnt;	/* Bus Level 0 Count */
	unsigned int l0indx;	/* Bus Level 0 Index */

	unsigned int l1sub;	/* Bus Level 1 Subordinate Bus Number */
	unsigned int l1cnt;	/* Bus Level 1 Count */
	unsigned int l1indx;	/* Bus Level 1 Index */

	unsigned int l2sub;	/* Bus Level 2 Subordinate Bus Number */
	unsigned int l2cnt;	/* Bus Level 2 Count */
	unsigned int l2indx;	/* Bus Level 2 Index */

	unsigned int l3sub;	/* Bus Level 3 Subordinate Bus Number */
	unsigned int l3cnt;	/* Bus Level 3 Count */
	unsigned int l3indx;	/* Bus Level 3 Index */

	unsigned int indx;	/* Just An Array Index */
	unsigned long size;	/* PCI Memory or I/O Space Size */
	int ret_code;		/* Function Return Code */
	char *func;		/* Function Name */

	/*
	   **    Validate the maximum node count against the
	   **    maximum allowed for bus nodes.
	 */

	func = "pci_alloc_businit";

	pr_debug("pci_alloc_bus_init: bus 0x%p\n", pbus);
	ret_code = PCI_OK;
	if ((unsigned int) MAX_PCI_NODES == 0)
		ret_code = PCI_ERR;
	if ((unsigned int) MAX_PCI_NODES > 255)
		ret_code = PCI_ERR;
	if (ret_code != PCI_OK) {
		pci_kern_err(func, "invalid number of bus nodes");
		return (PCI_ERR);
	}
	/*
	   **    Validate the I/O space address definitions.
	 */

	if ((unsigned int) CONFIG_PCI_IO_START >
		 (unsigned int) CONFIG_PCI_IO_END)
		ret_code = PCI_ERR;

	if ( ((unsigned int) CONFIG_PCI_IO_END -
		    (unsigned int) CONFIG_PCI_IO_START) < 0x1000)
		ret_code = PCI_ERR;

	if (ret_code != PCI_OK) {
		pci_kern_err(func, "invalid PCI I/O space addresses");
		return (PCI_ERR);
	}
	/*
	   **    Validate the 20-bit memory space address definitions.
	 */

	if ((unsigned int) CONFIG_PCI_MEM20_END > 0x000fffff)
		ret_code = PCI_ERR;

	if ((unsigned int) CONFIG_PCI_MEM20_START > (unsigned int) CONFIG_PCI_MEM20_END)
		ret_code = PCI_ERR;

	if (ret_code != PCI_OK) {
		pci_kern_err(func, "invalid PCI 20-bit memory space addresses");
		return (PCI_ERR);
	}
	if ((unsigned int) CONFIG_PCI_MEM_START > (unsigned int) CONFIG_PCI_MEM_END)
		ret_code = PCI_ERR;

	if (((unsigned int) CONFIG_PCI_MEM_END - (unsigned int) CONFIG_PCI_MEM_START) < 0x00100000UL)
		ret_code = PCI_ERR;

	if (ret_code != PCI_OK) {
		pci_kern_err(func, "invalid PCI memory space addresses");
		return (PCI_ERR);
	}
	/*
	   **    Validate the prefetchable memory space address definitions.
	 */

	if ((unsigned int) CONFIG_PCI_MEMPF_START > (unsigned int) CONFIG_PCI_MEMPF_END)
		ret_code = PCI_ERR;

	if (((unsigned int) CONFIG_PCI_MEMPF_END - (unsigned int) CONFIG_PCI_MEMPF_START) < 0x00100000)
		ret_code = PCI_ERR;

	if (ret_code != PCI_OK) {
		pci_kern_err(func, "invalid PCI prefetchable memory space addresses");
		return (PCI_ERR);
	}
#ifdef CONFIG_PCI_AGP_BRIDGE
	/*
	   **    Validate the I/O space size definition for the AGP bridge.
	 */

	if ((unsigned int) CONFIG_PCI_AGP_IO_SIZE >
	    ((unsigned int) (CONFIG_PCI_IO_END + 1) -
	     (unsigned int) CONFIG_PCI_IO_START)) {
		pci_kern_err(func, "invalid I/O size for AGP bridge");
		return (PCI_ERR);
	}
	/*
	   **    Validate the memory space size definition for the AGP bridge.
	 */

	if ((unsigned int) CONFIG_PCI_AGP_MEM_SIZE >
	    ((unsigned int) (CONFIG_PCI_MEM_END + 1) -
	     (unsigned int) CONFIG_PCI_MEM_START)) {
		pci_kern_err(func, "invalid memory size for AGP bridge");
		return (PCI_ERR);
	}
	/*
	   **    Validate the prefetchable memory space size definition
	   **    for the AGP bridge.
	 */

	if ((unsigned int) CONFIG_PCI_AGP_MEMPF_SIZE >
	    ((unsigned int) ((unsigned int) CONFIG_PCI_MEMPF_END + 1) -
	     (unsigned int) CONFIG_PCI_MEMPF_START)) {
		pci_kern_err(func, "invalid prefetchable memory size for AGP bridge");

		return (PCI_ERR);
	}
#endif				/* CONFIG_PCI_AGP_BRIDGE */

	/*
	   **    Validate the PCI local IRQs.
	 */

	if ((unsigned int) CONFIG_PCI_LOCAL_PIRQA > 255)
		ret_code = PCI_ERR;
	if ((unsigned int) CONFIG_PCI_LOCAL_PIRQB > 255)
		ret_code = PCI_ERR;
	if ((unsigned int) CONFIG_PCI_LOCAL_PIRQC > 255)
		ret_code = PCI_ERR;
	if ((unsigned int) CONFIG_PCI_LOCAL_PIRQD > 255)
		ret_code = PCI_ERR;
	if (ret_code != PCI_OK) {
		pci_kern_err(func, "invalid PCI local IRQ");
		return (PCI_ERR);
	}
#ifdef __powerpc__
#ifdef CONFIG_PCI_MCP750_BUS_0_DEVICES
	config_mcp750_bus_0_devices();

#endif				/* CONFIG_PCI_MCP750_BUS_0_DEVICES */
#endif				/* __powerpc__ */

#ifdef CONFIG_PCI_MULTI_DOMAIN
	{
#define IS_DOMAIN_B 0x80
#define BUSA_REG_OFFSET 0xE8

		u32 signature;
		u32 busa_reg;
		u32 bar0;
		void __iomem *hsc_regs_p, *busa_reg_p;
		struct pci_dev *dev0;
#ifdef CONFIG_PCI_HSC_BRIDGE
		u8 secondary_bus;
#endif

		/*
		   **    Validate the PCI remote IRQs.
		 */

		if ((unsigned int) CONFIG_PCI_REMOTE_PIRQA > 255)
			ret_code = PCI_ERR;
		if ((unsigned int) CONFIG_PCI_REMOTE_PIRQB > 255)
			ret_code = PCI_ERR;
		if ((unsigned int) CONFIG_PCI_REMOTE_PIRQC > 255)
			ret_code = PCI_ERR;
		if ((unsigned int) CONFIG_PCI_REMOTE_PIRQD > 255)
			ret_code = PCI_ERR;
		if (ret_code != PCI_OK) {
			pci_kern_err(func, "invalid PCI remote IRQ");
			return (PCI_ERR);
		}
		/*
		   **    Validate the PCI local domain bridge device/function.
		 */

		if ((unsigned int) CONFIG_PCI_LOCAL_DEV == 0x00)
			ret_code = PCI_ERR;
		if ((unsigned int) CONFIG_PCI_LOCAL_DEV > 0xff)
			ret_code = PCI_ERR;
		if (ret_code != PCI_OK) {
			pci_kern_err(func,
			   "invalid PCI local domain bridge device/function");
			return (PCI_ERR);
		}
		/*
		   **    Validate the PCI remote domain bridge device/function.
		 */

		if ((unsigned int) CONFIG_PCI_REMOTE_DEV == 0x00)
			ret_code = PCI_ERR;
		if ((unsigned int) CONFIG_PCI_REMOTE_DEV > 0xff)
			ret_code = PCI_ERR;
		if (ret_code != PCI_OK) {
			pci_kern_err(func,
			  "invalid PCI remote domain bridge device/function");

			return (PCI_ERR);
		}
		dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
		if (!dev0)
			return -ENOMEM;
		dev0->bus = pbus;
		dev0->sysdata = pbus->sysdata;
#ifdef CONFIG_PCI_HSC_BRIDGE
		dev0->devfn = CONFIG_PCI_HSC_DEV;
		pci_read_config_byte(dev0, PCI_SECONDARY_BUS, &secondary_bus);
		early_read_config_dword(pbus->sysdata, secondary_bus,
					CONFIG_PCI_LOCAL_HSC, PCI_VENDOR_ID, &signature);
		printk(KERN_INFO "HSC signature 0x%x\n", signature);
		early_read_config_dword(pbus->sysdata, secondary_bus,
					CONFIG_PCI_LOCAL_HSC, PCI_BASE_ADDRESS_0, &bar0);
#else
		dev0->devfn = CONFIG_PCI_LOCAL_HSC;
		pci_read_config_dword(dev0, PCI_VENDOR_ID, &signature);
		pci_read_config_dword(dev0, PCI_BASE_ADDRESS_0, &bar0);

#ifdef __powerpc__
#if !defined(CONFIG_MCP820) && !defined(CONFIG_PCI_MCP750_BUS_0_DEVICES)
		if (bar0 <= CONFIG_PCI_MEM_END) {
			printk(KERN_INFO "Rewriting BAR0 for HSC\n");
			pci_write_config_dword(dev0, PCI_BASE_ADDRESS_0, 0x3bffd000);
			pci_read_config_dword(dev0, PCI_BASE_ADDRESS_0, &bar0);
		}
#endif
#endif

#endif				/* CONFIG_PCI_HSC_BRIDGE */

#ifdef CONFIG_PPC_PREP

#ifdef CONFIG_PCI_MCP750_BUS_0_DEVICES
		hsc_regs_p = ioremap(bar0 + 0x80100000, 0xff);
#else
		hsc_regs_p = ioremap(bar0 + 0xc0000000, 0xff);
#endif

#else
		hsc_regs_p = ioremap(bar0, 0xff);
#endif
		if (!hsc_regs_p) {
			printk(KERN_ERR "Unable to map HSC register set\n");
			pci_processor = DOMAIN_A;
		} else {
			busa_reg_p = hsc_regs_p + BUSA_REG_OFFSET;
			busa_reg = readl(busa_reg_p);
			if (busa_reg & IS_DOMAIN_B)
				pci_processor = DOMAIN_B;
			else
				pci_processor = DOMAIN_A;
			printk(KERN_INFO "processor is in domain %s\n",
			       (pci_processor == DOMAIN_A) ? "A" : "B");
			iounmap(hsc_regs_p);
		}

		kfree(dev0);
	}

#endif				/* CONFIG_PCI_MULTI_DOMAIN */

	/*
	   **    Validate the device's cache line size.
	 */

	if ((unsigned int) CONFIG_PCI_CACHE_LINE_SIZE > 255) {
		pci_kern_err(func, "invalid device cache line size");
		return (PCI_ERR);
	}
	/*
	   **    Validate the devices' latency timer value.
	 */

	if ((unsigned int) CONFIG_PCI_LATENCY_TIMER > 255) {
		pci_kern_err(func, "invalid device latency timer value");
		return (PCI_ERR);
	}
	/*
	   **    Validate the bridges' secondary latency timer value.
	 */

	if ((unsigned int) CONFIG_PCI_SEC_LATENCY_TIMER > 255) {
		pci_kern_err(func,
			     "invalid bridge secondary latency timer value");
		return (PCI_ERR);
	}
	/*
	   **    Initialize the bus number, then loop
	   **    through the "Bus Level 0" node count.
	 */

	busnum = 0;

	for (l0cnt = 0; l0cnt < CONFIG_PCI_BUSLVL0_NODES; ++l0cnt) {
		/*
		   **    Generate "Bus Level 0" index, then
		   **    set "Bus Level 0" node bus number in
		   **    the array. Also save it as the "Bus
		   **    Level 0" subordinate bus number.
		 */

		l0indx = BUSLVL1_NODES * l0cnt;
		l0indx += l0cnt;
		pci_busnum_perlevel[l0indx] = l0sub = busnum;

		/*
		   **    Generate starting "Bus Level 1" index. Then
		   **    set the starting index of bus children for
		   **    this bus number and increment the bus number.
		 */

		l1indx = l0indx + 1;
		pci_busnum_indx[busnum++] = l1indx;

#ifdef CONFIG_PCI_AGP_BRIDGE
		/*
		   **    Reserve this bus number for the AGP PCI-2-PCI
		   **    bridge device.
		 */
		++busnum;
#endif

		/*
		   **    Loop through the "Bus Level 1" node count.
		 */

		for (l1cnt = 0; l1cnt < CONFIG_PCI_BUSLVL1_NODES; ++l1cnt) {
			/*
			   **    Set "Bus Level 1" node bus number in
			   **    the array. Also save it as the "Bus
			   **    Level 1" subordinate bus number.
			 */

			pci_busnum_perlevel[l1indx++] = l1sub = busnum;

			/*
			   **    Generate starting "Bus Level 2". Then
			   **    set the starting index of bus children
			   **    for this bus number and increment the
			   **    bus number.
			 */

			l2indx = (CONFIG_PCI_BUSLVL1_NODES + l0indx + 1);
			l2indx += (CONFIG_PCI_BUSLVL2_NODES * l1cnt);
			pci_busnum_indx[busnum++] = l2indx;

			/*
			   **    Loop through the "Bus Level 2" node count.
			 */

			for (l2cnt = 0; l2cnt < CONFIG_PCI_BUSLVL2_NODES; ++l2cnt) {
				/*
				   **    Set "Bus Level 2" node bus number in
				   **    the array. Also save it as the "Bus
				   **    Level 2" subordinate bus number.
				 */

				pci_busnum_perlevel[l2indx++] = l2sub = busnum;

				/*
				   **    Generate starting "Bus Level 3". Then
				   **    set the starting index of bus children
				   **    for this bus number and increment the
				   **    bus number.
				 */

				l3indx = (CONFIG_PCI_BUSLVL1_NODES + l0indx + 1);
				l3indx += (CONFIG_PCI_BUSLVL1_NODES * CONFIG_PCI_BUSLVL2_NODES);

				l3indx += (CONFIG_PCI_BUSLVL2_NODES * CONFIG_PCI_BUSLVL3_NODES * l1cnt);
				l3indx += (CONFIG_PCI_BUSLVL3_NODES * l2cnt);

				pci_busnum_indx[busnum++] = l3indx;

				/*
				   **    Loop through the "Bus Level 3" node count.
				 */

				for (l3cnt = 0; l3cnt < CONFIG_PCI_BUSLVL3_NODES; ++l3cnt) {
					/*
					   **    Set "Bus Level 3" node bus number in
					   **    the array. Also save it as the "Bus
					   **    Level 3" subordinate bus number.
					 */

					pci_busnum_perlevel[l3indx++] = l3sub = busnum;

					/*
					   **    Set "no more bus children" flag in 
					   **    the child index array and increment
					   **    the bus number.
					 */

					pci_busnum_indx[busnum++] = -1;

					/*
					   **    Set "Bus Level 3" subordinate bus number.
					   **    This is a leaf node, thus it has the
					   **    same number.
					 */

					pci_sub_busnum[l3sub] = l3sub;
				}

				/*
				   **    Set "Bus Level 2" subordinate bus number.
				 */

				pci_sub_busnum[l2sub] = busnum - 1;
			}

			/*
			   **    Set "Bus Level 1" subordinate bus number.
			 */

			pci_sub_busnum[l1sub] = busnum - 1;
		}
		/*
		   **    Set "Bus Level 0" subordinate bus number.
		 */
		pci_sub_busnum[l0sub] = busnum - 1;
	}

	/*
	   **    Clear the bus number allocation array.
	 */

	for (indx = 0; indx < MAX_PCI_NODES; ++indx)
		pci_busnum_alloc_status[indx] = PCI_BUS_NOT_ALLOC;

	/*
	   **    Set bus level 0 PCI "spaces". This is based upon
	   **    the ending address minus the starting address,
	   **    divided by the maximum number of "Bus Level 0"
	   **    nodes.
	 */

	pci_bus_io_size[0] = (CONFIG_PCI_IO_END + 1);
	pci_bus_io_size[0] -= CONFIG_PCI_IO_START;
#ifdef CONFIG_PCI_AGP_BRIDGE
	pci_bus_io_size[0] -= CONFIG_PCI_AGP_IO_SIZE;
#endif
	pci_bus_io_size[0] /= CONFIG_PCI_BUSLVL0_NODES;

	pci_bus_mem_size[0] = (CONFIG_PCI_MEM_END + 1);
	pci_bus_mem_size[0] -= CONFIG_PCI_MEM_START;
#ifdef __powerpc__
#ifdef CONFIG_PCI_MCP750_BUS_0_DEVICES
	pci_bus_mem_size[0] = (PciMemoryEnd - PciMemoryStart) + 1;
#endif				/* CONFIG_PCI_MCP750_BUS_0_DEVICES */
#endif				/* __powerpc__ */

#ifdef CONFIG_PCI_AGP_BRIDGE
	pci_bus_mem_size[0] -= CONFIG_PCI_AGP_MEM_SIZE;
#endif
	pci_bus_mem_size[0] /= CONFIG_PCI_BUSLVL0_NODES;

	pci_bus_mempf_size[0] = (unsigned int) CONFIG_PCI_MEMPF_END + 1;
	pci_bus_mempf_size[0] -= CONFIG_PCI_MEMPF_START;
#ifdef CONFIG_PCI_AGP_BRIDGE
	pci_bus_mempf_size[0] -= CONFIG_PCI_AGP_MEMPF_SIZE;
#endif
	pci_bus_mempf_size[0] /= CONFIG_PCI_BUSLVL0_NODES;

	pci_bus_mem20_size[0] = (CONFIG_PCI_MEM20_END + 1);
	pci_bus_mem20_size[0] -= CONFIG_PCI_MEM20_START;

	/*
	   **    Set PCI "spaces" for each subordinate
	   **    bus level, i.e. "Bus Level 1" and on.
	 */

	for (indx = 1; indx < MAX_PCI_LEVELS; ++indx) {
		/*
		   **    Clear the space sizes at this level.
		 */
		pci_bus_io_size[indx] = 0;
		pci_bus_mem_size[indx] = 0;
		pci_bus_mem20_size[indx] = 0;
		pci_bus_mempf_size[indx] = 0;
		/*
		   **    Check to see if we have any nodes
		   **    at this level.
		 */

		if (pci_max_busnodes_perlevel[indx] != 0) {
			/*
			   **    Generate PCI I/O space.
			 */
			size = pci_bus_io_size[indx - 1];
#ifdef CONFIG_PCI_EXT_BRIDGE
			if (indx == 3)
#ifdef CONFIG_PCI_MULTI_HOST_BRIDGE
				size /= (pci_max_busnodes_perlevel[indx] * CONFIG_PCI_BUSLVL0_NODES);
#else
				;
#endif
			else 
				size /= (pci_max_busnodes_perlevel[indx] * CONFIG_PCI_BUSLVL0_NODES);
#else
			size /= (pci_max_busnodes_perlevel[indx] * CONFIG_PCI_BUSLVL0_NODES);
#endif

			if (size > 0) {
				if (size > PCI_ALIGN_IO)
					size &= ~(PCI_ALIGN_IO - 1);
				else
					size = PCI_ALIGN_IO;

				pci_bus_io_size[indx] = size;
			}
			size = pci_bus_mem_size[indx - 1];
#ifdef CONFIG_PCI_EXT_BRIDGE
			if (indx == 3)
				size = pci_bus_mem_size[1] / (CONFIG_PCI_BUSLVL3_NODES + 4);
			else
				size /= (pci_max_busnodes_perlevel[indx] * CONFIG_PCI_BUSLVL0_NODES);
#else
			size /= (pci_max_busnodes_perlevel[indx] * CONFIG_PCI_BUSLVL0_NODES);
#endif				/* CONFIG_PCI_EXT_BRIDGE */

			if (size > 0) {
				if (size > PCI_ALIGN_MEM)
					size &= ~(PCI_ALIGN_MEM - 1);
				else
					size = PCI_ALIGN_MEM;

				pci_bus_mem_size[indx] = size;
			}
			size = pci_bus_mempf_size[indx - 1];
#ifdef CONFIG_PCI_EXT_BRIDGE
			if (indx == 3)
#ifdef CONFIG_PCI_MULTI_HOST_BRIDGE
				size /= (pci_max_busnodes_perlevel[indx] * CONFIG_PCI_BUSLVL0_NODES);
#else
				size = 0x100000;
#endif
			else
				size /= (pci_max_busnodes_perlevel[indx] * CONFIG_PCI_BUSLVL0_NODES);
#else
			size /= (pci_max_busnodes_perlevel[indx] * CONFIG_PCI_BUSLVL0_NODES);
#endif				/* CONFIG_PCI_EXT_BRIDGE */
			if (size > 0) {
				if (size > PCI_ALIGN_MEM)
					size &= ~(PCI_ALIGN_MEM - 1);
				else
					size = PCI_ALIGN_MEM;
				pci_bus_mempf_size[indx] = size;
			}
		}
	}
	return (PCI_OK);
}

#if (CONFIG_PCI_BUSLVL0_NODES > 1)
/*
   **    pci_alloc_hosts - Allocate Bus Numbers For Host Bridge Devices
   **
   **        This function will probe the PCI bus for Host
   **        Bridge Devices and reconfigure their bus numbers.
   **        This is done to prevent possible PCI bus access
   **        problems during the initial scanning.
 */

int __init 
pci_alloc_hosts(struct pci_bus *bus)
{
	struct pci_host *phost;	/* Pointer to Current Host */
	unsigned int number;	/* Bus Number */
	unsigned int devfn;	/* Device/Function Number */
	unsigned int buses;	/* Buses Config. Register */
	int numhosts;		/* Number of Hosts */
	struct pci_dev *dev0;
	struct pci_host hosts[CONFIG_PCI_BUSLVL0_NODES];

	/*
	   **    Initialize the number of Host Bridge devices. Then
	   **    loop through each bus/dev to find Host Bridges.
	 */
	pr_debug("pci_alloc_hosts: bus 0x%p\n", bus);
	numhosts = 0;
	dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
	if (!dev0)
		return -ENOMEM;
	dev0->bus = bus;
	dev0->sysdata = bus->sysdata;
	for (number = 0; number <= MAX_PCI_BUS; ++number) {
		/*
		 **    Look for a Host Bridge on this bus.
		 */

		for (devfn = 0; devfn < 0x100; devfn += 8) {
			dev0->devfn = devfn;
			ret_code = pci_find_host(dev0);
			if (ret_code == PCI_OK)
				break;
		}

		if (ret_code == PCI_OK) {
			/*
			   **    We have found one. Check to see if we
			   **    have exceeded the number of configurable
			   **    Host Bridge devices.
			 */
			if (numhosts >= CONFIG_PCI_BUSLVL0_NODES) {
				/*
				   **    We have exceeded the number of
				   **    configurable Host Bridge devices.
				 */
				pci_kern_err("pci_alloc_hosts",
					     "too many Host Bridge devices");

				return (PCI_ERR);
			}
			/*
			   **    Set the host structure pointer, along
			   **    with the device/function and current
			   **    bus number for the Host Bridge device.
			 */
			phost = &(hosts[numhosts]);
			phost->devfn = devfn;
			phost->number = number;

			/*
			   **    Set the new primary, secondary, and
			   **    subordinate bus numbers for the Host
			   **    Bridge device.
			 */

			phost->primary = 0;
			if (numhosts != 0) {
				phost->primary = hosts[numhosts - 1].subordinate;
				phost->primary += 1;
			}
			phost->secondary = phost->primary;
			phost->subordinate = pci_sub_busnum[phost->primary];
			++numhosts;
		}
	}
	kfree(dev0);

	/*
	   **    Starting with the last Host Bridge device,
	   **    reconfigure the bus numbers for each host.
	 */

	while (numhosts-- > 0) {
		/*
		   **    Set the host structure pointer. Then
		   **    set the new Subordinate, Primary, and
		   **    Secondary bus numbers.
		 */
		phost = &(hosts[numhosts]);
		buses = 0;
		buses = (buses << 8) | phost->subordinate;
		buses = (buses << 8) | phost->secondary;
		buses = (buses << 8) | phost->primary;
		pci_bus_write_config_dword(phost->number, phost->devfn,
					   PCI_PRIMARY_BUS, buses);
	}
	return (PCI_OK);
}
#endif				/* (CONFIG_PCI_BUSLVL0_NODES > 1) */

#ifdef CONFIG_PCI_AGP_BRIDGE
/*
   **    pci_agp_bridge - Is Device An AGP Bridge
   **
   **        This function will determine if the given device
   **        is an AGP bridge device (TRUE) or not (FALSE).
   **
   **        NOTE: This list will need to be filled in with
   **              future AGP bridge devices.
 */

int 
pci_agp_bridge(struct pci_bus *pbus, unsigned int devfn)
{
	unsigned short vendor;	/* Vendor ID */
	unsigned short device;	/* Device ID */
	struct pci_dev *dev0;
	int ret_code;		/* Function Return Code */

	ret_code = FALSE;
	dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
	if (!dev0)
		return -ENOMEM;
	dev0->bus = pbus;
	dev0->sysdata = pbus->sysdata;
	dev0->devfn = devfn;
	pci_read_config_word(dev0, PCI_VENDOR_ID, &vendor);
	pci_read_config_word(dev0, PCI_DEVICE_ID, &device);
	switch (vendor) {
	case PCI_VENDOR_ID_INTEL:
		{
			switch (device) {
			case 0x7181:	/* Intel 440LX    - 82443LX AGP bridge */
			case 0x7191:	/* Intel 440BX/ZX - 82443BX/ZX AGP bridge */
			case 0x71a1:	/* Intel 440GX    - 82443GX AGP bridge */
				{
					ret_code = TRUE;
					break;
				}
			}
		}
	}
	kfree(dev0);
	return (ret_code);
}

#endif				/* CONFIG_PCI_AGP_BRIDGE */

/*
   **    pci_find_level - Determine PCI Bus Level
   **
   **        This function will return the PCI bus level for
   **        the given bus node.
 */

int 
pci_find_level(struct pci_bus *pbus)
{
	struct pci_bus *nbus;	/* Pointer To Next Bus Parent */
	unsigned int bus_level;	/* Current Bus Level */

	/*
	   **    Initialize the bus level value, then
	   **    find the actual level of the bus node.
	 */

	bus_level = 0;
	for (nbus = pbus->parent; nbus != (struct pci_bus *) NULL;
	     nbus = nbus->parent)
		++bus_level;
	return (bus_level);
}

/*
   **    pci_alloc_busnum - PCI Bus Number Allocation
   **
   **        This is the PCI bus number allocation function. It will
   **        allocate secondary and subordinate bus numbers.
 */

int 
pci_alloc_busnum(struct pci_bus *pbus, unsigned int devfn)
{
	int bus_level;		/* Current Bus Level */
	int bus_num;		/* Current Bus Number */
	int child_indx;		/* Child Bus Index */
	int i;			/* Just An Array Index */
	u8 tmp_bus;
	int configured;		/* Configuration Status */
	char *func;		/* Function Name */
	struct pci_dev *dev0;
	unsigned long flags;

	/*
	   **    Initialize the bus number and the bus
	   **    level. Then check for bus level 0.
	 */

	pr_debug("pci_alloc_busnum: bus 0x%p devfn 0x%x\n", pbus, devfn);
	func = "pci_alloc_busnum";
	bus_num = 0;
	bus_level = pci_find_level(pbus);
	if (bus_level == 0) {
		/*
		   **    We are at bus level 0. Disable interrupts
		   **    for now.
		 */
		local_irq_save(flags);
		/*
		   **    Look for free bus numbers for this host node.
		 */
		for (i = 0; i < pci_max_busnodes_perlevel[0]; ++i) {
			if (pci_busnum_alloc_status[bus_num] == PCI_BUS_NOT_ALLOC) {
				pci_busnum_alloc_status[bus_num] = PCI_BUS_ALLOC;
				break;
			}
			bus_num += (pci_sub_busnum[bus_num] + 1);
		}
		local_irq_restore(flags);

		/*
		   **    Did we find free host bus numbers at this
		   **    level? If not, then set the bus number
		   **    to indicate an error.
		 */

		if (i < pci_max_busnodes_perlevel[0]) {
			/*
			   **    We have free host bus numbers. Hence, set
			   **    the primary, secondary, and subordinate
			   **    bus numbers.
			 */
			pbus->number = bus_num;
			pbus->primary = bus_num;
			pbus->secondary = bus_num;
			pbus->subordinate = pci_sub_busnum[bus_num];
			return (PCI_OK);
		}
		pci_kern_err(func,
			"couldn't find free bus numbers for the Host bridge");
		return (-ENOSPC);
	} else {
#ifdef CONFIG_PCI_AGP_BRIDGE
		/*
		   **    Check to see if this is an AGP bridge device.
		 */

		if (pci_agp_bridge(pbus->parent, devfn) == TRUE) {
			/*
			   **    Check the bus level. An AGP bridge device
			   **    can only be at bus level 1.
			 */

			if (bus_level == 1) {
				/*
				   **    We are an AGP bridge device. Look for free
				   **    bus numbers for this bridge.
				 */
				local_irq_save(flags);
				bus_num = (pbus->parent)->secondary + 1;
				if (pci_busnum_alloc_status[bus_num] == PCI_BUS_NOT_ALLOC) {
					/*
					   **    We have free AGP bridge bus numbers. Hence,
					   **    set the primary, secondary, and subordinate
					   **    bus numbers.
					 */

					pci_busnum_alloc_status[bus_num] = PCI_BUS_ALLOC;
					local_irq_restore(flags);
					pbus->number = bus_num;
					pbus->primary = (pbus->parent)->secondary;
					pbus->secondary = bus_num;
					pbus->subordinate = bus_num;
					return (PCI_OK);
				}
				/*
				   **    Couldn't find free bus numbers for the
				   **    AGP bridge device.
				 */
				local_irq_restore(flags);
				pci_kern_err(func, "couldn't find free bus numbers for the AGP bridge");
				return (-ENOSPC);
			}
			/*
			   **    An AGP bridge device is only allowed at
			   **    bus level 1.
			 */
			pci_kern_err(func, "an AGP bridge is only allowed at bus level 1");
			return (-EACCES);
		}
#endif				/* CONFIG_PCI_AGP_BRIDGE */

		/*
		   **    Get the bus child index. If no child buses
		   **    at this level, then display an error message
		   **    and return an error.
		 */

		dev0 = kzalloc(sizeof(*dev0), GFP_KERNEL);
		if (!dev0)
			return -ENOMEM;
		dev0->bus = pbus->parent;
		dev0->sysdata = pbus->sysdata;
		dev0->devfn = devfn;
		configured = pci_is_device_configured(dev0);
		if (configured) {
			pci_read_config_byte(dev0, PCI_SECONDARY_BUS, &tmp_bus);
			bus_num = tmp_bus;
			pci_read_config_byte(dev0, PCI_SUBORDINATE_BUS, &tmp_bus);
			kfree(dev0);
			local_irq_save(flags);
			pci_busnum_alloc_status[bus_num] = PCI_BUS_ALLOC;
			pbus->number = bus_num;
			pbus->primary = (pbus->parent)->secondary;
			pbus->secondary = bus_num;
			pbus->subordinate = pci_sub_busnum[bus_num];
			local_irq_restore(flags);
			if (pci_sub_busnum[bus_num] != tmp_bus) {
				printk(KERN_ERR "Bridge configuration incorrect\n");
				return (-ENODEV);
			}
			return (PCI_OK);
		} else {
			kfree(dev0);
			child_indx = pci_busnum_indx[(pbus->parent)->secondary];
			if (child_indx != -1) {

				local_irq_save(flags);

				/*
				 **    Initialize the bus number (not found), then
				 **    allocate the child bus.
				 */

				bus_num = -1;
				for (i = 0; i < pci_max_busnodes_perlevel[bus_level]; ++i) {
					bus_num = pci_busnum_perlevel[child_indx + i];
					if (pci_busnum_alloc_status [bus_num] == PCI_BUS_NOT_ALLOC) {
						pci_busnum_alloc_status[bus_num] = PCI_BUS_ALLOC;
						break;
					}
					bus_num = -1;
				}
				local_irq_restore(flags);

				/*
				 **    Did we find free bus numbers at this level?
				 */

				if (bus_num != -1) {
					/*
					 **    Set the primary, secondary, and
					 **    subordinate bus numbers.
					 */

					pbus->number = bus_num;
					pbus->primary = (pbus->parent)->secondary;
					pbus->secondary = bus_num;
					pbus->subordinate = pci_sub_busnum[bus_num];
					return (PCI_OK);
				}
			}
			pci_kern_err(func, "couldn't find free bus numbers");
			return (-ENOSPC);
		}
	}
}

/*
   **    pci_alloc_busres - PCI Bus Resource Allocation
   **
   **        This is the PCI bus resource allocation function. It will
   **        allocate I/O and memory space from it's parent. 64-bit
   **        and prefetch memory space are not allocated for bus nodes.
 */

int 
pci_alloc_busres(struct pci_bus *pbus)
{
	struct pci_res *pres;	/* Pointer To Bus Resources */
	struct pci_dev *pdev;	/* Pointer to Bus Device */
	unsigned int indx;	/* Resource Index */
	unsigned int host_num;	/* Host Number */
	int bus_level;		/* Current Bus Level */
	char *func;		/* Function Name */
	unsigned long flags;

#if (CONFIG_PCI_BUSLVL0_NODES > 1)
	unsigned int bus_min;	/* Host Minimum Bus Number */
	unsigned int bus_max;	/* Host Maximum Bus Number */
#endif

	pr_debug("pci_alloc_busres: bus 0x%p\n", pbus);
	/*
	   **    First, set the function name for possible error
	   **    messages. Then, ensure that there is a P2P bridge
	   **    device structure associated with this bus.
	 */
	func = "pci_alloc_busres";
	pdev = pbus->self;
	if (pdev != (struct pci_dev *) NULL) {
		/*
		   **    Next, initialize the bus level and
		   **    host number.
		 */
		bus_level = pci_find_level(pbus);
		host_num = 0;
#if (CONFIG_PCI_BUSLVL0_NODES > 1)
		/*
		   **    Initialize the starting minimum bus number.
		   **    Then loop until the host number is generated.
		 */
		for (bus_min = 0; host_num < pci_max_busnodes_perlevel[0]; ++host_num) {
			/*
			   **    Set the maximum bus number, which is the
			   **    subordinate bus number for the minimum
			   **    bus number.
			 */

			bus_max = pci_sub_busnum[bus_min];

			/*
			   **    Compare the bus number against the minimum
			   **    and maximum numbers. If within range, then
			   **    we have found our host number.
			 */

			if ((pbus->number >= bus_min) && (pbus->number <= bus_max))
				break;

			/*
			   **    Set the next minimum bus number, which is
			   **    just one more than the maximum number.
			 */
			bus_min = bus_max + 1;
		}
#endif

#ifdef CONFIG_PCI_AGP_BRIDGE
		/*
		   **    Does this bridge have a parent? If not, then it
		   **    is a host bridge and not a P2P bridge.
		 */

		if (pbus->parent != (struct pci_bus *) NULL) {
			/*
			   **    Check to see if this is an AGP bridge device.
			 */

			if (pci_agp_bridge(pbus->parent, pdev->devfn) == TRUE) {
				/*
				   **    Check the bus level. An AGP bridge device
				   **    can only be at bus level 1.
				 */

				if (bus_level == 1) {

					local_irq_save(flags);

					/*
					   **    Set the AGP bridge device's PCI bus node resources
					   **    here starting with the PCI I/O space resource.
					 */

					pres = &(pbus->resources[PCI_RES_INDX_IO]);
					pres->next = (struct pci_res *) NULL;
					pres->type = PCI_RES_TYPE_IO;
					pres->align = PCI_ALIGN_IO;
					pres->size = CONFIG_PCI_AGP_IO_SIZE;

					pres = &((pdev->bus)->resources[PCI_RES_INDX_IO]);
					pres->size += CONFIG_PCI_AGP_IO_SIZE;

					pres = &(pbus-> resources[PCI_RES_INDX_MEM]);

					pres->next = (struct pci_res *) NULL;
					pres->type = PCI_RES_TYPE_MEM;
					pres->align = PCI_ALIGN_MEM;
					pres->size = CONFIG_PCI_AGP_MEM_SIZE;

					pres = &((pdev->bus)->resources[PCI_RES_INDX_MEM]);
					pres->size += CONFIG_PCI_AGP_MEM_SIZE;

					pres = &(pbus->resources [PCI_RES_INDX_MEM20]);

					pres->next = (struct pci_res *) NULL;
					pres->type = PCI_RES_TYPE_MEM20;
					pres->align = PCI_ALIGN_MEM20;
					pres->size = 0;

					pres = &(pbus->resources[PCI_RES_INDX_MEMPF]);

					pres->next = (struct pci_res *) NULL;
					pres->type = PCI_RES_TYPE_MEMPF;
					pres->align = PCI_ALIGN_MEM;
					pres->size = CONFIG_PCI_AGP_MEMPF_SIZE;

					pres = &((pdev->bus)->resources [PCI_RES_INDX_MEMPF]);
					pres->size += CONFIG_PCI_AGP_MEMPF_SIZE;

					/*
					   **    Clear the child device resource link lists.
					 */

					pbus->res_head_io = (struct pci_res *) NULL;
					pbus->res_head_mem = (struct pci_res *) NULL;
					pbus->res_head_mem20 = (struct pci_res *) NULL;
					pbus->res_head_mempf = (struct pci_res *) NULL;

					local_irq_restore(flags);

					/*
					   **    For each node resource, allocate space.
					 */

					for (indx = PCI_RES_INDX_IO; indx <= PCI_RES_INDX_MEMPF; ++indx) {
						/*
						   **    Set the resource pointer. Then, if there
						   **    is space available, allocate it from the
						   **    parent bus node and link it up.
						 */
						pres = &(pbus->resources[indx]);
						if (pres->size != 0)
							pci_alloc_devres (pdev, pres, indx);
					}
					return (PCI_OK);
				}
				/*
				   **    An AGP bridge device is only allowed at
				   **    bus level 1.
				 */

				pci_kern_err(func, "an AGP bridge is only allowed at bus level 1");

				return (-EACCES);
			}
		}
#endif				/* CONFIG_PCI_AGP_BRIDGE */
		local_irq_save(flags);

		/*
		   **    Set the PCI bus node resources here
		   **    starting with the PCI I/O space resource.
		 */

		pres = &(pbus->resources[PCI_RES_INDX_IO]);

		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_IO;
		pres->align = PCI_ALIGN_IO;
		pres->size = pci_bus_io_size[bus_level];

#ifdef CONFIG_PCI_EXT_BRIDGE
		if ((pdev->class >> 8) == PCI_CLASS_BRIDGE_PCI) {

			switch (bus_level) {
			case 1:
				if ((CONFIG_PCI_IO_END - CONFIG_PCI_IO_START) < 64 * 1024) {
					switch (pdev->devfn) {
					case CONFIG_PCI_LOCAL_DEV:

						if (pci_processor == DOMAIN_A)
							pres->size = CONFIG_PCI_BUSLVL2_NODES * PCI_ALIGN_IO;

						else if (pci_processor == DOMAIN_B)
							pres->size = (CONFIG_PCI_IO_END + 1) -
						    (CONFIG_PCI_IO_START + (CONFIG_PCI_BUSLVL2_NODES * PCI_ALIGN_IO));
						break;

					case CONFIG_PCI_REMOTE_DEV:

						if (pci_processor == DOMAIN_A)
							pres->size = (CONFIG_PCI_IO_END + 1) -
						    (CONFIG_PCI_IO_START +
						     (CONFIG_PCI_BUSLVL2_NODES * PCI_ALIGN_IO));

						else if (pci_processor == DOMAIN_B)
							pres->size = CONFIG_PCI_BUSLVL2_NODES * PCI_ALIGN_IO;

						break;
					default:
						break;
					}
				}
				break;

			case 2:
				if ( (pbus->primary == pbus->parent->secondary)
					   && (((pci_processor == DOMAIN_A)
					      && (pbus->parent->self->devfn == CONFIG_PCI_REMOTE_DEV))
					       || ((pci_processor == DOMAIN_B)
						   && (pbus->parent->self->devfn == CONFIG_PCI_LOCAL_DEV)))) {

					if (pdev->devfn == CONFIG_PCI_EXT_BRIDGE_1 || pdev->devfn == CONFIG_PCI_EXT_BRIDGE_2) {
#ifndef CONFIG_PCI_MULTI_HOST_BRIDGE
						pres->size = 4 * PCI_ALIGN_IO;
#endif
					}
				}
				break;
			case 3:
				if (list_empty(&pbus->devices)) {
					pres->size = 0;
				}
				break;
			default:
				break;
			}	/* end of switch */
		}
		/* end of if */
#endif				/* CONFIG_PCI_EXT_BRIDGE */
		if (bus_level == 0) {
			pres->baddr = pci_bus_io_size[0];
			pres->baddr *= host_num;
			pres->baddr += CONFIG_PCI_IO_START;

#ifdef CONFIG_PCI_AGP_BRIDGE
			if (host_num != 0)
				pres->baddr += CONFIG_PCI_AGP_IO_SIZE;
#endif
		}
		/*
		   **    Set the PCI memory resource.
		 */

		pres = &(pbus->resources[PCI_RES_INDX_MEM]);

		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_MEM;
		pres->align = PCI_ALIGN_MEM;
		pres->size = pci_bus_mem_size[bus_level];

#ifdef CONFIG_PCI_EXT_BRIDGE

		if ((bus_level == 2) && ((pdev->class >> 8) == PCI_CLASS_BRIDGE_PCI)) {
			if ((pbus->primary == pbus->parent->secondary)
			    && (((pci_processor == DOMAIN_A)
				 && (pbus->parent->self->devfn == CONFIG_PCI_REMOTE_DEV))
				|| ((pci_processor == DOMAIN_B)
				    && (pbus->parent->self->devfn == CONFIG_PCI_LOCAL_DEV)))) {
				pres->size =
				    pci_bus_mem_size[1] / (CONFIG_PCI_BUSLVL3_NODES + 4);
				if (pres->size > 0) {
					if (pres->size > PCI_ALIGN_MEM)
						pres->size &= ~(PCI_ALIGN_MEM - 1);
					else
						pres->size = PCI_ALIGN_MEM;
				}
				if (pdev->devfn == CONFIG_PCI_EXT_BRIDGE_1
				    || pdev->devfn == CONFIG_PCI_EXT_BRIDGE_2) {
					pres->size *= CONFIG_PCI_BUSLVL3_NODES;
				}
			}
		}
#endif				/* CONFIG_PCI_EXT_BRIDGE */

		if (bus_level == 0) {
			pres->baddr = pci_bus_mem_size[0];
			pres->baddr *= host_num;
#ifdef __powerpc__
#ifdef CONFIG_PCI_MCP750_BUS_0_DEVICES
			pres->baddr += PciMemoryStart;
#else
			pres->baddr += CONFIG_PCI_MEM_START;
#endif				/* CONFIG_PCI_MCP750_BUS_0_DEVICES */
#else
			pres->baddr += CONFIG_PCI_MEM_START;
#endif				/* __powerpc__ */

#ifdef CONFIG_PCI_AGP_BRIDGE
			if (host_num != 0)
				pres->baddr += CONFIG_PCI_AGP_MEM_SIZE;
#endif
		}
		/*
		   **    Set the PCI 20-bit memory resource.
		 */

		pres = &(pbus->resources[PCI_RES_INDX_MEM20]);

		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_MEM20;
		pres->align = PCI_ALIGN_MEM20;
		pres->size = pci_bus_mem20_size[bus_level];

		if ((bus_level == 0) && (host_num == 0))
			pres->baddr = CONFIG_PCI_MEM20_START;

		/*
		   **    Set the PCI prefetchable memory resource.
		 */

		pres = &(pbus->resources[PCI_RES_INDX_MEMPF]);

		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_MEMPF;
		pres->align = PCI_ALIGN_MEM;
		pres->size = pci_bus_mempf_size[bus_level];

#ifdef CONFIG_PCI_EXT_BRIDGE

		if ((bus_level == 2)
		    && ((pdev->class >> 8) == PCI_CLASS_BRIDGE_PCI)) {

			if ((pbus->primary == pbus->parent->secondary)
			    && (((pci_processor == DOMAIN_A)
				 && (pbus->parent->self->devfn == CONFIG_PCI_REMOTE_DEV))
				|| ((pci_processor == DOMAIN_B)
				    && (pbus->parent->self->devfn == CONFIG_PCI_LOCAL_DEV)))) {

				if (pdev->devfn == CONFIG_PCI_EXT_BRIDGE_1
				    || pdev->devfn == CONFIG_PCI_EXT_BRIDGE_2) {
#ifndef CONFIG_PCI_MULTI_HOST_BRIDGE
					pres->size = CONFIG_PCI_BUSLVL3_NODES * 0x100000;
#endif
				}
			}
		}
#endif				/* CONFIG_PCI_EXT_BRIDGE */

		if (bus_level == 0) {
			pres->baddr = pci_bus_mempf_size[0];
			pres->baddr *= host_num;
			pres->baddr += CONFIG_PCI_MEMPF_START;

#ifdef CONFIG_PCI_AGP_BRIDGE
			if (host_num != 0)
				pres->baddr += CONFIG_PCI_AGP_MEMPF_SIZE;
#endif
		}
		/*
		   **    Clear the child device resource link lists.
		 */

		pbus->res_head_io = (struct pci_res *) NULL;
		pbus->res_head_mem = (struct pci_res *) NULL;
		pbus->res_head_mem20 = (struct pci_res *) NULL;
		pbus->res_head_mempf = (struct pci_res *) NULL;

		local_irq_restore(flags);

		/*
		   **    Check for bus level 0 - it's a special case.
		 */

		if (bus_level != 0) {
			/*
			   **    For each node resource, allocate space.
			 */

			for (indx = PCI_RES_INDX_IO; indx <= PCI_RES_INDX_MEMPF; ++indx) {
				/*
				   **    Set the resource pointer. Then, if there
				   **    is space available, allocate it from the
				   **    parent bus node and link it up.
				 */
				pres = &(pbus->resources[indx]);
				if (pres->size != 0)
					pci_alloc_devres(pdev, pres, indx);
			}
		}
		return (PCI_OK);
	}
	/*
	   **    No P2P bridge device structure.
	 */
	pci_kern_err(func, no_dev_struct);
	return (-ENOLINK);
}

/*
   **    pci_alloc_devres - PCI Device Resource Allocation
   **
   **        This is the PCI device resource allocation function. It will
   **        allocate I/O and memory space from it's parent bus. A linked
   **        list of resources is maintained in the parent node. There
   **        is one linked list per type. This routine searches on a
   **        first-fit basis for a gap to accomodate the new resource
   **        and inserts the resource into the list.
 */

int 
pci_alloc_devres(struct pci_dev *pdev, struct pci_res *pres, int bar)
{
	struct pci_bus *pbus;	/* Parent Bus Node Pointer */
	struct pci_res *pres_bus;	/* Bus Node Resource Pointer */
	struct pci_res **pres_head;	/* Resource Head Pointer */
	struct pci_res *pres_cur;	/* Current Resource Pointer */
	struct pci_res *pres_nxt;	/* Next Resource Pointer */
	unsigned long baddr;	/* Base Address */
	unsigned long align;	/* Base Address Alignment */
	long size;		/* PCI Space Size */
	long gap;		/* PCI Space Gap */
	int configured;		/* Configuration Status */
	char *func;		/* Function Name */
	char err_msg[256];	/* Error Message */
	unsigned long flags;

	pr_debug("pci_alloc_devres: dev 0x%p res 0x%p\n", pdev, pres);
	/*
	   **    First, set the function name for possible error
	   **    messages. Then, ensure that there is a P2P bridge
	   **    bus structure associated with this device.
	 */

	func = "pci_alloc_devres";

	pbus = pdev->bus;

	if (pbus != (struct pci_bus *) NULL) {
#ifndef CONFIG_PCI_BUS_0_DEVICES
		/*
		   **    Check for a Bus 0 non-P2P bridge device. 
		 */

		if ((pbus->number == 0)
		    && ((pdev->class >> 8) != PCI_CLASS_BRIDGE_PCI)) {
			u32 base_reg;	/* Base Address Register */

			u8 base;	/* Base Register Offset */

			/*
			   **    Bus 0 non-PCI-2-PCI device resources are allocated
			   **    by the BIOS and are static. Hence, just read the
			   **    Base Address register then adjust the resource
			   **    type and pointers based upon the value in the
			   **    Base Address register.
			 */

			base = PCI_BASE_ADDRESS_0 + (bar << 2);
			pci_read_config_dword(pdev, base, &base_reg);
			if ((base_reg & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO) {
				/*
				   **    PCI I/O space. Set PCI I/O variables.
				 */
				baddr = base_reg & PCI_BASE_ADDRESS_IO_MASK;
				pres_head = &(pbus->res_head_io);
				pres->type = PCI_RES_TYPE_IO;
				pres->align = pres->size;
			} else {
				/*
				   **    PCI memory space. Set default PCI
				   **    memory variables.
				 */
				baddr = base_reg & PCI_BASE_ADDRESS_MEM_MASK;
				pres_head = &(pbus->res_head_mem);
				pres->type = PCI_RES_TYPE_MEM;
				pres->align = pres->size;

				/*
				   **    Is it a 64-bit address?
				 */

				if ((base_reg & PCI_BASE_ADDRESS_MEM_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64) {
#if (BITS_PER_LONG == 64)
					/*
					   **    64-bit address. Thus, do a double read.
					 */
					base = PCI_BASE_ADDRESS_0 + ((bar + 1) << 2);
					pci_read_config_dword(pdev, base, &base_reg);
					baddr = base_reg;
					baddr <<= 32;
					base = PCI_BASE_ADDRESS_0 + (bar << 2);
					pci_read_config_dword(pdev, base, &base_reg);
					baddr |= (base_reg & PCI_BASE_ADDRESS_MEM_MASK);
#else
					printk
					    ("PCI: Unable to handle 64-bit address for device %02x:%02x\n",
					     pbus->number, pdev->devfn);
#endif
				}
				/*
				   **    Prefetchable memory?
				 */
				if ( (base_reg & PCI_BASE_ADDRESS_MEM_PREFETCH) != 0) {
					/*
					   **    Prefetchable memory. Set PCI
					   **    prefetch memory variables.
					 */
					pres_head = &(pbus->res_head_mempf);
					pres->type = PCI_RES_TYPE_MEMPF;
				}
				/*
				   **    Check to see if address is below 1M.
				 */

				if (baddr < 0x00100000UL) {
					/*
					   **    Below 1M. Set PCI memory variables.
					 */
					pres_head = &(pbus->res_head_mem20);
					pres->type = PCI_RES_TYPE_MEM20;
				}
			}

			/*
			   **    Clear the next field and set the
			   **    Base Address register field.
			 */

			pres->next = (struct pci_res *) NULL;
			pres->baddr = baddr;

			local_irq_save(flags);

			/*
			   **    Is this the first entry into the list?
			 */

			if ((*pres_head) == (struct pci_res *) NULL) {
				/*
				   **    This is the first entry in the linked list.
				 */
				(*pres_head) = pres;
			} else {
				/*
				   **    Should this be the new first entry?
				 */

				if ((*pres_head)->baddr > baddr) {
					/*
					   **    This is the new first entry in the linked list.
					 */
					pres->next = (*pres_head);
					(*pres_head) = pres;
				} else {
					pres_cur = (*pres_head);
					while (pres_cur->next != (struct pci_res *) NULL) {
						if ((pres_cur->next)-> baddr > baddr) {
							pres->next = pres_cur->next;
							break;
						} else
							pres_cur = pres_cur->next;
					}
					pres_cur->next = pres;
				}
			}
			local_irq_restore(flags);
			return (PCI_OK);
		}
#endif				/* !CONFIG_PCI_BUS_0_DEVICES */
		configured = pci_is_device_configured(pdev);
		if (configured) {
			if ((pdev->class >> 8) != PCI_CLASS_BRIDGE_PCI) {
				u32 base_reg;	/* Base Address Register */
				u8 base;	/* Base Register Offset */

				/*
				   **    Just read the
				   **    Base Address register and adjust the resource
				   **    type and pointers based upon the value in the
				   **    Base Address register.
				 */

				base = PCI_BASE_ADDRESS_0 + (bar << 2);
				pci_read_config_dword(pdev, base, &base_reg);
				if ((base_reg & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO) {
					/*
					   **    PCI I/O space. Set PCI I/O variables.
					 */

					baddr = base_reg & PCI_BASE_ADDRESS_IO_MASK;
					pres_head = &(pbus->res_head_io);
					pres->type = PCI_RES_TYPE_IO;
					pres->align = pres->size;
				} else {
					/*
					   **    PCI memory space. Set default PCI
					   **    memory variables.
					 */

					baddr = base_reg & PCI_BASE_ADDRESS_MEM_MASK;
					pres_head = &(pbus->res_head_mem);
					pres->type = PCI_RES_TYPE_MEM;
					pres->align = pres->size;

					/*
					   **    Is it a 64-bit address?
					 */

					if ( (base_reg & PCI_BASE_ADDRESS_MEM_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64) {
#if (BITS_PER_LONG == 64)
						/*
						   **    64-bit address. Thus, do a double read.
						 */
						base = PCI_BASE_ADDRESS_0 + ((bar + 1) << 2);
						pci_read_config_dword(pdev, base, &base_reg);
						baddr = base_reg;
						baddr <<= 32;
						base = PCI_BASE_ADDRESS_0 + (bar << 2);
						pci_read_config_dword(pdev, base, &base_reg);
						baddr |= (base_reg & PCI_BASE_ADDRESS_MEM_MASK);
#else
						printk
						    ("PCI: Unable to handle 64-bit address for device %02x:%02x\n",
						     pbus->number,
						     pdev->devfn);
#endif
					}
					/*
					   **    Prefetchable memory?
					 */

					if ( (base_reg & PCI_BASE_ADDRESS_MEM_PREFETCH) != 0) {
						/*
						   **    Prefetchable memory. Set PCI
						   **    prefetch memory variables.
						 */
						pres_head = &(pbus-> res_head_mempf);
						pres->type = PCI_RES_TYPE_MEMPF;
					}
					/*
					   **    Check to see if address is below 1M.
					 */

					if (baddr < 0x00100000UL) {
						/*
						   **    Below 1M. Set PCI memory variables.
						 */
						pres_head = &(pbus-> res_head_mem20);
						pres->type = PCI_RES_TYPE_MEM20;
					}
				}

			}
			/* not class PCI_BRIDGE */
			else {	/* class PCI_BRIDGE */
				u8 io_base_lo, io_limit_lo;
				u16 mem_base_lo, mem_limit_lo, io_base_hi, io_limit_hi;
				u32 mem_base_hi, mem_limit_hi;
				unsigned long base, limit;

				switch (pres->type) {

				case PCI_RES_TYPE_IO:
					pres_head = &(pbus->res_head_io);
					pres->align = pres->size;
					pci_read_config_byte(pdev, PCI_IO_BASE, &io_base_lo);
					pci_read_config_byte(pdev, PCI_IO_LIMIT, &io_limit_lo);
					pci_read_config_word(pdev, PCI_IO_BASE_UPPER16, &io_base_hi);
					pci_read_config_word(pdev, PCI_IO_LIMIT_UPPER16, &io_limit_hi);
					base = ((io_base_lo & PCI_IO_RANGE_MASK) << 8) | (io_base_hi << 16);
					limit = ((io_limit_lo & PCI_IO_RANGE_MASK) << 8) | (io_limit_hi << 16);
					if (base && base <= limit) {
						baddr = base;
					} else {
						printk(KERN_INFO
						       "PCI: A configured P2P bridge found at bus 0x%x devfn 0x%x with no PCI I/O allocated\n",
						       pbus->number,
						       pdev->devfn);
						return (PCI_OK);
					}

					break;

				case PCI_RES_TYPE_MEM:
					pres_head = &(pbus->res_head_mem);
					pres->align = pres->size;
					pci_read_config_word(pdev, PCI_MEMORY_BASE, &mem_base_lo);
					pci_read_config_word(pdev, PCI_MEMORY_LIMIT, &mem_limit_lo);
					base = (mem_base_lo & PCI_MEMORY_RANGE_MASK) << 16;
					limit = (mem_limit_lo & PCI_MEMORY_RANGE_MASK) << 16;
					if (base && base <= limit) {
						baddr = base;
					} else {
						printk(KERN_INFO
						       "PCI: A configured P2P bridge found at bus 0x%x devfn 0x%x with no PCI Memory allocated\n",
						       pbus->number,
						       pdev->devfn);
						return (PCI_OK);
					}
					break;

				case PCI_RES_TYPE_MEM20:
					pres_head = &(pbus->res_head_mem20);
					pres->align = pres->size;
					return (PCI_OK);
					break;

				case PCI_RES_TYPE_MEMPF:
					pres_head = &(pbus->res_head_mempf);
					pres->align = pres->size;

					pci_read_config_word(pdev, PCI_PREF_MEMORY_BASE, &mem_base_lo);
					pci_read_config_word(pdev, PCI_PREF_MEMORY_LIMIT, &mem_limit_lo);
					pci_read_config_dword(pdev, PCI_PREF_BASE_UPPER32, &mem_base_hi);
					pci_read_config_dword(pdev, PCI_PREF_LIMIT_UPPER32, &mem_limit_hi);
					base = (mem_base_lo & PCI_MEMORY_RANGE_MASK) << 16;
					limit = (mem_limit_lo & PCI_MEMORY_RANGE_MASK) << 16;
#if BITS_PER_LONG == 64
					base |= ((long) mem_base_hi) << 32;
					limit |= ((long) mem_limit_hi) << 32;
#else
					if (mem_base_hi || mem_limit_hi) {
						printk(KERN_ERR
						       "PCI: Unable to handle 64-bit address space for device at bus 0x%x devfn 0x%x\n",
						       pbus->number,
						       pdev->devfn);
						return (PCI_ERR);
					}
#endif
					if (base && base <= limit) {
						baddr = base;
					} else {
						pres_bus = &(pbus-> resources [PCI_RES_INDX_MEMPF]);
						baddr = PCI_ALIGN_ADDR(pres_bus->baddr, pres->align);
						printk(KERN_INFO
						       "PCI: A configured P2P bridge found at bus 0x%x devfn 0x%x with no PCI Prefetchable Memory allocated\n",
						       pbus->number,
						       pdev->devfn);
					}
					break;

				default:
					printk(KERN_ERR
					       "PCI: Unknown resource type for bus 0x%x devfn 0x%x bar %d\n",
					       pbus->number, pdev->devfn,
					       bar);
					return (-EIDRM);
				}
			}	/* end device class */
			/*
			 **    Clear the next field and set the
			 **    Base Address register field.
			 */

			pres->next = (struct pci_res *) NULL;
			pres->baddr = baddr;

			local_irq_save(flags);

			/*
			 **    Is this the first entry into the list?
			 */

			if ((*pres_head) == (struct pci_res *) NULL) {
				/*
				 **    This is the first entry in the linked list.
				 */

				(*pres_head) = pres;
			} else {
				/*
				 **    Should this be the new first entry?
				 */

				if ((*pres_head)->baddr > baddr) {
					/*
					 **    This is the new first entry in the linked list.
					 */

					pres->next = (*pres_head);
					(*pres_head) = pres;
				} else {
					/*
					 **    Find the entry point in the list.
					 */

					pres_cur = (*pres_head);
					while (pres_cur->next != (struct pci_res *) NULL) {
						if ((pres_cur->next)-> baddr > baddr) {
							pres->next = pres_cur->next;
							break;
						} else
							pres_cur = pres_cur->next;
					}
					pres_cur->next = pres;
				}
			}

			local_irq_restore(flags);
			return (PCI_OK);
		}
		/* not configured */
		else {

			/*
			   **    Determine which resource we will
			   **    allocate from.
			 */

			switch (pres->type) {
				/*
				   **    PCI I/O space
				 */

			case PCI_RES_TYPE_IO:
				{
					pres_bus = &(pbus->resources[PCI_RES_INDX_IO]);
					pres_head = &(pbus->res_head_io);
					break;
				}

				/*
				   **    PCI memory space
				 */

			case PCI_RES_TYPE_MEM:
				{
					pres_bus = &(pbus->resources[PCI_RES_INDX_MEM]);
					pres_head = &(pbus->res_head_mem);
					break;
				}

				/*
				   **    PCI 20-bit memory space
				 */

			case PCI_RES_TYPE_MEM20:
				{
					pres_bus = &(pbus->resources [PCI_RES_INDX_MEM20]);
					pres_head =
					    &(pbus->res_head_mem20);
					break;
				}

				/*
				   **    PCI prefetch memory space
				 */

			case PCI_RES_TYPE_MEMPF:
				{
					pres_bus = &(pbus->resources[PCI_RES_INDX_MEMPF]);
					pres_head = &(pbus->res_head_mempf);
					break;
				}

			default:
				{
					/*
					   **    Return an error if the child/parent resource
					   **    structure is not setup properly.
					 */

					sprintf(err_msg,
						"unknown resource type for bus %d, devfn = 0x%02x, bar = %d",
						pbus->number, pdev->devfn,
						bar);
					pci_kern_err(func, err_msg);
					return (-EIDRM);
				}
			}

			/*
			   **    Get the PCI size and alignment values.
			 */

			size = pres->size;
			align = pres->align;

			/*
			   **    Use a first fit algorithm for allocating the bus
			   **    address. There are four possible scenarios when
			   **    inserting into the resource list. 
			   **
			   **    1. This is a empty list and insertion happens at the
			   **       head.
			   **
			   **    2. This is a non empty list and there is room at the
			   **       head of the list. Insertion happens at the head
			   **       of the list.
			   **
			   **    3. This is a non empty list. Scanning for gaps in list
			   **       succeeds and insertion happens in between two
			   **       existing entries.
			   **
			   **    4. This is a non empty list. Scanning for gaps fails
			   **       and insertion happens at the end of the list.
			   **
			   **
			   **    Scenario 1: Empty list
			 */

			local_irq_save(flags);
			if (*pres_head == (struct pci_res *) NULL) {
				/*
				   **    This is the first entry in the linked list.
				 */

				pres->next = *pres_head;
				*pres_head = pres;

				pres->baddr = PCI_ALIGN_ADDR(pres_bus->baddr, align);
				local_irq_restore(flags);
				return (PCI_OK);
			}
			/*
			   **    Scenario 2: Non Empty list
			   **
			   **    First, check if there is a gap at the head.
			 */

			gap = (*pres_head)->baddr - pres_bus->baddr;
			if (gap >= size) {
				/*
				   **    Do an address alignment first.
				 */

				baddr = PCI_ALIGN_ADDR(pres_bus->baddr, align);
#ifdef CONFIG_PCI_BRIDGE_CTL_ISA_MODE
				/*
				   **    Is this an I/O address?
				 */

				if (pres->type == PCI_RES_TYPE_IO) {
					/*
					   **    Check for an "unusable ISA address". If
					   **    so, then adjust for a "usable ISA address".
					 */

					if ((baddr & 0x0300) != 0)
						baddr = PCI_ALIGN_ADDR(baddr, 0x400);
				}
#endif

				/*
				   **    Do a size check.
				 */

				gap = (*pres_head)->baddr - baddr;
				if (gap >= size) {
					/*
					   **    We can fit the resource at the head. Thus,
					   **    setup the linked list.
					 */

					pres->next = (*pres_head);
					(*pres_head) = pres;

					/*
					   **    The bus address is the start address of the bus
					 */

					pres->baddr = baddr;

					/*
					   **    Restore interrupts and return.
					 */

					local_irq_restore(flags);
					return (PCI_OK);
				}
			}
			/*
			   **    Scenario 3: Non Empty list
			   **
			   **    Check if there is a gap in the middle of the linked list.
			 */

			pres_cur = (*pres_head);
			pres_nxt = pres_cur->next;

			while (pres_nxt != (struct pci_res *) NULL) {
				/*
				   **    This check speeds up the search.
				 */

				gap = pres_nxt->baddr - (pres_cur->baddr + pres_cur->size);
				if (gap >= size) {
					/*
					   **    Do an address alignment first.
					 */

					baddr = pres_cur->baddr + pres_cur->size;
					baddr = PCI_ALIGN_ADDR(baddr, align);
#ifdef CONFIG_PCI_BRIDGE_CTL_ISA_MODE
					/*
					   **    Is this an I/O address?
					 */

					if (pres->type == PCI_RES_TYPE_IO) {
						/*
						   **    Check for an "unusable ISA address". If
						   **    so, then adjust for a "usable ISA address".
						 */
						if ((baddr & 0x0300) != 0)
							baddr = PCI_ALIGN_ADDR(baddr, 0x400);
					}
#endif

					/*
					   **    Do a size check.
					 */

					gap = pres_nxt->baddr - baddr;
					if (gap >= size) {
						/*
						   **    This is an aligned fit. Setup the
						   **    linked list.
						 */

						pres->next = pres_nxt;
						pres_cur->next = pres;
						/*
						   **    The bus address is the start address
						   **    of the bus.
						 */
						pres->baddr = baddr;
						local_irq_restore(flags);
						return (PCI_OK);
					}
				}
				pres_cur = pres_nxt;
				pres_nxt = pres_cur->next;
			}

			/*
			   **    Scenario 4: Non Empty list
			   **
			   **    Current pointer is at end of list. Thus,
			   **    append to the list. First, check here to
			   **    see that we are within bounds. If no space
			   **    to allocate the BAR resource, then return
			   **    an error.
			 */

			baddr = pres_cur->baddr + pres_cur->size;
			baddr = PCI_ALIGN_ADDR(baddr, align);

#ifdef CONFIG_PCI_BRIDGE_CTL_ISA_MODE
			/*
			   **    Is this an I/O address?
			 */

			if (pres->type == PCI_RES_TYPE_IO) {
				/*
				   **    Check for an "unusable ISA address". If
				   **    so, then adjust for a "usable ISA address".
				 */

				if ((baddr & 0x0300) != 0)
					baddr = PCI_ALIGN_ADDR(baddr, 0x400);
			}
#endif

			/*
			   **    Do a size check.
			 */

			if ((baddr + size) <= (pres_bus->baddr + pres_bus->size)) {
				/*
				   **    Setup the linked list.
				 */
				pres->next = pres_nxt;
				pres_cur->next = pres;
				/*
				   **    The bus address is the start address of the bus.
				 */
				pres->baddr = baddr;
				local_irq_restore(flags);
				return (PCI_OK);
			}
			local_irq_restore(flags);
			printk(KERN_ERR "pci_alloc_devres: no space left - type %d baddr 0x%lx size 0x%lx rbaddr 0x%lx rbsize 0x%lx\n",
				pres->type, baddr, size, pres_bus->baddr, pres_bus->size);
			return (-ENOMEM);
		}
	}
	/*
	   **    No P2P bridge bus structure.
	 */

	pci_kern_err(func, no_bus_struct);
	return (-ENOLINK);
}

/*
   **    pci_free_busnum - Free PCI Bus Number
   **
   **        This function will free the PCI bus number for
   **        re-allocation.
 */

int 
pci_free_busnum(struct pci_bus *pbus)
{
	/*
	   **    First, clear the bridge's bus numbers if it
	   **    still has a device structure.
	 */

	pr_debug("pci_free_busnum: bus 0x%p\n", pbus);
	if (pbus->self != (struct pci_dev *) NULL)
		pci_write_config_dword(pbus->self, PCI_PRIMARY_BUS, 0);

	pci_busnum_alloc_status[pbus->secondary] = PCI_BUS_NOT_ALLOC;
	return (PCI_OK);
}

/*
   **    pci_free_busres - Free PCI Bus Resources
   **
   **        This is the free PCI bus resources function. It will
   **        re-allocate I/O and memory space to it's parent.
 */

int 
pci_free_busres(struct pci_bus *pbus)
{
	struct pci_res *pres;	/* Pointer To Bus Resources */
	struct pci_dev *pdev;	/* Pointer to Bus Device */
	unsigned int indx;	/* Resource Index */
	int bus_level;		/* Current Bus Level */
	char *func;		/* Function Name */
	unsigned long flags;

	/*
	   **    First, set the function name for possible error
	   **    messages. Then, initialize the bus level.
	 */

	pr_debug("pci_free_busres: bus 0x%p\n", pbus);
	func = "pci_free_busres";
	bus_level = pci_find_level(pbus);

	/*
	   **    Next, ensure child resources have been freed for
	   **    this bus.
	 */

	if ((pbus->res_head_io == (struct pci_res *) NULL)
	    && (pbus->res_head_mem == (struct pci_res *) NULL)
	    && (pbus->res_head_mem20 == (struct pci_res *) NULL)
	    && (pbus->res_head_mempf == (struct pci_res *) NULL)) {
		/*
		   **    Check for bus level 0 - it's a special case.
		 */

		if (bus_level != 0) {
			/*
			   **    Ensure that there is a P2P bridge device
			   **    structure associated with this bus.
			 */
			pdev = pbus->self;
			if (pdev == (struct pci_dev *) NULL) {
				/*
				   **    No P2P bridge device structure.
				 */
				pci_kern_err(func, no_dev_struct);
				return (-ENOLINK);
			}
			/*
			   **    For each node resource, free the space.
			 */

			for (indx = PCI_RES_INDX_IO; indx <= PCI_RES_INDX_MEMPF; ++indx) {
				/*
				   **    Set the resource pointer. Then, if space
				   **    has been allocated, free it back to the
				   **    parent bus node and link it up.
				 */
				pres = &(pbus->resources[indx]);
				if (pres->size != 0)
					pci_free_devres(pdev, pres);
			}
		}
		local_irq_save(flags);

		/*
		   **    Free PCI I/O resources.
		 */

		pres = &(pbus->resources[PCI_RES_INDX_IO]);
		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_NOTUSED;
		pres->align = 0;
		pres->size = 0;
		pres->baddr = 0;

		/*
		   **    Free PCI memory resources.
		 */

		pres = &(pbus->resources[PCI_RES_INDX_MEM]);
		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_NOTUSED;
		pres->align = 0;
		pres->size = 0;
		pres->baddr = 0;

		/*
		   **    Free PCI 20-bit memory resources.
		 */

		pres = &(pbus->resources[PCI_RES_INDX_MEM20]);
		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_NOTUSED;
		pres->align = 0;
		pres->size = 0;
		pres->baddr = 0;

		/*
		   **    Free PCI prefetch memory resources.
		 */

		pres = &(pbus->resources[PCI_RES_INDX_MEMPF]);
		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_NOTUSED;
		pres->align = 0;
		pres->size = 0;
		pres->baddr = 0;

		local_irq_restore(flags);
		return (PCI_OK);
	}
	/*
	   **    Child resources not freed.
	 */

	pci_kern_err(func, "child resources not freed");
	return (-ENOTEMPTY);
}

/*
   **    pci_free_devres - Free PCI Device Resources
   **
   **        This is the free PCI device resources function. It will
   **        re-allocate I/O and memory space to it's parent.
 */

int 
pci_free_devres(struct pci_dev *pdev, struct pci_res *pres)
{
	struct pci_bus *pbus;	/* Parent Bus Node Pointer */
	struct pci_res **pres_head;	/* Resource Head Pointer */
	struct pci_res *pres_prv;	/* Previous Resource Pointer */
	char *func;		/* Function Name */
	unsigned long flags;

	pr_debug("pci_free_devres: dev 0x%p res 0x%p\n", pdev, pres);
	/*
	   **    First, set the function name for possible error
	   **    messages. Then, disable the device.
	 */

	func = "pci_free_devres";

	pci_write_config_word(pdev, PCI_COMMAND, 0);

	/*
	   **    Next, ensure that there is a P2P bridge
	   **    bus structure associated with this device.
	 */

	pbus = pdev->bus;
	if (pbus != (struct pci_bus *) NULL) {
		/*
		   **    Setup pointer to the device and parent resource
		   **    structures. Then determine which resource we will
		   **    allocate from.
		 */

		switch (pres->type) {
			/*
			   **    PCI I/O space
			 */

		case PCI_RES_TYPE_IO:
			{
				pres_head = &(pbus->res_head_io);
				break;
			}

			/*
			   **    PCI memory space
			 */

		case PCI_RES_TYPE_MEM:
			{
				pres_head = &(pbus->res_head_mem);
				break;
			}

			/*
			   **    PCI 20-bit memory space
			 */

		case PCI_RES_TYPE_MEM20:
			{
				pres_head = &(pbus->res_head_mem20);
				break;
			}

			/*
			   **    PCI prefetch memory space
			 */

		case PCI_RES_TYPE_MEMPF:
			{
				pres_head = &(pbus->res_head_mempf);
				break;
			}

			/*
			   **    Unused PCI resource
			 */

		case PCI_RES_TYPE_NOTUSED:
			{
				return (PCI_OK);
			}

		default:
			{
				/*
				   **    Return an error if the child/parent resource
				   **    structure is not setup properly.
				 */

				pci_kern_err(func, "child/parent resource structure not setup");
				return (-EIDRM);
			}
		}

		local_irq_save(flags);

		/*
		   **    The resource being removed can be the first on the
		   **    list and, hence, has no previous entry. The other
		   **    case is that the the resource has a previous entry.
		 */

		if (*pres_head == pres) {
			/*
			   **    Case 1: First on the list
			   **
			   **    Make the next in the list the resource head.
			 */
			*pres_head = pres->next;
		} else {
			/*
			   **    Case 2: search the linked list for the entry.
			 */
			pres_prv = *pres_head;
			while ((pres_prv != (struct pci_res *) NULL)
			       && (pres_prv->next != pres))
				pres_prv = pres_prv->next;

			/*
			   **    If we have come to the end of the list without
			   **    finding the resource, the list is somehow busted.
			 */

			if (pres_prv == (struct pci_res *) NULL) {
				local_irq_restore(flags);
				pci_kern_err(func, "resource structure not on list");
				return (-ENOENT);
			}
			/*
			   **    We have found the previous entry, fix the
			   **    list pointers.
			 */
			pres_prv->next = pres->next;
		}

		/*
		   **    Clear the entry.
		 */

		pres->next = (struct pci_res *) NULL;
		pres->type = PCI_RES_TYPE_NOTUSED;
		pres->align = 0;
		pres->size = 0;
		pres->baddr = 0;

		local_irq_restore(flags);
		return (PCI_OK);
	}
	/*
	   **    No P2P bridge bus structure.
	 */

	pci_kern_err(func, no_bus_struct);
	return (-ENOLINK);
}
