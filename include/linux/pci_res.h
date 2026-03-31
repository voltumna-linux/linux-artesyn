/*
**    PCI resource definitions and function prototypes
*/

#ifndef LINUX_PCI_RES_H
#define LINUX_PCI_RES_H

/*
**    PCI Bus Number/Level Definitions
**
**        MAX_PCI_NODES        - Maximum Number of Bus Nodes
**        MAX_PCI_LEVELS       - Maximum Number of Bus Levels
*/

#define BUSLVL3_NODES   CONFIG_PCI_BUSLVL3_NODES

#define BUSLVL2_NODES ( CONFIG_PCI_BUSLVL2_NODES  \
                      + ( CONFIG_PCI_BUSLVL2_NODES * BUSLVL3_NODES ) )

#define BUSLVL1_NODES ( CONFIG_PCI_BUSLVL1_NODES  \
                      + ( CONFIG_PCI_BUSLVL1_NODES * BUSLVL2_NODES ) )

#define BUSLVL0_NODES ( CONFIG_PCI_BUSLVL0_NODES  \
	                  + ( CONFIG_PCI_BUSLVL0_NODES * BUSLVL1_NODES ) )

#ifndef CONFIG_PCI_AGP_BRIDGE
#define MAX_PCI_NODES   BUSLVL0_NODES
#else
#define MAX_PCI_NODES   ( BUSLVL0_NODES + 1 )
#endif

#define MAX_PCI_LEVELS                4

/*
**    PCI IRQ Pin Definitions
**
*/

#define PCI_INT_NONE                  0
#define PCI_INTA                      1
#define PCI_INTB                      2
#define PCI_INTC                      3
#define PCI_INTD                      4

/*
**    PCI Memory and I/O Space Alignment Definitions
**
**        PCI_ALIGN_IO         - PCI I/O Alignment
**        PCI_ALIGN_MEM        - PCI Memory Alignment
**        PCI_ALIGN_MEM20      - PCI 20-bit Memory Alignment
**
**        PCI_ALIGN_ADDR       - Macro to Align PCI Memory or I/O Address
*/

#define PCI_ALIGN_IO         0x00001000
#define PCI_ALIGN_MEM        0x00100000
#define PCI_ALIGN_MEM20      0x00000010

#define PCI_ALIGN_ADDR(addr,align) ((addr + (align - 1)) & ~(align - 1))

/*
**    PCI Resource Array Indexes
**
**        PCI_RES_INDX_IO      - PCI I/O Resource Array Index
**        PCI_RES_INDX_MEM     - PCI Memory Resource Array Index
**        PCI_RES_INDX_MEM20   - PCI 20-bit Memory Resource Array Index
**        PCI_RES_INDX_MEMPF   - PCI Prefetch Memory Resource Array Index
*/

#define PCI_RES_INDX_IO               0
#define PCI_RES_INDX_MEM              1
#define PCI_RES_INDX_MEM20            2
#define PCI_RES_INDX_MEMPF            3

/*
**    PCI Resource Types
**
**        PCI_RES_TYPE_NOTUSED - Unused PCI Resouce Type
**
**        PCI_RES_TYPE_IO      - PCI I/O Resource Type
**        PCI_RES_TYPE_MEM     - PCI Memory Resource Type
**        PCI_RES_TYPE_MEM20   - PCI 20-bit Memory Resource Type
**        PCI_RES_TYPE_MEMPF   - PCI Prefetch Memory Resource Type
**
**        PCI_RES_ADDR_START   - Starting Address
**        PCI_RES_ADDR_END     - Ending Address
*/

#define PCI_RES_TYPE_NOTUSED          0
#define PCI_RES_TYPE_IO               1
#define PCI_RES_TYPE_MEM              2
#define PCI_RES_TYPE_MEM20            3
#define PCI_RES_TYPE_MEMPF            4

#define PCI_RES_ADDR_START            0
#define PCI_RES_ADDR_END              1

/*
**    PCI Bus/Device/Function
**
**        MAX_PCI_BUS          - Maximum PCI Bus Number
**        MAX_PCI_DEV          - Maximum PCI Device Number
**        MAX_PCI_FUNC         - Maximum PCI Function Number
*/

#define MAX_PCI_BUS                 255
#define MAX_PCI_DEV                  31
#define MAX_PCI_FUNC                  7

/*
**    Miscellaneous Definitions
**
**        PCI_OK               - Function Return Code - OK
**        PCI_ERR              - Function Return Code - Error
**
**        PCI_BUS_NOT_ALLOC    - PCI Bus Number Not Allocated
**        PCI_BUS_ALLOC        - PCI Bus Number Allocated
**
**        NUM_BARS             - Number of Base Address Registers
*/

#define PCI_OK                        0
#define PCI_ERR                      -1

#define PCI_BUS_NOT_ALLOC             0
#define PCI_BUS_ALLOC                 1

#define NUM_BARS                      6

#define DOMAIN_A		      0
#define DOMAIN_B		      1

#ifndef NULL
#define NULL                          0
#endif

#ifndef TRUE
#define TRUE                          1
#endif

#ifndef FALSE
#define FALSE                         0
#endif

/*
**    PCI bus number/device resource allocation arrays
**    and functions.
*/

int  pci_alloc_businit(struct pci_bus *) __init;

#if (CONFIG_PCI_BUSLVL0_NODES > 1)
int  pci_alloc_hosts(struct pci_bus *) __init;
#endif

#ifdef CONFIG_PCI_AGP_BRIDGE
int  pci_agp_bridge(struct pci_bus *pbus, unsigned int devfn);
#endif

struct pci_bus *pci_pks_scan_bus(int bus, struct pci_ops *ops, void *sysdata);
int  pci_alloc_busnum(struct pci_bus *pbus, unsigned int devfn);
int  pci_alloc_busres(struct pci_bus *pbus);
int  pci_alloc_devres(struct pci_dev *pdev, struct pci_res *pres, int bar);

int  pci_free_busnum(struct pci_bus *pbus);
int  pci_free_busres(struct pci_bus *pbus);
int  pci_free_devres(struct pci_dev *pdev, struct pci_res *pres);

/*
**    PCI bus/device node management functions.
*/

int  pci_add_dev(struct pci_bus *pbus, unsigned int devfn, struct pci_dev **pdev);
int  pci_add_bus(struct pci_bus *pbus, struct pci_dev *pdev, struct pci_bus **child);
int  pci_add_node(unsigned int number, unsigned int devfn);

int  pci_del_dev(struct pci_dev *pdev);
int  pci_del_bus(struct pci_bus *pbus);
int  pci_del_node(unsigned int number, unsigned int devfn);
int  pci_swap_node(unsigned int number, unsigned int devfn, unsigned int secdevfn);

/*
**    PCI bus/device configuration functions.
*/

int  pci_get_bars(struct pci_dev *pdev, int *num_bars);
int  pci_conf_dev(struct pci_dev *pdev, int res_flag);
int  pci_conf_bus(struct pci_bus *pbus, int res_flag);
void pci_conf_display(void);

/*
**    PCI bus scan/probe functions.
*/
/*
int  pci_find_host(unsigned int number, unsigned int *devfn) __init;
*/
int  pci_find_host(struct pci_dev *dev);
void pci_kern_err(char *func, char *err_msg);
int  pci_valid_devid(struct pci_dev *dev);
int  pci_multi_func(struct pci_dev *dev);
int  pci_probe_bus(struct pci_bus *pbus, unsigned int devfn);
int  pci_is_device_configured(struct pci_dev *pdev);

/*
**    PCI interrupt configuration function.
*/

#ifdef CONFIG_PCI_PIRQ_UPDATE
int  pci_hwints_update_pirqs(void) __init; 
#endif
void pci_hwints_getirq(struct pci_dev *pdev);

#endif /* LINUX_PCI_RES_H */
