/*
 * arch/ppc/platforms/cpci695.h
 * 
 * Definitions for Motorola CPCI695 board.
 *
 * Copyright 2007 Motorola, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __PPC_PLATFORMS_CPCI695_H
#define __PPC_PLATFORMS_CPCI695_H

#ifndef	MAX
#define	MAX(a,b)	(((a) > (b)) ? (a) : (b))
#endif

/*
 *  CPU Physical Memory Map setup.
 *
 *     00000000-DFFFFFFF       64 bit  Main memory
 *     E0000000-F7FFFFFF       64 bit  PCI memory
 *       0xe0000000-0xefffffff         PCI0 MEM
 *       0xf0000000-0xf5ffffff         PCI1 MEM
 *       0xf6000000-0xf6ffffff         PCI0 I/O
 *       0xf7000000-0xf7ffffff         PCI1 I/O
 *     F8000000-FBFFFFFF       8 bit   User Flash (CS3)
 *     FE000000-FE00FFFF       64 bit  System controller registers
 *     FE100000-FE10007F       8 bit   Board registers (CS0)
 *     FE100080-FE10010F       8 bit   Serial Controller (CS0)
 *     FE200000-FE207FFF       8 bit   RTC/NVRAM (CS1)
 *     FE300000-FE30000F       8 bit   Intelligent Board Management Unit (CS2)
 *     FE400000-FE43FFFF       64 bit  Integrated SRAM
 *     FE500000-FE5FFFFF       8 bit   Failsafe Boot Flash (BOOTCS) (512Kb/1Mb)  *     FFF00000-FFFFFFFF       8 bit   Primary Boot Flash (BOOTCS)  (1Mb)
 */

#define	CPCI695_BOOT_FLASH_BASE		0xfff00000
#define	CPCI695_BOOT_FLASH_SIZE_ACTUAL	0x00100000	/* 1024KB of BOOT FLASH */
#define	CPCI695_BOOT_FLASH_SIZE		MAX(MV64360_WINDOW_SIZE_MIN, \
						CPCI695_BOOT_FLASH_SIZE_ACTUAL)

#define CPCI695_USER_FLASH_BASE	        0xf8000000
#define CPCI695_USER_FLASH_SIZE_ACTUAL	0x04000000
#define	CPCI695_USER_FLASH_SIZE		MAX(MV64360_WINDOW_SIZE_MIN, \
						CPCI695_USER_FLASH_SIZE_ACTUAL)

#define	CPCI695_BRIDGE_REG_BASE		0xfe000000
#define	CPCI695_BRIDGE_REG_SIZE_ACTUAL	_64K
#define	CPCI695_BRIDGE_REG_SIZE		MAX(MV64360_WINDOW_SIZE_MIN, \
						CPCI695_BRIDGE_REG_SIZE_ACTUAL)
#define CPCI695_BOARD_REG_BASE         0xfe100000
#define CPCI695_BOARD_REG_SIZE_ACTUAL  0x180
#define CPCI695_BOARD_REG_SIZE         MAX(MV64360_WINDOW_SIZE_MIN,    \
                                                CPCI695_BOARD_REG_SIZE_ACTUAL)

#define CPCI695_UART_BASE              0xfe100080
#define CPCI695_UART_SIZE_ACTUAL       0x80
#define CPCI695_UART_SIZE              MAX(MV64360_WINDOW_SIZE_MIN,    \
                                                    CPCI695_UART_SIZE_ACTUAL)

#define CPCI695_TODC_BASE              0xfe200000
#define CPCI695_TODC_SIZE_ACTUAL       _32K	/* Size of NVRAM + RTC regs */
#define CPCI695_TODC_SIZE              MAX(MV64360_WINDOW_SIZE_MIN,    \
                                                CPCI695_TODC_SIZE_ACTUAL)

#define CPCI695_IPMI_BASE              0xfe300000
#define CPCI695_IPMI_SIZE_ACTUAL       0x10	/* 16 bytes of IPMI regs */
#define CPCI695_IPMI_SIZE              MAX(MV64360_WINDOW_SIZE_MIN,    \
                                                CPCI695_IPMI_SIZE_ACTUAL)

#define CPCI695_INTERNAL_SRAM_BASE     0xfe400000
#define CPCI695_INTERNAL_SRAM_SIZE_ACTUAL      _256K
#define CPCI695_INTERNAL_SRAM_SIZE     MAX(MV64360_WINDOW_SIZE_MIN,    \
                                            CPCI695_INTERNAL_SRAM_SIZE_ACTUAL)

#define CPCI695_PCI_DRAM_OFFSET        0x00000000
#define CPCI695_ISA_MEM_BASE           0x00000000

/* Memory Map */
#define CPCI695_PCI_0_MEM_SIZE          0x10000000U
#define CPCI695_PCI_0_IO_SIZE           0x01000000U

#define CPCI695_PCI_1_MEM_SIZE          0x06000000U
#define CPCI695_PCI_1_IO_SIZE           0x01000000U

#define CPCI695_PCI_0_MEM_BASE_ADDR_PROC  0xe0000000U	/* CPU Address */
#define CPCI695_PCI_0_IO_BASE_ADDR_PROC   0xf6000000U
#define CPCI695_PCI_1_MEM_BASE_ADDR_PROC  0xf0000000U	/* CPU Address */
#define CPCI695_PCI_1_IO_BASE_ADDR_PROC   0xf7000000U

#define CPCI695_PCI_0_MEM_BASE_ADDR       0xe0000000U	/* PCI Address */
#define CPCI695_PCI_0_IO_BASE_ADDR        0x00000000U
#define CPCI695_PCI_1_MEM_BASE_ADDR       0xf0000000U	/* PCI Address */
#define CPCI695_PCI_1_IO_BASE_ADDR        0x01000000U

/* PCI Bus 0 Definitions */

/* Processor Physical addresses */
#define CPCI695_PCI_0_MEM_START_PROC    CPCI695_PCI_0_MEM_BASE_ADDR_PROC
#define CPCI695_PCI_0_MEM_END_PROC      (CPCI695_PCI_0_MEM_START_PROC + \
                                         CPCI695_PCI_0_MEM_SIZE - 1)
#define CPCI695_PCI_0_IO_START_PROC     CPCI695_PCI_0_IO_BASE_ADDR_PROC
#define CPCI695_PCI_0_IO_END_PROC       (CPCI695_PCI_0_IO_START_PROC + \
                                         CPCI695_PCI_0_IO_SIZE - 1)

/* PCI 0 MEM address */
#define CPCI695_PCI_0_MEM_START         CPCI695_PCI_0_MEM_BASE_ADDR
#define CPCI695_PCI_0_MEM_END           (CPCI695_PCI_0_MEM_START + \
                                         CPCI695_PCI_0_MEM_SIZE - 1)

/* PCI 0 I/O address */
#define CPCI695_PCI_0_IO_START          CPCI695_PCI_0_IO_BASE_ADDR
#define CPCI695_PCI_0_IO_END            (CPCI695_PCI_0_IO_START + \
                                         CPCI695_PCI_0_IO_SIZE - 1)

/* PCI Bus 1 Definitions */

/* Processor Physical addresses */
#define CPCI695_PCI_1_MEM_START_PROC    CPCI695_PCI_1_MEM_BASE_ADDR_PROC
#define CPCI695_PCI_1_MEM_END_PROC      (CPCI695_PCI_1_MEM_START_PROC + \
                                         CPCI695_PCI_1_MEM_SIZE - 1)
#define CPCI695_PCI_1_IO_START_PROC     CPCI695_PCI_1_IO_BASE_ADDR_PROC
#define CPCI695_PCI_1_IO_END_PROC       (CPCI695_PCI_1_IO_START_PROC + \
                                         CPCI695_PCI_1_IO_SIZE - 1)

/* PCI 1 MEM address */
#define CPCI695_PCI_1_MEM_START         CPCI695_PCI_1_MEM_BASE_ADDR
#define CPCI695_PCI_1_MEM_END           (CPCI695_PCI_1_MEM_START + \
                                         CPCI695_PCI_1_MEM_SIZE - 1)

/* PCI 1 I/O address */
#define CPCI695_PCI_1_IO_START          CPCI695_PCI_1_IO_BASE_ADDR
#define CPCI695_PCI_1_IO_END            (CPCI695_PCI_1_IO_START + \
                                         CPCI695_PCI_1_IO_SIZE - 1)

#define CPCI695_PCI0_INTA_IRQ                  64+24
#define CPCI695_PCI0_INTB_IRQ                  64+25
#define CPCI695_PCI0_INTC_IRQ                  64+26
#define CPCI695_PCI0_INTD_IRQ                  64+27

#define CPCI695_PCI1_INTA_IRQ                  64+12
#define CPCI695_PCI1_INTB_IRQ                  64+13
#define CPCI695_PCI1_INTC_IRQ                  64+14
#define CPCI695_PCI1_INTD_IRQ                  64+15

#define CPCI695_ETH0_PHY_ADDR                   0
#define CPCI695_ETH1_PHY_ADDR                   1
#define CPCI695_ETH2_PHY_ADDR                   2

#define CPCI695_ETH_TX_QUEUE_SIZE               800
#define CPCI695_ETH_RX_QUEUE_SIZE               400

/* Board reg offsets */
#define CPCI695_LED_CTRL               0x00
#define CPCI695_SW_RESET               0x01
#define CPCI695_LAST_RESET_STATUS      0x02
#define CPCI695_SWITCH_STATUS          0x03
#define CPCI695_MISC_CTRL              0x04
#define CPCI695_FAILSAFE_CTRL          0x05
#define CPCI695_FAILSAFE_TIMER         0x06
#define CPCI695_WDT_TRIGGER            0x07
#define CPCI695_PCIX_STATUS            0x08
#define CPCI695_MEM_STATUS             0x09
#define CPCI695_SYS_CONF               0x0A
#define CPCI695_LAST_RESET_EXT_STATUS  0x0C

/* Switches */
#define CPCI695_SWITCH_STATUS_PR_BOOT_FLASH_WE 0x1	/* SW2 -> defines the Primary Boot flash WP */
#define CPCI695_SWITCH_STATUS_FS_BOOT_FLASH_WP 0x2	/* SW2 -> defines the Failsafe Boot flash WP */
#define CPCI695_SWITCH_STATUS_USER_FLASH_WE    0x4	/* SW2 -> defines the User Boot flash WP */
#define CPCI695_FAILSAFE_CTRL_FS_BOOT_SELECT   0x4	/* SW4 -> Primary or Failsafe Boot flash selected */
#define CPCI695_MISC_CTRL_USER_FLASH_SIZE      0x2	/* User flash size 32/64Mb */

#define CPCI695_PCIX_STATUS_PCIX       0x01
#define CPCI695_PCI_BUS_0_SPD_MASK     0x06
#define CPCI695_PCI_BUS_1_SPD_MASK     0x08
#define CPCI695_PCI_BUS_0_SPD_133      0x06
#define CPCI695_PCI_BUS_0_SPD_100      0x04
#define CPCI695_PCI_BUS_0_SPD_66       0x02
#define CPCI695_PCI_BUS_0_SPD_33       0x00

#define CPCI695_MAC_OFFSET             0x7c10	/* MAC addrs location in NVRAM */

#define CPCI695_SW_RESET_REG		0xFE100001
#define CPCI695_SW_RESET_MASK		0x11

/*
 * Serial driver setup.
 */
#define CPCI695_SERIAL_0_IOBASE		CPCI695_UART_BASE
#define CPCI695_SERIAL_1_IOBASE		CPCI695_UART_BASE + 0x80

#define	CPCI695_UART_0_IRQ		64+9
#define	CPCI695_UART_1_IRQ		64+10

#define CPCI695_BASE_BAUD 1843200
#define BASE_BAUD (CPCI695_BASE_BAUD / 16)

#ifdef CONFIG_SERIAL_MANY_PORTS
#define RS_TABLE_SIZE	64
#else
#define RS_TABLE_SIZE	2
#endif

#ifdef CONFIG_SERIAL_DETECT_IRQ
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST|ASYNC_AUTO_IRQ)
#else
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST)
#endif

#define STD_UART_OP(num) 						\
        { 0, BASE_BAUD, CPCI695_SERIAL_##num##_IOBASE,			\
	CPCI695_UART_##num##_IRQ, 					\
	STD_COM_FLAGS, 							\
	iomem_base: (u8 *)CPCI695_SERIAL_##num##_IOBASE,		\
	io_type: SERIAL_IO_MEM },

#define SERIAL_PORT_DFNS 	\
	STD_UART_OP(0)		\
	STD_UART_OP(1)

#endif				/* __PPC_PLATFORMS_CPCI695_H */
