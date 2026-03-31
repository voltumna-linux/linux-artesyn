/*
 * mcp905.h
 * 
 * Definitions for the MCP905 Board.
 *
 * Author: Ajit Prem (Ajit.Prem@motorola.com)
 *
 * Copyright 2003-2007 Motorola, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/*
 * The MV64360 has 2 PCI buses each with 1 window from the CPU bus to
 * PCI I/O space and 4 windows from the CPU bus to PCI MEM space.
 * We'll only use one PCI MEM window on each PCI bus.
 *
 * This is the CPU physical memory map (windows must be at least 1MB and start
 * on a boundary that is a multiple of the window size):
 *
 * 	0x80000000-0xcfffffff		- <hole>
 * 	0xd0000000-0xdfffffff		- PCI 0 MEM 
 * 	0xe0000000-0xefffffff		- PCI 1 MEM
 * 	0xf0000000-0xf07fffff		- PCI 0 I/O
 * 	0xf0800000-0xf08fffff		- PCI 1 I/O
 * 	0xf0900000-0xf0ffffff		- <hole>
 * 	0xf1000000-0xf10fffff		- MV64360 Registers
 * 	0xf1100000-0xf110ffff		- MV64360 Device 1 Bus Registers
 * 	0xf1110000-0xf1117fff		- TODC chip on device module
 * 	0xf1120000-0xf1120fff		- COM1 UART on device module
 * 	0xf1121000-0xf1121fff		- COM2 UART on device module
 * 	0xf1122000-0xf1f2ffff		- <hole>
 * 	0xf1130000-0xf113ffff		- Zircon BMC on device module
 * 	0xf1140000-0xf1ffffff		- <hole>
 * 	0xf2000000-0xf3ffffff		- Bank A Flash
 * 	0xf4000000-0xfefbffff		- <hole>
 * 	0xfefc0000-0xfeffffff		- Internal SRAM
 * 	0xff000000-0xff7fffff		- <hole>
 * 	0xff800000-0xffffffff		- Bank B Flash
 */

#ifndef __MCP905_H
#define __MCP905_H

#ifndef	MAX
#define	MAX(a,b)	(((a) > (b)) ? (a) : (b))
#endif

/*
 * CPU Physical Memory Map setup.
 */
#define	MCP905_BANK_B_FLASH_BASE	0xff800000	/* Bank B */
#define	MCP905_BANK_A_FLASH_BASE	0xf2000000	/* Bank A */
#define	MCP905_BRIDGE_REG_BASE		0xf1000000
#define	MCP905_DEVICE_CS1_BASE		0xf1100000
#define	MCP905_TODC_BASE		0xf1110000
#define	MCP905_UART_BASE		0xf1120000
#define	MCP905_BMC_BASE			0xf1130000
#define	MCP905_INTERNAL_SRAM_BASE	0xfefc0000

#define	MCP905_BANK_B_FLASH_SIZE	0x00800000	/* 8MB Bank B FLASH */
#define	MCP905_BANK_A_FLASH_SIZE	0x02000000	/* 32MB (Max) Bank A FLASH */
#define	MCP905_DEVICE_CS1_SIZE		0x00100000	/* 1MB */
#define	MCP905_INTERNAL_SRAM_SIZE	_256K	/* 256KByte internal SRAM */
#define	MCP905_TODC_SIZE		0x00008000	/* 32KB for TODC */

#define	MCP905_UART_SIZE_ACTUAL		0x00001000	/* 4K per UART */
#define	MCP905_UART_SIZE		0x8

#define MCP905_NVRAM_BASE_ADDRESS	0xf1110000U
#define MCP905_NVRAM_SIZE		0x8000

#define MCP905_PCI_DRAM_OFFSET        0x00000000
#define MCP905_ISA_MEM_BASE           0x00000000

/* MCP905 board register addresses. */
#define MCP905_BOARD_STATUS_REG_1       0xf1100000
#define MCP905_BOARD_STATUS_REG_2       0xf1100001
#define MCP905_BOARD_STATUS_REG_3       0xf1100002
#define MCP905_BOARD_GEO_ADDR_REG       0xf1100003
#define MCP905_BOARD_PRESENCE_REG       0xf1100004
#define MCP905_BOARD_SW_READ_REG        0xf1100005
#define MCP905_BOARD_TBEN_REG           0xf1100006
#define MCP905_BOARD_INT_STATUS_REG	0xf1100008

/* Status Register 1 */
#define MCP905_BOARD_REF_CLOCK		0x80
#define MCP905_BOARD_BANK_SEL_MASK	0x40
#define MCP905_BOARD_SAFE_START		0x20
#define MCP905_BOARD_ABORT_STATUS	0x10
#define MCP905_BOARD_FLASH_BUSY		0x08
#define MCP905_BOARD_SROM_INIT		0x02

/* Status Register 2 */
#define MCP905_BOARD_FAIL_MASK		0x80
#define MCP905_BOARD_EEPROM_WP_MASK	0x40
#define MCP905_BOARD_FLASH_WP_MASK	0x20
#define MCP905_BOARD_TSTAT_MASK		0x10

/* Status Register 3 */
#define MCP905_BOARD_RESET_MASK		0x80
#define MCP905_BOARD_PCI0_RESET_MASK	0x01
#define MCP905_BOARD_PCI1_RESET_MASK	0x02
#define MCP905_BOARD_ABORT_MASK		0x08

/* Geographic Address Register */
#define MCP905_BOARD_GEO_ADDR_MASK	0x1f

/* Presence Detect Register */
#define MCP905_BOARD_TM_PRESENT		0x10
#define MCP905_BOARD_EREADY2		0x08
#define MCP905_BOARD_EREADY1		0x04
#define MCP905_BOARD_PMC2		0x02
#define MCP905_BOARD_PMC1		0x01

/* TBEN register */
#define MCP905_BOARD_TBEN0		0x01

/* Interrupt Status Register */
#define MCP905_BOARD_TSTAT_INT		0x10

/* Memory Map */
#define MCP905_PCI_0_MEM_SIZE          0x10000000U
#define MCP905_PCI_0_IO_SIZE           0x00800000U

#define MCP905_PCI_1_MEM_SIZE          0x10000000U
#define MCP905_PCI_1_IO_SIZE           0x00800000U

#define MCP905_PCI_0_MEM_BASE_ADDR_PROC  0xd0000000U	/* CPU Address */
#define MCP905_PCI_0_IO_BASE_ADDR_PROC   0xf0000000U
#define MCP905_PCI_1_MEM_BASE_ADDR_PROC  0xe0000000U	/* CPU Address */
#define MCP905_PCI_1_IO_BASE_ADDR_PROC   0xf0800000U

#define MCP905_PCI_0_MEM_BASE_ADDR       0xd0000000U	/* PCI Address */
#define MCP905_PCI_0_IO_BASE_ADDR        0x00000000U
#define MCP905_PCI_1_MEM_BASE_ADDR       0xe0000000U	/* PCI Address */
#define MCP905_PCI_1_IO_BASE_ADDR        0x00800000U

/* PCI Bus 0 Definitions */

/* Processor Physical addresses */
#define MCP905_PCI_0_MEM_START_PROC    MCP905_PCI_0_MEM_BASE_ADDR_PROC
#define MCP905_PCI_0_MEM_END_PROC      (MCP905_PCI_0_MEM_START_PROC + \
                                         MCP905_PCI_0_MEM_SIZE - 1)
#define MCP905_PCI_0_IO_START_PROC     MCP905_PCI_0_IO_BASE_ADDR_PROC
#define MCP905_PCI_0_IO_END_PROC       (MCP905_PCI_0_IO_START_PROC + \
                                         MCP905_PCI_0_IO_SIZE - 1)

/* PCI 0 MEM address */
#define MCP905_PCI_0_MEM_START         MCP905_PCI_0_MEM_BASE_ADDR
#define MCP905_PCI_0_MEM_END           (MCP905_PCI_0_MEM_START + \
                                         MCP905_PCI_0_MEM_SIZE - 1)

/* PCI 0 I/O address */
#define MCP905_PCI_0_IO_START          MCP905_PCI_0_IO_BASE_ADDR
#define MCP905_PCI_0_IO_END            (MCP905_PCI_0_IO_START + \
                                         MCP905_PCI_0_IO_SIZE - 1)

/* PCI Bus 1 Definitions */

/* Processor Physical addresses */
#define MCP905_PCI_1_MEM_START_PROC    MCP905_PCI_1_MEM_BASE_ADDR_PROC
#define MCP905_PCI_1_MEM_END_PROC      (MCP905_PCI_1_MEM_START_PROC + \
                                         MCP905_PCI_1_MEM_SIZE - 1)
#define MCP905_PCI_1_IO_START_PROC     MCP905_PCI_1_IO_BASE_ADDR_PROC
#define MCP905_PCI_1_IO_END_PROC       (MCP905_PCI_1_IO_START_PROC + \
                                         MCP905_PCI_1_IO_SIZE - 1)

/* PCI 1 MEM address */
#define MCP905_PCI_1_MEM_START         MCP905_PCI_1_MEM_BASE_ADDR
#define MCP905_PCI_1_MEM_END           (MCP905_PCI_1_MEM_START + \
                                         MCP905_PCI_1_MEM_SIZE - 1)

/* PCI 1 I/O address */
#define MCP905_PCI_1_IO_START          MCP905_PCI_1_IO_BASE_ADDR
#define MCP905_PCI_1_IO_END            (MCP905_PCI_1_IO_START + \
                                         MCP905_PCI_1_IO_SIZE - 1)

#define	MCP905_UART_0_IRQ		64
#define	MCP905_UART_1_IRQ		64

#define	MCP905_UART_BASE		0xf1120000

/*
 * Serial driver setup.
 */
#define MCP905_SERIAL_0		MCP905_UART_BASE
#define MCP905_SERIAL_1		MCP905_UART_BASE +0x1000

#define MCP905_BASE_BAUD 1843200
#define BASE_BAUD (MCP905_BASE_BAUD / 16)

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

#define STD_SERIAL_PORT_DFNS 						\
        { 0, BASE_BAUD, MCP905_SERIAL_0, MCP905_UART_0_IRQ, STD_COM_FLAGS, /* ttyS0 */ \
	iomem_base: (u8 *)MCP905_SERIAL_0,				\
	iomem_reg_shift: 0,						\
	io_type: SERIAL_IO_MEM },					\
        { 0, BASE_BAUD, MCP905_SERIAL_1, MCP905_UART_1_IRQ, STD_COM_FLAGS, /* ttyS1 */ \
	iomem_base: (u8 *)MCP905_SERIAL_1,				\
	iomem_reg_shift: 0,						\
	io_type: SERIAL_IO_MEM },

#define SERIAL_PORT_DFNS \
        STD_SERIAL_PORT_DFNS

#define MCP905_ETH0_PHY_ADDR                   1
#define MCP905_ETH1_PHY_ADDR                   2
#define MCP905_ETH2_PHY_ADDR                   3

#define MCP905_ETH_TX_QUEUE_SIZE               800
#define MCP905_ETH_RX_QUEUE_SIZE               400

#endif				/* __MCP905_H */
