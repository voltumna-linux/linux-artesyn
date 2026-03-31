/*
 * mvme5500.h
 * 
 * Definitions for the MVME5500 Board.
 *
 * Author: Ajit Prem
 *
 * Copyright 2003-2007 Motorola Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/*
 * The GT64260 has 2 PCI buses each with 1 window from the CPU bus to
 * PCI I/O space and 4 windows from the CPU bus to PCI MEM space.
 * We'll only use one PCI MEM window on each PCI bus.
 *
 * This is the CPU physical memory map (windows must be at least 1MB and start
 * on a boundary that is a multiple of the window size):
 *
 * 	0x80000000-0xdfffffff		- PCI 0 MEM 
 * 	0xe0000000-0xefffffff		- PCI 1 MEM
 *      0xf0000000-0xf07fffff           - PCI 0 I/O
 *      0xf0800000-0xf08fffff           - PCI 1 I/O
 *      0xf0900000-0xf0ffffff           - <hole>
 *      0xf1000000-0xf10fffff           - GT64260 Registers
 *      0xf1100000-0xf110ffff           - GT64260 Device 1 Bus Registers
 *      0xf1110000-0xf1117fff           - TODC chip on device module
 *      0xf1120000-0xf1120fff           - COM1 UART on device module
 *      0xf1121000-0xf1121fff           - COM2 UART on device module   
 *      0xf1122000-0xf1ffffff           - <hole>
 *      0xf2000000-0xf3ffffff           - Bank A Flash
 *      0xf4000000-0xfefbffff           - <hole>
 *      0xfefc0000-0xfeffffff           - Internal SRAM 
 * 	0xff800000-0xffffffff		- Bank B FLASH 
 */

#ifndef __MVME5500_H
#define __MVME5500_H

/*
 * CPU Physical Memory Map setup.
 */
#define	MVME5500_BRIDGE_REG_BASE	0xf1000000
#define	MVME5500_TODC_BASE		0xf1110000
#define	MVME5500_UART_BASE		0xf1120000
#define	MVME5500_BANK_A_FLASH_BASE	0xf2000000
#define	MVME5500_INTERNAL_SRAM_BASE	0xfefc0000
#define	MVME5500_BANK_B_FLASH_BASE	0xff800000

#define	MVME5500_BANK_B_FLASH_SIZE	0x00800000	/* 8MB soldered FLASH */
#define	MVME5500_BANK_A_FLASH_SIZE	0x02000000	/* 32MB socketed FLASH */
#define	MVME5500_TODC_SIZE_ACTUAL	0x00008000	/* 32KB for TODC */
#define MVME5500_UART_SIZE_ACTUAL       0x00001000	/* 4K per UART */

#define	MVME5500_INTERNAL_SRAM_SIZE	0x40000	/* 256K */

#define MVME5500_TODC_SIZE              MVME5500_TODC_SIZE_ACTUAL

#define MVME5500_NVRAM_BASE_ADDRESS	0xf1110000U
#define MVME5500_NVRAM_SIZE		0x8000

#define MVME5500_PCI_DRAM_OFFSET        0x00000000
#define MVME5500_ISA_MEM_BASE           0x00000000

#define MVME5500_PCI_0_MEM_SIZE          0x60000000U
#define MVME5500_PCI_0_IO_SIZE           0x00800000U

#define MVME5500_PCI_1_MEM_SIZE          0x10000000U
#define MVME5500_PCI_1_IO_SIZE           0x00800000U

#define MVME5500_PCI_0_MEM_BASE_ADDR_PROC  0x80000000U	/* CPU Address */
#define MVME5500_PCI_1_MEM_BASE_ADDR_PROC  0xe0000000U	/* CPU Address */
#define MVME5500_PCI_0_IO_BASE_ADDR_PROC   0xf0000000U
#define MVME5500_PCI_1_IO_BASE_ADDR_PROC   0xf0800000U

#define MVME5500_PCI_0_MEM_BASE_ADDR       0x80000000U	/* PCI Address */
#define MVME5500_PCI_1_MEM_BASE_ADDR       0xe0000000U	/* PCI Address */
#define MVME5500_PCI_0_IO_BASE_ADDR        0x00000000U
#define MVME5500_PCI_1_IO_BASE_ADDR        0x00800000U

/* PCI Bus 0 Definitions */

/* Processor Physical addresses */
#define MVME5500_PCI_0_MEM_START_PROC    MVME5500_PCI_0_MEM_BASE_ADDR_PROC
#define MVME5500_PCI_0_MEM_END_PROC      (MVME5500_PCI_0_MEM_START_PROC + \
                                         MVME5500_PCI_0_MEM_SIZE - 1)

#define MVME5500_PCI_0_IO_START_PROC     MVME5500_PCI_0_IO_BASE_ADDR_PROC
#define MVME5500_PCI_0_IO_END_PROC       (MVME5500_PCI_0_IO_START_PROC + \
                                         MVME5500_PCI_0_IO_SIZE - 1)

/* PCI 0 MEM address */
#define MVME5500_PCI_0_MEM_START         MVME5500_PCI_0_MEM_BASE_ADDR
#define MVME5500_PCI_0_MEM_END           (MVME5500_PCI_0_MEM_START + \
                                         MVME5500_PCI_0_MEM_SIZE - 1)

/* PCI 0 I/O address */
#define MVME5500_PCI_0_IO_START          MVME5500_PCI_0_IO_BASE_ADDR
#define MVME5500_PCI_0_IO_END            (MVME5500_PCI_0_IO_START + \
                                         MVME5500_PCI_0_IO_SIZE - 1)

/*
 * PCI Bus 1 Definitions
 */
/* Processor Physical addresses */
#define MVME5500_PCI_1_MEM_START_PROC    MVME5500_PCI_1_MEM_BASE_ADDR_PROC
#define MVME5500_PCI_1_MEM_END_PROC      (MVME5500_PCI_1_MEM_START_PROC + \
                                         MVME5500_PCI_1_MEM_SIZE - 1)
#define MVME5500_PCI_1_IO_START_PROC     MVME5500_PCI_1_IO_BASE_ADDR_PROC
#define MVME5500_PCI_1_IO_END_PROC       (MVME5500_PCI_1_IO_START_PROC + \
                                         MVME5500_PCI_1_IO_SIZE - 1)

/* PCI 1 MEM address */
#define MVME5500_PCI_1_MEM_START         MVME5500_PCI_1_MEM_BASE_ADDR
#define MVME5500_PCI_1_MEM_END           (MVME5500_PCI_1_MEM_START + \
                                         MVME5500_PCI_1_MEM_SIZE - 1)

/* PCI 1 I/O address */
#define MVME5500_PCI_1_IO_START          MVME5500_PCI_1_IO_BASE_ADDR
#define MVME5500_PCI_1_IO_END            (MVME5500_PCI_1_IO_START + \
                                         MVME5500_PCI_1_IO_SIZE - 1)

#define MVME5500_UART_0_IRQ             64
#define MVME5500_UART_1_IRQ             64

#define	MVME5500_UART_BASE		0xf1120000
#define MVME5500_UART_SIZE              0x8

/* MVME5500 board register addresses. */
#define MVME5500_BOARD_STATUS_REG_1       0xf1100000
#define MVME5500_BOARD_STATUS_REG_2       0xf1100001
#define MVME5500_BOARD_STATUS_REG_3       0xf1100002
#define MVME5500_BOARD_PRESENCE_REG       0xf1100004
#define MVME5500_BOARD_SW_READ_REG        0xf1100005
#define MVME5500_BOARD_TBEN_REG           0xf1100006
#define MVME5500_BOARD_GEO_ADDR_REG       0xf1100007

/* Status Register 1 */
#define MVME5500_BOARD_REF_CLOCK   		0x80
#define MVME5500_BOARD_BANK_SEL_MASK   		0x40
#define MVME5500_BOARD_SAFE_START		0x20
#define MVME5500_BOARD_ABORT_STATUS		0x10
#define MVME5500_BOARD_FLASH_BUSY		0x08
#define MVME5500_BOARD_FUSE_STAT		0x04

/* Status Register 2 */
#define MVME5500_BOARD_FAIL_MASK     		0x80
#define MVME5500_BOARD_EEPROM_WP_MASK    	0x40
#define MVME5500_BOARD_FLASH_WP_MASK     	0x20
#define MVME5500_BOARD_TSTAT_MASK		0x10
#define MVME5500_BOARD_PCIC_M66EN		0x04
#define MVME5500_BOARD_PCIB_M66EN		0x02
#define MVME5500_BOARD_PCIA_M66EN		0x01

/* Status Register 3 */
#define MVME5500_BOARD_RESET_MASK		0x80
#define MVME5500_BOARD_ABORT_INT_MASK		0x40

/* Presence Detect Register */
#define MVME5500_BOARD_EREADY1			0x10
#define MVME5500_BOARD_EREADY0			0x08
#define MVME5500_BOARD_PMCSPAN			0x04
#define MVME5500_BOARD_PMC2			0x02
#define MVME5500_BOARD_PMC1			0x01

/* TBEN register */
#define MVME5500_BOARD_TBEN0			0x01

/* Geographic address register */
#define MVME5500_BOARD_GEO_ADDR_MASK		0x1f

/*
 * Serial driver setup.
 */
#define MVME5500_SERIAL_0               MVME5500_UART_BASE
#define MVME5500_SERIAL_1               MVME5500_UART_BASE + 0x1000

#define MVME5500_BASE_BAUD 1843200
#define BASE_BAUD (MVME5500_BASE_BAUD / 16)

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

/* Required for bootloader's ns16550.c code */
#define STD_SERIAL_PORT_DFNS 						\
        { 0, BASE_BAUD, MVME5500_SERIAL_0, 64, STD_COM_FLAGS, /* ttyS0 */ \
	iomem_base: (u8 *)MVME5500_SERIAL_0,				\
	iomem_reg_shift: 0,						\
	io_type: SERIAL_IO_MEM },					\
        { 0, BASE_BAUD, MVME5500_SERIAL_1, 64, STD_COM_FLAGS,  		 \
	iomem_base: (u8 *)MVME5500_SERIAL_1,				\
	iomem_reg_shift: 0,						\
	io_type: SERIAL_IO_MEM },

#define SERIAL_PORT_DFNS STD_SERIAL_PORT_DFNS

#endif				/* __MVME5500_H */
