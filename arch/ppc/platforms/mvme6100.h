/*
 * mvme6100.h
 * 
 * Definitions for the MVME6100 Board.
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
 * 	0xc0000000-0xdfffffff		- PCI 0 MEM 
 * 	0xe0000000-0xefffffff		- PCI 1 MEM 
 * 	0xf0800000-0xf08fffff		- PCI 0 I/O 
 * 	0xf0000000-0xf07fffff		- PCI 1 I/O 
 * 	0xf0900000-0xf0ffffff		- <hole>
 * 	0xf1000000-0xf10fffff		- MV64360 Registers
 * 	0xf1100000-0xf110ffff		- MV64360 Device 1 Bus Registers
 * 	0xf1110000-0xf1117fff		- TODC chip on device module
 * 	0xf1120000-0xf1120fff		- COM1 UART on device module
 * 	0xf1121000-0xf1121fff		- COM2 UART on device module
 * 	0xf1122000-0xf3ffffff		- <hole>
 * 	0xf4000000-0xf7ffffff		- Bank A Flash
 * 	0xf8000000-0xfbffffff		- Bank B Flash
 * 	0xfefc0000-0xfeffffff		- Integrated SRAM
 */

#ifndef __MVME6100_H
#define __MVME6100_H

#ifndef	MAX
#define	MAX(a,b)	(((a) > (b)) ? (a) : (b))
#endif

/*
 * CPU Physical Memory Map setup.
 */
#define	MVME6100_BANK_B_FLASH_BASE	0xf8000000	/* Bank B */
#define	MVME6100_BANK_A_FLASH_BASE	0xf4000000	/* Bank A */
#define	MVME6100_BRIDGE_REG_BASE	0xf1000000
#define	MVME6100_DEVICE_CS1_BASE	0xf1100000
#define	MVME6100_TODC_BASE		0xf1110000
#define	MVME6100_UART_BASE		0xf1120000
#define	MVME6100_INTERNAL_SRAM_BASE	0xfefc0000

#define	MVME6100_BANK_B_FLASH_SIZE	0x04000000	/* 64MB Bank B FLASH */
#define	MVME6100_BANK_A_FLASH_SIZE	0x04000000	/* 64MB (Max) Bank A FLASH */
#define	MVME6100_DEVICE_CS1_SIZE_ACTUAL	0x00100000	/* 1MB */
#define	MVME6100_INTERNAL_SRAM_SIZE	_256K	/* 256KByte internal SRAM */
#define	MVME6100_UART_SIZE_ACTUAL		0x00001000	/* 4K per UART */
#define	MVME6100_TODC_SIZE_ACTUAL		0x00008000	/* 32KB for TODC */

#define	MVME6100_DEVICE_CS1_SIZE		MAX(MV64360_WINDOW_SIZE_MIN,	\
						MVME6100_DEVICE_CS1_SIZE_ACTUAL)

#define	MVME6100_UART_SIZE		0x8
#define	MVME6100_TODC_SIZE		MVME6100_TODC_SIZE_ACTUAL

#define MVME6100_NVRAM_BASE_ADDRESS	0xf1110000U
#define MVME6100_NVRAM_SIZE 		0x8000

#define MVME6100_PCI_DRAM_OFFSET        0x00000000
#define MVME6100_ISA_MEM_BASE           0x00000000

#define _IO_BASE			isa_io_base
#define _ISA_MEM_BASE			isa_mem_base
#define PCI_DRAM_OFFSET			pci_dram_offset

/* MVME6100 board register addresses. */
#define MVME6100_BOARD_STATUS_REG_1       0xf1100000
#define MVME6100_BOARD_STATUS_REG_2       0xf1100001
#define MVME6100_BOARD_STATUS_REG_3       0xf1100002
#define MVME6100_BOARD_PRESENCE_REG       0xf1100004
#define MVME6100_BOARD_SW_READ_REG        0xf1100005
#define MVME6100_BOARD_TBEN_REG           0xf1100006
#define MVME6100_BOARD_GEO_ADDR_REG       0xf1100007

/* Status Register 1 */
#define MVME6100_BOARD_REF_CLOCK                0x80
#define MVME6100_BOARD_BANK_SEL_MASK            0x40
#define MVME6100_BOARD_SAFE_START               0x20
#define MVME6100_BOARD_ABORT_STATUS		0x10
#define MVME6100_BOARD_FLASH_BUSY               0x08
#define MVME6100_BOARD_FUSE_STAT                0x04
#define MVME6100_BOARD_SROM_INIT                0x02

/* Status Register 2 */
#define MVME6100_BOARD_FAIL_MASK                0x80
#define MVME6100_BOARD_FLASH0_SW_WP_MASK	0x20
#define MVME6100_BOARD_TSTAT_MASK               0x10
#define MVME6100_BOARD_FLASH1_BOOT_SW_WP_MASK	0x08
#define MVME6100_BOARD_FLASH0_HW_WP_MASK	0x04
#define MVME6100_BOARD_FLASH1_BOOT_HW_WP_MASK	0x02

/* Status Register 3 */
#define MVME6100_BOARD_RESET_MASK       	0x80
#define MVME6100_BOARD_PCI0_RESET_MASK  	0x01
#define MVME6100_BOARD_PCI1_RESET_MASK   	0x02

/* Presence Detect Register */
#define MVME6100_BOARD_EREADY1                  0x10
#define MVME6100_BOARD_EREADY0                  0x08
#define MVME6100_BOARD_PMCSPAN                  0x04
#define MVME6100_BOARD_PMC2                     0x02
#define MVME6100_BOARD_PMC1                     0x01

/* TBEN register */
#define MVME6100_BOARD_TBEN0                    0x01

/* Geographic Address Register 6 */
#define MVME6100_BOARD_GEO_ADDR_MASK     	0x1f

/* Memory Map */
#define MVME6100_PCI_0_MEM_SIZE          0x20000000U
#define MVME6100_PCI_0_IO_SIZE           0x00800000U

#define MVME6100_PCI_1_MEM_SIZE          0x10000000U
#define MVME6100_PCI_1_IO_SIZE           0x00800000U

#define MVME6100_PCI_0_MEM_BASE_ADDR_PROC  0xc0000000U	/* CPU Address */
#define MVME6100_PCI_0_IO_BASE_ADDR_PROC   0xf0000000U
#define MVME6100_PCI_1_MEM_BASE_ADDR_PROC  0xe0000000U	/* CPU Address */
#define MVME6100_PCI_1_IO_BASE_ADDR_PROC   0xf0800000U

#define MVME6100_PCI_0_MEM_BASE_ADDR       0xc0000000U	/* PCI Address */
#define MVME6100_PCI_0_IO_BASE_ADDR        0x00000000U
#define MVME6100_PCI_1_MEM_BASE_ADDR       0xe0000000U	/* PCI Address */
#define MVME6100_PCI_1_IO_BASE_ADDR        0x00800000U

/* PCI Bus 0 Definitions */

/* Processor Physical addresses */
#define MVME6100_PCI_0_MEM_START_PROC    MVME6100_PCI_0_MEM_BASE_ADDR_PROC
#define MVME6100_PCI_0_MEM_END_PROC      (MVME6100_PCI_0_MEM_START_PROC + \
                                         MVME6100_PCI_0_MEM_SIZE - 1)
#define MVME6100_PCI_0_IO_START_PROC     MVME6100_PCI_0_IO_BASE_ADDR_PROC
#define MVME6100_PCI_0_IO_END_PROC       (MVME6100_PCI_0_IO_START_PROC + \
                                         MVME6100_PCI_0_IO_SIZE - 1)

/* PCI 0 MEM address */
#define MVME6100_PCI_0_MEM_START         MVME6100_PCI_0_MEM_BASE_ADDR
#define MVME6100_PCI_0_MEM_END           (MVME6100_PCI_0_MEM_START + \
                                         MVME6100_PCI_0_MEM_SIZE - 1)

/* PCI 0 I/O address */
#define MVME6100_PCI_0_IO_START          MVME6100_PCI_0_IO_BASE_ADDR
#define MVME6100_PCI_0_IO_END            (MVME6100_PCI_0_IO_START + \
                                         MVME6100_PCI_0_IO_SIZE - 1)

/* PCI Bus 1 Definitions */

/* Processor Physical addresses */
#define MVME6100_PCI_1_MEM_START_PROC    MVME6100_PCI_1_MEM_BASE_ADDR_PROC
#define MVME6100_PCI_1_MEM_END_PROC      (MVME6100_PCI_1_MEM_START_PROC + \
                                         MVME6100_PCI_1_MEM_SIZE - 1)
#define MVME6100_PCI_1_IO_START_PROC     MVME6100_PCI_1_IO_BASE_ADDR_PROC
#define MVME6100_PCI_1_IO_END_PROC       (MVME6100_PCI_1_IO_START_PROC + \
                                         MVME6100_PCI_1_IO_SIZE - 1)

/* PCI 1 MEM address */
#define MVME6100_PCI_1_MEM_START         MVME6100_PCI_1_MEM_BASE_ADDR
#define MVME6100_PCI_1_MEM_END           (MVME6100_PCI_1_MEM_START + \
                                         MVME6100_PCI_1_MEM_SIZE - 1)

/* PCI 1 I/O address */
#define MVME6100_PCI_1_IO_START          MVME6100_PCI_1_IO_BASE_ADDR
#define MVME6100_PCI_1_IO_END            (MVME6100_PCI_1_IO_START + \
                                         MVME6100_PCI_1_IO_SIZE - 1)

#define MVME6100_SERIAL_0               MVME6100_UART_BASE
#define MVME6100_SERIAL_1               MVME6100_UART_BASE +0x1000

#define MVME6100_SERIAL_2   MVME6100_PCI_1_IO_BASE_ADDR_PROC + 0x03f8
#define MVME6100_SERIAL_3   MVME6100_PCI_1_IO_BASE_ADDR_PROC + 0x02f8

#define	MVME6100_UART_0_IRQ		64
#define	MVME6100_UART_1_IRQ		64
#define	MVME6100_UART_2_IRQ		100
#define	MVME6100_UART_3_IRQ		99

#define MVME6100_BASE_BAUD 1843200
#define BASE_BAUD (MVME6100_BASE_BAUD / 16)

#ifdef CONFIG_SERIAL_MANY_PORTS
#define RS_TABLE_SIZE   64
#else
#define RS_TABLE_SIZE   4
#endif

#ifdef CONFIG_SERIAL_DETECT_IRQ
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST|ASYNC_AUTO_IRQ)
#else
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST)
#endif

/* Required for bootloader's ns16550.c code */
#define STD_SERIAL_PORT_DFNS                                            \
        { 0, BASE_BAUD, MVME6100_SERIAL_0, MVME6100_UART_0_IRQ, STD_COM_FLAGS, /* ttyS0 */ \
        iomem_base: (u8 *)MVME6100_SERIAL_0,                            \
        iomem_reg_shift: 0,                                             \
        io_type: SERIAL_IO_MEM },                                       \
        { 0, BASE_BAUD, MVME6100_SERIAL_1, MVME6100_UART_1_IRQ, STD_COM_FLAGS, /* ttyS1 */  \
        iomem_base: (u8 *)MVME6100_SERIAL_1,                            \
        iomem_reg_shift: 0,                                             \
        io_type: SERIAL_IO_MEM },					\
        { 0, BASE_BAUD, MVME6100_SERIAL_2, MVME6100_UART_2_IRQ, STD_COM_FLAGS, /* ttyS2 */  \
        iomem_base: (u8 *)MVME6100_SERIAL_2,                            \
        iomem_reg_shift: 0,                                             \
        io_type: SERIAL_IO_MEM },					\
        { 0, BASE_BAUD, MVME6100_SERIAL_3, MVME6100_UART_3_IRQ, STD_COM_FLAGS, /* ttyS3 */  \
        iomem_base: (u8 *)MVME6100_SERIAL_3,                            \
        iomem_reg_shift: 0,                                             \
        io_type: SERIAL_IO_MEM },

#define SERIAL_PORT_DFNS 		STD_SERIAL_PORT_DFNS

#define MVME6100_ETH0_PHY_ADDR                   1
#define MVME6100_ETH1_PHY_ADDR                   2
#define MVME6100_ETH2_PHY_ADDR                   3

#define MVME6100_ETH_TX_QUEUE_SIZE               800
#define MVME6100_ETH_RX_QUEUE_SIZE               400

#endif				/* __MVME6100_H */
