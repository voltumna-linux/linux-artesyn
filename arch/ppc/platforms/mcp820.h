/*
 * mcp820.h
 * 
 * Definitions for the Motorola MCP820/CPCI6020 board
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2002-2007 Motorola Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
 /*
    * From Processor to PCI:
    *   PCI Mem Space: 0x80000000 - 0xefffffff -> 0x80000000 - 0xefffffff (1.75 GB)
    *   PCI I/O Space: 0xfe000000 - 0xfe7fffff -> 0x00000000 - 0x00800000 (8 MB)
    *
    * From PCI to Processor:
    *   System Memory: 0x00000000 -> 0x00000000
  */


#ifndef __MCP820_H
#define __MCP820_H

#define MCP820_PCI_CONFIG_1_ADDR		0xfe000cf8U
#define MCP820_PCI_CONFIG_1_DATA		0xfe000cfcU
#define MCP820_PCI_CONFIG_2_ADDR		0xfe400cf8U
#define MCP820_PCI_CONFIG_2_DATA		0xfe400cfcU

#define MCP820_PROC_PCI_IO_START		0xfe000000U
#define MCP820_PROC_PCI_IO_END			0xfe7fffffU
#define MCP820_PROC_PCI_IO_1_START		0xfe000000U
#define MCP820_PROC_PCI_IO_1_END		0xfe3fffffU
#define MCP820_PROC_PCI_IO_2_START		0xfe400000U
#define MCP820_PROC_PCI_IO_2_END		0xfe7fffffU

#define MCP820_PCI_IO_1_START			0x00000000U
#define MCP820_PCI_IO_1_END			0x003fffffU
#define MCP820_PCI_IO_2_START			0x00400000U
#define MCP820_PCI_IO_2_END			0x007fffffU

#define MCP820_PROC_PCI_MEM_START		0x80000000U
#define MCP820_PROC_PCI_MEM_END			0xefffffffU
#define MCP820_PROC_PCI_MEM_1_START		0x80000000U
#define MCP820_PROC_PCI_MEM_1_END		0xdfffffffU
#define MCP820_PROC_PCI_MEM_2_START		0xe0000000U
#define MCP820_PROC_PCI_MEM_2_END		0xefffffffU

#define MCP820_PCI_MEM_1_START			0x80000000U
#define MCP820_PCI_MEM_1_END			0xdfffffffU
#define MCP820_PCI_MEM_2_START			0xe0000000U
#define MCP820_PCI_MEM_2_END			0xefffffffU

#define MCP820_PCI_DRAM_OFFSET			0x00000000U
#define MCP820_PCI_PHY_MEM_OFFSET		0x00000000U

#define MCP820_ISA_IO_BASE			MCP820_PROC_PCI_IO_START
#define MCP820_ISA_MEM_BASE			0x00000000U

#define MCP820_HARRIER_1_XCSR_BASE		0xfeff0000U
#define MCP820_HARRIER_1_MPIC_BASE		0xff000000U

#define MCP820_HARRIER_2_XCSR_BASE		0xfeff1000U
#define MCP820_HARRIER_2_MPIC_BASE		0xff040000U

#define MCP820_HARRIER_1_SERIAL_1		0xfeff00c0U
#define MCP820_HARRIER_1_SERIAL_2		0xfeff00c8U

#define MCP820_HARRIER_2_SERIAL_1		0xfeff10c0U
#define MCP820_HARRIER_2_SERIAL_2		0xfeff10c8U

#define MCP820_MPIC1_VECTOR_OFFSET		16
#define MCP820_MPIC2_VECTOR_OFFSET		33

#define MCP820_BASE_BAUD			1843200

#define MCP820_NVRAM_BASE_ADDRESS 		0xff110000
#define MCP820_NVRAM_SIZE			0x8000

#ifdef CONFIG_SERIAL_MANY_PORTS
#define RS_TABLE_SIZE  64
#else
#define RS_TABLE_SIZE  4
#endif

/* Rate for the 1.8432 Mhz clock for the onboard serial chip */
#define BASE_BAUD (MCP820_BASE_BAUD / 16)

#define MCP820_HARRIER_1_SERIAL_1_IRQ   32
#define MCP820_HARRIER_1_SERIAL_2_IRQ   32
                                                                                
#ifdef CONFIG_SERIAL_DETECT_IRQ
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST|ASYNC_AUTO_IRQ)
#else
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST)
#endif

/* UARTS are at IRQs 32  */
#define STD_SERIAL_PORT_DFNS \
        { 0, BASE_BAUD, MCP820_HARRIER_1_SERIAL_1, 32, STD_COM_FLAGS, /* ttyS0 */\
        iomem_base: (unsigned char *)MCP820_HARRIER_1_SERIAL_1, \
        iomem_reg_shift: 0,                 \
        io_type: SERIAL_IO_MEM },               \
        { 0, BASE_BAUD, MCP820_HARRIER_1_SERIAL_2, 32, STD_COM_FLAGS, /* ttyS1 */\
        iomem_base: (unsigned char *)MCP820_HARRIER_1_SERIAL_2,     \
        iomem_reg_shift: 0,                 \
        io_type: SERIAL_IO_MEM },

#define SERIAL_PORT_DFNS \
        STD_SERIAL_PORT_DFNS

#endif				/* __MCP820_H */
