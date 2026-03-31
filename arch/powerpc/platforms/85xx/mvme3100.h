/*
 * arch/powerpc/platforms/85xx/mvme3100.h
 *
 * MVME3100 board definitions
 *
 * Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2004-2007 Motorola Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __MACH_MVME3100_H__
#define __MACH_MVME3100_H__

#include <linux/init.h>
#include <linux/seq_file.h>
#include <asm/ppcboot.h>
#include <linux/initrd.h>

#ifndef CONFIG_UBOOT

#define BINFO_INTFREQ 		833333333
#define BINFO_BUSFREQ		333333333
#define BINFO_IMMR_BASE		0xe1000000
#define BINFO_MEMSIZE		0x20000000
#define BINFO_BAUDRATE		9600

#endif

#define BOARD_CCSRBAR		((uint)0xe1000000)

/* PCI interrupt controller */
#define PIRQA		MPC85xx_IRQ_EXT1
#define PIRQB		MPC85xx_IRQ_EXT2
#define PIRQC		MPC85xx_IRQ_EXT3
#define PIRQD		MPC85xx_IRQ_EXT4

#define MPC85XX_PCI1_LOWER_IO	0x00000000
#define MPC85XX_PCI1_UPPER_IO	0x00ffffff

#define MPC85XX_PCI1_LOWER_MEM	0x80000000
#define MPC85XX_PCI1_UPPER_MEM	0xdfffffff

#define MPC85XX_PCI1_IO_BASE	0xe0000000
#define MPC85XX_PCI1_MEM_OFFSET	0x00000000

#define MPC85XX_PCI1_IO_SIZE	0x01000000

/* FCC1 Clock Source Configuration.  These can be
 * redefined in the board specific file.
 *    Can only choose from CLK9-12 */
#define F1_RXCLK       12
#define F1_TXCLK       11
                                                                                
/* FCC2 Clock Source Configuration.  These can be
 * redefined in the board specific file.
 *    Can only choose from CLK13-16 */
#define F2_RXCLK       13
#define F2_TXCLK       14
                                                                                
/* FCC3 Clock Source Configuration.  These can be
 * redefined in the board specific file.
 *    Can only choose from CLK13-16 */
#define F3_RXCLK       15
#define F3_TXCLK       16
                                                                                

/* Flash */
#define MVME3100_FLASH_BASE_128M	0xf8000000
#define MVME3100_FLASH_SIZE_128M	0x08000000

#define MVME3100_FLASH_BASE_64M		0xfc000000
#define MVME3100_FLASH_SIZE_64M		0x04000000

/* System Control and Status Registers */
#define MVME3100_SYSTEM_STATUS_REG	0xE2000000
#define MVME3100_SYSTEM_CONTROL_REG	0xE2000001
#define MVME3100_STATUS_INDICATOR_REG	0xE2000002
#define MVME3100_FLASH_CTRL_STAT_REG	0xE2000003
#define MVME3100_PCI_BUS_A_STATUS_REG	0xE2000004
#define MVME3100_PCI_BUS_B_STATUS_REG	0xE2000005
#define MVME3100_PCI_BUS_C_STATUS_REG	0xE2000006
#define MVME3100_INTERRUPT_DETECT_REG	0xE2000007
#define MVME3100_PRESENCE_DETECT_REG	0xE2000008
#define MVME3100_PLD_REVISION_REG	0xE2000009
#define MVME3100_PLD_DATE_CODE_REG	0xE200000C

/* System Status Register */
#define MVME3100_SAFE_START		0x10
#define MVME3100_ABORT_STATUS		0x08
#define MVME3100_BOARD_TYPE_MASK	0x03
#define MVME3100_BOARD_TYPE_PRPMC	0x01
#define MVME3100_BOARD_TYPE_VME		0x00

/* System Control Register */
#define MVME3100_BOARD_RESET		0xA0
#define MVME3100_FEC_PHY_MASK		0x04
#define MVME3100_EEPROM_WP		0x02
#define MVME3100_TSTAT_MASK		0x01

/* Status Indicator Register */
#define MVME3100_BRD_FAIL_LED		0x01
#define MVME3100_USR1_LED		0x02
#define MVME3100_USR2_LED		0x04
#define MVME3100_USR3_LED		0x08

/* Flash Control/Status Register */
#define MVME3100_FLASH_MAP_SELECT	0x10
#define MVME3100_FLASH_WP_SW		0x08
#define MVME3100_FLASH_WP_HW		0x04
#define MVME3100_FLASH_BLK_SEL		0x02
#define MVME3100_FLASH_RDY		0x01

/* PCI Bus A Status Register */
#define MVME3100_PCI_BUS_A_64B		0x08
#define MVME3100_PCI_BUS_A_PCIX		0x04
#define MVME3100_PCI_BUS_A_SPD_MASK	0x03
#define MVME3100_PCI_BUS_A_SPD_133	0x03
#define MVME3100_PCI_BUS_A_SPD_100	0x02
#define MVME3100_PCI_BUS_A_SPD_66	0x01
#define MVME3100_PCI_BUS_A_SPD_33	0x00

/* PCI Bus B Status Register */
#define MVME3100_PCI_BUS_B_3_3V_VIO	0x80
#define MVME3100_PCI_BUS_B_5_0V_VIO	0x40
#define MVME3100_PCI_BUS_B_ERDY2	0x20
#define MVME3100_PCI_BUS_B_ERDY1	0x10
#define MVME3100_PCI_BUS_B_64B		0x08
#define MVME3100_PCI_BUS_B_PCIX		0x04
#define MVME3100_PCI_BUS_B_SPD_MASK	0x03
#define MVME3100_PCI_BUS_B_SPD_133	0x03
#define MVME3100_PCI_BUS_B_SPD_100	0x02
#define MVME3100_PCI_BUS_B_SPD_66	0x01
#define MVME3100_PCI_BUS_B_SPD_33	0x00

/* PCI Bus C Status Register */
#define MVME3100_PCI_BUS_C_64B		0x08
#define MVME3100_PCI_BUS_C_PCIX		0x04
#define MVME3100_PCI_BUS_C_SPD_MASK	0x03
#define MVME3100_PCI_BUS_C_SPD_133	0x03
#define MVME3100_PCI_BUS_C_SPD_100	0x02
#define MVME3100_PCI_BUS_C_SPD_66	0x01
#define MVME3100_PCI_BUS_C_SPD_33	0x00

/* Interrupt Detect Register */
#define MVME3100_FEC_PHY_INTERRUPT	0x04
#define MVME3100_TSEC2_PHY_INTERRUPT	0x02
#define MVME3100_TSEC1_PHY_INTERRUPT	0x01

/* Presence Detect Register */
#define MVME3100_PMCSPAN_PRESENT	0x04
#define MVME3100_PMC2_PRESENT		0x02
#define MVME3100_PMC1_PRESENT		0x01

#define MVME3100_BASE_BAUD		1843200
#define QUART_BASE_BAUD			(MVME3100_BASE_BAUD / 16)
#define MVME3100_UART_SIZE		0x8

#define MVME3100_SERIAL_1	0xE2011000U
#define MVME3100_SERIAL_2	0xE2012000U
#define MVME3100_SERIAL_3	0xE2013000U
#define MVME3100_SERIAL_4	0xE2014000U

#define MVME3100_IDE_INT0	MPC85xx_IRQ_EXT2
#define MVME3100_SERIAL_IRQ	MPC85xx_IRQ_EXT3

extern int add_bridge(struct device_node *dev);

/* Some crap left out of headers where it belongs */

#define MPC85xx_OPENPIC_IRQ_OFFSET 0

#define MPC85xx_IRQ_L2CACHE     ( 0 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_ECM         ( 1 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_DDR         ( 2 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_LBIU        ( 3 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_DMA0        ( 4 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_DMA1        ( 5 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_DMA2        ( 6 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_DMA3        ( 7 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_PCI1        ( 8 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_PCI2        ( 9 + MPC85xx_OPENPIC_IRQ_OFFSET)

/* The 12 external interrupt lines */
#define MPC85xx_IRQ_EXT0        (48 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT1        (49 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT2        (50 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT3        (51 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT4        (52 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT5        (53 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT6        (54 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT7        (55 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT8        (56 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT9        (57 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT10       (58 + MPC85xx_OPENPIC_IRQ_OFFSET)
#define MPC85xx_IRQ_EXT11       (59 + MPC85xx_OPENPIC_IRQ_OFFSET)

#endif				/* __MACH_MVME3100_H__ */
