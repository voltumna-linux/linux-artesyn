/*
 * arch/ppc/boot/simple/misc-mvme6100.S
 * 
 * Host bridge init code for the MVME6100 board
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <asm/ppc_asm.h>
#include <asm/processor.h>
#include <asm/cache.h>
#include <asm/mv64x60_defs.h>

#include "../../platforms/mvme6100.h"

#define MV643XX_ETH_RECEIVE_QUEUE_COMMAND_REG(port)     (0x2680 + (port<<10))

#define _64K            0x00010000

	.globl	mvme6100_init
mvme6100_init:
	li	r20,0
	li	r23,16


	/* stack in cache - cache touch and invalidate first 2MByte */
	/* R24 = 2MByte */
	lis	r24, 0x0020
	/* R25 = 0 (the counter */
	lis	r25,0x0
	/* R26 = 0x80000000 - BAT offset mapping physical memory at offset 0x0 - cacheable */
	lis	r26,0x8000
	
1:	dcbf	r26,r25
	icbi	r26,r25
	addi	r25,r25,32
	cmplw   r25,r24
	ble	1b

        /* Stop GigE Rx DMA engines */
        addis	r25,0,MVME6100_BRIDGE_REG_BASE@h
        ori	r25,r25,MV643XX_ETH_RECEIVE_QUEUE_COMMAND_REG(0)             
	addis	r26,0,0
	ori	r26,r26,0xff00
        stwbrx  r26,0,(r25)
	sync

        addis	r25,0,MVME6100_BRIDGE_REG_BASE@h
        ori	r25,r25,MV643XX_ETH_RECEIVE_QUEUE_COMMAND_REG(1)             
	addis	r26,0,0
	ori	r26,r26,0xff00
        stwbrx  r26,0,(r25)
	sync

        addis	r25,0,MVME6100_BRIDGE_REG_BASE@h
        ori	r25,r25,MV643XX_ETH_RECEIVE_QUEUE_COMMAND_REG(2)             
	addis	r26,0,0
	ori	r26,r26,0xff00
        stwbrx  r26,0,(r25)
	sync

	blr


