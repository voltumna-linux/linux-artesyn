/*
 * Copied from <file:arch/powerpc/kernel/misc_32.S>
 *
 * This file contains miscellaneous low-level functions.
 *    Copyright (C) 1995-1996 Gary Thomas (gdt@linuxppc.org)
 *
 * Largely rewritten by Cort Dougan (cort@cs.nmt.edu)
 * and Paul Mackerras.
 *
 * kexec bits:
 * Copyright (C) 2002-2003 Eric Biederman  <ebiederm@xmission.com>
 * GameCube/ppc32 port Copyright (C) 2004 Albert Herranz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#include "ppc_asm.h"

#define SPRN_PVR        0x11F   /* Processor Version Register */
#define SPRN_HID0       0x3F0 
#define HID0_ICE        (1<<15)
#define HID0_ICFI       (1<<11)
#define HID0_DCE        (1<<14)
#define HID0_DCI        (1<<10) 
#if defined(CONFIG_8xx) || defined(CONFIG_403GCX)
#define L1_CACHE_SHIFT          4
#define MAX_COPY_PREFETCH       1
#elif defined(CONFIG_PPC32)
#define L1_CACHE_SHIFT          5
#define MAX_COPY_PREFETCH       4
#else /* CONFIG_PPC64 */
#define L1_CACHE_SHIFT          7
#endif

#define L1_CACHE_BYTES          (1 << L1_CACHE_SHIFT)

	.text

/* udelay (on non-601 processors) needs to know the period of the
 * timebase in nanoseconds.  This used to be hardcoded to be 60ns
 * (period of 66MHz/4).  Now a variable is used that is initialized to
 * 60 for backward compatibility, but it can be overridden as necessary
 * with code something like this:
 *    extern unsigned long timebase_period_ns;
 *    timebase_period_ns = 1000000000 / bd->bi_tbfreq;
 */
	.data
	.globl timebase_period_ns
timebase_period_ns:
	.long	60

	.text
/*
 * Delay for a number of microseconds
 */
	.globl	udelay
udelay:
	mfspr	r4,SPRN_PVR
	srwi	r4,r4,16
	cmpwi	0,r4,1		/* 601 ? */
	bne	.udelay_not_601
00:	li	r0,86	/* Instructions / microsecond? */
	mtctr	r0
10:	addi	r0,r0,0 /* NOP */
	bdnz	10b
	subic.	r3,r3,1
	bne	00b
	blr

.udelay_not_601:
	mulli	r4,r3,1000	/* nanoseconds */
	/*  Change r4 to be the number of ticks using:
	 *	(nanoseconds + (timebase_period_ns - 1 )) / timebase_period_ns
	 *  timebase_period_ns defaults to 60 (16.6MHz) */
	mflr	r5
	bl	0f
0:	mflr	r6
	mtlr	r5
	lis	r5,0b@ha
	addi	r5,r5,0b@l
	subf	r5,r5,r6	/* In case we're relocated */
	addis	r5,r5,timebase_period_ns@ha
	lwz	r5,timebase_period_ns@l(r5)
	add	r4,r4,r5
	addi	r4,r4,-1
	divw	r4,r4,r5	/* BUS ticks */
1:	mftbu	r5
	mftb	r6
	mftbu	r7
	cmpw	0,r5,r7
	bne	1b		/* Get [synced] base time */
	addc	r9,r6,r4	/* Compute end time */
	addze	r8,r5
2:	mftbu	r5
	cmpw	0,r5,r8
	blt	2b
	bgt	3f
	mftb	r6
	cmpw	0,r6,r9
	blt	2b
3:	blr


/*
 * Flush and enable instruction cache
 * First, flush the data cache in case it was enabled and may be
 * holding instructions for copy back.
 */
        .globl flush_instruction_cache
flush_instruction_cache:
        mflr    r6
        bl      flush_data_cache

#ifdef CONFIG_8xx
        lis     r3, IDC_INVALL@h
        mtspr   SPRN_IC_CST, r3
        lis     r3, IDC_ENABLE@h
        mtspr   SPRN_IC_CST, r3
        lis     r3, IDC_DISABLE@h
        mtspr   SPRN_DC_CST, r3
#elif CONFIG_4xx
        lis     r3,start@h              # r9 = &_start
        lis     r4,_etext@ha
        addi    r4,r4,_etext@l          # r8 = &_etext
1:      dcbf    r0,r3                   # Flush the data cache
        icbi    r0,r3                   # Invalidate the instruction cache
        addi    r3,r3,0x10              # Increment by one cache line
        cmplw   cr0,r3,r4               # Are we at the end yet?
        blt     1b                      # No, keep flushing and invalidating
#elif CONFIG_FSL_BOOKE
        mfspr   r3,SPRN_L1CSR1
        ori     r3,r3,L1CSR1_ICFI|L1CSR1_ICLFR
        mtspr   SPRN_L1CSR1,r3
        isync
#else
        /* Enable, invalidate and then disable the L1 icache/dcache. */
        li      r3,0
        ori     r3,r3,(HID0_ICE|HID0_DCE|HID0_ICFI|HID0_DCI)
        mfspr   r4,SPRN_HID0
        or      r5,r4,r3
        isync
        mtspr   SPRN_HID0,r5
        sync
        isync
        ori     r5,r4,HID0_ICE  /* Enable cache */
        mtspr   SPRN_HID0,r5
        sync
        isync
#endif
        mtlr    r6
        blr

#define NUM_CACHE_LINES 128*8
#define cache_flush_buffer 0x1000

/*
 * Flush data cache
 * Do this by just reading lots of stuff into the cache.
 */
        .globl flush_data_cache
flush_data_cache:
        lis     r3,cache_flush_buffer@h
        ori     r3,r3,cache_flush_buffer@l
        li      r4,NUM_CACHE_LINES
        mtctr   r4
00:     lwz     r4,0(r3)
        addi    r3,r3,L1_CACHE_BYTES    /* Next line, please */
        bdnz    00b
10:     blr

