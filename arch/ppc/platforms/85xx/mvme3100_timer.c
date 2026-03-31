/*
 * arch/powerpc/platforms/85xx/mvme3100_timer.c
 *
 * MVME3100 Tick Timers Support
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2005-2007 Motorola Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>

#include <asm/io.h>
#include <asm/mpc85xx.h>
#include "mvme3100.h"
#include <asm/mvme3100_timer.h>

#define MVME3100_TICK_TIMER_BASE                0xE2020000
#define MVME3100_TICK_TIMER_BLOCK_SIZE          0x4C

#define MVME3100_TICK_TIMER_PRESCALER_REG_OFFSET        0x0
#define MVME3100_TICK_TIMER_CONTROL_REG_OFFSET          0x10
#define MVME3100_TICK_TIMER_COMPARE_REG_OFFSET          0x14
#define MVME3100_TICK_TIMER_COUNTER_REG_OFFSET          0x18

#define MVME3100_TICK_TIMER_ENC                 0x00000001
#define MVME3100_TICK_TIMER_COC                 0x00000002
#define MVME3100_TICK_TIMER_COVF                0x00000004
#define MVME3100_TICK_TIMER_OVF                 0x000000F0
#define MVME3100_TICK_TIMER_ENINT               0x00000100
#define MVME3100_TICK_TIMER_CINT                0x00000200
#define MVME3100_TICK_TIMER_INTS                0x00000400

#define	MIN_MICROSECONDS	10000
#define MAX_TICKS		0xFFFFFFFF

struct mvme3100_timer_t {
	mvme3100_timer_f func;
	void *func_data;
	int started;
	int periodic;
	ulong period;
	ulong ticks;
};

#define ALL_MSG "mvme3100_timer: "

#ifdef MVME3100_TIMER_DEBUG
int mvme3100_timer_debug = MVME3100_TIMER_DEBUG;
MODULE_PARM(mvme3100_timer_debug, "i");
#define DEBUG(n, fmt, args...) if (mvme3100_timer_debug>(n)) printk(KERN_INFO ALL_MSG fmt, ## args)
static const char *version = "mvme3100_timer.c 1.0.1 (Ajit Prem)";
#else
#define DEBUG(n, fmt, args...)
#endif

static struct mvme3100_timer_t mvme3100_timer[4];
static void __iomem *mvme3100_timer_base;
static u32 clk_out = 1;		/* Ticks per microsecond */
static int mvme3100_timer_irq = MPC85xx_IRQ_EXT1;
static spinlock_t mvme3100_timer_lock = SPIN_LOCK_UNLOCKED;

static irqreturn_t mvme3100_timer_int_handler(int irq, void *data)
{
	struct mvme3100_timer_t *info = data;
	mvme3100_timer_f func;
	void *func_data;
	int timer_number;
	u32 ctrl_reg;

	for (timer_number = 0; timer_number <= 3; timer_number++) {
		if (swab32(readl(mvme3100_timer_base +
			 (timer_number + 1) *
			 MVME3100_TICK_TIMER_CONTROL_REG_OFFSET)) &
		   	 MVME3100_TICK_TIMER_INTS) {
			info = &mvme3100_timer[timer_number];
			func_data = info->func_data;
			func = info->func;

			if (!info->periodic) {
				mvme3100_timer_stop(timer_number);
			} else {
				ctrl_reg = swab32(readl(mvme3100_timer_base +
					((timer_number + 1) *
					 MVME3100_TICK_TIMER_CONTROL_REG_OFFSET)));
				ctrl_reg &= ~MVME3100_TICK_TIMER_ENC;
				ctrl_reg =
				    MVME3100_TICK_TIMER_COVF |
				    MVME3100_TICK_TIMER_CINT;
				writel(swab32(ctrl_reg),
				       mvme3100_timer_base +
				       ((timer_number + 1) *
					MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
				writel(0,
				       mvme3100_timer_base +
				       ((timer_number + 1) *
					MVME3100_TICK_TIMER_CONTROL_REG_OFFSET)
				       +
				       (MVME3100_TICK_TIMER_COUNTER_REG_OFFSET -
					MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
				DEBUG(1,
				      ("Timer %d Counter 0x%x Compare 0x%x\n",
				       timer_number,
				       swab32(readl
					      (mvme3100_timer_base +
					       ((timer_number + 1) *
						MVME3100_TICK_TIMER_CONTROL_REG_OFFSET)
					       +
					       (MVME3100_TICK_TIMER_COUNTER_REG_OFFSET
						-
						MVME3100_TICK_TIMER_CONTROL_REG_OFFSET))),
				       swab32(readl
					      (mvme3100_timer_base +
					       ((timer_number + 1) *
						MVME3100_TICK_TIMER_CONTROL_REG_OFFSET)
					       +
					       (MVME3100_TICK_TIMER_COMPARE_REG_OFFSET
						-
						MVME3100_TICK_TIMER_CONTROL_REG_OFFSET)))));
				writel(swab32
				       (MVME3100_TICK_TIMER_ENC |
					MVME3100_TICK_TIMER_COC |
					MVME3100_TICK_TIMER_ENINT),
				       mvme3100_timer_base + 
					((timer_number + 1) *
					MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
			}
			if (func) 
				func(func_data);
		}
	}

	return IRQ_HANDLED;
}

/* 
 * Stop the timer
 */
EXPORT_SYMBOL(mvme3100_timer_stop);
int mvme3100_timer_stop(int timer_number)
{
	u32 reset_value;
	unsigned long flags;

	if ((timer_number < 0) || (timer_number > 3))
		return -EINVAL;

	spin_lock_irqsave(&mvme3100_timer_lock, flags);
	reset_value = swab32(readl(mvme3100_timer_base +
				   ((timer_number + 1) *
				    MVME3100_TICK_TIMER_CONTROL_REG_OFFSET)));
	reset_value &= ~MVME3100_TICK_TIMER_ENC;
	reset_value &= ~MVME3100_TICK_TIMER_ENINT;
	reset_value = MVME3100_TICK_TIMER_COVF | MVME3100_TICK_TIMER_CINT;
	writel(swab32(reset_value),
	       mvme3100_timer_base +
	       ((timer_number + 1) * MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
	writel(0, mvme3100_timer_base +
	       ((timer_number + 1) * MVME3100_TICK_TIMER_CONTROL_REG_OFFSET) +
	       (MVME3100_TICK_TIMER_COUNTER_REG_OFFSET -
		MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
	writel(0xffffffff, mvme3100_timer_base +
	       ((timer_number + 1) * MVME3100_TICK_TIMER_CONTROL_REG_OFFSET) +
	       (MVME3100_TICK_TIMER_COMPARE_REG_OFFSET -
		MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
	mvme3100_timer[timer_number].func = NULL;
	mvme3100_timer[timer_number].func_data = NULL;
	mvme3100_timer[timer_number].started = 0;
	mvme3100_timer[timer_number].periodic = 0;
	mvme3100_timer[timer_number].period = 0;
	mvme3100_timer[timer_number].ticks = 0xFFFFFFFF;
	spin_unlock_irqrestore(&mvme3100_timer_lock, flags);

	return 0;
}

/* 
 * Start the timer
 */
EXPORT_SYMBOL(mvme3100_timer_start);
int mvme3100_timer_start(int timer_number, unsigned long microseconds,
			 mvme3100_timer_f func, void *func_data, int periodic)
{
	int res = 0;
	unsigned long flags;
	unsigned long ticks;

	printk(KERN_INFO
	       "mvme3100_timer: start_timer: num %d microseconds 0x%lx periodic %d\n",
	       timer_number, microseconds, periodic);
	if ((timer_number < 0) || (timer_number > 3))
		return -EINVAL;
	if ((microseconds < MIN_MICROSECONDS) ||
	    (microseconds > MAX_TICKS / clk_out))
		return -EINVAL;
	if (func == NULL)
		return -EINVAL;
	if (mvme3100_timer[timer_number].started == 1)
		return -EBUSY;
	if ((res = mvme3100_timer_stop(timer_number)) < 0)
		return res;
	ticks = microseconds * clk_out;
	spin_lock_irqsave(&mvme3100_timer_lock, flags);
	mvme3100_timer[timer_number].started = 1;
	mvme3100_timer[timer_number].func = func;
	mvme3100_timer[timer_number].func_data = func_data;
	mvme3100_timer[timer_number].periodic = periodic;
	mvme3100_timer[timer_number].period = microseconds;
	mvme3100_timer[timer_number].ticks = ticks;
	writel(swab32(ticks), mvme3100_timer_base +
	       ((timer_number + 1) * MVME3100_TICK_TIMER_CONTROL_REG_OFFSET) +
	       (MVME3100_TICK_TIMER_COMPARE_REG_OFFSET -
		MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
	writel(0, mvme3100_timer_base +
	       ((timer_number + 1) * MVME3100_TICK_TIMER_CONTROL_REG_OFFSET) +
	       (MVME3100_TICK_TIMER_COUNTER_REG_OFFSET -
		MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
	writel(swab32(MVME3100_TICK_TIMER_ENC | MVME3100_TICK_TIMER_COC |
		      MVME3100_TICK_TIMER_ENINT),
	       mvme3100_timer_base +
	       ((timer_number + 1) * MVME3100_TICK_TIMER_CONTROL_REG_OFFSET));
	spin_unlock_irqrestore(&mvme3100_timer_lock, flags);

	return 0;
}

/* 
 * Get the timer resolution, in microseconds/tick
 */
EXPORT_SYMBOL(mvme3100_timer_get_resolution);
unsigned long mvme3100_timer_get_resolution(void)
{
	u32 prescaler_adjust;

	/* 
	 * Prescaler_adjust = 256 - CLKIN/CLKOUT
	 * CLKIN = 25 MHz
	 */

	prescaler_adjust = readl(mvme3100_timer_base +
				 MVME3100_TICK_TIMER_PRESCALER_REG_OFFSET);
	return (25 / (256 - prescaler_adjust));
}

/* 
 * Set the timer resolution, in ticks/microsecond
 */
EXPORT_SYMBOL(mvme3100_timer_set_resolution);
int mvme3100_timer_set_resolution(unsigned long ticks)
{
	u32 prescaler_adjust;
	int i;
	unsigned long flags;

	/* 
	 * Prescaler_adjust = 256 - CLKIN/CLKOUT
	 * CLKIN = 25 MHz
	 */

	if ((ticks < MIN_TICKS_PER_MICROSECOND) ||
	    (ticks > MAX_TICKS_PER_MICROSECOND))
		return -EINVAL;

	clk_out = ticks;
	prescaler_adjust = 256 - (25 / clk_out);

	spin_lock_irqsave(&mvme3100_timer_lock, flags);
	for (i = 0; i <= 3; i++) {
		if (mvme3100_timer[i].started == 1) {
			spin_unlock_irqrestore(&mvme3100_timer_lock, flags);
			return -EBUSY;
		}
	}

	writel(prescaler_adjust, mvme3100_timer_base +
	       MVME3100_TICK_TIMER_PRESCALER_REG_OFFSET);
	spin_unlock_irqrestore(&mvme3100_timer_lock, flags);

	return 0;
}

/* 
 * Get the timer value, in ticks
 */
EXPORT_SYMBOL(mvme3100_timer_get_ticks);
unsigned long mvme3100_timer_get_ticks(int timer_number)
{
	if ((timer_number < 0) || (timer_number > 3))
		return -EINVAL;

	return swab32(readl(mvme3100_timer_base +
			    ((timer_number +
			      1) * MVME3100_TICK_TIMER_CONTROL_REG_OFFSET) +
			    (MVME3100_TICK_TIMER_COUNTER_REG_OFFSET -
			     MVME3100_TICK_TIMER_CONTROL_REG_OFFSET)));
}

#ifdef CONFIG_PROC_FS

static int
mvme3100_timer_proc_read(char *buf, char **start, off_t off, int len, int *eof,
			 void *data)
{
	int i;

	len = 0;
	len +=
	    sprintf(buf + len,
		    "\nTimer     Active  Periodic  Period(ticks)   Period(microseconds)\n");
	len +=
	    sprintf(buf + len,
		    "_____     ______  ________  _____________   ____________________\n");
	for (i = 0; i <= 3; i++) {
		len +=
		    sprintf(buf + len,
			    "#%d        %c       %c         0x%08lx      %ld\n",
			    i + 1,
			    ((mvme3100_timer[i].started == 1) ? 'y' : 'n'),
			    ((mvme3100_timer[i].periodic == 1) ? 'y' : 'n'),
			    ((mvme3100_timer[i].periodic ==
			      1) ? mvme3100_timer[i].ticks : 0),
			    ((mvme3100_timer[i].periodic ==
			      1) ? mvme3100_timer[i].period : 0));
	}
	return len;
}

#endif				/* CONFIG_PROC_FS */

static __init int mvme3100_timer_init(void)
{
	int res = 0;
	int i;

	printk(KERN_INFO "mvme3100-timer: MVME3100 Tick Timers\n");

	mvme3100_timer_base = ioremap(MVME3100_TICK_TIMER_BASE,
				     MVME3100_TICK_TIMER_BLOCK_SIZE);
	if (mvme3100_timer_base == NULL)
		return -ENOMEM;

	for (i = 0; i <= 3; i++) {
		mvme3100_timer[i].func = NULL;
		mvme3100_timer[i].func_data = NULL;
		mvme3100_timer[i].started = 0;
		mvme3100_timer[i].periodic = 0;
		mvme3100_timer[i].period = 0;
		mvme3100_timer[i].ticks = 0;
	}
	res = request_irq(mvme3100_timer_irq, mvme3100_timer_int_handler,
			  IRQF_SHARED,
			  "MVME3100 Tick Timers", &mvme3100_timer[0]);
	if (res < 0) {
		printk(KERN_ERR "MVME3100 Timer: Can't allocate IRQ %d.\n",
		       mvme3100_timer_irq);
		return res;
	}
#ifdef CONFIG_PROC_FS
	create_proc_read_entry("mvme3100_timers", 0, NULL,
			       mvme3100_timer_proc_read, NULL);
#endif
	return 0;
}

static __exit void mvme3100_timer_exit(void)
{
	int timer_number;

	for (timer_number = 0; timer_number <= 3; timer_number++) {
		mvme3100_timer_stop(timer_number);
	}
	free_irq(mvme3100_timer_irq, &mvme3100_timer[0]);
	iounmap(mvme3100_timer_base);
}

MODULE_LICENSE("GPL");

MODULE_AUTHOR("Ajit Prem <Ajit.Prem@motorola.com>");
MODULE_DESCRIPTION("MVME3100 Tick Timer driver");

module_init(mvme3100_timer_init);
module_exit(mvme3100_timer_exit);
