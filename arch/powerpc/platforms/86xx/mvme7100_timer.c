/*
 * arch/ppc/platforms/86xx/mvme7100_timer.c
 *
 * MVME7100 Tick Timers Support
 *
 * Author: Ajit Prem <ajit.prem@emerson.com>
 *
 * Copyright 2007 Motorola Inc.
 * Copyright 2008 Emerson Network Power Embedded Computing Inc.
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
#include <asm/mpc86xx.h>
#include "mvme7100.h"
#include <asm/mvme7100_timer.h>

#define MVME7100_TICK_TIMER_BASE                0xF2020000
#define MVME7100_TICK_TIMER_BLOCK_SIZE          0x4C

#define MVME7100_TICK_TIMER_PRESCALER_REG_OFFSET        0x0
#define MVME7100_TICK_TIMER_CONTROL_REG_OFFSET          0x10
#define MVME7100_TICK_TIMER_COMPARE_REG_OFFSET          0x14
#define MVME7100_TICK_TIMER_COUNTER_REG_OFFSET          0x18

#define MVME7100_TICK_TIMER_ENC                 0x00000001
#define MVME7100_TICK_TIMER_COC                 0x00000002
#define MVME7100_TICK_TIMER_COVF                0x00000004
#define MVME7100_TICK_TIMER_OVF                 0x000000F0
#define MVME7100_TICK_TIMER_ENINT               0x00000100
#define MVME7100_TICK_TIMER_CINT                0x00000200
#define MVME7100_TICK_TIMER_INTS                0x00000400

#define	MIN_MICROSECONDS	10000
#define MAX_TICKS		0xFFFFFFFF

struct mvme7100_timer_t {
	mvme7100_timer_f func;
	void *func_data;
	int started;
	int periodic;
	ulong period;
	ulong ticks;
};

#define ALL_MSG "mvme7100_timer: "

#ifdef MVME7100_TIMER_DEBUG
int mvme7100_timer_debug = MVME7100_TIMER_DEBUG;
MODULE_PARM(mvme7100_timer_debug, "i");
#define DEBUG(n, fmt, args...) if (mvme7100_timer_debug>(n)) printk(KERN_INFO ALL_MSG fmt, ## args)
static const char *version = "mvme7100_timer.c 1.0.0 (Ajit Prem)";
#else
#define DEBUG(n, fmt, args...)
#endif

extern int mvme7100_timer_irq;

static struct mvme7100_timer_t mvme7100_timer[4];
static void __iomem *mvme7100_timer_base;
static u32 clk_out = 1;		/* Ticks per microsecond */
static spinlock_t mvme7100_timer_lock = SPIN_LOCK_UNLOCKED;

static irqreturn_t mvme7100_timer_int_handler(int irq, void *data)
{
	struct mvme7100_timer_t *info = data;
	mvme7100_timer_f func;
	void *func_data;
	int timer_number;
	u32 ctrl_reg;

	if (data != (void *)&mvme7100_timer[0])
		return IRQ_NONE;

	for (timer_number = 0; timer_number <= 3; timer_number++) {
		if (swab32(readl(mvme7100_timer_base +
			 (timer_number + 1) *
			 MVME7100_TICK_TIMER_CONTROL_REG_OFFSET)) &
		   	 MVME7100_TICK_TIMER_INTS) {
			info = &mvme7100_timer[timer_number];
			func_data = info->func_data;
			func = info->func;

			if (!info->periodic) {
				mvme7100_timer_stop(timer_number);
			} else {
				ctrl_reg = swab32(readl(mvme7100_timer_base +
					((timer_number + 1) *
					 MVME7100_TICK_TIMER_CONTROL_REG_OFFSET)));
				ctrl_reg &= ~MVME7100_TICK_TIMER_ENC;
				ctrl_reg =
				    MVME7100_TICK_TIMER_COVF |
				    MVME7100_TICK_TIMER_CINT;
				writel(swab32(ctrl_reg),
				       mvme7100_timer_base +
				       ((timer_number + 1) *
					MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
				writel(0,
				       mvme7100_timer_base +
				       ((timer_number + 1) *
					MVME7100_TICK_TIMER_CONTROL_REG_OFFSET)
				       +
				       (MVME7100_TICK_TIMER_COUNTER_REG_OFFSET -
					MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
				DEBUG(1,
				      ("Timer %d Counter 0x%x Compare 0x%x\n",
				       timer_number,
				       swab32(readl
					      (mvme7100_timer_base +
					       ((timer_number + 1) *
						MVME7100_TICK_TIMER_CONTROL_REG_OFFSET)
					       +
					       (MVME7100_TICK_TIMER_COUNTER_REG_OFFSET
						-
						MVME7100_TICK_TIMER_CONTROL_REG_OFFSET))),
				       swab32(readl
					      (mvme7100_timer_base +
					       ((timer_number + 1) *
						MVME7100_TICK_TIMER_CONTROL_REG_OFFSET)
					       +
					       (MVME7100_TICK_TIMER_COMPARE_REG_OFFSET
						-
						MVME7100_TICK_TIMER_CONTROL_REG_OFFSET)))));
				writel(swab32
				       (MVME7100_TICK_TIMER_ENC |
					MVME7100_TICK_TIMER_COC |
					MVME7100_TICK_TIMER_ENINT),
				       mvme7100_timer_base + 
					((timer_number + 1) *
					MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
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
EXPORT_SYMBOL(mvme7100_timer_stop);
int mvme7100_timer_stop(int timer_number)
{
	u32 reset_value;
	unsigned long flags;

	if ((timer_number < 0) || (timer_number > 3))
		return -EINVAL;

	spin_lock_irqsave(&mvme7100_timer_lock, flags);
	reset_value = swab32(readl(mvme7100_timer_base +
				   ((timer_number + 1) *
				    MVME7100_TICK_TIMER_CONTROL_REG_OFFSET)));
	reset_value &= ~MVME7100_TICK_TIMER_ENC;
	reset_value &= ~MVME7100_TICK_TIMER_ENINT;
	reset_value = MVME7100_TICK_TIMER_COVF | MVME7100_TICK_TIMER_CINT;
	writel(swab32(reset_value),
	       mvme7100_timer_base +
	       ((timer_number + 1) * MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
	writel(0, mvme7100_timer_base +
	       ((timer_number + 1) * MVME7100_TICK_TIMER_CONTROL_REG_OFFSET) +
	       (MVME7100_TICK_TIMER_COUNTER_REG_OFFSET -
		MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
	writel(0xffffffff, mvme7100_timer_base +
	       ((timer_number + 1) * MVME7100_TICK_TIMER_CONTROL_REG_OFFSET) +
	       (MVME7100_TICK_TIMER_COMPARE_REG_OFFSET -
		MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
	mvme7100_timer[timer_number].func = NULL;
	mvme7100_timer[timer_number].func_data = NULL;
	mvme7100_timer[timer_number].started = 0;
	mvme7100_timer[timer_number].periodic = 0;
	mvme7100_timer[timer_number].period = 0;
	mvme7100_timer[timer_number].ticks = 0xFFFFFFFF;
	spin_unlock_irqrestore(&mvme7100_timer_lock, flags);

	return 0;
}

/* 
 * Start the timer
 */
EXPORT_SYMBOL(mvme7100_timer_start);
int mvme7100_timer_start(int timer_number, unsigned long microseconds,
			 mvme7100_timer_f func, void *func_data, int periodic)
{
	int res = 0;
	unsigned long flags;
	unsigned long ticks;

	printk(KERN_INFO
	       "mvme7100_timer: start_timer: num %d microseconds 0x%lx periodic %d\n",
	       timer_number, microseconds, periodic);
	if ((timer_number < 0) || (timer_number > 3))
		return -EINVAL;
	if ((microseconds < MIN_MICROSECONDS) ||
	    (microseconds > MAX_TICKS / clk_out))
		return -EINVAL;
	if (func == NULL)
		return -EINVAL;
	if (mvme7100_timer[timer_number].started == 1)
		return -EBUSY;
	if ((res = mvme7100_timer_stop(timer_number)) < 0)
		return res;
	ticks = microseconds * clk_out;
	spin_lock_irqsave(&mvme7100_timer_lock, flags);
	mvme7100_timer[timer_number].started = 1;
	mvme7100_timer[timer_number].func = func;
	mvme7100_timer[timer_number].func_data = func_data;
	mvme7100_timer[timer_number].periodic = periodic;
	mvme7100_timer[timer_number].period = microseconds;
	mvme7100_timer[timer_number].ticks = ticks;
	writel(swab32(ticks), mvme7100_timer_base +
	       ((timer_number + 1) * MVME7100_TICK_TIMER_CONTROL_REG_OFFSET) +
	       (MVME7100_TICK_TIMER_COMPARE_REG_OFFSET -
		MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
	writel(0, mvme7100_timer_base +
	       ((timer_number + 1) * MVME7100_TICK_TIMER_CONTROL_REG_OFFSET) +
	       (MVME7100_TICK_TIMER_COUNTER_REG_OFFSET -
		MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
	writel(swab32(MVME7100_TICK_TIMER_ENC | MVME7100_TICK_TIMER_COC |
		      MVME7100_TICK_TIMER_ENINT),
	       mvme7100_timer_base +
	       ((timer_number + 1) * MVME7100_TICK_TIMER_CONTROL_REG_OFFSET));
	spin_unlock_irqrestore(&mvme7100_timer_lock, flags);

	return 0;
}

/* 
 * Get the timer resolution, in microseconds/tick
 */
EXPORT_SYMBOL(mvme7100_timer_get_resolution);
unsigned long mvme7100_timer_get_resolution(void)
{
	u32 prescaler_adjust;

	/* 
	 * Prescaler_adjust = 256 - CLKIN/CLKOUT
	 * CLKIN = 25 MHz
	 */

	prescaler_adjust = readl(mvme7100_timer_base +
				 MVME7100_TICK_TIMER_PRESCALER_REG_OFFSET);
	return (25 / (256 - prescaler_adjust));
}

/* 
 * Set the timer resolution, in ticks/microsecond
 */
EXPORT_SYMBOL(mvme7100_timer_set_resolution);
int mvme7100_timer_set_resolution(unsigned long ticks)
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

	spin_lock_irqsave(&mvme7100_timer_lock, flags);
	for (i = 0; i <= 3; i++) {
		if (mvme7100_timer[i].started == 1) {
			spin_unlock_irqrestore(&mvme7100_timer_lock, flags);
			return -EBUSY;
		}
	}

	writel(prescaler_adjust, mvme7100_timer_base +
	       MVME7100_TICK_TIMER_PRESCALER_REG_OFFSET);
	spin_unlock_irqrestore(&mvme7100_timer_lock, flags);

	return 0;
}

/* 
 * Get the timer value, in ticks
 */
EXPORT_SYMBOL(mvme7100_timer_get_ticks);
unsigned long mvme7100_timer_get_ticks(int timer_number)
{
	if ((timer_number < 0) || (timer_number > 3))
		return -EINVAL;

	return swab32(readl(mvme7100_timer_base +
			    ((timer_number +
			      1) * MVME7100_TICK_TIMER_CONTROL_REG_OFFSET) +
			    (MVME7100_TICK_TIMER_COUNTER_REG_OFFSET -
			     MVME7100_TICK_TIMER_CONTROL_REG_OFFSET)));
}

#ifdef CONFIG_PROC_FS

static int
mvme7100_timer_proc_read(char *buf, char **start, off_t off, int len, int *eof,
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
			    ((mvme7100_timer[i].started == 1) ? 'y' : 'n'),
			    ((mvme7100_timer[i].periodic == 1) ? 'y' : 'n'),
			    ((mvme7100_timer[i].periodic ==
			      1) ? mvme7100_timer[i].ticks : 0),
			    ((mvme7100_timer[i].periodic ==
			      1) ? mvme7100_timer[i].period : 0));
	}
	return len;
}

#endif				/* CONFIG_PROC_FS */

static __init int mvme7100_timer_init(void)
{
	int res = 0;
	int i;

	printk(KERN_INFO "mvme7100-timer: MVME7100 Tick Timers\n");

	mvme7100_timer_base = ioremap(MVME7100_TICK_TIMER_BASE,
				     MVME7100_TICK_TIMER_BLOCK_SIZE);
	if (mvme7100_timer_base == NULL)
		return -ENOMEM;

	for (i = 0; i <= 3; i++) {
		mvme7100_timer[i].func = NULL;
		mvme7100_timer[i].func_data = NULL;
		mvme7100_timer[i].started = 0;
		mvme7100_timer[i].periodic = 0;
		mvme7100_timer[i].period = 0;
		mvme7100_timer[i].ticks = 0;
	}
	res = request_irq(mvme7100_timer_irq, mvme7100_timer_int_handler,
			  IRQF_SHARED,
			  "MVME7100 Tick Timers", &mvme7100_timer[0]);
	if (res < 0) {
		printk(KERN_ERR "MVME7100 Timer: Can't allocate IRQ %d.\n",
		       mvme7100_timer_irq);
		return res;
	}
#ifdef CONFIG_PROC_FS
	create_proc_read_entry("mvme7100_timers", 0, NULL,
			       mvme7100_timer_proc_read, NULL);
#endif
	return 0;
}

static __exit void mvme7100_timer_exit(void)
{
	int timer_number;

	for (timer_number = 0; timer_number <= 3; timer_number++) {
		mvme7100_timer_stop(timer_number);
	}
	free_irq(mvme7100_timer_irq, &mvme7100_timer[0]);
	iounmap(mvme7100_timer_base);
}

MODULE_LICENSE("GPL");

MODULE_AUTHOR("Ajit Prem <ajit.prem@emerson.com>");
MODULE_DESCRIPTION("MVME7100 Tick Timer driver");

module_init(mvme7100_timer_init);
module_exit(mvme7100_timer_exit);
