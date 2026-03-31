/*
 *	MVME7100 Watchdog Support
 *
 *	Author: Ajit Prem <ajit.prem@emerson.com>
 *   
 *	Copyright 2008 Emerson Network Power - Embedded Computing, Inc.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <platforms/86xx/mvme7100.h>

#define WDT_INTERVAL	(HZ/2 + 1)

static unsigned long mvme7100_wdt_is_open;
static char mvme7100_expect_close;
static struct timer_list mvme7100_wd_timer;
static unsigned long next_heartbeat;

#define WD_LOAD_REG_OFFSET		0
#define WD_CONTROL_REG_OFFSET		4
#define WD_RESOLUTION_REG_OFFSET	5
#define WD_COUNT_REG_OFFSET		6

#define WD_LOAD_MAGIC_VALUE			0xDB
#define WD_ENABLE				0x80
#define WD_VMEBUS_SYS_RESET			0x40

#define WD_COUNTER_RES_4_MICROSEC		0x1
#define WD_COUNTER_RES_8_MICROSEC		0x2
#define WD_COUNTER_RES_16_MICROSEC		0x3
#define WD_COUNTER_RES_32_MICROSEC		0x4
#define WD_COUNTER_RES_64_MICROSEC		0x5
#define WD_COUNTER_RES_128_MICROSEC		0x6
#define WD_COUNTER_RES_256_MICROSEC		0x7
#define WD_COUNTER_RES_512_MICROSEC		0x8
#define WD_COUNTER_RES_1024_MICROSEC		0x9
#define WD_COUNTER_RES_2048_MICROSEC		0xa
#define WD_COUNTER_RES_4096_MICROSEC		0xb
#define WD_COUNTER_RES_8192_MICROSEC		0xc
#define WD_COUNTER_RES_16384_MICROSEC		0xd
#define WD_COUNTER_RES_32768_MICROSEC		0xe
#define WD_COUNTER_RES_65536_MICROSEC		0xf


/* Module parameters */

#define WD_TIM0 60			/* Default heartbeat = 60 seconds */

static int heartbeat = WD_TIM0;
static int wd_heartbeat = (WD_TIM0 * 1000) >> 6;

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat in seconds. (0<heartbeat<3600, default=" __MODULE_STRING(WD_TIMO) ")");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");


static void __iomem *mvme7100_wdt_regs;

/* Start the watchdog driver */

static int mvme7100_wdt_start(void)
{
	u8 ctrl_reg;
	u16 count_reg;

	/* Disable watchdog */
	ctrl_reg = readb(mvme7100_wdt_regs + WD_CONTROL_REG_OFFSET);
	if (ctrl_reg & WD_ENABLE)
		writeb(ctrl_reg & ~WD_ENABLE, mvme7100_wdt_regs + WD_CONTROL_REG_OFFSET);

	/* Load counter */
	count_reg = wd_heartbeat;
	writew(swab16(count_reg), mvme7100_wdt_regs + WD_COUNT_REG_OFFSET);

	/* Start the kernel timer */
	next_heartbeat = jiffies + (heartbeat * HZ);
	mod_timer(&mvme7100_wd_timer, jiffies + WDT_INTERVAL); 

	/* Enable watchdog */
	ctrl_reg = readb(mvme7100_wdt_regs + WD_CONTROL_REG_OFFSET);
	writeb(ctrl_reg | WD_ENABLE, mvme7100_wdt_regs + WD_CONTROL_REG_OFFSET);

	return 0;
}

/*
 *	mvme7100_wdt_stop:
 *
 *	Stop the watchdog driver.
 */

static int mvme7100_wdt_stop(void)
{
	u8 ctrl_reg;

	printk(KERN_INFO "mvme7100_wdt: Stopping watchdog timer\n");
	del_timer(&mvme7100_wd_timer);

	/* Turn the watchdog off */
	ctrl_reg = readb(mvme7100_wdt_regs + WD_CONTROL_REG_OFFSET);
	ctrl_reg &= ~WD_ENABLE;
	writeb(ctrl_reg, mvme7100_wdt_regs + WD_CONTROL_REG_OFFSET);

	return 0;
}

/*
 *	mvme7100_wdt_ping:
 *
 *	Reload counter with the watchdog heartbeat. 
 */

static void mvme7100_wdt_ping(unsigned long data)
{
	if(time_before(jiffies, next_heartbeat)) {
		/* Write the magic value  to the LOAD Register */
		writeb(WD_LOAD_MAGIC_VALUE, mvme7100_wdt_regs + WD_LOAD_REG_OFFSET);
		mod_timer(&mvme7100_wd_timer, jiffies + WDT_INTERVAL);
	} else {
		printk(KERN_WARNING "Heartbeat lost! Will not ping the watchdog.\n");
	}
	return;
}


static int mvme7100_wdt_keepalive(void)
{
	/* userland ping */
	next_heartbeat = jiffies + (heartbeat * HZ);
	return 0;
}


/*
 *	mvme7100_wdt_set_heartbeat:
 *	@t:		the new heartbeat value that needs to be set.
 *
 *	Set a new heartbeat value for the watchdog device. If the heartbeat 
 *      value is incorrect we keep the old value and return -EINVAL. 
 *      If successfull we return 0.
 */
static int mvme7100_wdt_set_heartbeat(int t)
{
	if ((t < 1) || (t > 3600))
		return -EINVAL;

	heartbeat = t;
	/* heartbeat is in seconds. The watchdog resolution has been 
         * set to countdown every 65536 microseconds */
	wd_heartbeat = ((t * 1000) >> 6) & 0xffff;
	return 0;
}

/**
 *	mvme7100_wdt_write:
 *	@file: file handle to the watchdog
 *	@buf: buffer to write (unused as data does not matter here
 *	@count: count of bytes
 *	@ppos: pointer to the position to write. No seeks allowed
 *
 *	A write to a watchdog device is defined as a keepalive signal. Any
 *	write of data will do, as we we don't define content meaning.
 */

static ssize_t mvme7100_wdt_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if(count) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			mvme7100_expect_close = 0;

			for (i = 0; i != count; i++) {
				char c;
				if (get_user(c, buf + i))
					return -EFAULT;
				if (c == 'V')
					mvme7100_expect_close = 42;
			}
		}
		mvme7100_wdt_keepalive();
	}
	return count;
}

/**
 *	mvme7100_wdt_ioctl:
 *	@inode: inode of the device
 *	@file: file handle to the device
 *	@cmd: watchdog command
 *	@arg: argument pointer
 *
 *	The watchdog API defines a common set of functions for all watchdogs
 *	according to their available features. We only actually usefully support
 *	querying capabilities and current status.
 */

static int mvme7100_wdt_ioctl(struct inode *inode, struct file *file, 
		              unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_heartbeat;
	int options;

	static struct watchdog_info ident = {
		.options =		WDIOF_SETTIMEOUT |
					WDIOF_MAGICCLOSE |
					WDIOF_KEEPALIVEPING,
		.firmware_version =	1,
		.identity =		"MVME7100_WDT",
	};

	switch(cmd)
	{
		case WDIOC_GETSUPPORT:
			return copy_to_user(argp, &ident, sizeof(ident))?-EFAULT:0;

		case WDIOC_GETSTATUS:
		case WDIOC_GETBOOTSTATUS:
			return put_user(0, p);
		case WDIOC_KEEPALIVE:
			mvme7100_wdt_keepalive();
			return 0;
		case WDIOC_SETTIMEOUT:
			if (get_user(new_heartbeat, p))
				return -EFAULT;

			if (mvme7100_wdt_set_heartbeat(new_heartbeat))
				return -EINVAL;

			mvme7100_wdt_keepalive();
			/* Fall */
		case WDIOC_GETTIMEOUT:
			return put_user(heartbeat, p);
		case WDIOC_SETOPTIONS:
			if (get_user(options, p))
				return -EFAULT;
			if (options & WDIOS_DISABLECARD) {
				mvme7100_wdt_stop();
				return 0;
			}

			if (options & WDIOS_ENABLECARD) {
				mvme7100_wdt_start();
				return 0;
			}
			break;
		default:
			return -ENOTTY;

	}
	return 0;
}

/*
 *	mvme7100_wdt_open:
 *	@inode: inode of device
 *	@file: file handle to device
 *
 */

static int mvme7100_wdt_open(struct inode *inode, struct file *file)
{
	if(test_and_set_bit(0, &mvme7100_wdt_is_open))
		return -EBUSY;

	/* Activate */
	printk(KERN_INFO "mvme7100_wdt: Starting watchdog timer\n");
	mvme7100_wdt_start();

	return nonseekable_open(inode, file);
}

/*
 *	mvme7100_wdt_release:
 *	@inode: inode to board
 *	@file: file handle to board
 *
 *	The watchdog has a configurable API. There is a religious dispute
 *	between people who want their watchdog to be able to shut down and
 *	those who want to be sure if the watchdog manager dies the machine
 *	reboots. In the former case we disable the watchdog timer, in the 
 *      latter case you have to open it again very soon.
 */

static int mvme7100_wdt_release(struct inode *inode, struct file *file)
{
	if (mvme7100_expect_close == 42) {
		mvme7100_wdt_stop();
	} else {
		printk(KERN_CRIT "mvme7100_wdt:  watchdog device closed unexpectedly - watchdog will not stop!\n");
		mvme7100_wdt_keepalive();
	}
	mvme7100_expect_close = 0;
	clear_bit(0, &mvme7100_wdt_is_open);
	return 0;
}

/**
 *	notify_sys:
 *	@this: our notifier block
 *	@code: the event being reported
 *	@unused: unused
 *
 *	Our notifier is called on system shutdowns. We want to turn the 
 *      watchdog off at reboot.
 */

static int mvme7100_wdt_notify_sys(struct notifier_block *this, unsigned long code,
	void *unused)
{
	if(code==SYS_DOWN || code==SYS_HALT) {
		/* Turn the card off */
		mvme7100_wdt_stop();
	}
	return NOTIFY_DONE;
}

/*
 *	Kernel Interfaces
 */


static const struct file_operations mvme7100_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= mvme7100_wdt_write,
	.ioctl		= mvme7100_wdt_ioctl,
	.open		= mvme7100_wdt_open,
	.release	= mvme7100_wdt_release,
};

static struct miscdevice mvme7100_wdt_miscdev = {
	.minor	= WATCHDOG_MINOR,
	.name	= "watchdog",
	.fops	= &mvme7100_wdt_fops,
};

/*
 *	The watchdog timer needs to be stopped on reboots/shutdowns 
 */

static struct notifier_block mvme7100_wdt_notifier = {
	.notifier_call = mvme7100_wdt_notify_sys,
};

/*
 *	cleanup_module:
 *
 *	Unload the watchdog. You cannot do this with any file handles open.
 *	If your watchdog is set to continue ticking on close and you unload
 *	it, well it keeps ticking.  You just have to load a new
 *	module in 60 seconds or reboot.
 */

static void __devexit mvme7100_wdt_exit(void)
{
	if (!nowayout) 
		mvme7100_wdt_stop();
	iounmap(mvme7100_wdt_regs);	
	misc_deregister(&mvme7100_wdt_miscdev);
	unregister_reboot_notifier(&mvme7100_wdt_notifier);
}

/*
 * 	mvme7100_wdt_init:
 *
 *	Set up the MVME7100 watchdog. 
 *	The open() function will actually kick the watchdog.
 */

static int __devinit mvme7100_wdt_init(void)
{
	int ret = 0;

	/* Check that the heartbeat value is within it's range ; if not reset to the default */
	if (mvme7100_wdt_set_heartbeat(heartbeat)) {
		mvme7100_wdt_set_heartbeat(WD_TIM0);
		printk(KERN_INFO "wdt: heartbeat value must be 0<heartbeat<3600 secs, using %d secs\n",
			WD_TIM0);
	}

	mvme7100_wdt_regs = ioremap(MVME7100_WATCHDOG_TIMER_LOAD_REG, 8);
	if (mvme7100_wdt_regs == NULL) {
		printk(KERN_ERR "mvme7100_wdt: ioremap() failed\n");
		ret = -ENOMEM;
		goto out;
	}

	writeb(WD_COUNTER_RES_65536_MICROSEC, mvme7100_wdt_regs + WD_RESOLUTION_REG_OFFSET);

	init_timer(&mvme7100_wd_timer);
	mvme7100_wd_timer.function = mvme7100_wdt_ping;
	mvme7100_wd_timer.data = 0;

	ret = register_reboot_notifier(&mvme7100_wdt_notifier);
	if(ret) {
		printk(KERN_ERR "mvme7100_wdt: cannot register reboot notifier (err=%d)\n", ret);
		goto out;
	}

	ret = misc_register(&mvme7100_wdt_miscdev);
	if (ret) {
		printk(KERN_ERR "mvme7100_wdt: cannot register miscdev on minor=%d (err=%d)\n",
			WATCHDOG_MINOR, ret);
		goto outmisc;
	}

	printk(KERN_INFO "mvme7100 watchdog driver ver 1.0: (base=0x%p, heartbeat=%d sec, nowayout=%d\n",
		mvme7100_wdt_regs, heartbeat, nowayout);
out:
	return ret;

outmisc:
	unregister_reboot_notifier(&mvme7100_wdt_notifier);
	goto out;
}

module_init(mvme7100_wdt_init);
module_exit(mvme7100_wdt_exit);

MODULE_AUTHOR("Ajit Prem");
MODULE_DESCRIPTION("Watchdog Driver for the MVME7100 watchdog");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_LICENSE("GPL");
