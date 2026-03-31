/*
 * nvram.c: NVRAM driver for the M48T37V used on Motorola's 
 * MCP805/MCP820/MCP905/MVME5500/MVME6100 boards
 *
 * Ajit Prem, (Ajit.Prem@motorola.com).
 *
 * Copyright 2003-2006 Motorola, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#ifdef CONFIG_MCP805
#include <platforms/mcp805.h>
#endif
#ifdef CONFIG_MCP820
#include <platforms/mcp820.h>
#endif
#ifdef CONFIG_MCP905
#include <platforms/mcp905.h>
#endif
#ifdef CONFIG_MVME5500
#include <platforms/mvme5500.h>
#endif
#ifdef CONFIG_MVME6100
#include <platforms/mvme6100.h>
#endif


static ulong nvram_base;
static uint nvram_size = 0x8000 - 16;  
static uint nvram_low_protect = 0x1400;
static uint nvram_high_protect = 0x7600;


module_param(nvram_base, ulong, 0);
module_param(nvram_size, uint, 0);
module_param(nvram_low_protect, uint, 0);
module_param(nvram_high_protect, uint, 0);

#define ALL_MSG "nvram: "

#undef NVRAM_DEBUG 

#ifdef NVRAM_DEBUG
int nvram_debug = NVRAM_DEBUG;
MODULE_PARM(nvram_debug, "i");
#define DEBUG(n, fmt, args...) if (nvram_debug>(n)) printk(KERN_INFO ALL_MSG fmt, ## args)
static const char *version = "nvram.c 1.0.2";
#else
#define DEBUG(n, fmt, args...)
#endif

struct nvram_dev_t {
	u_int size;
	u_int base;
	u8 *virt;
};


static struct nvram_dev_t nvram;
static unsigned long  open_for_write;

static int 
nvram_open(struct inode *inode, struct file *file)
{
	DEBUG(1, "open()\n");

	if ((file->f_flags & O_ACCMODE) != O_RDONLY) {
		if (test_and_set_bit(0, &open_for_write))
			return -EBUSY;
	}

	return 0;
}				/* nvram_open */


static int 
nvram_release(struct inode *inode, struct file *file)
{
	DEBUG(1, "release()\n");

	if ((file->f_flags & O_ACCMODE) != O_RDONLY)
		clear_bit(0, &open_for_write);

	return 0;
}				/* nvram_release */

static ssize_t 
nvram_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p;
	unsigned long i, odd;
	u_int tmpw;
	u_short tmps;
	u_char tmpc;

	DEBUG(1, "read(pos 0x%lx, count 0x%lx)\n", (u_long) p, (u_long) count);

        if (!access_ok(VERIFY_WRITE, buf, count))
                return -EFAULT;

	p = *ppos;
	if (p < 0 || p > nvram.size || count < 0)
		return -EINVAL;

	if (count > nvram.size - p)
		count = nvram.size - p;

	odd = count & 3;
	count &= ~3;

        for (i = 0; i < count; i += 4, p += 4, buf +=4) {
		tmpw = readl(nvram.virt + p);
		if (__put_user(swab32(tmpw), (u_int *)buf))
			return -EFAULT;
	}

	if (odd & 2) {
		tmps = readw(nvram.virt + p);
		if (__put_user(swab16(tmps), (u_short *) buf))
			return -EFAULT;
		p += 2;
		buf += 2;
	}

	if (odd & 1) {
		tmpc = readb(nvram.virt + p);
		if (__put_user(tmpc, (u_char *) buf))
			return -EFAULT;
	}


	*ppos += count + odd;
	return count + odd;
}				/* nvram_read */


static ssize_t 
nvram_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
        unsigned long p;
	unsigned long i, odd;
	u_int tmpw;
	u_short tmps;
	u_char tmpc;

        DEBUG(1, "write(0x%lx, 0x%lx)\n", (u_long) p, (u_long) count);

        if (!access_ok(VERIFY_READ, buf, count))
                return -EFAULT;

	p = *ppos;
	if (p < 0 || p >= nvram.size || count < 0)
		return -EINVAL;

	if ((p < nvram_low_protect) || (p >= nvram_high_protect)) {
		printk(KERN_INFO 
			"nvram: Attempt to write to offset 0x%lu which is in a write protected area\n",
	p);
		return -EPERM;
	}

	if (p + count > nvram.size) {
		count = nvram.size - p;
	}

        if (p + count > nvram_high_protect) {
		printk(KERN_INFO
		"nvram: Attempt to write beyond nvram_high_protect. Offset 0x%lu, Count 0x%x\n",
			p, count);
                count = nvram_high_protect - p;
	}

	odd = count & 3;
	count &= ~3;

        for (i = 0; i < count; i += 4, p += 4, buf +=4) {
		if (__get_user(tmpw, (u_int *)buf))
			return -EFAULT;
		writel(swab32(tmpw), nvram.virt + p);
	}

	if (odd & 2) {
		if (__get_user(tmps, (u_short *) buf))
			return -EFAULT;
		writew(swab16(tmps), nvram.virt + p);
		p += 2;
		buf += 2;
	}

	if (odd & 1) {
		if (__get_user(tmpc, (u_char *) buf))
			return -EFAULT;
		writeb(tmpc, nvram.virt + p);
	}

        *ppos += count + odd;

        return count + odd;

}                               /* nvram_write */

static long long
nvram_llseek(struct file *file, long long offset, int origin)
{
	DEBUG(1, "llseek(offset 0x%lx origin 0x%x)\n", (u_long) offset, origin);

	switch (origin) {
	case 0:
                break;
        case 1:
                offset += file->f_pos;
                break;
        case 2:
                offset += nvram.size;
                break;
        default:
                return -EINVAL;
        }
        if (offset < 0 || offset > nvram.size)
                return -EINVAL;

        file->f_pos = offset;
        return file->f_pos;
}                               /* nvram_llseek */


static struct file_operations nvram_fops = {
	.owner		= THIS_MODULE,
	.open		= nvram_open,
	.release	= nvram_release,
	.read		= nvram_read,
	.write		= nvram_write,
	.llseek		= nvram_llseek,
};

static struct miscdevice nvram_miscdev = {
        .minor          = NVRAM_MINOR,
        .name           = "nvram",
        .fops           = &nvram_fops,
};



static int __init 
nvram_m48txxx_init(void)
{
	int ret;

	DEBUG(0, "%s\n", version);

#if defined(CONFIG_MCP805)
	nvram.base = MCP805_NVRAM_BASE_ADDRESS;
	nvram.size = MCP805_NVRAM_SIZE - 16; /* exclude RTC and watchdog */
#elif defined(CONFIG_MCP820)
	nvram.base = MCP820_NVRAM_BASE_ADDRESS;
	nvram.size = MCP820_NVRAM_SIZE - 16; /* exclude RTC and watchdog */
#elif defined(CONFIG_MCP905)
	nvram_base = MCP905_NVRAM_BASE_ADDRESS;
	nvram.size = MCP905_NVRAM_SIZE - 16; /* exclude RTC and watchdog */
#elif defined(CONFIG_MVME5500)
	nvram_base = MVME5500_NVRAM_BASE_ADDRESS;
	nvram.size = MVME5500_NVRAM_SIZE - 16; /* exclude RTC and watchdog */
#elif defined(CONFIG_MVME6100)
	nvram_base = MVME6100_NVRAM_BASE_ADDRESS;
	nvram.size = MVME6100_NVRAM_SIZE - 16; /* exclude RTC and watchdog */
#else
	return -ENODEV;
#endif
	if (nvram.base == 0) {
		if (nvram_base == 0) {
			return -ENODEV;
		} else {
			nvram.base = nvram_base;
			nvram.size = nvram_size;
		}
	}

	if ((nvram_low_protect < 0) || (nvram_low_protect > nvram_high_protect) ||
		(nvram_high_protect > nvram_size)) {
		printk(KERN_INFO "nvram: Bad module parameters\n");
		return -ENODEV;
	}
	DEBUG(0, "base: 0x%x size: 0x%x low_protect_offset: 0x%x high_protect_offset 0x%x\n",
			nvram.base, nvram.size, nvram_low_protect, nvram_high_protect);
 
	nvram.virt = ioremap(nvram.base, nvram.size);

	/* Set up character device for user mode clients */
        ret = misc_register(&nvram_miscdev);
        if (ret) {
		printk (KERN_ERR "Cannot register miscdev on minor=%d (err=%d)\n", NVRAM_MINOR, ret);
		iounmap(nvram.virt);
		return ret;
	}

	return 0;
}		


static void __exit 
nvram_m48txxx_exit(void)
{
	DEBUG(0, "exiting\n");
	iounmap(nvram.virt);
	misc_deregister(&nvram_miscdev);
	return;
}	

module_init(nvram_m48txxx_init);
module_exit(nvram_m48txxx_exit);

MODULE_AUTHOR("Ajit Prem <Ajit.Prem@motorola.com>");
MODULE_DESCRIPTION("M48TXXX NVRAM Driver");
MODULE_LICENSE("GPL");
