/*
 * 
 * Mapping for Motorola MVME7100 flash
 *
 * Ajit Prem (Ajit.Prem@emerson.com)
 *
 * Copyright 2008 Emerson Network Power - Embedded Computing, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <platforms/86xx/mvme7100.h>

static struct mtd_info *flash;

static struct map_info mvme7100_map = {
	.name =		"mvme7100-flash",
	.size =		MVME7100_FLASH_SIZE,
	.bankwidth =	4,
};

static struct mtd_partition mvme7100_partitions[] = {
	{
		.name =   "Linux Kernel + INITRD",
		.offset = 0x00000000,
		.size =   0x01000000,
	},
	{
		.name =   "Linux JFFS2 Filesystem",
		.offset = 0x01000000,
		.size =   0x06e00000,
	},
	{
		.name	= "Bootloader Block B",
		.offset	= 0x07e00000,
		.size	= 0x00100000,
	},
	{
		.name	= "Bootloader Block A",
		.offset	= 0x07f00000,
		.size	= 0x00100000,
	}
};

int __init mvme7100_map_init(void)
{
	void __iomem *nor_ctrl_status_reg;
	u8 value;

	mvme7100_map.phys = MVME7100_FLASH_BASE;
	mvme7100_map.virt = ioremap(mvme7100_map.phys, mvme7100_map.size);

	if (!mvme7100_map.virt) {
		printk(KERN_ERR "mvme7100_flash: Failed to ioremap flash\n");
		return -EIO;
	}

	simple_map_init(&mvme7100_map);

	nor_ctrl_status_reg = ioremap(MVME7100_NOR_FLASH_CTRL_STAT_REG, 1);
	if (!nor_ctrl_status_reg) {
		printk(KERN_ERR "mvme7100_flash: ioremap() failed\n");
		iounmap((void *)mvme7100_map.virt);
		return -EIO;
	}

	value = readb(nor_ctrl_status_reg);
	iounmap(nor_ctrl_status_reg);

	if (value & MVME7100_NOR_FLASH_WP_HW) {
		printk(KERN_INFO "mvme7100_flash: Flash HW write protected - mapping read-only\n");
		flash = do_map_probe("map_rom", &mvme7100_map);
	} else if (value & MVME7100_NOR_FLASH_WP_SW) {
		printk(KERN_INFO "mvme7100_flash: Flash SW write protected - mapping read-only\n");
		flash = do_map_probe("map_rom", &mvme7100_map);
	} else {
		flash = do_map_probe("cfi_probe", &mvme7100_map);
		if (!flash) {
			flash = do_map_probe("jedec", &mvme7100_map);
		}
		if (!flash) {
			flash = do_map_probe("map_rom", &mvme7100_map);
		}
	}
	
	if (flash) {
		flash->owner = THIS_MODULE;
		add_mtd_partitions(flash, mvme7100_partitions,
					ARRAY_SIZE(mvme7100_partitions));
	} else {
		printk(KERN_ERR "mvme7100_flash: map probe failed for flash\n");
		iounmap((void *)mvme7100_map.virt);
		return -ENXIO;
	}

	return 0;
}

static void __exit mvme7100_map_exit(void)
{
	if (flash) {
		del_mtd_partitions(flash);
		map_destroy(flash);
	}

	if (mvme7100_map.virt) {
		iounmap((void *)mvme7100_map.virt);
		mvme7100_map.virt = NULL;
	}
}

module_init(mvme7100_map_init);
module_exit(mvme7100_map_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ajit Prem <ajit.prem@emerson.com>");
MODULE_DESCRIPTION("MTD map and partitions for the Emerson MVME7100 board");
