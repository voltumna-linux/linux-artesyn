/*
 * 
 * Mapping for Motorola MVME3100 flash
 *
 * Ajit Prem (Ajit.Prem@motorola.com)
 *
 * Copyright 2005-2006 Motorola Inc.
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
#include <platforms/85xx/mvme3100.h>

static struct mtd_info *flash;

static struct map_info mvme3100_128M_map = {
	.name =		"mvme3100-flash",
	.size =		MVME3100_FLASH_SIZE_128M,
	.bankwidth =	4,
};

static struct map_info mvme3100_64M_map = {
	.name =		"mvme3100-flash",
	.size =		MVME3100_FLASH_SIZE_64M,
	.bankwidth =	4,
};

static struct map_info mvme3100_map;
 
static struct mtd_partition mvme3100_128M_partitions[] = {
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

static struct mtd_partition mvme3100_64M_partitions[] = {
	{
		.name =   "Linux Kernel + INITRD",
		.offset = 0x00000000,
		.size =   0x01000000,
	},
	{
		.name =   "Linux JFFS2 Filesystem",
		.offset = 0x01000000,
		.size =   0x02e00000,
	},
	{
		.name	= "Bootloader Block B",
		.offset	= 0x03e00000,
		.size	= 0x00100000,
	},
	{
		.name	= "Bootloader Block A",
		.offset	= 0x03f00000,
		.size	= 0x00100000,
	}
};


int __init mvme3100_map_init(void)
{
	unsigned long mvme3100_flash_base;

	mvme3100_flash_base = MVME3100_FLASH_BASE_128M;

	mvme3100_map = mvme3100_128M_map;

	mvme3100_map.phys = mvme3100_flash_base;
	mvme3100_map.virt = (unsigned long)ioremap(mvme3100_flash_base, 
						mvme3100_map.size);

	if (!mvme3100_map.virt) {
		printk("Failed to ioremap flash\n");
		return -EIO;
	}

	simple_map_init(&mvme3100_map);

	flash = do_map_probe("cfi_probe", &mvme3100_map);
	if (flash) {
		flash->owner = THIS_MODULE;
		add_mtd_partitions(flash, mvme3100_128M_partitions,
					ARRAY_SIZE(mvme3100_128M_partitions));
	} else {
		iounmap((void *)mvme3100_map.virt);
		mvme3100_map.virt = 0;

		mvme3100_flash_base = MVME3100_FLASH_BASE_64M;
		mvme3100_map = mvme3100_64M_map;

		mvme3100_map.phys = mvme3100_flash_base;
		mvme3100_map.virt = (unsigned long)ioremap(
						mvme3100_flash_base, 
						mvme3100_map.size);

		if (!mvme3100_map.virt) {
			printk("Failed to ioremap flash\n");
			return -EIO;
		}

		simple_map_init(&mvme3100_map);

		flash = do_map_probe("cfi_probe", &mvme3100_map);
		if (flash) {
			flash->owner = THIS_MODULE;
			add_mtd_partitions(flash, mvme3100_64M_partitions,
					ARRAY_SIZE(mvme3100_64M_partitions));
		} else {
			printk("map probe failed for flash\n");
			iounmap((void *)mvme3100_map.virt);
			return -ENXIO;
		}
	}

	return 0;
}

static void __exit mvme3100_map_exit(void)
{
	if (flash) {
		del_mtd_partitions(flash);
		map_destroy(flash);
	}

	if (mvme3100_map.virt) {
		iounmap((void *)mvme3100_map.virt);
		mvme3100_map.virt = 0;
	}
}

module_init(mvme3100_map_init);
module_exit(mvme3100_map_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ajit Prem <ajit.prem@motorola.com>");
MODULE_DESCRIPTION("MTD map and partitions for Motorola MVME3100");
