/*
 * 
 * Mapping for Motorola MVME6100 flash
 *
 * Ajit Prem (Ajit.Prem@motorola.com)

 * Copyright 2006 Motorola Inc.
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
#include <platforms/mvme6100.h>

/* Bank A is flash 0 and Bank B is flash 1 */

static struct mtd_info *flash[2];

static struct map_info mvme6100_flash0_map = {
	.name =		"mvme6100-flash0",
	.size =		MVME6100_BANK_A_FLASH_SIZE,
	.bankwidth =	4,
};

static struct map_info mvme6100_flash1_map = {
	.name =		"mvme6100-flash1",
	.size =		MVME6100_BANK_B_FLASH_SIZE,
	.bankwidth =	4,
};

static struct mtd_partition mvme6100_flash0_partitions[] = {
	{
		.name =   "Linux Kernel + INITRD",
		.offset = 0x00000000,
		.size =   0x01400000,
	},
	{
		.name =   "Linux JFFS2 Filesystem",
		.offset = 0x01400000,
		.size =   0x02b00000,
	},
	{
		.name =   "Bootloader Flash 0",
		.offset = 0x03f00000,
		.size =   0x00100000,
	},
};

static struct mtd_partition mvme6100_flash1_partitions[] = {
	{
		.name =   "Linux JFFS2 Filesystem",
		.offset = 0x00000000,
		.size =   0x03f00000,
	},
	{
		.name	= "Bootloader Flash 1",
		.offset	= 0x03f00000,
		.size	= 0x00100000,
	}
};


int __init mvme6100_map_init(void)
{
	unsigned long mvme6100_flash0_base;
	unsigned long mvme6100_flash1_base;

	mvme6100_flash0_base = MVME6100_BANK_A_FLASH_BASE;

	mvme6100_flash0_map.phys = mvme6100_flash0_base;
	mvme6100_flash0_map.virt = (unsigned long)ioremap(mvme6100_flash0_base, 
						mvme6100_flash0_map.size);

	if (!mvme6100_flash0_map.virt) {
		printk(KERN_ERR "Failed to ioremap flash0\n");
		return -EIO;
	}

	simple_map_init(&mvme6100_flash0_map);

	flash[0] = do_map_probe("cfi_probe", &mvme6100_flash0_map);
	if (flash[0]) {
		flash[0]->owner = THIS_MODULE;
		add_mtd_partitions(flash[0], mvme6100_flash0_partitions,
					ARRAY_SIZE(mvme6100_flash0_partitions));
	} else {
		printk(KERN_ERR "map probe failed for flash0\n");
		iounmap((void *)mvme6100_flash0_map.virt);
		return -ENXIO;
	}

	mvme6100_flash1_base = MVME6100_BANK_B_FLASH_BASE;

	mvme6100_flash1_map.phys = mvme6100_flash1_base;
	mvme6100_flash1_map.virt = (unsigned long)ioremap(
						mvme6100_flash1_base, 
						mvme6100_flash1_map.size);

	if (!mvme6100_flash1_map.virt) {
		printk(KERN_ERR "Failed to ioremap flash1\n");
		return -EIO;
	}

	simple_map_init(&mvme6100_flash1_map);

	flash[1] = do_map_probe("cfi_probe", &mvme6100_flash1_map);
	if (flash[1]) {
		flash[1]->owner = THIS_MODULE;
		add_mtd_partitions(flash[1], mvme6100_flash1_partitions,
				ARRAY_SIZE(mvme6100_flash1_partitions));
	} else {
		printk(KERN_ERR "map probe failed for flash1\n");
		iounmap((void *)mvme6100_flash1_map.virt);
		return -ENXIO;
	}

	return 0;
}

static void __exit mvme6100_map_exit(void)
{
	int i;

        for (i=0; i<2; i++) {
		if (flash[i]) {
			del_mtd_partitions(flash[i]);
			map_destroy(flash[i]);
		}
	}

	if (mvme6100_flash0_map.virt) {
		iounmap((void *)mvme6100_flash0_map.virt);
	}
	if (mvme6100_flash1_map.virt) {
		iounmap((void *)mvme6100_flash1_map.virt);
	}
}

module_init(mvme6100_map_init);
module_exit(mvme6100_map_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ajit Prem <ajit.prem@motorola.com>");
MODULE_DESCRIPTION("MTD map and partitions for Motorola MVME6100");
