/*
 * 
 * Mapping for Motorola MVME5500 flash
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
#include <platforms/mvme5500.h>

/* Bank A is flash 0 and Bank B is flash 1 */

extern unsigned long mvme5500_bank_A_flash_size;

static struct mtd_info *flash[2];

static struct map_info mvme5500_flash1_map = {
	.name =		"mvme5500-flash1",
	.size =		MVME5500_BANK_B_FLASH_SIZE,
	.bankwidth =	4,
};

static struct map_info mvme5500_flash0_map = {
	.name =		"mvme5500-flash0",
	.size =		MVME5500_BANK_A_FLASH_SIZE,
	.bankwidth =	4,
};

static struct mtd_partition mvme5500_flash1_partitions[] = {
	{
		.name =   "Linux JFFS2 Filesystem",
		.offset = 0x00000000,
		.size =   0x00700000,
	},
	{
		.name	= "Bootloader Flash 1",
		.offset	= 0x00700000,
		.size	= 0x00100000,
	}
};


static struct mtd_partition mvme5500_flash0_8M_partitions[] = {
	{
		.name =   "Linux Kernel + INITRD",
		.offset = 0x00000000,
		.size =   0x00700000,
	},
	{
		.name =   "Bootloader Flash 0",
		.offset = 0x00700000,
		.size =   0x00100000,
	},
};

static struct mtd_partition mvme5500_flash0_16M_partitions[] = {
	{
		.name =   "Linux Kernel + INITRD",
		.offset = 0x00000000,
		.size =   0x00f00000,
	},
	{
		.name =   "Bootloader Flash 0",
		.offset = 0x00f00000,
		.size =   0x00100000,
	},
};

static struct mtd_partition mvme5500_flash0_32M_partitions[] = {
	{
		.name =   "Linux Kernel + INITRD",
		.offset = 0x00000000,
		.size =   0x01400000,
	},
	{
		.name =   "Linux JFFS2 Filesystem",
		.offset = 0x01400000,
		.size =   0x00b00000,
	},
	{
		.name =   "Bootloader Flash 0",
		.offset = 0x01f00000,
		.size =   0x00100000,
	},
};

static struct mtd_partition mvme5500_flash0_64M_partitions[] = {
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

int __init mvme5500_map_init(void)
{
	unsigned long mvme5500_flash0_base;
	unsigned long mvme5500_flash1_base;

	mvme5500_flash1_base = MVME5500_BANK_B_FLASH_BASE;

	mvme5500_flash1_map.phys = mvme5500_flash1_base;
	mvme5500_flash1_map.virt = (unsigned long)ioremap(
						mvme5500_flash1_base, 
						mvme5500_flash1_map.size);

	if (!mvme5500_flash1_map.virt) {
		printk(KERN_ERR "mvme5500_flash: Failed to ioremap flash1\n");
		return -EIO;
	}

	simple_map_init(&mvme5500_flash1_map);

	flash[1] = do_map_probe("cfi_probe", &mvme5500_flash1_map);
	if (flash[1]) {
		flash[1]->owner = THIS_MODULE;
		add_mtd_partitions(flash[1], mvme5500_flash1_partitions,
				ARRAY_SIZE(mvme5500_flash1_partitions));
	} else {
		printk(KERN_ERR "mvme5500_flash: map probe failed for flash1\n");
		iounmap((void *)mvme5500_flash1_map.virt);
		return -ENXIO;
	}

	mvme5500_flash0_base = MVME5500_BANK_A_FLASH_BASE;

	mvme5500_flash0_map.phys = mvme5500_flash0_base;
	if (mvme5500_bank_A_flash_size != 0) {
		mvme5500_flash0_map.size = mvme5500_bank_A_flash_size;
		mvme5500_flash0_map.virt = 
		(unsigned long)ioremap(mvme5500_flash0_base, 
						mvme5500_flash0_map.size);

		if (!mvme5500_flash0_map.virt) {
			printk(KERN_ERR "mvme5500_flash: Failed to ioremap flash0\n");
			return -EIO;
		}

		simple_map_init(&mvme5500_flash0_map);

		flash[0] = do_map_probe("cfi_probe", &mvme5500_flash0_map);
		if (flash[0]) {
			flash[0]->owner = THIS_MODULE;

			if (mvme5500_flash0_map.size == 8 * 1024 * 1024) {
				add_mtd_partitions(flash[0], 
					mvme5500_flash0_8M_partitions,
					ARRAY_SIZE(mvme5500_flash0_8M_partitions));
			} else if (mvme5500_flash0_map.size == 16 * 1024 * 1024) {
				add_mtd_partitions(flash[0], 
					mvme5500_flash0_16M_partitions,
					ARRAY_SIZE(mvme5500_flash0_16M_partitions));
			} else if (mvme5500_flash0_map.size == 32 * 1024 * 1024) {
				add_mtd_partitions(flash[0], 
					mvme5500_flash0_32M_partitions,
					ARRAY_SIZE(mvme5500_flash0_32M_partitions));
			} else if (mvme5500_flash0_map.size == 64 * 1024 * 1024) {
				add_mtd_partitions(flash[0], 
					mvme5500_flash0_64M_partitions,
					ARRAY_SIZE(mvme5500_flash0_64M_partitions));
			} else {
				printk(KERN_ERR "mvme5500_flash: Unexpected size for flash0\n");
				iounmap((void *)mvme5500_flash0_map.virt);
				return -ENXIO;
			}
		} else {
			printk(KERN_ERR "mvme5500_flash: map probe failed for flash0\n");
			iounmap((void *)mvme5500_flash0_map.virt);
			return -ENXIO;
		}
	}

	return 0;
}

static void __exit mvme5500_map_exit(void)
{
	int i;

        for (i=0; i<2; i++) {
		if (flash[i]) {
			del_mtd_partitions(flash[i]);
			map_destroy(flash[i]);
		}
	}

	if (mvme5500_flash0_map.virt) {
		iounmap((void *)mvme5500_flash0_map.virt);
	}
	if (mvme5500_flash1_map.virt) {
		iounmap((void *)mvme5500_flash1_map.virt);
	}
}

module_init(mvme5500_map_init);
module_exit(mvme5500_map_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ajit Prem <ajit.prem@motorola.com>");
MODULE_DESCRIPTION("MTD map and partitions for Motorola MVME5500");
