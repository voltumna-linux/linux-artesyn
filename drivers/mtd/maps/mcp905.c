/*
 * 
 * Mapping for Motorola MCP905 flash
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
#include <platforms/mcp905.h>

static struct mtd_info *flash[2];

static struct map_info mcp905_flash0_map = {
	.name =		"mcp905-flash0",
	.size =		MCP905_BANK_A_FLASH_SIZE,
	.bankwidth =	4,
};

static struct map_info mcp905_flash1_map = {
	.name =		"mcp905-flash1",
	.size =		MCP905_BANK_B_FLASH_SIZE,
	.bankwidth =	4,
};

static struct mtd_partition mcp905_flash0_partitions[] = {
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

static struct mtd_partition mcp905_flash1_partitions[] = {
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


int __init mcp905_map_init(void)
{
	unsigned long mcp905_flash0_base;
	unsigned long mcp905_flash1_base;

	mcp905_flash0_base = MCP905_BANK_A_FLASH_BASE;

	mcp905_flash0_map.phys = mcp905_flash0_base;
	mcp905_flash0_map.virt = (unsigned long)ioremap(mcp905_flash0_base, 
						mcp905_flash0_map.size);

	if (!mcp905_flash0_map.virt) {
		printk("Failed to ioremap flash0\n");
		return -EIO;
	}

	simple_map_init(&mcp905_flash0_map);

	flash[0] = do_map_probe("cfi_probe", &mcp905_flash0_map);
	if (flash[0]) {
		flash[0]->owner = THIS_MODULE;
		add_mtd_partitions(flash[0], mcp905_flash0_partitions,
					ARRAY_SIZE(mcp905_flash0_partitions));
	} else {
		printk("map probe failed for flash0\n");
		iounmap((void *)mcp905_flash0_map.virt);
		return -ENXIO;
	}

	mcp905_flash1_base = MCP905_BANK_B_FLASH_BASE;

	mcp905_flash1_map.phys = mcp905_flash1_base;
	mcp905_flash1_map.virt = (unsigned long)ioremap(
						mcp905_flash1_base, 
						mcp905_flash1_map.size);

	if (!mcp905_flash1_map.virt) {
		printk("Failed to ioremap flash1\n");
		return -EIO;
	}

	simple_map_init(&mcp905_flash1_map);

	flash[1] = do_map_probe("cfi_probe", &mcp905_flash1_map);
	if (flash[1]) {
		flash[1]->owner = THIS_MODULE;
		add_mtd_partitions(flash[1], mcp905_flash1_partitions,
				ARRAY_SIZE(mcp905_flash1_partitions));
	} else {
		printk("map probe failed for flash1\n");
		iounmap((void *)mcp905_flash0_map.virt);
		iounmap((void *)mcp905_flash1_map.virt);
		return -ENXIO;
	}

	return 0;
}

static void __exit mcp905_map_exit(void)
{
	int i;

        for (i=0; i<2; i++) {
		if (flash[i]) {
			del_mtd_partitions(flash[i]);
			map_destroy(flash[i]);
		}
	}

	if (mcp905_flash0_map.virt) {
		iounmap((void *)mcp905_flash0_map.virt);
	}
	if (mcp905_flash1_map.virt) {
		iounmap((void *)mcp905_flash1_map.virt);
	}
}

module_init(mcp905_map_init);
module_exit(mcp905_map_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ajit Prem <ajit.prem@motorola.com>");
MODULE_DESCRIPTION("MTD map and partitions for Motorola MCP905");
