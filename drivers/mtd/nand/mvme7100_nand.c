/*
 * drivers/mtd/nand/mvme7100_nand.c
 *
 * A glue driver for the NAND chips on the MVME7100 board
 *
 * Author: Ajit Prem (ajit.prem@emerson.com)
 * Copyright 2008 Emerson Network Power Embedded Computing Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <platforms/86xx/mvme7100.h>

static int n_physical_chips;

struct mvme7100_nand_host {
	struct nand_chip nand_chip;
	struct mtd_info mtd;
	void __iomem *io_ctrl;
	void __iomem *io_select;
	void __iomem *io_status;
	void __iomem *io_data;
	int chip;
	int chip_enable;
};

static struct mvme7100_nand_host *chips[2];

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };

/*
 * Define static partitions for flash device
 */
static struct mtd_partition partition_info[] = {
	{
	 .name = "Linux YAFFS 1",
	 .offset = 0x00000000,
	 .size =   0x80000000,
	}, 
	{
	 .name = "Linux YAFFS 2",
	 .offset = 0x80000000,
	 .size =   0x80000000,
	},
};

#endif

/*
 *      hardware specific access to control-lines
 *
 *      ctrl:
 *      NAND_NCE: bit 0 -> bits 4->7 (0xf0) in SELECT Reg
 *      NAND_CLE: bit 1 -> bit 7 (0x80) in CONTROL Reg
 *      NAND_ALE: bit 2 -> bit 6 (0x40) in CONTROL Reg
 */

static void mvme7100_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mvme7100_nand_host *host = nand_chip->priv;

	if (ctrl & NAND_CTRL_CHANGE) {
		unsigned char bits;

		bits = readb(host->io_ctrl);
		bits &= 0x3f;
		bits = (ctrl & NAND_CLE) << 6;
		bits |= (ctrl & NAND_ALE) << 4;
		writeb(bits, host->io_ctrl);
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, host->io_data);
}

static void mvme7100_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mvme7100_nand_host *host = nand_chip->priv;
	unsigned char sel;

	if (chip > 3)
		return;

	sel = readb(host->io_select);
	sel &= ~(MVME7100_NAND_FLASH_CE1 | MVME7100_NAND_FLASH_CE2 |
		 MVME7100_NAND_FLASH_CE3 | MVME7100_NAND_FLASH_CE4);

	switch (chip) {
	case 0:
		sel |= MVME7100_NAND_FLASH_CE1; 
		break;
	case 1:
		sel |= MVME7100_NAND_FLASH_CE2; 
		break;
	case 2:
		sel |= MVME7100_NAND_FLASH_CE3; 
		break;
	case 3:
		sel |= MVME7100_NAND_FLASH_CE4; 
		break;
	default:
		break;
	}
	host->chip_enable = chip;
	writeb(sel, host->io_select);
}

static int mvme7100_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mvme7100_nand_host *host = nand_chip->priv;
	u8 io_status;
	int status;

	io_status = readb(host->io_status);

        switch (host->chip_enable) {
        case 0:
                status = io_status & MVME7100_NAND_FLASH_RB1;
                break;
        case 1:
                status = io_status & MVME7100_NAND_FLASH_RB2;
                break;
        case 2:
                status = io_status & MVME7100_NAND_FLASH_RB3;
                break;
        case 3:
                status = io_status & MVME7100_NAND_FLASH_RB4;
                break;
        default:
                status = 1;
                break;
        }
        return status;
}

static int __init mvme7100_init_one(int chip)
{
	struct mvme7100_nand_host *host;
	struct mtd_info *mtd;
	struct nand_chip *nand_chip;
	unsigned char reg;
	int err = 0;
#ifdef CONFIG_MTD_PARTITIONS
	const char *part_type = 0;
	int mtd_parts_nb = 0;
	struct mtd_partition *mtd_parts = 0;
#endif

	/* Allocate memory for MTD device structure and private data */
	host = kzalloc(sizeof(struct mvme7100_nand_host), GFP_KERNEL);
	if (!host) {
		printk(KERN_WARNING
		       "mvme7100_nand: Unable to allocate device structure.\n");
		return -ENOMEM;
	}

	if (chip == 1) {
		host->io_ctrl = ioremap(MVME7100_NAND_FLASH1_CONTROL_REG, 1);
		if (!host->io_ctrl) {
			printk(KERN_ERR "mvme7100_nand: Unable to ioremap()");
			err = -ENOMEM;
			goto init_dhost;
		}
		host->io_select = ioremap(MVME7100_NAND_FLASH1_SELECT_REG, 1);
		if (!host->io_select) {
			printk(KERN_ERR "mvme7100_nand: Unable to ioremap()");
			err = -ENOMEM;
			goto init_d1;
		}
		host->io_status = ioremap(MVME7100_NAND_FLASH1_STATUS_REG, 1);
		if (!host->io_status) {
			printk(KERN_ERR "mvme7100_nand: Unable to ioremap()");
			err = -ENOMEM;
			goto init_d2;
		}
		host->io_data = ioremap(MVME7100_NAND_FLASH1_DATA_REG, 1);
		if (!host->io_data) {
			printk(KERN_ERR "mvme7100_nand: Unable to ioremap()");
			err = -ENOMEM;
			goto init_d3;
		}
		chips[0] = host;
	} else if (chip == 2) {
		host->io_ctrl = ioremap(MVME7100_NAND_FLASH2_CONTROL_REG, 1);
		if (!host->io_ctrl) {
			printk(KERN_ERR "mvme7100_nand: Unable to ioremap()");
			err = -ENOMEM;
			goto init_dhost;
		}
		host->io_select = ioremap(MVME7100_NAND_FLASH2_SELECT_REG, 1);
		if (!host->io_select) {
			printk(KERN_ERR "mvme7100_nand: Unable to ioremap()");
			err = -ENOMEM;
			goto init_d1;
		}
		host->io_status = ioremap(MVME7100_NAND_FLASH2_STATUS_REG, 1);
		if (!host->io_status) {
			printk(KERN_ERR "mvme7100_nand: Unable to ioremap()");
			err = -ENOMEM;
			goto init_d2;
		}
		host->io_data = ioremap(MVME7100_NAND_FLASH2_DATA_REG, 1);
		if (!host->io_data) {
			printk(KERN_ERR "mvme7100_nand: Unable to ioremap()");
			err = -ENOMEM;
			goto init_d3;
		}
		chips[1] = host;
	} else {
		err = -EINVAL;
		goto init_dhost;
	}

	host->chip = chip;
	reg = readb(host->io_ctrl);
	reg &= ~MVME7100_NAND_FLASH_WP;
	writeb(reg, host->io_ctrl);
	mtd = &host->mtd;
	nand_chip = &host->nand_chip;

	/* Link the private data with the MTD structure */
	nand_chip->priv = host;
	mtd->priv = nand_chip;
	mtd->owner = THIS_MODULE;

	/* Set address of NAND IO lines */
	nand_chip->IO_ADDR_R = host->io_data;
	nand_chip->IO_ADDR_W = host->io_data;

	nand_chip->cmd_ctrl = mvme7100_hwcontrol;
	nand_chip->select_chip = mvme7100_select_chip;
	nand_chip->dev_ready = mvme7100_device_ready;
	/* We'll let YAFFS do ECC*/
	nand_chip->ecc.mode = NAND_ECC_NONE;
	nand_chip->chip_delay = 20;

	nand_chip->options = NAND_NO_AUTOINCR;

	/* Scan to find existence of the device */
	if (nand_scan(mtd, 4)) {
		err = -ENXIO;
		goto init_d4;
	}

#ifdef CONFIG_MTD_PARTITIONS
	if (chip == 1)
		mtd->name = "MVME7100 NAND Chip 1";
	else if (chip == 2)
		mtd->name = "MVME7100 NAND Chip 2";
	/* First look for partitions on the command line, these take
	 * take precedence over statically defined partitions */
	mtd_parts_nb = parse_mtd_partitions(mtd, part_probes, &mtd_parts, 0);
	if (mtd_parts_nb > 0)
		part_type = "command line";
	else
		mtd_parts_nb = 0;

	if (mtd_parts_nb == 0) {
		part_type = "static";
		mtd_parts = partition_info;
		mtd_parts_nb = 2;
	}

	/* Register the partitions */
	printk(KERN_INFO "Using %s partition definition\n", part_type);
	add_mtd_partitions(mtd, mtd_parts, mtd_parts_nb);
#else
	add_mtd_device(mtd);
#endif
	goto init_ret;

init_d4:
        /* Set WP */
        reg = readb(host->io_ctrl);
        reg |= MVME7100_NAND_FLASH_WP;
        writeb(reg, host->io_ctrl);
	iounmap(host->io_data);
init_d3:
	iounmap(host->io_status);
init_d2:
	iounmap(host->io_select);
init_d1:
	iounmap(host->io_ctrl);
init_dhost:
	kfree(host);
init_ret:
	return err;
}

static int __init mvme7100_init(void)
{
	int err = -ENXIO;
	void __iomem *preg;

	preg = ioremap(MVME7100_NAND_FLASH1_PRESENCE_REG, 1);
	if (!preg) {
		printk(KERN_WARNING "mvme7100_nand: ioremap failed\n");

		return -ENOMEM;
	}
	if (readb(preg) & MVME7100_NAND_FLASH_CP) {
		err = mvme7100_init_one(1);
		if (err) {
			iounmap(preg);
			return err;
		}
		n_physical_chips++;
	}
	iounmap(preg);

	preg = ioremap(MVME7100_NAND_FLASH2_PRESENCE_REG, 1);
	if (!preg) {
		printk(KERN_WARNING "mvme7100_nand: ioremap failed\n");

		return -ENOMEM;
	}
	if (readb(preg) & MVME7100_NAND_FLASH_CP) {
		err = mvme7100_init_one(2);
		if (err) {
			iounmap(preg);
			return err;
		}
		n_physical_chips++;
	}
	iounmap(preg);

	return err;
}

module_init(mvme7100_init);

static void __exit mvme7100_cleanup(void)
{
	int i;

	for (i = 0; i < n_physical_chips; i++) {
		struct mvme7100_nand_host *host = chips[i];
		struct mtd_info *mtd;
		unsigned char reg;

		if (host) {
			mtd = &host->mtd;

			if (mtd)
				nand_release(mtd);
                        reg = readb(host->io_ctrl);
                        reg |= MVME7100_NAND_FLASH_WP;
                        writeb(reg, host->io_ctrl);
			iounmap(host->io_data);
			iounmap(host->io_status);
			iounmap(host->io_select);
			iounmap(host->io_ctrl);

			kfree(host);
		}
	}
	return;
}

module_exit(mvme7100_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ajit Prem (ajit.prem@emerson.com");
MODULE_DESCRIPTION("NAND driver for the nand chips on the MVME7100");
