// SPDX-License-Identifier: GPL-2.0
/*
 * Platform device setup for Marvell mv64360/mv64460 host bridges (Discovery)
 *
 * Author: Dale Farnsworth <dale@farnsworth.org>
 *
 * 2007 (c) MontaVista, Software, Inc.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/mv643xx.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>

#include <asm/prom.h>

/* These functions provide the necessary setup for the mv64x60 drivers. */

static const struct of_device_id of_mv64x60_devices[] __initconst = {
	{ .compatible = "marvell,mv64306-devctrl", },
	{}
};

/*
 * Create mv64x60_eth platform devices
 */
static struct platform_device * __init mv64x60_eth_register_shared_pdev(
						struct device_node *np, int id)
{
	struct platform_device *pdev;
	struct resource r[2];
	int err;

	err = of_address_to_resource(np, 0, &r[0]);
	if (err)
		return ERR_PTR(err);

	/* register an orion mdio bus driver */
	r[1].start = r[0].start + 0x4;
	r[1].end = r[0].start + 0x84 - 1;
	r[1].flags = IORESOURCE_MEM;

	if (id == 0) {
		pdev = platform_device_register_simple("orion-mdio", -1, &r[1], 1);
		if (IS_ERR(pdev))
			return pdev;
	}

	pdev = platform_device_register_simple(MV643XX_ETH_SHARED_NAME, id,
					       &r[0], 1);

	return pdev;
}

static int __init mv64x60_eth_device_setup(struct device_node *np, int id,
					   struct platform_device *shared_pdev)
{
	struct resource r[1];
	struct mv643xx_eth_platform_data pdata;
	struct platform_device *pdev;
	struct device_node *phy;
	const u8 *mac_addr;
	const int *prop;
	const phandle *ph;
	int err;

	memset(r, 0, sizeof(r));
	of_irq_to_resource(np, 0, &r[0]);

	memset(&pdata, 0, sizeof(pdata));

	pdata.shared = shared_pdev;

	prop = of_get_property(np, "reg", NULL);
	if (!prop)
		return -ENODEV;
	pdata.port_number = *prop;

	mac_addr = of_get_mac_address(np);
	if (!IS_ERR(mac_addr))
		ether_addr_copy(pdata.mac_addr, mac_addr);

	prop = of_get_property(np, "speed", NULL);
	if (prop)
		pdata.speed = *prop;

	prop = of_get_property(np, "tx_queue_size", NULL);
	if (prop)
		pdata.tx_queue_size = *prop;

	prop = of_get_property(np, "rx_queue_size", NULL);
	if (prop)
		pdata.rx_queue_size = *prop;

	prop = of_get_property(np, "tx_sram_addr", NULL);
	if (prop)
		pdata.tx_sram_addr = *prop;

	prop = of_get_property(np, "tx_sram_size", NULL);
	if (prop)
		pdata.tx_sram_size = *prop;

	prop = of_get_property(np, "rx_sram_addr", NULL);
	if (prop)
		pdata.rx_sram_addr = *prop;

	prop = of_get_property(np, "rx_sram_size", NULL);
	if (prop)
		pdata.rx_sram_size = *prop;

	ph = of_get_property(np, "phy", NULL);
	if (!ph)
		return -ENODEV;

	phy = of_find_node_by_phandle(*ph);
	if (phy == NULL)
		return -ENODEV;

	prop = of_get_property(phy, "reg", NULL);
	if (prop)
		pdata.phy_addr = MV643XX_ETH_PHY_ADDR(*prop);

	of_node_put(phy);

	pdev = platform_device_alloc(MV643XX_ETH_NAME, id);
	if (!pdev)
		return -ENOMEM;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	err = platform_device_add_resources(pdev, r, 1);
	if (err)
		goto error;

	err = platform_device_add_data(pdev, &pdata, sizeof(pdata));
	if (err)
		goto error;

	err = platform_device_add(pdev);
	if (err)
		goto error;

	return 0;

error:
	platform_device_put(pdev);
	return err;
}

/*
 * Create mv64x60_i2c platform devices
 */
static int __init mv64x60_i2c_device_setup(struct device_node *np, int id)
{
	struct resource r[2];
	struct platform_device *pdev;
	struct mv64xxx_i2c_pdata pdata;
	const unsigned int *prop;
	int err;

	memset(r, 0, sizeof(r));

	err = of_address_to_resource(np, 0, &r[0]);
	if (err)
		return err;

	of_irq_to_resource(np, 0, &r[1]);

	memset(&pdata, 0, sizeof(pdata));

	pdata.freq_m = 8;	/* default */
	prop = of_get_property(np, "freq_m", NULL);
	if (prop)
		pdata.freq_m = *prop;

	pdata.freq_n = 3;	/* default */
	prop = of_get_property(np, "freq_n", NULL);
	if (prop)
		pdata.freq_n = *prop;

	pdata.timeout = 1000;				/* default: 1 second */

	pdev = platform_device_alloc(MV64XXX_I2C_CTLR_NAME, id);
	if (!pdev)
		return -ENOMEM;

	err = platform_device_add_resources(pdev, r, 2);
	if (err)
		goto error;

	err = platform_device_add_data(pdev, &pdata, sizeof(pdata));
	if (err)
		goto error;

	err = platform_device_add(pdev);
	if (err)
		goto error;

	return 0;

error:
	platform_device_put(pdev);
	return err;
}

/*
 * Create mv64x60_wdt platform devices
 */
static int __init mv64x60_wdt_device_setup(struct device_node *np, int id)
{
	struct resource r;
	struct platform_device *pdev;
	struct mv64x60_wdt_pdata pdata;
	const unsigned int *prop;
	int err;

	err = of_address_to_resource(np, 0, &r);
	if (err)
		return err;

	memset(&pdata, 0, sizeof(pdata));

	pdata.timeout = 10;			/* Default: 10 seconds */

	np = of_get_parent(np);
	if (!np)
		return -ENODEV;

	prop = of_get_property(np, "clock-frequency", NULL);
	of_node_put(np);
	if (!prop)
		return -ENODEV;
	pdata.bus_clk = *prop / 1000000; /* wdt driver wants freq in MHz */

	pdev = platform_device_alloc(MV64x60_WDT_NAME, id);
	if (!pdev)
		return -ENOMEM;

	err = platform_device_add_resources(pdev, &r, 1);
	if (err)
		goto error;

	err = platform_device_add_data(pdev, &pdata, sizeof(pdata));
	if (err)
		goto error;

	err = platform_device_add(pdev);
	if (err)
		goto error;

	return 0;

error:
	platform_device_put(pdev);
	return err;
}

static int __init mv64x60_device_setup(void)
{
	struct device_node *np, *np2;
	struct platform_device *pdev;
	int id, id2;
	int err;

	id = 0;
	id2 = 0;
	for_each_compatible_node(np, NULL, "marvell,mv64360-eth-group") {
		pdev = mv64x60_eth_register_shared_pdev(np, id++);
		if (IS_ERR(pdev)) {
			err = PTR_ERR(pdev);
			pr_err("Failed to initialize MV64x60 network block %pOF: error %d.\n",
			       np, err);
			continue;
		}
		for_each_child_of_node(np, np2) {
			if (!of_device_is_compatible(np2,
					"marvell,mv64360-eth"))
				continue;
			err = mv64x60_eth_device_setup(np2, id2++, pdev);
			if (err)
				pr_err("Failed to initialize MV64x60 network device %pOF: error %d.\n",
				       np2, err);
		}
	}

	id = 0;
	for_each_compatible_node(np, "i2c", "marvell,mv64360-i2c") {
		err = mv64x60_i2c_device_setup(np, id++);
		if (err)
			pr_err("Failed to initialize MV64x60 I2C bus %pOF: error %d.\n",
			       np, err);
	}

	/* support up to one watchdog timer */
	np = of_find_compatible_node(np, NULL, "marvell,mv64360-wdt");
	if (np) {
		if ((err = mv64x60_wdt_device_setup(np, id)))
			pr_err("Failed to initialize MV64x60 Watchdog %pOF: error %d.\n",
			       np, err);
		of_node_put(np);
	}

	/* Now add every node that is on the device bus */
	for_each_compatible_node(np, NULL, "marvell,mv64360")
		of_platform_bus_probe(np, of_mv64x60_devices, NULL);

	return 0;
}
arch_initcall(mv64x60_device_setup);
