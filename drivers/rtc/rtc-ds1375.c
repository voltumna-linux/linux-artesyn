/*
 * drivers/rtc/ds1375.c
 *
 * RTC driver for the Maxim/Dallas DS1375 Real-Time Clock
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Based on various RTC drivers 
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include <asm/time.h>

#define DS1375_SECONDS_REGISTER       0x00
#define DS1375_MINUTES_REGISTER       0x01
#define DS1375_HOURS_REGISTER         0x02
#define DS1375_DAY_REGISTER           0x03
#define DS1375_DATE_REGISTER          0x04
#define DS1375_MONTH_REGISTER         0x05
#define DS1375_YEAR_REGISTER          0x06
#define DS1375_CENTURY_REGISTER       0x05
#define DS1375_CONTROL_REGISTER       0x0E
#define DS1375_STATUS_REGISTER        0x0F

#define DS1375_12HOUR_MODE_FLAG       0x40
#define DS1375_PM_HOUR_FLAG           0x20
#define DS1375_BIT_CENTURY            0x80

#define DS1375_CLOCK_ENABLE		0x80

#define DS1375_NVRAM_OFFSET             0x10
#define DS1375_MIN_NVRAM_ADDRESS        0x10
#define DS1375_MAX_NVRAM_ADDRESS        0x1f
#define DS1375_NVRAM_SIZE               0x10

#define	DS1375_DRV_NAME		"ds1375"


static struct i2c_driver ds1375_driver;

static unsigned short normal_i2c[] = { 0x68, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(ds1375);

struct ds1375_data {
	u8	reg_addr;
	u8	regs[8];
	struct i2c_msg	  msg[2];
	struct i2c_client client;
	struct rtc_device *rtc;
};

int ds1375_get_rtc_time(struct device *dev, struct rtc_time *t)
{
        struct ds1375_data   *ds1375 = dev_get_drvdata(dev);
        int             tmp;

        /* read the RTC registers all at once */
        ds1375->msg[1].flags = I2C_M_RD;
        ds1375->msg[1].len = 7;

        tmp = i2c_transfer(ds1375->client.adapter, ds1375->msg, 2);
        if (tmp != 2) {
                dev_err(dev, "%s error %d\n", "read", tmp);
                return -EIO;
        }

        dev_dbg(dev, "%s: %02x %02x %02x %02x %02x %02x %02x\n",
                        "read",
                        ds1375->regs[0], ds1375->regs[1],
                        ds1375->regs[2], ds1375->regs[3],
                        ds1375->regs[4], ds1375->regs[5],
                        ds1375->regs[6]);

        t->tm_sec = BCD2BIN(ds1375->regs[DS1375_SECONDS_REGISTER] & 0x7f);
        t->tm_min = BCD2BIN(ds1375->regs[DS1375_MINUTES_REGISTER] & 0x7f);
        tmp = ds1375->regs[DS1375_HOURS_REGISTER] & 0x3f;
        t->tm_hour = BCD2BIN(tmp);
        t->tm_wday = BCD2BIN(ds1375->regs[DS1375_DAY_REGISTER] & 0x07) - 1;
        t->tm_mday = BCD2BIN(ds1375->regs[DS1375_DATE_REGISTER] & 0x3f);
        tmp = ds1375->regs[DS1375_MONTH_REGISTER] & 0x1f;
        t->tm_mon = BCD2BIN(tmp) - 1;

        /* assume 20YY not 19YY, and ignore DS1375_BIT_CENTURY */
        t->tm_year = BCD2BIN(ds1375->regs[DS1375_YEAR_REGISTER]) + 100;

        dev_dbg(dev, "%s secs=%d, mins=%d, "
                "hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
                "read", t->tm_sec, t->tm_min,
                t->tm_hour, t->tm_mday,
                t->tm_mon, t->tm_year, t->tm_wday);

        return 0;
}

int ds1375_set_rtc_time(struct device *dev, struct rtc_time *t)
{
        struct ds1375_data   *ds1375 = dev_get_drvdata(dev);
        int             result;
        int             tmp;
        u8              *buf = ds1375->regs;

        dev_dbg(dev, "%s secs=%d, mins=%d, "
                "hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
                "write", t->tm_sec, t->tm_min,
                t->tm_hour, t->tm_mday,
                t->tm_mon, t->tm_year, t->tm_wday);

        *buf++ = 0;             /* first register addr */
        buf[DS1375_SECONDS_REGISTER] = BIN2BCD(t->tm_sec);
        buf[DS1375_MINUTES_REGISTER] = BIN2BCD(t->tm_min);
        buf[DS1375_HOURS_REGISTER] = BIN2BCD(t->tm_hour);
        buf[DS1375_DAY_REGISTER] = BIN2BCD(t->tm_wday + 1);
        buf[DS1375_DATE_REGISTER] = BIN2BCD(t->tm_mday);
        buf[DS1375_MONTH_REGISTER] = BIN2BCD(t->tm_mon + 1);

        /* assume 20YY not 19YY */
        tmp = t->tm_year - 100;
        buf[DS1375_YEAR_REGISTER] = BIN2BCD(tmp);

        buf[DS1375_MONTH_REGISTER] |= DS1375_BIT_CENTURY;

        ds1375->msg[1].flags = 0;
        ds1375->msg[1].len = 8;

        dev_dbg(dev, "%s: %02x %02x %02x %02x %02x %02x %02x\n",
                "write", buf[0], buf[1], buf[2], buf[3],
                buf[4], buf[5], buf[6]);

        result = i2c_transfer(ds1375->client.adapter, &ds1375->msg[1], 1);

        if (result != 1) {
                dev_err(dev, "%s error %d\n", "write", tmp);
                return -EIO;
        }
        return 0;
}

static void ds1375_init_client(struct i2c_client *client)
{
	s32 val;

	/* Enable clock */
	val = 0x84;
	i2c_smbus_write_byte_data(client, DS1375_CONTROL_REGISTER, val);

	/* Clear status */
	val = 0;
	i2c_smbus_write_byte_data(client, DS1375_STATUS_REGISTER, val);

	/* Ensure that device is set in 24-hour mode */
	val = i2c_smbus_read_byte_data(client, DS1375_HOURS_REGISTER);
	if ((val >= 0) && (val & (1 << 6)))
		i2c_smbus_write_byte_data(client, DS1375_HOURS_REGISTER,
					  val & 0x3f);
}

static const struct rtc_class_ops ds1375_rtc_ops = {
        .read_time      = ds1375_get_rtc_time,
        .set_time       = ds1375_set_rtc_time,
};

static int __devinit ds1375_detect(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	struct ds1375_data *data;
	int rc = 0;

	if (!i2c_check_functionality(adap, I2C_FUNC_I2C))
		return 0;

	if (!(data = kzalloc(sizeof(struct ds1375_data), GFP_KERNEL))) 
		return -ENOMEM;
	
	client = &data->client;
	client->addr = addr;
	client->adapter = adap;
	client->driver = &ds1375_driver;
	strncpy(client->name, DS1375_DRV_NAME, I2C_NAME_SIZE);
	client->flags = 0;

	i2c_set_clientdata(client, data);

        data->msg[0].addr = client->addr;
        data->msg[0].flags = 0;
        data->msg[0].len = 1;
        data->msg[0].buf = &data->reg_addr;

        data->msg[1].addr = client->addr;
        data->msg[1].flags = I2C_M_RD;
        data->msg[1].len = sizeof(data->regs);
        data->msg[1].buf = data->regs;

	if (kind == 0) {
		kind = ds1375;
		data->reg_addr = 0; 
	}

	if ((rc = i2c_attach_client(client)) != 0)
		goto exit_free;

	data->rtc = rtc_device_register(client->name, &client->dev,
				&ds1375_rtc_ops, THIS_MODULE);	

        if (IS_ERR(data->rtc)) {
                rc = PTR_ERR(data->rtc);
                dev_err(&client->dev,
                        "unable to register the class device\n");
                goto exit_detach;
        }

	ds1375_init_client(client);

	return 0;

exit_detach:
	i2c_detach_client(client);
exit_free:
	kfree(data);
	return rc;
}

static int __devinit ds1375_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, ds1375_detect);
}

static int __devexit ds1375_detach(struct i2c_client *client)
{
	int rc;
        struct ds1375_data   *ds1375 = i2c_get_clientdata(client);

        rtc_device_unregister(ds1375->rtc);

	if ((rc = i2c_detach_client(client)))
		return rc;

	kfree(ds1375);
	return 0;
}

static struct i2c_driver ds1375_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DS1375_DRV_NAME,
	},
	.id = I2C_DRIVERID_DS1375,
	.attach_adapter = ds1375_attach,
	.detach_client = __devexit_p(ds1375_detach),
};

static int __init ds1375_init(void)
{
	return i2c_add_driver(&ds1375_driver);
}

static void __exit ds1375_exit(void)
{
	i2c_del_driver(&ds1375_driver);
}

module_init(ds1375_init);
module_exit(ds1375_exit);

MODULE_AUTHOR("Ajit Prem <Ajit.Prem@motorola.com>");
MODULE_DESCRIPTION("Maxim/Dallas DS1375 RTC I2C Client Driver");
MODULE_LICENSE("GPL");
