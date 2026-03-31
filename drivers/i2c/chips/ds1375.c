/*
 * drivers/i2c/chips/ds1375.c
 *
 * I2C client/driver for the Maxim/Dallas DS1375 Real-Time Clock
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2005-2007 Motorola, Inc. 
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

#define DS1375_CLOCK_ENABLE		0x80

#define DS1375_NVRAM_OFFSET             0x10
#define DS1375_MIN_NVRAM_ADDRESS        0x10
#define DS1375_MAX_NVRAM_ADDRESS        0x1f
#define DS1375_NVRAM_SIZE               0x10

#define	DS1375_DRV_NAME		"ds1375"

static DEFINE_MUTEX(ds1375_mutex);

static struct i2c_driver ds1375_driver;
static struct i2c_client *save_client;

static unsigned short normal_i2c[] = { 0x68, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(ds1375);

struct ds1375_data {
	struct i2c_client client;
	int sysctl_id;
};

static inline int ds1375_read(struct i2c_client *client, u8 reg, u8 *value)
{
	s32 tmp = i2c_smbus_read_byte_data(client, reg);

	if (tmp < 0)
		return -EIO;

	*value = tmp;

	return 0;
}

static ulong ds1375_read_rtc(void)
{
	u8 yr, month, mday, wday, hours, minutes, seconds;
	u8 temp;
	unsigned int year;

	if (ds1375_read(save_client, DS1375_SECONDS_REGISTER, &seconds) < 0) {
		dev_warn(&save_client->dev, "RTC read error SECOND register\n");
		return 0;
	}
	if (ds1375_read(save_client, DS1375_MINUTES_REGISTER, &minutes) < 0) {
		dev_warn(&save_client->dev, "RTC read error MINUTE register\n");
		return 0;
	}
	if (ds1375_read(save_client, DS1375_HOURS_REGISTER, &hours) < 0) {
		dev_warn(&save_client->dev, "RTC read error HOUR register\n");
		return 0;
	}
	if (ds1375_read(save_client, DS1375_DAY_REGISTER, &wday) < 0) {
		dev_warn(&save_client->dev, "RTC read error DAY register\n");
		return 0;
	}
	if (ds1375_read(save_client, DS1375_DATE_REGISTER, &mday) < 0) {
		dev_warn(&save_client->dev, "RTC read error DATE register\n");
		return 0;
	}
	if (ds1375_read(save_client, DS1375_MONTH_REGISTER, &month) < 0) {
		dev_warn(&save_client->dev, "RTC read error MONTH register\n");
		return 0;
	}
	if (ds1375_read(save_client, DS1375_YEAR_REGISTER, &yr) < 0) {
		dev_warn(&save_client->dev, "RTC read error YEAR register\n");
		return 0;
	}

	dev_dbg(&save_client->dev, "%s: %02x %02x %02x %02x %02x %02x %02x\n",
		__FUNCTION__, seconds, minutes, hours, wday, mday, month, yr);

	seconds = BCD2BIN(seconds);
	minutes = BCD2BIN(minutes);
	hours = hours & 0x3f;
	hours = BCD2BIN(hours);
	wday = BCD2BIN(wday) - 1;
	mday = BCD2BIN(mday);
	temp = month;
	month = month & 0x7f;
	month = BCD2BIN(month);
	year = BCD2BIN(yr);
	if (temp & 0x80)
		year += 100;
	year += 1900;

	dev_dbg(&save_client->dev, "%s: secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		__FUNCTION__, seconds, minutes, hours, mday, month, year, wday);

	return (mktime(year, month, mday, hours, minutes, seconds));
}

static void ds1375_write_rtc(ulong time)
{
	struct rtc_time tm;
	s32 buf[7];
	u32 year;

	to_tm(time, &tm);

	dev_dbg(&save_client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n", __FUNCTION__,
		tm.tm_sec, tm.tm_min, tm.tm_hour,
		tm.tm_mday, tm.tm_mon, tm.tm_year, tm.tm_wday);

	buf[0] = BIN2BCD(tm.tm_sec);
	buf[1] = BIN2BCD(tm.tm_min);
	buf[2] = BIN2BCD(tm.tm_hour);
	buf[3] = BIN2BCD(tm.tm_wday) + 1;
	buf[4] = BIN2BCD(tm.tm_mday);
	buf[5] = BIN2BCD(tm.tm_mon);
	year = tm.tm_year - 1900;
	if (year >= 100) {
		year -= 100;
		buf[5] |= (1 << 7);
	}
	buf[6] = BIN2BCD(year);

	i2c_smbus_write_byte_data(save_client, DS1375_SECONDS_REGISTER, buf[0]);
	i2c_smbus_write_byte_data(save_client, DS1375_MINUTES_REGISTER, buf[1]);
	i2c_smbus_write_byte_data(save_client, DS1375_HOURS_REGISTER, buf[2]);
	i2c_smbus_write_byte_data(save_client, DS1375_DAY_REGISTER, buf[3]);
	i2c_smbus_write_byte_data(save_client, DS1375_DATE_REGISTER, buf[4]);
	i2c_smbus_write_byte_data(save_client, DS1375_MONTH_REGISTER, buf[5]);
	i2c_smbus_write_byte_data(save_client, DS1375_YEAR_REGISTER, buf[6]);

	return;
}

#ifdef CONFIG_PPC_OF
void ds1375_get_rtc_time(struct rtc_time *tm)
#else
ulong ds1375_get_rtc_time(void)
#endif
{
	ulong t1, t2;
	int limit = 10;		/* arbitrary retry limit */

	if (save_client == NULL) {
		printk(KERN_ERR "I2C not initialized!!!\n");
#ifdef CONFIG_PPC_OF
		return;
#else
		return 0;
#endif
	}

	mutex_lock(&ds1375_mutex);

	/*
	 * Since the reads are being performed one byte at a time using
	 * the SMBus vs a 4-byte i2c transfer, there is a chance that a
	 * carry will occur during the read. To detect this, 2 reads are
	 * performed and compared.
	 */
	do {
		t1 = ds1375_read_rtc();
		t2 = ds1375_read_rtc();
	} while (t1 != t2 && limit--);

	mutex_unlock(&ds1375_mutex);

	if (t1 != t2) {
		dev_warn(&save_client->dev,
			 "Cannot get consistent time from RTC chip\n");
		t1 = 0;
	}

#ifdef CONFIG_PPC_OF
	to_tm(t1, tm);
	tm->tm_year -= 1900;
	tm->tm_mon -= 1;
	return;
#else
	return t1;
#endif
}

static ulong new_time;

static void ds1375_set_work(struct work_struct *work)
{
	ulong t1, t2;
	int limit = 10;		/* arbitrary retry limit */

	t1 = new_time;

	mutex_lock(&ds1375_mutex);

	/*
	 * Since the writes are being performed one byte at a time using
	 * the SMBus vs a 4-byte i2c transfer, there is a chance that a
	 * carry will occur during the write. To detect this, the write
	 * value is read back and compared.
	 */
	do {
		ds1375_write_rtc(t1);
		t2 = ds1375_read_rtc();
	} while (t1 != t2 && limit--);

	mutex_unlock(&ds1375_mutex);

	if (t1 != t2)
		dev_warn(&save_client->dev,
			 "Cannot confirm time set from RTC chip\n");
}

static struct workqueue_struct *ds1375_workqueue;
static DECLARE_WORK(ds1375_work, ds1375_set_work);

#ifdef CONFIG_PPC_OF
int ds1375_set_rtc_time(struct rtc_time *time)
#else
int ds1375_set_rtc_time(ulong nowtime)
#endif
{
#ifdef CONFIG_PPC_OF
	new_time = mktime(time->tm_year+1900, time->tm_mon+1,
				time->tm_mday, time->tm_hour, time->tm_min,
				time->tm_sec);
#else
	new_time = nowtime;
#endif

	if (in_interrupt())
		queue_work(ds1375_workqueue, &ds1375_work);
	else
		ds1375_set_work(NULL);

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

/*
 *****************************************************************************
 *
 *	Driver Interface
 *
 *****************************************************************************
 */
static int ds1375_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	struct ds1375_data *data;
	int rc = 0;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_I2C))
		return 0;

	if (!(data = kzalloc(sizeof(struct ds1375_data), GFP_KERNEL))) {
		return -ENOMEM;
	}
	client = &data->client;
	i2c_set_clientdata(client, data);
	client->addr = addr;
	client->adapter = adap;
	client->driver = &ds1375_driver;
	strncpy(client->name, DS1375_DRV_NAME, I2C_NAME_SIZE);
	client->flags = 0;

	/*
	 * Now we do the remaining detection. A negative kind means that
	 * the driver was loaded with no force parameter (default), so we
	 * must both detect and identify the chip. A zero kind means that
	 * the driver was loaded with the force parameter, the detection
	 * step shall be skipped. A positive kind means that the driver
	 * was loaded with the force parameter and a given kind of chip is
	 * requested, so both the detection and the identification steps
	 * are skipped.
	 *
	 * For detection, we read registers that are most likely to cause
	 * detection failure, i.e. those that have more bits with fixed
	 * or reserved values.
	 */

	/* Default to an DS1375 if forced */
	if (kind == 0)
		kind = ds1375;

	if (kind < 0) {
		u8 data;

		/* Check for a valid day register value */
		if ((ds1375_read(client, DS1375_DAY_REGISTER, &data) < 0) ||
		    (data == 0) || (data & 0xf8))
			goto exit_free;

		/* Check for a valid date register value */
		if ((ds1375_read(client, DS1375_DATE_REGISTER, &data) < 0) ||
		    (data == 0) || (data & 0xc0) || ((data & 0x0f) > 9) ||
		    (data >= 0x32))
			goto exit_free;

		/* Check for a valid month register value */
		if ((ds1375_read(client, DS1375_MONTH_REGISTER, &data) < 0) ||
		    (data == 0) || (data & 0x60) || ((data & 0x0f) > 9) ||
		    ((data >= 0x13) && (data <= 0x19)))
			goto exit_free;

		kind = ds1375;
	}

	ds1375_workqueue = create_singlethread_workqueue("ds1375");
	if (!ds1375_workqueue) {
		kfree(data);
		return -ENOMEM;	/* most expected reason */
	}

	if ((rc = i2c_attach_client(client)) != 0)
		goto exit_free;

	save_client = client;

	ds1375_init_client(client);

	return 0;

exit_free:
	kfree(data);
	return rc;
}

static int ds1375_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, ds1375_probe);
}

static int ds1375_detach(struct i2c_client *client)
{
	int rc;

	if ((rc = i2c_detach_client(client)) == 0) {
		kfree(i2c_get_clientdata(client));
		destroy_workqueue(ds1375_workqueue);
	}
	return rc;
}

static struct i2c_driver ds1375_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DS1375_DRV_NAME,
	},
	.id = I2C_DRIVERID_DS1375,
	.attach_adapter = ds1375_attach,
	.detach_client = ds1375_detach,
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
