/*
    ds1621.c - Part of lm_sensors, Linux kernel modules for hardware
             monitoring
    Christian W. Zuckschwerdt  <zany@triq.net>  2000-11-23
    based on lm75.c by Frodo Looijaard <frodol@dds.nl>
    Ported to Linux 2.6 by Aurelien Jarno <aurelien@aurel32.net> with 
    the help of Jean Delvare <khali@linux-fr.org>

    Added board specific interrupt support for the MVME3100, MVME5500, 
    MVME6100, and MCP905 boards (Ajit Prem <Ajit.Prem@motorola.com)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/hwmon.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <asm/irq.h>
#ifdef CONFIG_MVME3100
#include <platforms/85xx/mvme3100.h>
#endif
#ifdef CONFIG_MVME5500
#include <platforms/mvme5500.h>
#endif
#ifdef CONFIG_MVME6100
#include <platforms/mvme6100.h>
#endif
#ifdef CONFIG_MCP905
#include <platforms/mcp905.h>
#endif
#include <asm/io.h>
#include "lm75.h"

/* Addresses to scan */
static unsigned short normal_i2c[] = { 0x48, 0x49, 0x4a, 0x4b, 0x4c,
					0x4d, 0x4e, 0x4f, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(ds1621);
static int polarity = 0;
module_param(polarity, int, 0);
MODULE_PARM_DESC(polarity, "Output's polarity: 0 = active high, 1 = active low");

/* Many DS1621 constants specified below */
/* Config register used for detection         */
/*  7    6    5    4    3    2    1    0      */
/* |Done|THF |TLF |NVB | X  | X  |POL |1SHOT| */
#define DS1621_REG_CONFIG_MASK          0x0C
#define DS1621_REG_CONFIG_VAL           0x0C
#define DS1621_REG_CONFIG_NVB		0x10
#define DS1621_REG_CONFIG_POLARITY	0x02
#define DS1621_REG_CONFIG_1SHOT		0x01
#define DS1621_REG_CONFIG_DONE		0x80

/* The DS1621 registers */
#define DS1621_REG_TEMP			0xAA /* word, RO */
#define DS1621_REG_TEMP_MAX             0xA1 /* word, RW */
#define DS1621_REG_TEMP_MIN             0xA2 /* word, RW */
#define DS1621_REG_CONF			0xAC /* byte, RW */
#define DS1621_REG_TEMP_COUNTER         0xA8 /* byte, RO */
#define DS1621_REG_TEMP_SLOPE           0xA9 /* byte, RO */
#define DS1621_COM_START		0xEE /* no data */
#define DS1621_COM_STOP			0x22 /* no data */

/* The DS1621 configuration register */
#define DS1621_ALARM_TEMP_HIGH		0x40
#define DS1621_ALARM_TEMP_LOW		0x20

#define ALARMS_FROM_REG(val) ((val) & \
                              (DS1621_ALARM_TEMP_HIGH | DS1621_ALARM_TEMP_LOW))


/* Each client has this additional data */
struct ds1621_data {
	struct i2c_client client;
	struct class_device *class_dev;
	struct mutex update_lock;
	char valid;			/* !=0 if following fields are valid */
	unsigned long last_updated;	/* In jiffies */

	u16 temp, temp_max, temp_min;	/* Register values, word */
	u8 conf;			/* Register encoding, combined */
        char enable;    /* !=0 if we're expected to restart the conversion */
        u8 temp_int, temp_counter, temp_slope;  /* Register values, byte */
#if defined(CONFIG_MVME3100) || defined(CONFIG_MVME5500) || defined(CONFIG_MVME6100) || defined(CONFIG_MCP905)
        struct work_struct tqueue;
#endif
};

static int ds1621_attach_adapter(struct i2c_adapter *adapter);
static int ds1621_detect(struct i2c_adapter *adapter, int address,
			 int kind);
static void ds1621_init_client(struct i2c_client *client);
static int ds1621_detach_client(struct i2c_client *client);
static struct ds1621_data *ds1621_update_client(struct device *dev);

/* This is the driver that will be inserted */
static struct i2c_driver ds1621_driver = {
	.driver = {
		.name	= "ds1621",
	},
	.id		= I2C_DRIVERID_DS1621,
	.attach_adapter	= ds1621_attach_adapter,
	.detach_client	= ds1621_detach_client,
};

static inline s32
i2c_smbus_read_word_swapped(struct i2c_client *client, u8 command)
{
        s32 value = i2c_smbus_read_word_data(client, command);

        return (value < 0) ? value : swab16(value);
}

static inline s32
i2c_smbus_write_word_swapped(struct i2c_client *client,
                             u8 command, u16 value)
{
        return i2c_smbus_write_word_data(client, command, swab16(value));
}

/* All registers are word-sized, except for the configuration register. */
static int ds1621_read_value(struct i2c_client *client, u8 reg)
{
        if ((reg == DS1621_REG_CONF) || (reg == DS1621_REG_TEMP_COUNTER)
            || (reg == DS1621_REG_TEMP_SLOPE))
		return i2c_smbus_read_byte_data(client, reg);
	else
		return i2c_smbus_read_word_swapped(client, reg);
}

/* All registers are word-sized, except for the configuration register. */
static int ds1621_write_value(struct i2c_client *client, u8 reg, u16 value)
{
        if ( (reg == DS1621_COM_START) || (reg == DS1621_COM_STOP) )
                return i2c_smbus_write_byte(client, reg);
        else
        if ((reg == DS1621_REG_CONF) || (reg == DS1621_REG_TEMP_COUNTER)
            || (reg == DS1621_REG_TEMP_SLOPE))
                return i2c_smbus_write_byte_data(client, reg, value);
        else
                return i2c_smbus_write_word_swapped(client, reg, value);
}

#ifdef CONFIG_MVME3100

#define DS1621_INTERRUPT        57
#define BOARD_TSTAT_MASK        MVME3100_TSTAT_MASK
#define BOARD_TSTAT_REG		MVME3100_SYSTEM_CONTROL_REG

#endif

#ifdef CONFIG_MVME5500

#define DS1621_INTERRUPT        67
#define BOARD_TSTAT_MASK        MVME5500_BOARD_TSTAT_MASK
#define BOARD_TSTAT_REG		MVME5500_BOARD_STATUS_REG_2

#endif

#ifdef CONFIG_MVME6100

#define DS1621_INTERRUPT        67
#define BOARD_TSTAT_MASK        MVME6100_BOARD_TSTAT_MASK
#define BOARD_TSTAT_REG		MVME6100_BOARD_STATUS_REG_2

#endif

#ifdef CONFIG_MCP905

#define DS1621_INTERRUPT        67
#define BOARD_TSTAT_MASK        MCP905_BOARD_TSTAT_MASK
#define BOARD_TSTAT_REG		MCP905_BOARD_STATUS_REG_2

#endif

#if defined(CONFIG_MVME3100) || defined(CONFIG_MVME5500) || defined(CONFIG_MVME6100) || defined(CONFIG_MCP905)
                                                                                
static void __iomem *control_reg_mapped_addr;
static int control_reg_mapped;

static void ds1621_softint(struct work_struct *work)
{
        s32 conf;
        struct ds1621_data *data = container_of(work, struct ds1621_data, tqueue);
        struct i2c_client *client = &data->client ;
                                                                                
        data = ds1621_update_client(&client->dev);
                                                                                
        printk(KERN_ERR "ds1621: the current temperature is %d C\n", LM75_TEMP_FROM_REG(data->temp)/1000);
        printk(KERN_ERR "ds1621: the high temperature trip register (TH) is set to %d C\n", LM75_TEMP_FROM_REG(data->temp_max)/1000);
        printk(KERN_ERR "ds1621: the low temperature trigger register (TL) is set to %d C\n", LM75_TEMP_FROM_REG(data->temp_min)/1000);
                                                                                
        if ((conf = i2c_smbus_read_byte_data(client,DS1621_REG_CONF)) < 0) {
                dev_warn(&client->dev, "Cannot read DS1621_REG_CONF\n");
                return;
        }
        printk(KERN_ERR "The ds1621 config register is: 0x%x\n", conf);
        return;
}
                                                                                
                                                                                
static irqreturn_t ds1621_int_handler(int irq, void *dev_id)
{
        u16 value;
        struct i2c_client *client = (struct i2c_client *)dev_id;
        struct ds1621_data *data = i2c_get_clientdata(client);
                                                                                
        printk(KERN_ERR "ds1621: Interrupt!\n");
        printk(KERN_ERR "Masking off ds1621 interrupt.\n");
                                                                                
        value = readb(control_reg_mapped_addr);
        value |= BOARD_TSTAT_MASK;
        writeb(value, control_reg_mapped_addr);

        schedule_work(&data->tqueue);
                                                                                
        return IRQ_HANDLED;
}
                                                                                
#endif

static void ds1621_init_client(struct i2c_client *client)
{
	int reg = ds1621_read_value(client, DS1621_REG_CONF);
        struct ds1621_data *data = i2c_get_clientdata(client);
        int result;
        int value;

	/* switch to continuous conversion mode */
	reg &= ~ DS1621_REG_CONFIG_1SHOT;

	/* setup output polarity */
	if (polarity == 0)
		reg &= ~DS1621_REG_CONFIG_POLARITY;
	else if (polarity == 1)
		reg |= DS1621_REG_CONFIG_POLARITY;
	
	ds1621_write_value(client, DS1621_REG_CONF, reg);
	
	/* start conversion */
	i2c_smbus_write_byte(client, DS1621_COM_START);
#if defined(CONFIG_MVME3100) || defined(CONFIG_MVME5500) || defined(CONFIG_MVME6100) || defined(CONFIG_MCP905)
        INIT_WORK(&data->tqueue, ds1621_softint);
                                                                                
        control_reg_mapped_addr = ioremap(BOARD_TSTAT_REG, 1);
        control_reg_mapped = 1;

        result = request_irq(DS1621_INTERRUPT, ds1621_int_handler,
                                IRQF_DISABLED, "ds1621", client);
        if (result) {
                printk(KERN_ERR "ds1621: Cannot get irq %d\n", DS1621_INTERRUPT);
                return;
        }
                                                                                
        /* Unmask ds1621 temperature sensor thermostat output */
       	value = readb(control_reg_mapped_addr);
       	value &= ~BOARD_TSTAT_MASK;
       	writeb(value, control_reg_mapped_addr);
#endif
}

static ssize_t show_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct ds1621_data *data = ds1621_update_client(dev);
                                                                                
        return sprintf(buf, "%d\n",
                       LM75_TEMP_FROM_REG(data->temp));
}

static ssize_t show_temp_max(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct ds1621_data *data = ds1621_update_client(dev);
                                                                                
        return sprintf(buf, "%d\n",
                       LM75_TEMP_FROM_REG(data->temp_max));
}

static ssize_t show_temp_min(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct ds1621_data *data = ds1621_update_client(dev);
                                                                                
        return sprintf(buf, "%d\n",
                       LM75_TEMP_FROM_REG(data->temp_min));
}

static ssize_t set_temp_max(struct device *dev, struct device_attribute *attr, const char *buf,	size_t count)				
{									
	struct i2c_client *client = to_i2c_client(dev);			
        struct ds1621_data *data = i2c_get_clientdata(client);		
	long val;

	val = simple_strtol(buf, NULL, 10);	
									
	mutex_lock(&data->update_lock);					
        data->temp_max = LM75_TEMP_TO_REG(val);
        i2c_smbus_write_word_swapped(client, DS1621_REG_TEMP_MAX,
                                     data->temp_max);
	mutex_unlock(&data->update_lock);				
	return count;							
}

static ssize_t set_temp_min(struct device *dev, struct device_attribute *attr, const char *buf,	size_t count)				
{									
	struct i2c_client *client = to_i2c_client(dev);			
        struct ds1621_data *data = i2c_get_clientdata(client);		
	long val;

	val = simple_strtol(buf, NULL, 10);	
									
	mutex_lock(&data->update_lock);					
        data->temp_min = LM75_TEMP_TO_REG(val);
        i2c_smbus_write_word_swapped(client, DS1621_REG_TEMP_MIN,
                                     data->temp_min);
	mutex_unlock(&data->update_lock);				
	return count;							
}


static ssize_t show_alarms(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds1621_data *data = ds1621_update_client(dev);
	return sprintf(buf, "%d\n", ALARMS_FROM_REG(data->conf));
}

static ssize_t show_continuous(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct ds1621_data *data = ds1621_update_client(dev);
                                                                                
        return sprintf(buf, "%d\n", !(data->conf & DS1621_REG_CONFIG_1SHOT));
}
                                                                                
static ssize_t set_continuous(struct device *dev, struct device_attribute *attr, const char *buf,
                                 size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct ds1621_data *data = ds1621_update_client(dev);
        ulong   value;
                                                                                
        value = simple_strtoul(buf, NULL, 10);
        if (value == 1)
                ds1621_write_value(client, DS1621_REG_CONF,
                        data->conf & ~DS1621_REG_CONFIG_1SHOT);
        else
                ds1621_write_value(client, DS1621_REG_CONF,
                        data->conf | DS1621_REG_CONFIG_1SHOT);
        return count;
}
                                                                                
static ssize_t show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct ds1621_data *data = ds1621_update_client(dev);
                                                                                
        return sprintf(buf, "%d\n", !(data->conf & DS1621_REG_CONFIG_DONE));
}
                                                                                
static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t count)
                                                                                
{
        struct i2c_client *client = to_i2c_client(dev);
        struct ds1621_data *data = i2c_get_clientdata(client);
        ulong value;
                                                                                
        value = simple_strtoul(buf, NULL, 10);
        if (value == 1) {
                ds1621_write_value(client, DS1621_COM_START, 0);
                data->enable = 1;
        } else {
                ds1621_write_value(client, DS1621_COM_STOP, 0);
                data->enable = 0;
        }
        return count;                                                   \
}
                                                                                
static ssize_t show_polarity(struct device *dev, struct device_attribute *attr,
char *buf)
{
        struct ds1621_data *data = ds1621_update_client(dev);
                                                                                
        return sprintf(buf, "%d\n", !(!(data->conf & DS1621_REG_CONFIG_POLARITY)));
}
                                                                                
static ssize_t set_polarity(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct ds1621_data *data = i2c_get_clientdata(client);
        ulong value;
                                                                                
        value = simple_strtoul(buf, NULL, 10);
        if (value == 1)
                ds1621_write_value(client, DS1621_REG_CONF,
                                data->conf | DS1621_REG_CONFIG_POLARITY);
         else
                ds1621_write_value(client, DS1621_REG_CONF,
                                data->conf & ~DS1621_REG_CONFIG_POLARITY);
        return count;                                                   \
}
                                                                                
#if defined(CONFIG_MVME3100) || defined(CONFIG_MVME5500) || defined(CONFIG_MVME6100) || defined(CONFIG_MCP905)
                                                                                
static ssize_t show_irq_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        u8      cr;
                                                                                
        cr = readb(control_reg_mapped_addr);
        return sprintf(buf, "%d\n", (!(cr & BOARD_TSTAT_MASK)));
}
                                                                                
static ssize_t set_irq_enable(struct device *dev, struct device_attribute *attr,                                 const char *buf, size_t count)
{
        ulong value;
        u8  cr;
                                                                                
        value = simple_strtoul(buf, NULL, 10);
        cr = readb(control_reg_mapped_addr);
        if (value == 1)  {
                cr &= ~BOARD_TSTAT_MASK;
        } else {
                cr |= BOARD_TSTAT_MASK;
        }
        writeb(cr, control_reg_mapped_addr);
        return count;
}
                                                                                
static DEVICE_ATTR(irq_enable, S_IWUSR | S_IRUGO, show_irq_enable, set_irq_enable);
                                                                                
#endif

static DEVICE_ATTR(alarms, S_IRUGO, show_alarms, NULL);
static DEVICE_ATTR(continuous, S_IWUSR | S_IRUGO, show_continuous, set_continuous);
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, show_enable, set_enable);
static DEVICE_ATTR(polarity, S_IWUSR | S_IRUGO, show_polarity, set_polarity);
static DEVICE_ATTR(temp1_input, S_IRUGO , show_temp, NULL);
static DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO , show_temp_max, set_temp_max);
static DEVICE_ATTR(temp1_min, S_IWUSR | S_IRUGO, show_temp_min, set_temp_min);

static struct attribute *ds1621_attributes[] = {
	&dev_attr_temp1_min.attr,
	&dev_attr_temp1_max.attr,
	&dev_attr_temp1_input.attr,
	&dev_attr_polarity.attr,
	&dev_attr_enable.attr,
	&dev_attr_continuous.attr,
	&dev_attr_alarms.attr,
#if defined(CONFIG_MVME3100) || defined(CONFIG_MVME5500) || defined(CONFIG_MVME6100) || defined(CONFIG_MCP905)
	&dev_attr_irq_enable.attr,
#endif
	NULL
};

static const struct attribute_group ds1621_group = {
	.attrs = ds1621_attributes,
};


static int ds1621_attach_adapter(struct i2c_adapter *adapter)
{
	if (!(adapter->class & I2C_CLASS_HWMON))
		return 0;
	return i2c_probe(adapter, &addr_data, ds1621_detect);
}

/* This function is called by i2c_probe */
static int ds1621_detect(struct i2c_adapter *adapter, int address,
			 int kind)
{
	int conf, temp;
	struct i2c_client *new_client;
	struct ds1621_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA 
				     | I2C_FUNC_SMBUS_WORD_DATA 
				     | I2C_FUNC_SMBUS_WRITE_BYTE))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet.
	   But it allows us to access ds1621_{read,write}_value. */
	if (!(data = kzalloc(sizeof(struct ds1621_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	
	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &ds1621_driver;
	new_client->flags = 0;


	/* Now, we do the remaining detection. It is lousy. */
	if (kind < 0) {
		/* The NVB bit should be low if no EEPROM write has been 
		   requested during the latest 10ms, which is highly 
		   improbable in our case. */
		conf = ds1621_read_value(new_client, DS1621_REG_CONF);
		if (conf & DS1621_REG_CONFIG_NVB)
			goto exit_free;
		/* The 7 lowest bits of a temperature should always be 0. */
		temp = ds1621_read_value(new_client, DS1621_REG_TEMP);
		if (temp & 0x007f)
			goto exit_free;
		temp = ds1621_read_value(new_client, DS1621_REG_TEMP_MAX);
		if (temp & 0x007f)
			goto exit_free;
		temp = ds1621_read_value(new_client, DS1621_REG_TEMP_MIN);
		if (temp & 0x007f)
			goto exit_free;
	}

	/* Determine the chip type - only one kind supported! */
	if (kind <= 0)
		kind = ds1621;

	/* Fill in remaining client fields and put it into the global list */
	strlcpy(new_client->name, "ds1621", I2C_NAME_SIZE);
	data->valid = 0;
	mutex_init(&data->update_lock);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_free;

	/* Initialize the DS1621 chip */
	ds1621_init_client(new_client);

	/* Register sysfs hooks */
	if ((err = sysfs_create_group(&new_client->dev.kobj, &ds1621_group)))
		goto exit_detach;

	data->class_dev = hwmon_device_register(&new_client->dev);
	if (IS_ERR(data->class_dev)) {
		err = PTR_ERR(data->class_dev);
		goto exit_remove_files;
	}
	data = ds1621_update_client(&new_client->dev);
	printk("ds1621: the current temperature is %d C\n", LM75_TEMP_FROM_REG(data->temp)/1000);

	return 0;

      exit_remove_files:
	sysfs_remove_group(&new_client->dev.kobj, &ds1621_group);
      exit_detach:
	i2c_detach_client(new_client);
      exit_free:
	kfree(data);
      exit:
	return err;
}

static int ds1621_detach_client(struct i2c_client *client)
{
	struct ds1621_data *data = i2c_get_clientdata(client);
	int err;
	u8  value;

	hwmon_device_unregister(data->class_dev);
	sysfs_remove_group(&client->dev.kobj, &ds1621_group);

	if ((err = i2c_detach_client(client)))
		return err;
#if defined(CONFIG_MVME3100) || defined(CONFIG_MVME5500) || defined(CONFIG_MVME6100) || defined(CONFIG_MCP905)
        if (control_reg_mapped) {
                /* Mask DS1621 temperature interrupt */
                value = readb(control_reg_mapped_addr);
                value |= BOARD_TSTAT_MASK;
                writeb(value, control_reg_mapped_addr);
                iounmap(control_reg_mapped_addr);
                control_reg_mapped = 0;
        }
        free_irq(DS1621_INTERRUPT, client);
        flush_scheduled_work();
#endif

	kfree(data);

	return 0;
}


static struct ds1621_data *ds1621_update_client(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds1621_data *data = i2c_get_clientdata(client);
	u8 new_conf;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
	    || !data->valid) {

		dev_dbg(&client->dev, "Starting ds1621 update\n");

		data->conf = ds1621_read_value(client, DS1621_REG_CONF);

		data->temp = ds1621_read_value(client, DS1621_REG_TEMP);
		
		data->temp_max = ds1621_read_value(client,
		                                    DS1621_REG_TEMP_MAX);
		data->temp_min= ds1621_read_value(client,
						    DS1621_REG_TEMP_MIN);

		/* reset alarms if necessary */
		new_conf = data->conf;
		if (data->temp > data->temp_min)     /* input > min */
			new_conf &= ~DS1621_ALARM_TEMP_LOW;
		if (data->temp < data->temp_max)     /* input < max */
			new_conf &= ~DS1621_ALARM_TEMP_HIGH;
		if (data->conf != new_conf)
			ds1621_write_value(client, DS1621_REG_CONF,
					   new_conf);

		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

static int __init ds1621_init(void)
{
	return i2c_add_driver(&ds1621_driver);
}

static void __exit ds1621_exit(void)
{
	i2c_del_driver(&ds1621_driver);
}


MODULE_AUTHOR("Christian W. Zuckschwerdt <zany@triq.net>");
MODULE_DESCRIPTION("DS1621 driver");
MODULE_LICENSE("GPL");

module_init(ds1621_init);
module_exit(ds1621_exit);
