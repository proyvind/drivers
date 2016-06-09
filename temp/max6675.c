/*
 * Max6675 driver for BBP board
 *
 * Copyright (c) 2014 Truby Zong <truby.zong@gmail.com>
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

static int debug = 0;
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "enable debug");

#define max_info(fmt, args...)  \
    if (debug) printk(KERN_EMERG "[max6675]:" fmt,##args)
#define max_err(fmt, args...)  \
    printk(KERN_ERR "[max6675] Err:" fmt,##args)
#define st_info_enter(fmt,...) \
    st_info("[max6675]: Enter func:%s\n", __func__);
#define lmsw_info_leave(fmt,...) \
    st_info("[max6675]: Leave func:%s\n", __func__);

#define TX_BUF_SIZE 2
#define RX_BUF_SIZE 2

struct max6675_data {
    struct mutex        lock;
    struct spi_device   *spi;
    struct device       *hwmon_dev;
    struct spi_message  msg;
    struct spi_transfer xfer;
    uint8_t tx_buf[TX_BUF_SIZE];
    uint8_t rx_buf[RX_BUF_SIZE];
};

static int max6675_read(struct device *dev)
{
    unsigned int val = 0;
	unsigned int ret = 0;
    struct spi_message msg;
    struct spi_transfer xfer; 
    
    struct max6675_data *data = dev_get_drvdata(dev);

    mutex_lock(&data->lock);

    u16 rx_buf = 0;
    xfer.rx_buf = &rx_buf;
    xfer.tx_buf = NULL;
    xfer.len    = 2;
    xfer.speed_hz = 1000000;
    xfer.bits_per_word = 16;
    spi_message_init(&msg);

    spi_message_add_tail(&xfer, &msg);
    if (data->spi == NULL){
		max_err("spi device instance is NULL\n");
    	mutex_unlock(&data->lock);
        return 0;
	}

    ret = spi_sync(data->spi, &msg);
    val = (rx_buf >> 3) & 0xFFF;

    max_info("rx_buf:0x%x\n", rx_buf);
    max_info("temp:0x%x, %d\n", val, val/4);

    mutex_unlock(&data->lock);

    return val;
}

static ssize_t show_name(struct device *dev, 
                         struct device_attribute *attr,
                         char *buf)
{
    return sprintf(buf, "max6675\n");
}

static ssize_t show_adc(struct device *dev,
                        struct device_attribute *attr,
                        char *buf)
{
    int ret;
    ret = max6675_read(dev);
    if (ret < 0) {
        return ret;
    }
    return sprintf(buf, "%d\n", ret);
}

static SENSOR_DEVICE_ATTR(vout, S_IRUGO, show_adc, NULL, 0);
static SENSOR_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);

static struct attribute *max6675_attributes[] = {
    &sensor_dev_attr_vout.dev_attr.attr,
    &sensor_dev_attr_name.dev_attr.attr,
    NULL,
};

static const struct attribute_group max6675_attr_group = {
    .attrs = max6675_attributes, 
};

#ifdef CONFIG_PM
static int max6675_suspend(struct spi_device *spi, pm_message_t state)
{
    return 0;
}

static int max6675_resume(struct spi_device *spi)
{
    return 0;
}
#else
#define max6675_suspend NULL
#define max6675_resume NULL
#endif

static int max6675_probe(struct spi_device *spi)
{
    int ret;
    struct max6675_data *data; 
    
    spi->bits_per_word = 16;
    spi->mode = SPI_MODE_0;
	spi->max_speed_hz  = 10000000;
    ret = spi_setup(spi);
    if (ret < 0) {
        max_err("spi_setup failed\n");
        return ret;
    }
    
    data = kzalloc(sizeof(struct max6675_data), GFP_KERNEL);
    if (data == NULL) { 
        max_err("failed to allocate memory\n");
        return -ENOMEM;
    }

    mutex_init(&data->lock);
    data->spi = spi;
    spi_set_drvdata(spi, data);
    
    ret = sysfs_create_group(&spi->dev.kobj, &max6675_attr_group);
    if (ret) { 
        max_err("failed to create attribute group\n");
        goto free_data;
    }

    data->hwmon_dev = hwmon_device_register(&spi->dev);
    if (IS_ERR(data->hwmon_dev)) {
        max_err("failed to create hwmon device\n");
        ret = PTR_ERR(data->hwmon_dev);
        goto remove;
    }

	max_info("%s: sensor\n", dev_name(data->hwmon_dev));
/*
*/

    return 0;

remove:
    sysfs_remove_group(&spi->dev.kobj, &max6675_attr_group);
free_data:
    kfree(data);
    return ret;
}

static int max6675_remove(struct spi_device *spi)
{
    struct max6675_data *data = spi_get_drvdata(spi);

    hwmon_device_unregister(data->hwmon_dev);
    sysfs_remove_group(&spi->dev.kobj, &max6675_attr_group); 
    mutex_destroy(&data->lock);
    kfree(data);
    return 0;
}

static struct spi_driver max6675_driver = {
    .driver = {
        .name  = "max6675",
        .bus   = &spi_bus_type,
        .owner = THIS_MODULE,
    },
    .probe   = max6675_probe,
    .remove  = max6675_remove,
    .suspend = max6675_suspend,
    .resume  = max6675_resume,
};

static int __init max6675_init(void)
{
    return spi_register_driver(&max6675_driver);
}
module_init(max6675_init);

static void __exit max6675_exit(void)
{
    spi_unregister_driver(&max6675_driver);
}
module_exit(max6675_exit);

MODULE_AUTHOR("Truby Zong <truby.zong@gmail.com>");
MODULE_DESCRIPTION("Max6675 thermocouple driver");
MODULE_LICENSE("GPL");
