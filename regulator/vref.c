/*
 * Vref consumer driver for replicator unicron board
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <linux/stepper.h>

static int debug = 0;
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "enable debug");

#define vref_info(fmt, args...)  \
    if (debug) printk(KERN_EMERG "[vref]:" fmt,##args)
#define vref_err(fmt, args...)  \
    printk(KERN_ERR "[vref] Err:" fmt,##args)
#define vref_info_enter(fmt,...) \
    vref_info("[vref]: Enter func:%s\n", __func__);
#define lmsw_info_leave(fmt,...) \
    vref_info("[vref]: Leave func:%s\n", __func__);

struct vref_consumer_data {
	struct mutex lock;
	struct regulator *regulator;
	bool enabled;
    int current_uV;
    int target_uV;
	unsigned int mode;
};

static void update_voltage_constraints(struct device *dev,
				       struct vref_consumer_data *data)
{
	int ret;
    
    if (data->current_uV != data->target_uV) {
        vref_info("Requesting %duV\n", data->target_uV);

        //TODO: Change Vref to ad5300 voltage

        ret = regulator_set_voltage(data->regulator,
                    data->target_uV, data->target_uV);
        if (ret != 0) {
            vref_err("regulator_set_voltage() failed: %d\n", ret);
            return;
        }
        data->current_uV = data->target_uV;
    }

	if (data->target_uV && !data->enabled) {
		vref_info("Enabling regulator\n");
		ret = regulator_enable(data->regulator);
		if (ret == 0) {
			data->enabled = true;
        } else {
			vref_err("regulator_enable() failed: %d\n", ret);
        }
	}

	if (!(data->target_uV) && data->enabled) {
		vref_info("Disabling regulator\n");
		ret = regulator_disable(data->regulator);
		if (ret == 0) {
			data->enabled = false;
        } else {
			vref_err("regulator_disable() failed: %d\n", ret);
        }
	}
}

static ssize_t show_voltage(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct vref_consumer_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->current_uV);
}

static ssize_t set_voltage(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct vref_consumer_data *data = dev_get_drvdata(dev);
	long val;

	if (strict_strtol(buf, 10, &val) != 0)
		return count;

	mutex_lock(&data->lock);

	data->target_uV = val;
	update_voltage_constraints(dev, data);

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_enable(struct device *dev, 
        struct device_attribute *attr, char *buf)
{
	struct vref_consumer_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->enabled);
}

static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
    int ret;
	long val;
	struct vref_consumer_data *data = dev_get_drvdata(dev);

	if (strict_strtol(buf, 10, &val) != 0)
		return count;

	mutex_lock(&data->lock);
    
    if (val) {
        if (!data->enabled) {
            ret = regulator_enable(data->regulator);
            if (ret == 0) {
                data->enabled = true;
            }
        }
    } else {
        if (data->enabled) {
            ret = regulator_disable(data->regulator);
            if (ret == 0) {
                data->enabled = false;
            }
        }
    }

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_mode(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct vref_consumer_data *data = dev_get_drvdata(dev);

	switch (data->mode) {
	case REGULATOR_MODE_FAST:
		return sprintf(buf, "fast\n");
	case REGULATOR_MODE_NORMAL:
		return sprintf(buf, "normal\n");
	case REGULATOR_MODE_IDLE:
		return sprintf(buf, "idle\n");
	case REGULATOR_MODE_STANDBY:
		return sprintf(buf, "standby\n");
	default:
		return sprintf(buf, "unknown\n");
	}
}

static ssize_t set_mode(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct vref_consumer_data *data = dev_get_drvdata(dev);
	unsigned int mode;
	int ret;

	/*
	 * sysfs_streq() doesn't need the \n's, but we add them so the strings
	 * will be shared with show_mode(), above.
	 */
	if (sysfs_streq(buf, "fast\n"))
		mode = REGULATOR_MODE_FAST;
	else if (sysfs_streq(buf, "normal\n"))
		mode = REGULATOR_MODE_NORMAL;
	else if (sysfs_streq(buf, "idle\n"))
		mode = REGULATOR_MODE_IDLE;
	else if (sysfs_streq(buf, "standby\n"))
		mode = REGULATOR_MODE_STANDBY;
	else {
		vref_err("Configuring invalid mode\n");
		return count;
	}

	mutex_lock(&data->lock);
	ret = regulator_set_mode(data->regulator, mode);
	if (ret == 0) {
		data->mode = mode;
    } else {
		vref_err("Failed to configure mode: %d\n", ret);
    }
	mutex_unlock(&data->lock);

	return count;
}

static DEVICE_ATTR(microvolts, 0666, show_voltage, set_voltage);
static DEVICE_ATTR(enable, 0666, show_enable, set_enable);
static DEVICE_ATTR(mode, 0666, show_mode, set_mode);

static struct attribute *vref_consumer_attributes[] = {
	&dev_attr_microvolts.attr,
	&dev_attr_enable.attr,
	&dev_attr_mode.attr,
	NULL
};

static const struct attribute_group vref_consumer_attr_group = {
	.attrs	= vref_consumer_attributes,
};

static int __devinit vref_consumer_probe(struct platform_device *pdev)
{
    struct vref_platform_data *pdata = pdev->dev.platform_data;
	struct vref_consumer_data *drvdata;
	int ret;
    
	drvdata = kzalloc(sizeof(struct vref_consumer_data), GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	mutex_init(&drvdata->lock);

    vref_info("platform device name %s\n", pdata->name);

	drvdata->regulator = regulator_get(&pdev->dev, pdata->name);
	if (IS_ERR(drvdata->regulator)) {
		ret = PTR_ERR(drvdata->regulator);
		vref_err("Failed to obtain supply '%s': %d\n",
			pdata->name, ret);
		goto err;
	}

	ret = sysfs_create_group(&pdev->dev.kobj,
				 &vref_consumer_attr_group);
	if (ret != 0) {
		vref_err("Failed to create attribute group: %d\n", ret);
		goto err_regulator;
	}

	drvdata->mode = regulator_get_mode(drvdata->regulator);

	platform_set_drvdata(pdev, drvdata);

#ifdef DEBUG 
    vref_info("vref probe ok\n");
#endif
	return 0;

err_regulator:
	regulator_put(drvdata->regulator);
err:
	kfree(drvdata);
	return ret;
}

static int __devexit vref_consumer_remove(struct platform_device *pdev)
{
	struct vref_consumer_data *drvdata = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &vref_consumer_attr_group);

	if (drvdata->enabled)
		regulator_disable(drvdata->regulator);
	regulator_put(drvdata->regulator);

	kfree(drvdata);

	platform_set_drvdata(pdev, NULL);

    vref_info("vref remove ok\n");

	return 0;
}

static struct platform_driver vref_consumer_x = {
	.probe		= vref_consumer_probe,
	.remove		= __devexit_p(vref_consumer_remove),
	.driver		= {
		.name		= "vref_consumer_x",
		.owner		= THIS_MODULE,
	},
};

static struct platform_driver vref_consumer_y = {
	.probe		= vref_consumer_probe,
	.remove		= __devexit_p(vref_consumer_remove),
	.driver		= {
		.name		= "vref_consumer_y",
		.owner		= THIS_MODULE,
	},
};

static struct platform_driver vref_consumer_z = {
	.probe		= vref_consumer_probe,
	.remove		= __devexit_p(vref_consumer_remove),
	.driver		= {
		.name		= "vref_consumer_z",
		.owner		= THIS_MODULE,
	},
};

static struct platform_driver vref_consumer_ext1 = {
	.probe		= vref_consumer_probe,
	.remove		= __devexit_p(vref_consumer_remove),
	.driver		= {
		.name		= "vref_consumer_ext1",
		.owner		= THIS_MODULE,
	},
};

static struct platform_driver vref_consumer_ext2 = {
	.probe		= vref_consumer_probe,
	.remove		= __devexit_p(vref_consumer_remove),
	.driver		= {
		.name		= "vref_consumer_ext2",
		.owner		= THIS_MODULE,
	},
};

static struct platform_driver vref_consumer_ext3 = {
	.probe		= vref_consumer_probe,
	.remove		= __devexit_p(vref_consumer_remove),
	.driver		= {
		.name		= "vref_consumer_ext3",
		.owner		= THIS_MODULE,
	},
};

static struct platform_driver vref_consumer_ext4 = {
	.probe		= vref_consumer_probe,
	.remove		= __devexit_p(vref_consumer_remove),
	.driver		= {
		.name		= "vref_consumer_ext4",
		.owner		= THIS_MODULE,
	},
};

static struct platform_driver vref_consumer_ext5 = {
	.probe		= vref_consumer_probe,
	.remove		= __devexit_p(vref_consumer_remove),
	.driver		= {
		.name		= "vref_consumer_ext5",
		.owner		= THIS_MODULE,
	},
};

static int __init vref_consumer_init(void)
{
	platform_driver_register(&vref_consumer_x);
	platform_driver_register(&vref_consumer_y);
	platform_driver_register(&vref_consumer_z);
	platform_driver_register(&vref_consumer_ext1);
	platform_driver_register(&vref_consumer_ext2);
	platform_driver_register(&vref_consumer_ext3);
	platform_driver_register(&vref_consumer_ext4);
	platform_driver_register(&vref_consumer_ext5);
    return 0;
}
module_init(vref_consumer_init);

static void __exit vref_consumer_exit(void)
{
	platform_driver_unregister(&vref_consumer_x);
	platform_driver_unregister(&vref_consumer_y);
	platform_driver_unregister(&vref_consumer_z);
	platform_driver_unregister(&vref_consumer_ext1);
	platform_driver_unregister(&vref_consumer_ext2);
	platform_driver_unregister(&vref_consumer_ext3);
	platform_driver_unregister(&vref_consumer_ext4);
	platform_driver_unregister(&vref_consumer_ext5);
}
module_exit(vref_consumer_exit);

MODULE_AUTHOR("Truby Zong <truby.zong@gmail.com>");
MODULE_DESCRIPTION("Replicator Vref consumer driver");
MODULE_LICENSE("GPL");
