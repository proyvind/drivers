/**
 * lmsw.c - limit switch input event Driver
 *
 * Copyright (C) 2014 Truby Zong <truby.zong@gmail.com>
 *
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/input/lmsw.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/clk.h>

#include <linux/gpio.h>

#include <asm/atomic.h>

static int debug = 0;
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "enable debug");

#define lmsw_info(fmt, args...)  \
    if (debug) printk(KERN_EMERG "[lmsw]:" fmt,##args)
#define lmsw_err(fmt, args...)  \
    printk(KERN_ERR "[lmsw] Err:" fmt,##args)
#define lmsw_info_enter(fmt,...) \
    lmsw_info("[lmsw]: Enter func:%s\n", __func__);
#define lmsw_info_leave(fmt,...) \
    lmsw_info("[lmsw]: Leave func:%s\n", __func__);

struct lmsw_dev {
    struct input_dev *input;
    struct timer_list timer;
    atomic_t checking;
    char *name;
    u32 gpio;
    u32 level;
    u32 value;
};

static int lmsw_request_gpio(struct lmsw_dev *lmsw)
{
    if (gpio_request(lmsw->gpio, lmsw->name) < 0) {
        lmsw_err("failed to request gpio%d for %s\n", 
                lmsw->gpio, lmsw->name);
        return -ENXIO;
    }
    gpio_direction_input(lmsw->gpio);

    return 0;
}

static void lmsw_free_gpio(struct lmsw_dev *lmsw)
{
    gpio_free(lmsw->gpio);
}

static inline int lmsw_check_level(struct lmsw_dev *lmsw)
{
    if (lmsw->level) {
        return gpio_get_value(lmsw->gpio) ? 1 : 0;
    } else {
        return gpio_get_value(lmsw->gpio) ? 0 : 1;
    }
}

static void lmsw_do_timer(unsigned long arg)
{
    struct lmsw_dev *lmsw = (struct lmsw_dev *)arg;
    
	lmsw_info("%s hited\n", lmsw->name);

	input_report_key(lmsw->input, lmsw->value, 1);
	input_report_key(lmsw->input, lmsw->value, 0);
	input_sync(lmsw->input);

    del_timer(&lmsw->timer);
    atomic_set(&lmsw->checking, 1);
}


static irqreturn_t lmsw_irq(int irq, void *pwr)
{
    struct lmsw_dev *lmsw = (struct lmsw_dev *)pwr;

    if (atomic_read(&lmsw->checking)) {
    	atomic_set(&lmsw->checking, 0);
		lmsw->timer.expires = jiffies + HZ / 10;
        add_timer(&lmsw->timer);
    }
    
	return IRQ_HANDLED;
}

static int lmsw_request_irq(struct lmsw_dev *lmsw)
{
    int ret;
    unsigned long flags;
    
		init_timer(&lmsw->timer);
		lmsw->timer.function = lmsw_do_timer;
		lmsw->timer.data = (unsigned long)lmsw;
    if (lmsw->level) {
        flags = IRQF_TRIGGER_RISING & IRQF_TRIGGER_MASK;
    } else {
        flags = IRQF_TRIGGER_FALLING & IRQF_TRIGGER_MASK;
    }

	ret = request_threaded_irq(gpio_to_irq(lmsw->gpio),
                               NULL, 
                               lmsw_irq,
                               flags,
			                   lmsw->name, 
                               lmsw);
	if (ret < 0) {
        lmsw_err("request %s irq err\n", lmsw->name);
	}
    
    return ret;
}

static void lmsw_free_irq(struct lmsw_dev *lmsw)
{
    free_irq(gpio_to_irq(lmsw->gpio), lmsw);
}

static int __init lmsw_probe(struct platform_device *pdev)
{
	int ret;
	struct input_dev *input;
    struct lmsw_dev *lmsw; 
    struct lmsw_platform_data *pdata;

    pdata = pdev->dev.platform_data;
    if (!pdata) {
        lmsw_err("platform data is required!\n");
        return -EINVAL;
    }
    
    lmsw = kzalloc(sizeof(struct lmsw_dev), GFP_KERNEL);
    if (!lmsw) {
        return -ENOMEM;
    }
    atomic_set(&lmsw->checking, 1);
    
    lmsw->gpio  = pdata->gpio;
    lmsw->level = pdata->level; 
    lmsw->value = pdata->value;
    lmsw->name  = kstrdup(pdev->name, GFP_KERNEL);

    ret = lmsw_request_gpio(lmsw);
    if (ret) {
       return ret;
    }

	input = input_allocate_device();
	if (!input) {
		lmsw_err("Can't allocate power button\n");
		return -ENOMEM;
	}

    __set_bit(EV_KEY, input->evbit);
    __set_bit(lmsw->value, input->keybit);

	input->name = lmsw->name;
	input->phys = "gpio lmsw";
    input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

    input->id.bustype = BUS_HOST;
    input->id.vendor  = 0x0522;    
    input->id.product = 0x0502;
    input->id.version = 0x0101;

    if (lmsw_request_irq(lmsw) < 0) {
		lmsw_err("Can't get IRQ for lmsw\n");
        goto out;
    }

	ret = input_register_device(input);
	if (ret) {
		lmsw_err("Can't register lmsw input device: %d\n", ret);
		goto out;
	}

    lmsw->input = input;
	platform_set_drvdata(pdev, lmsw);

	return 0;
out:
	input_free_device(input);
	return ret;
}

static int __exit lmsw_remove(struct platform_device *pdev)
{
    struct lmsw_dev *lmsw = platform_get_drvdata(pdev);
	struct input_dev *input = lmsw->input;
    
    if (lmsw) {
	    lmsw_free_irq(lmsw);
        lmsw_free_gpio(lmsw);

	    input_unregister_device(input);
        kfree(lmsw);
    } 

	return 0;
}

static struct platform_driver lmsw_min_x_driver = {
	.remove		= __exit_p(lmsw_remove),
	.driver		= {
		.name	= "lmsw_min_x",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver lmsw_min_y_driver = {
	.remove		= __exit_p(lmsw_remove),
	.driver		= {
		.name	= "lmsw_min_y",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver lmsw_min_z_driver = {
	.remove		= __exit_p(lmsw_remove),
	.driver		= {
		.name	= "lmsw_min_z",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver lmsw_max_x_driver = {
	.remove		= __exit_p(lmsw_remove),
	.driver		= {
		.name	= "lmsw_max_x",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver lmsw_max_y_driver = {
	.remove		= __exit_p(lmsw_remove),
	.driver		= {
		.name	= "lmsw_max_y",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver lmsw_max_z_driver = {
	.remove		= __exit_p(lmsw_remove),
	.driver		= {
		.name	= "lmsw_max_z",
		.owner	= THIS_MODULE,
	},
};

static int __init lmsw_init(void)
{
	platform_driver_probe(&lmsw_min_x_driver, lmsw_probe);
	platform_driver_probe(&lmsw_min_y_driver, lmsw_probe);
	platform_driver_probe(&lmsw_min_z_driver, lmsw_probe);
	platform_driver_probe(&lmsw_max_x_driver, lmsw_probe);
	platform_driver_probe(&lmsw_max_y_driver, lmsw_probe);
	platform_driver_probe(&lmsw_max_z_driver, lmsw_probe);
    return 0;
}
module_init(lmsw_init);

static void __exit lmsw_exit(void)
{
	platform_driver_unregister(&lmsw_min_x_driver);
	platform_driver_unregister(&lmsw_min_y_driver);
	platform_driver_unregister(&lmsw_min_z_driver);
	platform_driver_unregister(&lmsw_max_x_driver);
	platform_driver_unregister(&lmsw_max_y_driver);
	platform_driver_unregister(&lmsw_max_z_driver);
}
module_exit(lmsw_exit);

MODULE_ALIAS("platform:lmsw");
MODULE_DESCRIPTION("Unicorn Limit Switch");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Truby Zong <truby.zong@gmail.com>");
