/**
 * pwr_button.c - Power Button Input Driver
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
#include <linux/platform_device.h>
#include <linux/input/pwr_button.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/clk.h>

#include <asm/atomic.h>

static int debug = 0;
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "enable debug");

#define pb_info(fmt, args...)  \
    if (debug) printk(KERN_EMERG "[pwr_button]:" fmt,##args)
#define pb_err(fmt, args...)  \
    printk(KERN_ERR "[pwr_button] Err:" fmt,##args)
#define pb_info_enter(fmt,...) \
    pb_info("[pwr_button]: Enter func:%s\n", __func__);
#define lmsw_info_leave(fmt,...) \
    pb_info("[pwr_button]: Leave func:%s\n", __func__);

struct pwr_button_dev {
    struct input_dev *input;
    struct timer_list timer;
    struct work_struct check_work;
    struct pwr_button_platform_data *pdata;
    atomic_t checking;
    int count;
};

static void pwr_button_do_work(struct work_struct *work)
{
	u8 val = 0;
    struct pwr_button_dev *pwr_dev = container_of(work, struct pwr_button_dev, check_work);
    
    if (pwr_dev->pdata->get_status()) {
        pwr_dev->count++; 
        if (pwr_dev->count >= 2) 
        { 
            val = 1;        
            pwr_dev->count = 0;
            del_timer(&pwr_dev->timer);
            atomic_set(&pwr_dev->checking, 1);
        } else {
            val = 0;
            mod_timer(&pwr_dev->timer, jiffies + HZ);
        }
    } else {
        val = 0;
        pwr_dev->count = 0;
        del_timer(&pwr_dev->timer);
        atomic_set(&pwr_dev->checking, 1);
    }
    
    input_report_key(pwr_dev->input, KEY_POWER, val);
    input_sync(pwr_dev->input);
}

static void pwr_button_do_timer(unsigned long arg)
{
    struct pwr_button_dev *pwr_dev = (struct pwr_button_dev *)arg;
    schedule_work(&pwr_dev->check_work);
}

static irqreturn_t pwr_button_irq(int irq, void *pwr)
{
    struct pwr_button_dev *pwr_dev = (struct pwr_button_dev *)pwr;

    pb_info("[pwr_button]: irq\n"); 

    if (atomic_dec_and_test(&pwr_dev->checking)) {
        init_timer(&pwr_dev->timer);
        pwr_dev->timer.function = pwr_button_do_timer;
        pwr_dev->timer.data = (unsigned long)pwr_dev;
        pwr_dev->timer.expires = jiffies + HZ;
        add_timer(&pwr_dev->timer);
        
        INIT_WORK(&pwr_dev->check_work, pwr_button_do_work);
    }

	return IRQ_HANDLED;
}

static int __init pwr_button_probe(struct platform_device *pdev)
{
	int err;
	struct input_dev *input;
    struct pwr_button_dev *pwr_dev; 
    struct pwr_button_platform_data *pdata;
    int irq; 

    pdata = pdev->dev.platform_data;
    if (!pdata) {
        dev_err(&pdev->dev, "platform data is required!\n");
        return -EINVAL;
    }
    
    if (!pdata->init || !pdata->exit || !pdata->get_status) {
        dev_err(&pdev->dev, "no platform-specific callbacks provided\n");
        return -EINVAL;
    }
    irq = pdata->irq;

    pwr_dev = kzalloc(sizeof(struct pwr_button_dev), GFP_KERNEL);
    if (!pwr_dev) {
        return -ENOMEM;
    }
    pwr_dev->pdata = pdata;

    err = pdata->init();
    if (err < 0) { 
        dev_err(&pdev->dev, "failed to initialize IRQ\n");
        return -ENXIO;
    }

    atomic_set(&pwr_dev->checking, 1);

	input = input_allocate_device();
	if (!input) {
		pb_err("Can't allocate power button\n");
		return -ENOMEM;
	}

	input->evbit[0] = BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	input->name = "pwr_button";
	input->phys = "pwr_button/input3";
    input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;
    
	err = request_threaded_irq(irq, NULL, pwr_button_irq,
			                   IRQF_TRIGGER_RISING & IRQF_TRIGGER_MASK,
			                   "pwr_button", pwr_dev);
	if (err < 0) {
		pb_err("Can't get IRQ for pwr_button: %d\n", err);
		goto free_input_dev;
	}
    
	err = input_register_device(input);
	if (err) {
		pb_err("Can't register power button: %d\n", err);
		goto free_irq;
	}

    pwr_dev->input = input;
	platform_set_drvdata(pdev, pwr_dev);

	return 0;

free_irq:
	free_irq(irq, NULL);
free_input_dev:
	input_free_device(input);
	return err;
}

static int __exit pwr_button_remove(struct platform_device *pdev)
{
    struct pwr_button_dev *pwr_dev = platform_get_drvdata(pdev);
	struct input_dev *input = pwr_dev->input;
    int irq = pwr_dev->pdata->irq;
    
    if (pwr_dev) {
        if (pwr_dev->pdata->exit) {
            pwr_dev->pdata->exit();
        }

	    free_irq(irq, pwr_dev);
	    input_unregister_device(input);
        kfree(pwr_dev);
    } 

	return 0;
}

static struct platform_driver pwr_button_driver = {
	.remove		= __exit_p(pwr_button_remove),
	.driver		= {
		.name	= "pwr_button",
		.owner	= THIS_MODULE,
	},
};

static int __init pwr_button_init(void)
{
	return platform_driver_probe(&pwr_button_driver,
			                      pwr_button_probe);
}
module_init(pwr_button_init);

static void __exit pwr_button_exit(void)
{
	platform_driver_unregister(&pwr_button_driver);
}
module_exit(pwr_button_exit);

MODULE_ALIAS("platform:pwr_button");
MODULE_DESCRIPTION("Unicorn Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Truby Zong <truby.zong@gmail.com>");
