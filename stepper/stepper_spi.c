/*---------------------------------------------------------------------------
 * stepper_spi.c
 * SPI driver for stepper motor mode control
 * Copyright (c) 2014 Truby Zong <truby.zong@gmail.com>
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
 ---------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>

#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <linux/gpio.h>
#include <linux/stepper.h>
#include <linux/input/lmsw.h>

#include "stepper_spi.h"

static int debug = 0;
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "enable debug");

#define st_info(fmt, args...)  \
    if (debug) printk(KERN_EMERG "[stepper_spi]:" fmt,##args)
#define st_err(fmt, args...)  \
    printk(KERN_ERR "[stepper_spi] Err:" fmt,##args)
#define st_info_enter(fmt,...) \
    st_info("[stepper_spi]: Enter func:%s\n", __func__);
#define lmsw_info_leave(fmt,...) \
    st_info("[stepper_spi]: Leave func:%s\n", __func__);

#define STEPPER_MAJOR          (242)
static int stepper_major = STEPPER_MAJOR;

static unsigned autoLevel_gpio = 14; //gpio0_14;
static char autoLevel_gpio_desc[] = "autoLevel";

struct stepper_spi {
    /* spi device */
    struct spi_device *spi;

    /* char device */
    struct cdev  cdev;
    struct class *class; 
    dev_t devno;

    /* file sem */
    struct semaphore sem;
    struct mutex lock;   
    struct mutex wr_lock;   
    struct mutex rd_lock;   

    /* stepper information */
    struct stepper_info *infos;
    
    int (*check_min_x)(void); 
    int (*check_min_y)(void); 
    int (*check_min_z)(void); 

    int (*check_max_x)(void); 
    int (*check_max_y)(void); 
    int (*check_max_z)(void); 

    int (*check_autoLevel_z)(void); 

    /* gpio list */
    u32 step_x;       
    u32 step_y;
    u32 step_z;
    u32 step_e;
    
    u32 dir_x;
    u32 dir_y;
    u32 dir_z;
    u32 dir_e;

    u32 fault_x;
    u32 fault_y;
    u32 fault_z;
    u32 fault_e;

    /* spi command */
    stepper_cmd_t cmd;
    u8 spi_cmd;

    /* stepper state */
    u8 state;
};


static int gpio_num = 0;
static ssize_t show_gpio_dev(struct class *class,
						struct class_attribute *attr, char *buf)
{
	int ret = -1;
	if (gpio_num >0 && gpio_num < 128) {
		 ret = gpio_get_value(gpio_num);
	} 
    return sprintf(buf, "%d\n", ret);
}
static ssize_t set_gpio_dev(struct class *cls, struct class_attribute *attr, const char *buf, size_t count)
{
	if (sscanf(buf, "%d", &gpio_num) != 1){
		return -EINVAL;
	}
	return count;
}

static int gpio = -1, direction = -1, level = -1;
static ssize_t show_config_gpio_dev(struct class *class,
						struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "gpio:%d,direction:%d,level:%d\n", gpio, direction, level);
}


//echo "gpio,direction,level"  -> 0 for input, 1 for output.  0 for low level, 1 for high level
static ssize_t set_config_gpio_dev(struct class *cls, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	//int gpio = -1, direction = -1, level = -1;
	if (sscanf(buf, "%d,%d,%d", &gpio, &direction, &level) != 3) {
		return -EINVAL;
	}

	if (gpio < 0 || gpio > 128) {
		printk("echo \"gpio is out of range\n");
		return count;
	}

    //printk("<0> gpio:%d,direction:%d,level:%d\n", gpio, direction, level);

	if(((direction != 0) && (direction !=1)) || ((level != 0) && (level != 1))){
		printk("echo \"gpio,direction,level\"  -> 0 for input, 1 for output.  0 for low level, 1 for high level\n");
		return count;
	}

	if (direction == 0) {
	    ret = gpio_direction_input(gpio); 
	} else {
		ret = gpio_direction_output(gpio, level);
	}
	
	return count;
}

static CLASS_ATTR(gpio, 0666, show_gpio_dev, set_gpio_dev); 
static CLASS_ATTR(config_gpio, 0666, show_config_gpio_dev, set_config_gpio_dev); 

static void spi_write_cmd(struct spi_device *spi, u8 cmd)
{
    u8 tx;
    u8 rx;
    struct spi_message msg;
    struct spi_transfer xfer;
    
    tx = cmd;
    spi_message_init(&msg);

    xfer.tx_buf = &tx;
    xfer.rx_buf = &rx;
    xfer.len    = 1;
    xfer.speed_hz = 100000;
    xfer.bits_per_word = 8;
    spi_message_add_tail(&xfer, &msg);

    spi_sync(spi, &msg);
}

static int stepper_open(struct inode *inode, struct file *file)
{
    struct stepper_spi *stepper = container_of(inode->i_cdev, struct stepper_spi, cdev);;
    
    if (!stepper) {
        return -EFAULT;
    }

    if (mutex_lock_interruptible(&stepper->lock)) {
        return -ERESTARTSYS;
    }
    file->private_data = stepper;
    nonseekable_open(inode, file); 

    mutex_unlock(&stepper->lock);

    return 0;
}

static int stepper_release(struct inode *inode, struct file *file)
{
    struct stepper_spi *stepper = file->private_data;

    if (mutex_lock_interruptible(&stepper->lock)) {
        return -ERESTARTSYS;
    }
    file->private_data = NULL;

    mutex_unlock(&stepper->lock);

    return 0;
}

static int stepper_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    int ret;
    unsigned int copied;
    struct stepper_spi *stepper = file->private_data;

    if (mutex_lock_interruptible(&stepper->wr_lock)) {
        return -ERESTARTSYS;
    }

    mutex_unlock(&stepper->wr_lock);

    return ret ? ret : copied;
}

static ssize_t stepper_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int ret;
    unsigned int copied;
    
    struct stepper_spi *stepper = file->private_data;
    
    if (mutex_lock_interruptible(&stepper->rd_lock)) {
        return -ERESTARTSYS;
    }

    mutex_unlock(&stepper->rd_lock);

    return ret ? ret : copied;
}

static long stepper_ioctl(struct file *file, uint cmd, ulong arg)
{
    int ret = 0; 
    u8 spi_cmd = 0;
    struct stepper_spi *stepper = file->private_data;
    stepper_cmd_t *command = (stepper_cmd_t *)&stepper->cmd;    

    switch (cmd)
    {
    case STEPPER_SET_CMD:
        if (copy_from_user(command, (void *)arg, sizeof(stepper_cmd_t))) {
            return -EFAULT;
        }

        /*
         * SPI send MSB first!
         * ------------------------------------
         *  7     6      5     4    3  2  1  0
         *  H     G      F     E    D  C  B  A
         * ------------------------------------
         * Reset Sleep Decay Enable M0 M1 M2 X 
         * ------------------------------------
         */
        spi_cmd = ((command->mode & 0x07) << 1) | ((command->enable & 0x01) << 4) 
                | ((command->decay & 0x01) << 5) | ((command->sleep & 0x01) << 6)
                | ((command->reset & 0x01) << 7);
        st_info("spi_cmd %#x\n", spi_cmd);

        spi_write_cmd(stepper->spi, spi_cmd);
        stepper->spi_cmd = spi_cmd;
        return ret;

    case STEPPER_GET_CMD:
        if (copy_to_user((void *)arg, command, sizeof(stepper_cmd_t))) {
            return -EFAULT;
        }
        return ret;

    case STEPPER_CH_MIN_X:
        return stepper->check_min_x();

    case STEPPER_CH_MIN_Y:
        return stepper->check_min_y();
    
    case STEPPER_CH_MIN_Z:
        return stepper->check_min_z();

    case STEPPER_CH_MAX_X:
        return stepper->check_max_x();

    case STEPPER_CH_MAX_Y:
        return stepper->check_max_y();
    
    case STEPPER_CH_MAX_Z:
        return stepper->check_max_z();

    case AUTOLEVEL_Z:
        return stepper->check_autoLevel_z();

    case AUTOLEVEL_Z_GPIO_INPUT:
	    ret = gpio_direction_input(autoLevel_gpio); 
        return 0;

    case AUTOLEVEL_Z_GPIO_OUTPUT: 
		ret = gpio_direction_output(autoLevel_gpio, 1); 
        return 0;

    default:
        return -EINVAL;
    }
}


static const struct file_operations stepper_fops = 
{
    .owner   = THIS_MODULE,
    .read    = stepper_read, 
    .write   = stepper_write,
    .open    = stepper_open,
    .release = stepper_release,
    .llseek  = no_llseek,
    .unlocked_ioctl = stepper_ioctl,
};

#ifdef CONFIG_PM
static int stepper_suspend(struct spi_device *spi, pm_message_t state)
{
    return 0;
}

static int stepper_resume(struct spi_device *spi)
{
    return 0;
}
#else
#define stepper_suspend NULL
#define stepper_resume  NULL
#endif

static int stepper_probe(struct spi_device *spi)
{
    int i, ret = 0;
    struct stepper_spi *stepper = NULL;
    struct stepper_info *info = NULL;
    struct stepper_platform_data *pdata = spi->dev.platform_data;
    dev_t devno = MKDEV(stepper_major, 0);
    
    /* Check platform data */
    if (!pdata) {
        return -EINVAL;
    }
    
    if (pdata->nsteppers <= 0 || pdata->nsteppers > MAX_STEPPER_NUM) {
        return -EINVAL;
    }
        
    /* Alloc device number */
    if (stepper_major) {
        ret = register_chrdev_region(devno, 1, "stepper_spi");
    } else {
        ret = alloc_chrdev_region(&devno, 0, 1, "stepper_spi");
        stepper_major = MAJOR(devno);
    }
    if (ret < 0) {
        return ret;
    }
    
    /* Alloc device struct */
    stepper = kzalloc(sizeof(struct stepper_spi), GFP_KERNEL); 
    if (unlikely(!stepper)) {
        ret = -ENOMEM;
        goto out;
    }

    stepper->infos = kzalloc(sizeof(struct stepper_info) * pdata->nsteppers, 
            GFP_KERNEL);
    if (unlikely(!stepper->infos)) {
        ret = -ENOMEM;
        goto out;
    }
    memcpy(stepper->infos, pdata->steppers, 
            sizeof(struct stepper_info) * pdata->nsteppers);
    
    info = stepper->infos;
    for (i = 0; i < pdata->nsteppers; i++, info++) {
        st_info("Found %s step: %d, dir: %d, fault: %d\n",
                info->name, info->step, info->dir, info->fault);
        if (!strcmp(info->name, "stepper_x")) {
            stepper->step_x  = info->step;
            stepper->dir_x   = info->dir;
            stepper->fault_x = info->fault;
        }
        if (!strcmp(info->name, "stepper_y")) {
            stepper->step_y  = info->step;
            stepper->dir_y   = info->dir;
            stepper->fault_y = info->fault;
        }
        if (!strcmp(info->name, "stepper_z")) {
            stepper->step_z  = info->step;
            stepper->dir_z   = info->dir;
            stepper->fault_z = info->fault;
        }
        if (!strcmp(info->name, "stepper_ext1")) {
            stepper->step_e  = info->step;
            stepper->dir_e   = info->dir;
            stepper->fault_e = info->fault;
        }

    }

    stepper->check_min_x = pdata->check_min_x;
    stepper->check_min_y = pdata->check_min_y;
    stepper->check_min_z = pdata->check_min_z;

    stepper->check_max_x = pdata->check_max_x;
    stepper->check_max_y = pdata->check_max_y;
    stepper->check_max_z = pdata->check_max_z;
    stepper->check_autoLevel_z = pdata->check_autoLevel_z;

    //set step signal to low
    gpio_set_value(stepper->step_x, 0);
    gpio_set_value(stepper->step_y, 0);
    gpio_set_value(stepper->step_z, 0);
    gpio_set_value(stepper->step_e, 0);

    stepper->devno = devno; 
    
    mutex_init(&stepper->lock);
    mutex_init(&stepper->wr_lock);
    mutex_init(&stepper->rd_lock);

    /* Setup spi device */
    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_0;
    ret = spi_setup(spi);
    if (ret < 0) {
        goto out;
    }
    stepper->spi = spi;
    dev_set_drvdata(&spi->dev, stepper);
    
    /* Register char device */
    cdev_init(&stepper->cdev, &stepper_fops);
    stepper->cdev.owner = THIS_MODULE;
    stepper->cdev.ops = &stepper_fops;

    ret = cdev_add(&stepper->cdev, devno, 1);
    if (ret) {
        st_err("Error add char dev:%d\n", ret);
        goto out;
    }
   
    stepper->class = class_create(THIS_MODULE, "stepper_spi_class");
    if (IS_ERR(stepper->class)) {
        st_err("Failed to create class %s\n", "stepper_spi_class");
        goto out;
    }
    
    device_create(stepper->class, NULL, devno, NULL, "stepper_spi");

    ret = class_create_file(stepper->class, &class_attr_gpio);
    if (ret != 0) {
        st_err("Failed to create attribute gpio group: %d\n", ret);
        goto out;
    }
    ret = class_create_file(stepper->class, &class_attr_config_gpio);
    if (ret != 0) {
        st_err("Failed to create attribute config gpio group: %d\n", ret);
        goto out;
    }


#if 0
	if (gpio_request(autoLevel_gpio, autoLevel_gpio_desc) < 0) {
		st_err("failed to request gpio%d for %s\n",
				autoLevel_gpio, autoLevel_gpio_desc);
		return -ENXIO;
	}
#endif

    st_info("probe ok\n");    
    return 0;
out:
    unregister_chrdev_region(devno, 1);

    if (stepper) {
        kfree(stepper);
    }
    return ret;
}

static int stepper_remove(struct spi_device *spi)
{
    struct stepper_spi *stepper = dev_get_drvdata(&spi->dev); 
    
    /* destory char device */
    if (stepper) {
    	class_remove_file(stepper->class, &class_attr_gpio);
    	class_remove_file(stepper->class, &class_attr_config_gpio);
        cdev_del(&stepper->cdev);
        unregister_chrdev_region(MKDEV(stepper_major, 0), 1);
        device_destroy(stepper->class, stepper->devno); 
        class_destroy(stepper->class);
    }
    
    dev_set_drvdata(&spi->dev, NULL);

    st_info("remove ok\n");    

    return 0;
}

static struct spi_driver stepper_driver = {
    .driver   = {
        .name = "stepper_spi",
        .bus  = &spi_bus_type,
        .owner = THIS_MODULE,
    },
    .probe    = stepper_probe,
    .remove   = stepper_remove,
    .suspend  = stepper_suspend,
    .resume   = stepper_resume,
};

static int __init stepper_init(void)
{
    return spi_register_driver(&stepper_driver);
}
module_init(stepper_init);

static void __exit stepper_exit(void)
{
    spi_unregister_driver(&stepper_driver);
}
module_exit(stepper_exit);

MODULE_AUTHOR("Truby Zong <truby.zong@gmail.com>");
MODULE_DESCRIPTION("Unicorn stepper spi driver");
MODULE_LICENSE("GPL");
