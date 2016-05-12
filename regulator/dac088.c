/*
 * DAC088 Regulator driver for replicator unicron board
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
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

static int debug = 0;
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "enable debug");

#define dac_info(fmt, args...)  \
    if (debug) printk(KERN_EMERG "[dac]:" fmt,##args)
#define dac_err(fmt, args...)  \
    printk(KERN_ERR "[dac] Err:" fmt,##args)
#define dac_info_enter(fmt,...) \
    dac_info("[dac]: Enter func:%s\n", __func__);
#define lmsw_info_leave(fmt,...) \
    dac_info("[dac]: Leave func:%s\n", __func__);

#define DAC088_VREF 3200000  /* uV */
#define DAC088_NUM_REGULATOR (8)

#define DAC088_VOUT_A        (0)
#define DAC088_VOUT_B        (1)
#define DAC088_VOUT_C        (2)
#define DAC088_VOUT_D        (3)
#define DAC088_VOUT_E        (4)
#define DAC088_VOUT_F        (5)
#define DAC088_VOUT_G        (6)
#define DAC088_VOUT_H        (7)

#define DAC088_WRM           (0x8F)
#define DAC088_WTM           (0x9F)
#define DAC088_UPDATE_SEL    (0xAF)
#define DAC088_CH_A_WR       (0xBF)
#define DAC088_BROADCAST     (0xCF)
#define DAC088_PWR_DOWN_A    (0xDF)
#define DAC088_PWR_DOWN_B    (0xEF)
#define DAC088_PWR_DOWN_C    (0xFF)

struct reg_info {
    const char *name;
    char id;
    int min_uV;
    int max_uV;
    int (*vsel_to_uv)(unsigned int vsel);
    int (*uv_to_vsel)(int uv, unsigned int *vsel);
};

static int dac088_uv_to_vsel(int uv, unsigned int *vsel)
{
    *vsel = 0;

    return 0;
}

static int dac088_vsel_to_uv(unsigned int vsel)
{
    int uv = 0;
    
    return uv;
}

#define DAC088_INFO(_nm, _id,  _min, _max, _f1, _f2)	\
	{						\
		.name		= _nm,			\
		.id		    = _id,			\
		.min_uV		= _min,			\
		.max_uV		= _max,			\
		.vsel_to_uv	= _f1,			\
		.uv_to_vsel	= _f2,			\
	}

static struct reg_info dac088_infos[] = {
    DAC088_INFO("vout_a", DAC088_VOUT_A, 10000, 3300000, dac088_vsel_to_uv, dac088_uv_to_vsel),
    DAC088_INFO("vout_b", DAC088_VOUT_B, 10000, 3300000, dac088_vsel_to_uv, dac088_uv_to_vsel),
    DAC088_INFO("vout_c", DAC088_VOUT_C, 10000, 3300000, dac088_vsel_to_uv, dac088_uv_to_vsel),
    DAC088_INFO("vout_d", DAC088_VOUT_D, 10000, 3300000, dac088_vsel_to_uv, dac088_uv_to_vsel),
    DAC088_INFO("vout_e", DAC088_VOUT_E, 10000, 3300000, dac088_vsel_to_uv, dac088_uv_to_vsel),
    DAC088_INFO("vout_f", DAC088_VOUT_F, 10000, 3300000, dac088_vsel_to_uv, dac088_uv_to_vsel),
    DAC088_INFO("vout_g", DAC088_VOUT_G, 10000, 3300000, dac088_vsel_to_uv, dac088_uv_to_vsel),
    DAC088_INFO("vout_h", DAC088_VOUT_H, 10000, 3300000, dac088_vsel_to_uv, dac088_uv_to_vsel),
};

struct dac088_reg {
    struct spi_device *spi;
    struct regulator_desc desc[DAC088_NUM_REGULATOR];
    struct regulator_dev *rdev[DAC088_NUM_REGULATOR];
    struct reg_info *info[DAC088_NUM_REGULATOR];
    int mode[DAC088_NUM_REGULATOR];
    int current_uV[DAC088_NUM_REGULATOR];
    int is_enabled;
};

static void spi_set_value(struct spi_device *spi, u16 value)
{
    struct spi_message msg;
    struct spi_transfer xfer;

    u8 rx_buf[2] = {0, 0};
    u8 tx_buf[2] = {0, 0};

    tx_buf[0] = (value >> 8) & 0xff;
    tx_buf[1] = value & 0xff;

    spi_message_init(&msg);

    xfer.tx_buf = tx_buf;
    xfer.rx_buf = rx_buf;
    xfer.len    = 2;
    xfer.speed_hz = 100000;
    xfer.bits_per_word = 8;
    spi_message_add_tail(&xfer, &msg);

    spi_sync(spi, &msg);
}

static int dac088_set_mode(struct regulator_dev *dev, unsigned int mode)
{
    struct dac088_reg *reg = rdev_get_drvdata(dev);
    int id = rdev_get_id(dev);
    u16 val;
    
    switch (mode) {
        case REGULATOR_MODE_FAST:
        case REGULATOR_MODE_NORMAL:
            //FIXME: should read val first
            val = DAC088_PWR_DOWN_A | (0x1 << id);  
            spi_set_value(reg->spi, val);
            break;
        case REGULATOR_MODE_IDLE:
        case REGULATOR_MODE_STANDBY:
            //FIXME: should read val first
            val = DAC088_PWR_DOWN_A | ~(0x1 << id);
            spi_set_value(reg->spi, val);
            break;
        default:
            return -EINVAL;
    }

    reg->mode[id] = mode;
    return 0;
}

static unsigned int dac088_get_mode(struct regulator_dev *dev)
{
    struct dac088_reg *reg = rdev_get_drvdata(dev);
    int id = rdev_get_id(dev);
    
    return reg->mode[id];
}

static int dac088_is_enabled(struct regulator_dev *rdev)
{
    struct dac088_reg *reg = rdev_get_drvdata(rdev);

    return reg->is_enabled;
}

static int dac088_enable(struct regulator_dev *rdev)
{
    u16 val;

    struct dac088_reg *reg = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    
    if (id < 0 || id >= DAC088_NUM_REGULATOR) {
        return -EINVAL;
    }

    /* spi write cmd */
    /* WTM */
    val = DAC088_WTM << 8;
    spi_set_value(reg->spi, val);

    reg->is_enabled = 1;
    return 0;
}

static int dac088_disable(struct regulator_dev *rdev)
{
    u16 val;
    struct dac088_reg *reg = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);

    //FIXME: should read val first
    val = DAC088_PWR_DOWN_A | ~(0x1 << id);
    spi_set_value(reg->spi, val);

    reg->is_enabled = 0;
    return 0;
}


static int dac088_get_voltage(struct regulator_dev *rdev)
{
    struct dac088_reg *reg = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);

    if (id < 0 || id >= DAC088_NUM_REGULATOR) {
        return -EINVAL;
    }

    return reg->current_uV[id];
}

static int dac088_set_voltage(struct regulator_dev *rdev, 
                              int min_uV, int max_uV,
                              unsigned *selector)
{
    u16 val = 0;
    u8 level = 0;

    struct dac088_reg *reg = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    struct reg_info *info = NULL;
   
    if (id < 0 || id >= DAC088_NUM_REGULATOR) {
        return -EINVAL;
    }
    
    /* check if enabled */
    if (!dac088_is_enabled(rdev)) {
        dac088_enable(rdev);
    }

    info = &dac088_infos[id];
    
    if (min_uV < info->min_uV || min_uV > info->max_uV) {
        return -EINVAL;
    }
    if (max_uV > info->max_uV || max_uV < info->min_uV) {
        return -EINVAL;
    }
    
    reg->current_uV[id] = min_uV; 

    level = 256 * min_uV / DAC088_VREF;
    val = ((id & 0x07) << 12) | (level << 4);

    dac_info("set ch%d to level %d, val %x\n", id, level, val);

    spi_set_value(reg->spi, val);
    
    return 0;
}

static int dac088_get_status(struct regulator_dev *rdev)
{
    struct dac088_reg *reg = rdev_get_drvdata(rdev);

    return reg->is_enabled ? REGULATOR_STATUS_NORMAL : REGULATOR_STATUS_OFF;
}

static struct regulator_ops dac088_ops = {
    .enable       = dac088_enable,
    .disable      = dac088_disable,
    .is_enabled   = dac088_is_enabled,
    .get_voltage  = dac088_get_voltage,
    .set_voltage  = dac088_set_voltage,
    .set_mode     = dac088_set_mode,
    .get_mode     = dac088_get_mode,
    .get_status   = dac088_get_status,
};

#ifdef CONFIG_PM
static int dac088_reg_suspend(struct spi_device *spi, pm_message_t state)
{
    return 0;
}

static int dac088_reg_resume(struct spi_device *spi)
{
    return 0;
}
#else
#define dac088_suspend NULL
#define dac088_resume  NULL
#endif

static int __devinit dac088_reg_probe(struct spi_device *spi)
{
    int i;
    int ret;
    struct dac088_reg *reg;
    struct regulator_dev *rdev;
    struct reg_info *info = dac088_infos;
    struct regulator_init_data *reg_data = spi->dev.platform_data;  //TODO

    if (!reg_data) {
        return -ENXIO;
    }

    reg = kzalloc(sizeof(struct dac088_reg), GFP_KERNEL);
    if (unlikely(!reg)) {
        return -ENOMEM;
    }

    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_1;
    ret = spi_setup(spi);
    if (ret < 0) {
        return ret;
    }
    reg->spi = spi;
    dev_set_drvdata(&spi->dev, reg);
    
    for (i = 0; i < DAC088_NUM_REGULATOR; i++, info++, reg_data++) {
        
        info->min_uV = reg_data->constraints.min_uV;
        info->max_uV = reg_data->constraints.max_uV;

        /* register the regulators */
        reg->info[i]      = info;
        reg->desc[i].name = info->name;
        reg->desc[i].id   = info->id;
        reg->desc[i].n_voltages = 64;
        reg->desc[i].ops   = &dac088_ops;
        reg->desc[i].type  = REGULATOR_VOLTAGE;
        reg->desc[i].owner = THIS_MODULE;

        rdev = regulator_register(&reg->desc[i], &spi->dev, reg_data, reg);
        if (IS_ERR(rdev)) {
            ret = PTR_ERR(rdev);
            dac_err("failed to register %s\n", info->name);
            goto err;
        } 

        reg->rdev[i] = rdev;
    } 

    dac_info("dac088 probe ok\n");

    return 0;

err:
    while (--i >= 0) {
        regulator_unregister(reg->rdev[i]);
    }

    if (reg) {
        kfree(reg);
    }
    return ret;
}

static int __devexit dac088_reg_remove(struct spi_device *spi)
{
    int i;
    struct dac088_reg *reg = dev_get_drvdata(&spi->dev);
    
    for (i = 0; i < DAC088_NUM_REGULATOR; i++) {
        regulator_unregister(reg->rdev[i]);
    }

    if (reg) {
        kfree(reg);
    }

    dac_info("dac088 remove ok\n");

    return 0;
}

static struct spi_driver dac088_reg_driver = {
    .driver = {
        .name  = "dac088_regulator",
        .bus   = &spi_bus_type,
        .owner = THIS_MODULE,
    },
    .probe   = dac088_reg_probe,
    .remove  = __devexit_p(dac088_reg_remove),
    .suspend = dac088_reg_suspend,
    .resume  = dac088_reg_resume,
};

static int __init dac088_reg_init(void)
{
    return spi_register_driver(&dac088_reg_driver);
}
module_init(dac088_reg_init);

static void __exit dac088_reg_exit(void)
{
    spi_unregister_driver(&dac088_reg_driver);
}
module_exit(dac088_reg_exit);

MODULE_AUTHOR("Truby Zong <truby.zong@gmail.com>");
MODULE_DESCRIPTION("Replicator DAC088 regulator driver");
MODULE_LICENSE("GPL");
