/*
 * Driver for PCA9685 16-channel 12-bit PWM LED controller
 *
 * Copyright (C) 2013 Steffen Trumtrar <s.trumtrar@pengutronix.de>
 * Copyright (C) 2014 Truby Zong <truby.zong@gmail.com>
 *
 * based on the pwm-twl-led.c driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/pwm/pwm.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/i2c.h>

static int debug = 0;
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "enable debug");

#define pwm_info(fmt, args...)  \
    if (debug) printk(KERN_EMERG "[pwm]:" fmt,##args)
#define pwm_err(fmt, args...)  \
    printk(KERN_ERR "[pwm] Err:" fmt,##args)
#define lmsw_info_enter(fmt,...) \
    pwm_info("[pwm]: Enter func:%s\n", __func__);
#define lmsw_info_leave(fmt,...) \
    pwm_info("[pwm]: Leave func:%s\n", __func__);

#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_SUBADDR1        0x02
#define PCA9685_SUBADDR2        0x03
#define PCA9685_SUBADDR3        0x04
#define PCA9685_ALLCALLADDR     0x05
#define PCA9685_LEDX_ON_L       0x06
#define PCA9685_LEDX_ON_H       0x07
#define PCA9685_LEDX_OFF_L      0x08
#define PCA9685_LEDX_OFF_H      0x09
 
#define PCA9685_ALL_LED_ON_L    0xFA
#define PCA9685_ALL_LED_ON_H    0xFB
#define PCA9685_ALL_LED_OFF_L   0xFC
#define PCA9685_ALL_LED_OFF_H   0xFD
#define PCA9685_PRESCALE        0xFE
 
#define PCA9685_NUMREGS         0xFF
#define PCA9685_MAXCHAN         0x10
 
#define LED_FULL        (1 << 4)
#define MODE1_SLEEP     (1 << 4)
#define MODE1_ALLCALL   (1 << 0)
#define MODE2_INVRT     (1 << 4)
#define MODE2_OUTDRV    (1 << 2)
 
#define LED_N_ON_H(N)   (PCA9685_LEDX_ON_H + (4 * (N)))
#define LED_N_ON_L(N)   (PCA9685_LEDX_ON_L + (4 * (N)))
#define LED_N_OFF_H(N)  (PCA9685_LEDX_OFF_H + (4 * (N)))
#define LED_N_OFF_L(N)  (PCA9685_LEDX_OFF_L + (4 * (N)))
 
#define PCA9685_CLK     (25000000)

struct pca9685 {
    struct pwm_device pwm[PCA9685_MAXCHAN];
    u8 duty_percent[PCA9685_MAXCHAN];
    struct regmap *regmap;
    int active_cnt[PCA9685_MAXCHAN];
};

static inline struct pca9685 *to_pca(struct pwm_device *pwm)
{
    return pwm_get_drvdata(pwm);
}

static void dump_pwm(struct pwm_device *pwm)
{
    pwm_info("duty_ticks %ld\n",   pwm_ns_to_ticks(pwm, pwm->duty_ns));
    pwm_info("period_ticks %ld\n", pwm_ns_to_ticks(pwm, pwm->period_ns));
    pwm_info("duty_ns %ld\n",      pwm->duty_ns);
    pwm_info("period_ns %ld\n",    pwm->period_ns);
    pwm_info("tick_hz %ld\n",      pwm->tick_hz);
}

static int pca9685_pwm_start(struct pwm_device *pwm)
{
    unsigned int reg;
    struct pca9685 *pca = to_pca(pwm);
    int chan = pwm - &pca->pwm[0];
    
    if (chan >= PCA9685_MAXCHAN) {
        reg = PCA9685_ALL_LED_ON_L;
    } else {
        reg = LED_N_ON_L(chan);
    }
    regmap_write(pca->regmap, reg, 0);

    if (chan >= PCA9685_MAXCHAN) {
        reg = PCA9685_ALL_LED_ON_H;
    } else {
        reg = LED_N_ON_H(chan);
    }
    regmap_write(pca->regmap, reg, 0);

    if (chan >= PCA9685_MAXCHAN) {
        reg = PCA9685_ALL_LED_OFF_H;
    } else {
        reg = LED_N_OFF_H(chan);
    }
    regmap_update_bits(pca->regmap, reg, LED_FULL, 0x0);

    return 0;
}

static int pca9685_pwm_stop(struct pwm_device *pwm)
{
    unsigned int reg;
    struct pca9685 *pca = to_pca(pwm);
    int chan = pwm - &pca->pwm[0];

    if (chan >= PCA9685_MAXCHAN) {
        reg = PCA9685_ALL_LED_OFF_H;
    } else {
        reg = LED_N_OFF_H(chan);
    }
    regmap_write(pca->regmap, reg, LED_FULL);

    if (chan >= PCA9685_MAXCHAN) {
        reg = PCA9685_ALL_LED_OFF_L;
    } else {
        reg = LED_N_OFF_L(chan);
    }
    regmap_write(pca->regmap, reg, 0x0);
    return 0;
}

static int pca9685_pwm_set_polarity(struct pwm_device *pwm, char pol)
{
    int mode2 = 0;
    struct pca9685 *pca = to_pca(pwm);
    
    regmap_read(pca->regmap, PCA9685_MODE2, &mode2);
    if (pol) {
        mode2 |= MODE2_INVRT;
    } else {
        mode2 &= ~MODE2_INVRT;
    } 
    
    //FIXME:
    //mode2 &= ~MODE2_OUTDRV;
    pwm_info("mode 2 %#x\n", mode2);

    regmap_write(pca->regmap, PCA9685_MODE2, mode2);

    return 0;
}

static int pca9685_pwm_config_duty(struct pwm_device *pwm)
{
    unsigned int reg;
    unsigned long long duty;
    struct pca9685 *pca = to_pca(pwm);
    int chan = pwm - &pca->pwm[0];
    
    if (pwm->duty_ns < 1) {
        if (chan >= PCA9685_MAXCHAN) {
            reg = PCA9685_ALL_LED_OFF_H;
        } else {
            reg = LED_N_OFF_H(chan);
        }

        regmap_write(pca->regmap, reg, LED_FULL);
        return 0;
    }

    duty = 4096 * (unsigned long long)pwm->duty_ns;
    duty = DIV_ROUND_UP_ULL(duty, pwm->period_ns) - 1;
    
    pca->duty_percent[chan] = DIV_ROUND_UP_ULL(pwm->duty_ns * 100, 
                                               pwm->period_ns);

    if (chan >=  PCA9685_MAXCHAN) {
        reg = PCA9685_ALL_LED_OFF_L;
    } else {
        reg = LED_N_OFF_L(chan);
    }
    regmap_write(pca->regmap, reg, (int)duty & 0xff);

    if (chan >= PCA9685_ALL_LED_OFF_H) {
        reg = PCA9685_ALL_LED_OFF_H;
    } else {
        reg = LED_N_OFF_H(chan);
    }
    regmap_write(pca->regmap, reg, ((int)duty >> 8) & 0xf);
    return 0;
}

static int pca9685_pwm_config_freq(struct pwm_device *pwm)
{
    u32 prescale = 0;
    unsigned long freq = NSEC_PER_SEC / pwm->period_ns;
    struct pca9685 *pca = to_pca(pwm);
    int chan = pwm - &pca->pwm[0];
    u32 mode;
    
    regmap_update_bits(pca->regmap, PCA9685_MODE1, MODE1_SLEEP, MODE1_SLEEP);

    regmap_read(pca->regmap, PCA9685_PRESCALE, &prescale); 
    prescale = PCA9685_CLK / (4096 * freq) - 1;
    regmap_write(pca->regmap, PCA9685_PRESCALE, prescale);
    
    /* update duty */
    pwm->duty_ns = pca->duty_percent[chan] * pwm->period_ns / 100;

    if (pwm_is_running(pwm)) {
        regmap_update_bits(pca->regmap, PCA9685_MODE1, MODE1_SLEEP, 0x0);
    }

    return 0;
}

static int pca9685_pwm_config(struct pwm_device *p, 
        struct pwm_config *c)
{
    int ret = 0; 

    switch (c->config_mask) {
	case BIT(PWM_CONFIG_DUTY_TICKS):
		p->duty_ticks = c->duty_ticks;
		ret = pca9685_pwm_config_duty(p);
		break;

	case BIT(PWM_CONFIG_PERIOD_TICKS):
		p->period_ticks = c->period_ticks;
        ret = pca9685_pwm_config_freq(p);
		break;

	case BIT(PWM_CONFIG_POLARITY):
		p->active_high = c->polarity;
		ret = pca9685_pwm_set_polarity(p, c->polarity);
		break;

	case BIT(PWM_CONFIG_START):
		ret = pca9685_pwm_start(p);
		break;

	case BIT(PWM_CONFIG_STOP):
		ret = pca9685_pwm_stop(p);
		break;
    default:
        ret = -EINVAL;
        break;
	}

    dump_pwm(p); 
    
	return ret;
}

static int pca9685_pwm_request(struct pwm_device *pwm)
{
    int ret;
    struct pca9685 *pca = to_pca(pwm);
    int chan = pwm - &pca->pwm[0];
    u32 mode; 

    if (pca->active_cnt[chan]++ == 0) {
        ret = regmap_update_bits(pca->regmap, PCA9685_MODE1, MODE1_SLEEP, 0x0);
        return ret;
    }

    regmap_read(pca->regmap, PCA9685_MODE1, &mode);
    pwm_info("mode %#x\n", mode);

    return 0;
}

static void pca9685_pwm_release(struct pwm_device *pwm)
{
    struct pca9685 *pca = to_pca(pwm);
    int chan = pwm - &pca->pwm[0];

    if (--pca->active_cnt[chan] == 0) {
        regmap_update_bits(pca->regmap, PCA9685_MODE1, MODE1_SLEEP, MODE1_SLEEP);
    }
}
 
static struct pwm_device_ops pca9685_pwm_ops = {
    .config  = pca9685_pwm_config,
    .request = pca9685_pwm_request,
    .release = pca9685_pwm_release,
};

static struct regmap_config pca9685_regmap_i2c_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = PCA9685_NUMREGS,
    .cache_type = REGCACHE_NONE,
};

static int pca9685_pwm_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    int i;
    int ret;
    struct pca9685 *pca;

    pca = devm_kzalloc(&client->dev, sizeof(*pca), GFP_KERNEL);
    if (!pca) {
        return -ENOMEM;
    }

    pca->regmap = regmap_init_i2c(client, &pca9685_regmap_i2c_config);
    if (IS_ERR(pca->regmap)) {
        ret = PTR_ERR(pca->regmap);
        pwm_err("Failed to initialize register map: %d\n", ret);
        return ret;
    }
 
    i2c_set_clientdata(client, pca);

    for (i = 0; i < PCA9685_MAXCHAN; i++) {
        pca->pwm[i].ops = &pca9685_pwm_ops;

        pca->pwm[i].tick_hz = PCA9685_CLK;

        pwm_set_drvdata(&pca->pwm[i], pca);

        ret = pwm_register(&pca->pwm[i], &client->dev, i);
        if (ret) {
            pwm_err("register pwm[%d] err\n", i);
        }
        
        pca->duty_percent[i] = 0;

        regmap_write(pca->regmap, LED_N_ON_H(i), 0x0);
        regmap_write(pca->regmap, LED_N_ON_L(i), 0x0);
    }

    regmap_update_bits(pca->regmap, PCA9685_MODE1, MODE1_ALLCALL, 0);

    pwm_info("pwm-pca9685 probe ok\n");

    return 0;
}
 
static int pca9685_pwm_remove(struct i2c_client *client)
{
    int i, ret;
    struct pca9685 *pca = i2c_get_clientdata(client);

    regmap_update_bits(pca->regmap, PCA9685_MODE1, MODE1_SLEEP,
               MODE1_SLEEP);

    for (i = 0; i < PCA9685_MAXCHAN; i++) {
        ret = pwm_unregister(&pca->pwm[i]);
    }

    pwm_info("pwm-pca9685 remove ok\n");

    return ret;
}
 
static const struct i2c_device_id pca9685_id[] = {
    { "pca9685-pwm", 0x7F },
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, pca9685_id);

static struct i2c_driver pca9685_i2c_driver = {
    .driver = {
        .name = "pca9685-pwm",
        .owner = THIS_MODULE,
    },
    .probe = pca9685_pwm_probe,
    .remove = pca9685_pwm_remove,
    .id_table = pca9685_id,
};
 
static __init int pca9685_init(void)
{
    return i2c_add_driver(&pca9685_i2c_driver);
}
module_init(pca9685_init);

static __exit void pca9685_exit(void)
{
    i2c_del_driver(&pca9685_i2c_driver);
}
module_exit(pca9685_exit);

MODULE_AUTHOR("Truby Zong <truby.zong@gmail.com>");
MODULE_DESCRIPTION("PWM driver for PCA9685");
MODULE_LICENSE("GPL");

