/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>

#include <linux/workqueue.h>

#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/delay.h>

#include <asm/uaccess.h>

typedef struct {
	int  duty_ns;
	int  period_ns;
	int  number;
} qiyang_ioctl_arg_pwm;

struct backlight_device *qy_bl;
struct backlight_device *qy_pwm0 = NULL;
struct backlight_device *qy_pwm1 = NULL;
int pwmflag = 0;
#define	IOCTL_PWM_LEVEL_UPDATE            _IOR('Q', 0x05, int)



struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	int			(*notify)(struct device *,
					  int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
};

static int pwm_backlight_update_status(struct backlight_device *bl)
{
#if 0
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		brightness = pb->lth_brightness +
			(brightness * (pb->period - pb->lth_brightness) / max);
		pwm_config(pb->pwm, brightness, pb->period);
		pwm_enable(pb->pwm);
	}
#endif
	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
					struct fb_info *info)
{
	struct pwm_bl_data *pb = bl_get_data(bl);
	char *id = info->fix.id;
	int ret = 0;

	if (pb->check_fb) {
		ret = pb->check_fb(pb->dev, info);

		if (ret)
			return ret;
	}

	if (!strcmp(id, "DISP3 BG") ||
		!strcmp(id, "DISP3 BG - DI1") ||
		!strcmp(id, "DISP4 BG") ||
		!strcmp(id, "DISP4 BG - DI1"))
	    return 1;
	else
	return 0;
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

static int qy_pwm_update_status(qiyang_ioctl_arg_pwm *pwm)
{
	if(pwm->number == 0) {
		qy_bl = qy_pwm0;
	}else if (pwm->number == 1) {
		qy_bl = qy_pwm1;
	}

	struct pwm_bl_data *pb = dev_get_drvdata(&qy_bl->dev);
	int brightness = pwm->duty_ns;
	int max = qy_bl->props.max_brightness;

	if (qy_bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (qy_bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pwm->period_ns);
		pwm_disable(pb->pwm);
	}else {
		brightness = pb->lth_brightness +
			(brightness * (pwm->period_ns - pb->lth_brightness) / max);
		pwm_config(pb->pwm, brightness, pwm->period_ns);
		pwm_enable(pb->pwm);
	}
	return 0;
}

static int imx6_pwm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	static qiyang_ioctl_arg_pwm pwm;

	if (0 == arg)
		return -EINVAL;

	memset(&pwm, 0, sizeof(pwm));
	if(copy_from_user(&pwm, (qiyang_ioctl_arg_pwm *)arg, sizeof (qiyang_ioctl_arg_pwm))){
		printk(KERN_ERR "imx6_pwm:copy datas from user failed\n");
		return -EINVAL;
	}

	if (pwm.number == 0) {
		if(qy_pwm0 == NULL){
			printk("unregistered pwm0\n");
			return -EINVAL;
		} 
	}

	if (pwm.number == 1) {
		if(qy_pwm1 == NULL){
			printk("unregistered pwm1\n");
			return -EINVAL;
		}
	}

	if (pwm.number >= 2) {
		printk("number Please enter less than 2\n");
		return -EINVAL;
	}

	if (pwm.duty_ns < 0 || pwm.duty_ns > 100) {
		printk("pwm.duty_ns Value 0 ~100\n");
		return -EINVAL;
	}

	if (pwm.period_ns < 0 ) {
		printk("number Please enter greater than 0\n");
		return -EINVAL;
	}

	switch(cmd) {
	case IOCTL_PWM_LEVEL_UPDATE:
		ret = qy_pwm_update_status(&pwm);
		break;

	default:
		printk(KERN_ERR "imx6_pwm:unsupport ioctl cmd!\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int imx6_pwm_open(struct inode *node, struct file *file)
{
	return 0;
}

static int imx6_pwm_release(struct inode *node, struct file *file)
{
	return 0;
}


static const struct file_operations pwm_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl = imx6_pwm_ioctl,
	.open		= imx6_pwm_open,
	.release	= imx6_pwm_release,
};


static struct miscdevice pwm_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "qiyang_imx6_pwm",
	.fops		= &pwm_fops,
};


static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->check_fb = data->check_fb;
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);

	if (data->pwm_id == 0) {
		qy_pwm0 = bl;
	}else if (data->pwm_id == 1) {
		qy_pwm1 = bl;
	}

	pwmflag = pwmflag+1;
	if (pwmflag == 1) { 
		ret = misc_register(&pwm_dev);
		if (ret != 0) {
			printk(KERN_ERR "imx6_pwm:misc register failed: %d\n", ret);
			return -ENODEV;
		}
	}
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (data->pwm_id == 0) {
		qy_pwm0 = NULL;
	}else if (data->pwm_id == 1) {
		qy_pwm1 = NULL;
	}

	pwmflag = pwmflag-1;
	if (pwmflag == 0) { 
		misc_deregister(&pwm_dev);
	}

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

static int __init pwm_backlight_init(void)
{
	return platform_driver_register(&pwm_backlight_driver);
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
	platform_driver_unregister(&pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

