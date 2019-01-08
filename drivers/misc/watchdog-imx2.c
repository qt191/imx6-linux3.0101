/*
 * Watchdog driver for IMX2 and later processors
 *
 *  Copyright (C) 2010 Wolfram Sang, Pengutronix e.K. <w.sang@pengutronix.de>
 *
 * some parts adapted by similar drivers from Darius Augulis and Vladimir
 * Zapolskiy, additional improvements by Wim Van Sebroeck.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * NOTE: MX1 has a slightly different Watchdog than MX2 and later:
 *
 *            MX1:        MX2+:
 *            ----        -----
 * Registers:        32-bit        16-bit
 * Stopable timer:    Yes        No
 * Need to enable clk:    No        Yes
 * Halt on suspend:    Manual        Can be automatic
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <mach/irqs.h>

#define DRIVER_NAME "imx2-wdt"

//定义一些宏，方便使用。

#define IMX2_WDT_WCR        0x00        /* Control Register */
#define IMX2_WDT_WCR_WT        (0xFF << 8)    /* -> Watchdog Timeout Field */
#define IMX2_WDT_WCR_WRE    (1 << 3)    /* -> WDOG Reset Enable */
#define IMX2_WDT_WCR_WDE    (1 << 2)    /* -> Watchdog Enable */
#define IMX2_WDT_WCR_WDZST    (1 << 0)    /* -> Watchdog timer Suspend */


#define IMX2_WDT_WSR        0x02        /* Service Register */
#define IMX2_WDT_SEQ1        0x5555        /* -> service sequence 1 */
#define IMX2_WDT_SEQ2        0xAAAA        /* -> service sequence 2 */

#define IMX2_WDT_WICR        0x06        /*Interrupt Control Register*/
#define IMX2_WDT_WICR_WIE    (1 << 15)    /* -> Interrupt Enable */
#define IMX2_WDT_WICR_WTIS    (1 << 14)    /* -> Interrupt Status */
#define IMX2_WDT_WICR_WICT    (0xFF << 0)    /* -> Watchdog Interrupt Timeout Field */

#define IMX2_WDT_MAX_TIME    128
#define IMX2_WDT_DEFAULT_TIME    60        /* in seconds */

#define WDOG_SEC_TO_COUNT(s)    ((s * 2 - 1) << 8)
#define WDOG_SEC_TO_PRECOUNT(s)    (s * 2)        /* set WDOG pre timeout count*/

#define IMX2_WDT_STATUS_OPEN    0
#define IMX2_WDT_STATUS_STARTED    1
#define IMX2_WDT_EXPECT_CLOSE    2

static struct {
    struct clk *clk;
    void __iomem *base;
    unsigned timeout;
    unsigned pretimeout;
    unsigned long status;
    struct timer_list timer;    /* Pings the watchdog when closed */
} imx2_wdt;

static struct miscdevice imx2_wdt_miscdev;

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);//内核参数
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
                __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");


static unsigned timeout = IMX2_WDT_DEFAULT_TIME;
module_param(timeout, uint, 0);//内核参数
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default="
                __MODULE_STRING(IMX2_WDT_DEFAULT_TIME) ")");

//这个结构体很重要，描述了驱动所支持的操作。
//应用程序使用ioctl GETSUPPORT调用来获取这个结构体
//然后在应用层使用位与检测每一位的功能。
static const struct watchdog_info imx2_wdt_info = {
    .identity = "imx2+ watchdog",
    .options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE | WDIOF_PRETIMEOUT,
    //支持喂狗
    //支持设置超时时间
    //支持魔幻字符关闭（关闭只能使用特殊字符）
    //支持预超时设置
};

static inline void imx2_wdt_setup(void)
{
    u16 val = __raw_readw(imx2_wdt.base + IMX2_WDT_WCR);

    /* Suspend watch dog timer in low power mode, write once-only */
    val |= IMX2_WDT_WCR_WDZST;
    /* Strip the old watchdog Time-Out value */
    val &= ~IMX2_WDT_WCR_WT;
    /* Generate reset if WDOG times out */
    val &= ~IMX2_WDT_WCR_WRE;
    /* Keep Watchdog Disabled */
    val &= ~IMX2_WDT_WCR_WDE;
    /* Set the watchdog's Time-Out value */
    val |= WDOG_SEC_TO_COUNT(imx2_wdt.timeout);

    __raw_writew(val, imx2_wdt.base + IMX2_WDT_WCR);

    /* enable the watchdog */
    val |= IMX2_WDT_WCR_WDE;
    __raw_writew(val, imx2_wdt.base + IMX2_WDT_WCR);
}
//喂狗操作，根据手册就是往寄存器顺序写入0x5555,0xaaaa.
static inline void imx2_wdt_ping(void)
{
    __raw_writew(IMX2_WDT_SEQ1, imx2_wdt.base + IMX2_WDT_WSR);
    __raw_writew(IMX2_WDT_SEQ2, imx2_wdt.base + IMX2_WDT_WSR);
}

static void imx2_wdt_timer_ping(unsigned long arg)
{
    /* ping it every imx2_wdt.timeout / 2 seconds to prevent reboot */
    imx2_wdt_ping();
    //启动定时器，用于喂狗
    mod_timer(&imx2_wdt.timer, jiffies + imx2_wdt.timeout * HZ / 2);
}

static void imx2_wdt_start(void)
{
    if (!test_and_set_bit(IMX2_WDT_STATUS_STARTED, &imx2_wdt.status)) {
        /* at our first start we enable clock and do initialisations */
        clk_enable(imx2_wdt.clk);

        imx2_wdt_setup();
    } else    /* delete the timer that pings the watchdog after close */
        del_timer_sync(&imx2_wdt.timer);

    /* Watchdog is enabled - time to reload the timeout value */
    imx2_wdt_ping();
}

static void imx2_wdt_stop(void)
{
    /* we don't need a clk_disable, it cannot be disabled once started.
     * We use a timer to ping the watchdog while /dev/watchdog is closed */
    imx2_wdt_timer_ping(0);
}

//设置超时时间，根本上就是读写寄存器。
static void imx2_wdt_set_timeout(int new_timeout)
{
    u16 val = __raw_readw(imx2_wdt.base + IMX2_WDT_WCR);

    /* set the new timeout value in the WSR */
    val &= ~IMX2_WDT_WCR_WT;
    val |= WDOG_SEC_TO_COUNT(new_timeout);
    __raw_writew(val, imx2_wdt.base + IMX2_WDT_WCR);
}


static int imx2_wdt_check_pretimeout_set(void)
{
    u16 val = __raw_readw(imx2_wdt.base + IMX2_WDT_WICR);
    return (val & IMX2_WDT_WICR_WIE) ? 1 : 0;
}

static void imx2_wdt_set_pretimeout(int new_timeout)
{
    u16 val = __raw_readw(imx2_wdt.base + IMX2_WDT_WICR);

    /* set the new pre-timeout value in the WSR */
    val &= ~IMX2_WDT_WICR_WICT;
    val |= WDOG_SEC_TO_PRECOUNT(new_timeout);

    if (!imx2_wdt_check_pretimeout_set())
        val |= IMX2_WDT_WICR_WIE;    /*enable*/
    __raw_writew(val, imx2_wdt.base + IMX2_WDT_WICR);
}
//预超时时间往往会引发一个中断
//例：
//如果设置超时时间为60秒，预超时为10秒
//则在50秒时会引发一个预超时中断，通过用于记录日志，刷新flash.

static irqreturn_t imx2_wdt_isr(int irq, void *dev_id)
{
    //读取状态寄存器，因为中断可能是共享的，有好几个中断源能引发同一个中断
    //所以我们需要知道是哪个源引起来的。
    //__raw_readw/__raw_writew与readl/writel是有区别的。
    //readl/writel是小端操作，__raw_readw/writew是native操作。
    u16 val = __raw_readw(imx2_wdt.base + IMX2_WDT_WICR);
    if (val & IMX2_WDT_WICR_WTIS) {
        /*clear interrupt status bit*/
        __raw_writew(val, imx2_wdt.base + IMX2_WDT_WICR);
        printk(KERN_INFO "watchdog pre-timeout:%d, %d Seconds remained\n", \
            imx2_wdt.pretimeout, imx2_wdt.timeout-imx2_wdt.pretimeout);
    }
    return IRQ_HANDLED;
}

//驱动确保这个特殊的设备只能打开1次。
//设备文件被打开则启动看门狗
//如果应用不喂狗则超时复位
static int imx2_wdt_open(struct inode *inode, struct file *file)
{
    //set_bit和test_and_set_bit是内核定义的位操作函数
    //原型为：set_bit(nr,addr):将addr的第nr位设置为1
    //test_and_set_bit(nr,addr)将addr的第nr位设置为1并返回它原来的值
    if (test_and_set_bit(IMX2_WDT_STATUS_OPEN, &imx2_wdt.status))
        return -EBUSY;

    imx2_wdt_start();
    return nonseekable_open(inode, file);
}

static int imx2_wdt_close(struct inode *inode, struct file *file)
{
    if (test_bit(IMX2_WDT_EXPECT_CLOSE, &imx2_wdt.status) && !nowayout)
        imx2_wdt_stop();
    else {
        dev_crit(imx2_wdt_miscdev.parent,
            "Unexpected close: Expect reboot!\n");
        imx2_wdt_ping();
    }

    //clear_bit是内核定义的位操作函数,原型为clear_bit(nr,addr)
    //将addr的第nr位清0.
    clear_bit(IMX2_WDT_EXPECT_CLOSE, &imx2_wdt.status);
    clear_bit(IMX2_WDT_STATUS_OPEN, &imx2_wdt.status);
    return 0;
}

static long imx2_wdt_ioctl(struct file *file, unsigned int cmd,
                            unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int __user *p = argp;
    int new_value;

    switch (cmd) {
    case WDIOC_GETSUPPORT: //应用获取驱动支持的信息，看来就是拷贝了一个结构体到用户空间。
        //应该是这个结构体中包含了所支持的操作吧？
        return copy_to_user(argp, &imx2_wdt_info,
            sizeof(struct watchdog_info)) ? -EFAULT : 0;

    case WDIOC_GETSTATUS:
    case WDIOC_GETBOOTSTATUS:
        return put_user(0, p); //难道这是不支持，直接返回0？？？

    case WDIOC_KEEPALIVE: //应用程序调用这个ioctl来喂狗。
        imx2_wdt_ping();//喂狗函数
        return 0;

    case WDIOC_SETTIMEOUT://设置超时时间
        if (get_user(new_value, p))//get_user应该跟copy_from_user()差不多的功能。
            return -EFAULT;
        if ((new_value < 1) || (new_value > IMX2_WDT_MAX_TIME))
            return -EINVAL;
        imx2_wdt_set_timeout(new_value); //将该值设置到寄存器中
        imx2_wdt.timeout = new_value; //更新结构体中保存的值
        imx2_wdt_ping();

        /* Fallthrough to return current value */
    case WDIOC_GETTIMEOUT:    //获取超时值，直接返回保存在结构体中的值。
        return put_user(imx2_wdt.timeout, p);

    case WDIOC_SETPRETIMEOUT: // 设置预超时值
        if (get_user(new_value, p))
            return -EFAULT;
        if ((new_value < 0) || (new_value >= imx2_wdt.timeout))
            return -EINVAL;
        imx2_wdt_set_pretimeout(new_value);
        imx2_wdt.pretimeout = new_value; //保存起来，以便一会儿用户获取时直接返回

    case WDIOC_GETPRETIMEOUT: //获取预超时值
        return put_user(imx2_wdt.pretimeout, p);

    default:
        return -ENOTTY; //这应该是不支持的操作吧
    }
}

static ssize_t imx2_wdt_write(struct file *file, const char __user *data,
                        size_t len, loff_t *ppos)
{
    size_t i;
    char c;

    if (len == 0)    /* Can we see this even ? */
        return 0;

    clear_bit(IMX2_WDT_EXPECT_CLOSE, &imx2_wdt.status);
    /* scan to see whether or not we got the magic character */
    for (i = 0; i != len; i++) {
        if (get_user(c, data + i))
            return -EFAULT;
        if (c == 'V')  //如果这一串数据中有魔幻字符V则表示用户期望关闭看门狗
            //但是这里只是设置了一个标志位。具体在哪里关闭呢？
            //内核中老是喜欢用set_bit/clear_bit来位来保存信息为了节省内存吗？
            set_bit(IMX2_WDT_EXPECT_CLOSE, &imx2_wdt.status);
    }

    imx2_wdt_ping();
    return len;
}
//具体的操作指针
static const struct file_operations imx2_wdt_fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,//这个函数应该是内核提供的，表示这个驱动不支持lseek操作。
    .unlocked_ioctl = imx2_wdt_ioctl,
    .open = imx2_wdt_open,
    .release = imx2_wdt_close,
    .write = imx2_wdt_write,
};

//混杂设备属于一类设备，所有混杂设备都拥有同一个主设备号
//根据次设备号来区分
//使用传统的文件结构指针
static struct miscdevice imx2_wdt_miscdev = {
    .minor = WATCHDOG_MINOR,
    .name = "watchdog",
    .fops = &imx2_wdt_fops, //函数指针
};
//驱动注册时使用遍历设备总线，根据驱动名称查找到设备名称一致的设备
//若一致则调用驱动的这个probe()函数并将找到的设备作为参数传输进来！
static int __init imx2_wdt_probe(struct platform_device *pdev)
{
    int ret;
    int res_size;
    int irq;
    struct resource *res;
//内核的这个定义很奇特：驱动跟设备分开，驱动向设备要数据，这里请求的内存地址！
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "can't get device resources\n");
        return -ENODEV;
    }
//这里是驱动向设备要中断号！
    irq = platform_get_irq(pdev, 0);
    if (irq < 0) {
        dev_err(&pdev->dev, "can't get device irq number\n");
        return -ENODEV;
    }

    res_size = resource_size(res);
//向内核注册这段地址，说这段地址已经有人在用了，别的模块请不要使用。
    if (!devm_request_mem_region(&pdev->dev, res->start, res_size,
        res->name)) {
        dev_err(&pdev->dev, "can't allocate %d bytes at %d address\n",
            res_size, res->start);
        return -ENOMEM;
    }
//物理地址映射到虚拟地址，以便进行读写操作
    imx2_wdt.base = devm_ioremap_nocache(&pdev->dev, res->start, res_size);
    if (!imx2_wdt.base) {
        dev_err(&pdev->dev, "ioremap failed\n");
        return -ENOMEM;
    }
//这里不明白？
//这是驱动从哪里提取时钟呢？驱动根据一个设备结构体就能得到时钟？
//难道时钟也有时钟链表，挂接在链表上，根据设备或名称来驱动？？
    imx2_wdt.clk = clk_get(&pdev->dev, NULL);
    if (IS_ERR(imx2_wdt.clk)) {
        dev_err(&pdev->dev, "can't get Watchdog clock\n");
        return PTR_ERR(imx2_wdt.clk);
    }
//这里是申请中断了
    ret = request_irq(irq, imx2_wdt_isr, 0, pdev->name, NULL);
    if (ret) {
        dev_err(&pdev->dev, "can't claim irq %d\n", irq);
        goto fail;
    }

    imx2_wdt.timeout = clamp_t(unsigned, timeout, 1, IMX2_WDT_MAX_TIME);
    if (imx2_wdt.timeout != timeout)
        dev_warn(&pdev->dev, "Initial timeout out of range! "
            "Clamped from %u to %u\n", timeout, imx2_wdt.timeout);
//这应该是启动定时器，定时喂狗操作！
    setup_timer(&imx2_wdt.timer, imx2_wdt_timer_ping, 0);

    imx2_wdt_miscdev.parent = &pdev->dev;
//注册混杂设备，看门狗没有分类了吗？
    ret = misc_register(&imx2_wdt_miscdev);
    if (ret)
        goto fail;

    dev_info(&pdev->dev,
        "IMX2+ Watchdog Timer enabled. timeout=%ds (nowayout=%d)\n",
                        imx2_wdt.timeout, nowayout);
    return 0;

fail:
    imx2_wdt_miscdev.parent = NULL;
    clk_put(imx2_wdt.clk);
    return ret;
}

static int __exit imx2_wdt_remove(struct platform_device *pdev)
{
    misc_deregister(&imx2_wdt_miscdev);

    if (test_bit(IMX2_WDT_STATUS_STARTED, &imx2_wdt.status)) {
        del_timer_sync(&imx2_wdt.timer);

        dev_crit(imx2_wdt_miscdev.parent,
            "Device removed: Expect reboot!\n");
    } else
        clk_put(imx2_wdt.clk);

    imx2_wdt_miscdev.parent = NULL;
    return 0;
}

static void imx2_wdt_shutdown(struct platform_device *pdev)
{
    if (test_bit(IMX2_WDT_STATUS_STARTED, &imx2_wdt.status)) {
        /* we are running, we need to delete the timer but will give
         * max timeout before reboot will take place */
        del_timer_sync(&imx2_wdt.timer);
        imx2_wdt_set_timeout(IMX2_WDT_MAX_TIME);
        imx2_wdt_ping();

        dev_crit(imx2_wdt_miscdev.parent,
            "Device shutdown: Expect reboot!\n");
    }
}
//看门狗属于SoC设备，所以这里使用平台驱动实现。
//很奇怪这里没有probe域的定义，可能是因为下面使用了platform_driver_probe()进行了注册！
static struct platform_driver imx2_wdt_driver = {
    .remove        = __exit_p(imx2_wdt_remove),
    .shutdown    = imx2_wdt_shutdown,//驱动移除时会调用的函数
    .driver        = {
        .name    = DRIVER_NAME,//驱动名称必须跟设备名称一致才能probe成功！
        .owner    = THIS_MODULE,
    },
};

static int __init imx2_wdt_init(void)
{
//这里的platform_driver_probe()跟platform_driver_register()是一样的
//唯一的区别是它不能再被以后其他的设备probe了
//也就是说这个driver只能跟一个device绑定！
    return platform_driver_probe(&imx2_wdt_driver, imx2_wdt_probe);
}
module_init(imx2_wdt_init);

static void __exit imx2_wdt_exit(void)
{
    platform_driver_unregister(&imx2_wdt_driver);
}  
module_exit(imx2_wdt_exit);

MODULE_AUTHOR("Wolfram Sang");
MODULE_DESCRIPTION("Watchdog driver for IMX2 and later");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:" DRIVER_NAME);
//作者：shell_albert 
//来源：CSDN 
//原文：https://blog.csdn.net/shell_albert/article/details/46771517 

