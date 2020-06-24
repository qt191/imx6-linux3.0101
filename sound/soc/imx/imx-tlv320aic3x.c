/*
 * qiyang-tlv320.c  --  SoC audio for qiyang_cpuimxXX in I2S mode
 *
 * Copyright 2010 Eric Bénard, Eukréa Electromatique <eric@qiyang.com>
 *
 * based on sound/soc/s3c24xx/s3c24xx_simtec_tlv320aic23.c
 * which is Copyright 2009 Simtec Electronics
 * and on sound/soc/imx/phycore-ac97.c which is
 * Copyright 2009 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 * 
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <mach/audmux.h>

#include "../codecs/tlv320aic23.h"
#include "imx-ssi.h"

#define CODEC_CLOCK 24000000

static int qiyang_tlv320_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		pr_err("%s: failed set cpu dai format\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		pr_err("%s: failed set codec dai format\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
				     CODEC_CLOCK, SND_SOC_CLOCK_OUT);
	if (ret) {
		pr_err("%s: failed setting codec sysclk\n", __func__);
		return ret;
	}
	snd_soc_dai_set_tdm_slot(cpu_dai, 0xffffffc, 0xffffffc, 2, 0);

	ret = snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0,
				SND_SOC_CLOCK_IN);
	if (ret) {
		pr_err("can't set CPU system clock IMX_SSP_SYS_CLK\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops qiyang_tlv320_snd_ops = {
	.hw_params	= qiyang_tlv320_hw_params,
};

static struct snd_soc_dai_link qiyang_tlv320_dai = {
	.name		= "tlv320aic3x",
	.stream_name	= "TLV320AIC3X",
	.codec_dai_name	= "tlv320aic3x-hifi",
	.codec_name	= "tlv320aic3x-codec.1-0018",
	.platform_name  = "imx-pcm-audio.1",
	.cpu_dai_name	= "imx-ssi.1",
	.ops		= &qiyang_tlv320_snd_ops,
};

static struct snd_soc_card qiyang_tlv320 = {
	.name		= "tlv320-audio",
	.dai_link	= &qiyang_tlv320_dai,
	.num_links	= 1,
};
static int imx_audmux_config(int slave, int master)
{
        unsigned int ptcr, pdcr;
        slave = slave - 1;
        master = master - 1;

        // SSI0 mastered by port 4 
        ptcr = MXC_AUDMUX_V2_PTCR_SYN |
                MXC_AUDMUX_V2_PTCR_TFSDIR |
                MXC_AUDMUX_V2_PTCR_TFSEL(master) |
                MXC_AUDMUX_V2_PTCR_TCLKDIR |
                MXC_AUDMUX_V2_PTCR_TCSEL(master);
        pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
        mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

        ptcr = MXC_AUDMUX_V2_PTCR_SYN;
        pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
        mxc_audmux_v2_configure_port(master, ptcr, pdcr);

        return 0;
}
static int __devinit imx_tlv320_probe(struct platform_device *pdev)
{
	//printk("*****************888imx_tlv320_probe\n");
        struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

        int ret = 0;

        imx_audmux_config(plat->src_port, plat->ext_port);

        ret = -EINVAL;
        if (plat->init && plat->init())
                return ret;
        return 0;
}
static int imx_tlv320_remove(struct platform_device *pdev)
{
        struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

        if (plat->finit)
                plat->finit();

        return 0;
}

static struct platform_driver imx_tlv320_audio_driver = {
        .probe = imx_tlv320_probe,
        .remove = imx_tlv320_remove,
        .driver = {
                   .name = "imx-tlv320",
                   },
};

static struct platform_device *qiyang_tlv320_snd_device;

static int __init qiyang_tlv320_init(void)
{
	int ret;
	//printk("********************************************************fuck!%d\n",machine_arch_type);
	ret = platform_driver_register(&imx_tlv320_audio_driver);
        if (ret)
                return -ENOMEM;
	if(!machine_is_mx6q_qiyang())
		return 0;
	qiyang_tlv320_snd_device = platform_device_alloc("soc-audio", 6);
	if (!qiyang_tlv320_snd_device)
		return -ENOMEM;
	
	platform_set_drvdata(qiyang_tlv320_snd_device, &qiyang_tlv320);
	ret = platform_device_add(qiyang_tlv320_snd_device);
	
	
	
	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		platform_device_put(qiyang_tlv320_snd_device);
	}
	//printk("********************************************************fuck2!\n");
	return ret;
}

static void __exit qiyang_tlv320_exit(void)
{
	platform_driver_unregister(&imx_tlv320_audio_driver);
	platform_device_unregister(qiyang_tlv320_snd_device);
}

module_init(qiyang_tlv320_init);
module_exit(qiyang_tlv320_exit);


MODULE_AUTHOR("allen young <yangcenhao@gmail.com>");
MODULE_DESCRIPTION("FUCK ALSA SoC driver");
MODULE_LICENSE("GPL");
