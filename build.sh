#!/bin/bash/
#IAC-IMX6 LINUX3.0.101 arm-fsl-linux-gnueabi-gcc gcc4.6.2
#cheng 1910347219@qq.com
make imx6s_defconfig
#make imx6_defconfig
make uImage -j4
