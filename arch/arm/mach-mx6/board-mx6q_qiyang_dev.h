/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _BOARD_MX6Q_QIYANG_DEV_H
#define _BOARD_MX6Q_QIYANG_DEV_H
#include <mach/iomux-mx6q.h>

static iomux_v3_cfg_t mx6q_qy_imx6s_i2c_pads[] =
{
    MX6Q_PAD_CSI0_DAT8__I2C1_SDA,
    MX6Q_PAD_CSI0_DAT9__I2C1_SCL,
    MX6Q_PAD_KEY_COL3__I2C2_SCL,
    MX6Q_PAD_KEY_ROW3__I2C2_SDA,
    MX6Q_PAD_GPIO_5__I2C3_SCL,
    MX6Q_PAD_GPIO_6__I2C3_SDA,
};
static iomux_v3_cfg_t mx6q_qy_imx6s_spi_pads[] = 
{
	MX6Q_PAD_SD1_CMD__ECSPI5_MOSI,
	MX6Q_PAD_SD1_CLK__ECSPI5_SCLK,
	MX6Q_PAD_SD1_DAT0__ECSPI5_MISO,
	MX6Q_PAD_SD1_DAT1__GPIO_1_17, //cs0
	MX6Q_PAD_SD1_DAT2__GPIO_1_19, //cs1
};
static iomux_v3_cfg_t mx6q_qy_imx6s_misc_pads[] = 
{
	MX6Q_PAD_GPIO_0__CCM_CLKO,   //CLK ouput

	MX6Q_PAD_SD3_RST__GPIO_7_8, //phy rst
	MX6Q_PAD_SD3_DAT4__GPIO_7_1, //sd3 wr
	MX6Q_PAD_SD3_DAT5__GPIO_7_0, //sd3 cd
	
    MX6Q_PAD_CSI0_MCLK__GPIO_5_19,     //pcie disable
    MX6Q_PAD_CSI0_VSYNC__GPIO_5_21,     //pcie reset

    MX6Q_PAD_DISP0_DAT4__GPIO_4_25,//ft5x06 touch screen INT
	MX6Q_PAD_DISP0_DAT6__GPIO_4_27,//ft5x06 touch screen reset

    MX6Q_PAD_DISP0_DAT23__GPIO_5_17,//ads7846 INT 
    
	/*J16 gpio*/
    MX6Q_PAD_GPIO_4__GPIO_1_4,
    MX6Q_PAD_GPIO_7__GPIO_1_7,
    MX6Q_PAD_GPIO_8__GPIO_1_8,
    MX6Q_PAD_GPIO_9__GPIO_1_9,
    MX6Q_PAD_GPIO_16__GPIO_7_11,
    MX6Q_PAD_GPIO_17__GPIO_7_12,
    MX6Q_PAD_GPIO_18__GPIO_7_13,
    MX6Q_PAD_GPIO_19__GPIO_4_5,

	/*J27 gpio*/
    MX6Q_PAD_DISP0_DAT3__GPIO_4_24,
    MX6Q_PAD_DISP0_DAT5__GPIO_4_26,
    MX6Q_PAD_DISP0_DAT7__GPIO_4_28,
//    MX6Q_PAD_DISP0_DAT9__GPIO_4_30,
    MX6Q_PAD_DISP0_DAT11__GPIO_5_5,
    MX6Q_PAD_DISP0_DAT13__GPIO_5_7,
    MX6Q_PAD_DISP0_DAT15__GPIO_5_9,
    MX6Q_PAD_DISP0_DAT17__GPIO_5_11,
    
	/*pwm*/
	MX6Q_PAD_DISP0_DAT9__PWM2_PWMO,
        MX6Q_PAD_DISP0_DAT8__PWM1_PWMO,
	MX6Q_PAD_GPIO_1__GPIO_1_1,  //otg pwr

	MX6Q_PAD_CSI0_DATA_EN__GPIO_5_20,  //enable pin
	MX6Q_PAD_CSI0_PIXCLK__GPIO_5_18,  //feed_dog pin

    MX6Q_PAD_GPIO_2__GPIO_1_2, //buzzer for iac


    MX6Q_PAD_DI0_DISP_CLK__GPIO_4_16,   //mipi reset
    MX6Q_PAD_DI0_PIN3__GPIO_4_19,       //mipi enable
    MX6Q_PAD_DI0_PIN15__GPIO_4_17,       //mipi power
    
};
static iomux_v3_cfg_t mx6q_qy_imx6s_matrix_keys_pads[] = 
{
	MX6Q_PAD_NANDF_D0__GPIO_2_0,
	MX6Q_PAD_NANDF_D1__GPIO_2_1,
	MX6Q_PAD_NANDF_D2__GPIO_2_2,
	MX6Q_PAD_NANDF_D3__GPIO_2_3,

	MX6Q_PAD_NANDF_D4__GPIO_2_4,
	MX6Q_PAD_NANDF_D5__GPIO_2_5,
	MX6Q_PAD_NANDF_D6__GPIO_2_6,
	MX6Q_PAD_NANDF_D7__GPIO_2_7,
};
static iomux_v3_cfg_t mx6q_qy_imx6s_csi0_pads[] =
{
    /*
    MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
    MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
    MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
    MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
    MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
    MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
    MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
    MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	//MX6Q_PAD_CSI0_DAT4__GPIO_5_22, //j22_8	
	MX6Q_PAD_CSI0_DAT5__GPIO_5_23, //j22_18
	MX6Q_PAD_CSI0_DAT6__GPIO_5_24, //j22_20
    MX6Q_PAD_CSI0_DATA_EN__IPU1_CSI0_DATA_EN,
    //MX6Q_PAD_CSI0_DATA_EN__GPIO_5_20,CSI0_DAT5
    MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
    MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
    MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
    */
};

static iomux_v3_cfg_t mx6q_qy_imx6s_disp0_pads[] =
{
    /*
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
    MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,             // DE
    MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,               // HSync
    MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,               // VSync
    MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,               // Contrast
    MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
    MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
    MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
    MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
    MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
    MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
    MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
    MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
    MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
    MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
    MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
    MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
    MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
    MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
    MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
    MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
    MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
    MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
    MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
    MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
    MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
    MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
    MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
    MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
    */
};

static iomux_v3_cfg_t mx6q_qy_imx6s_sd_pads[] = {
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
    MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
    MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
    MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
    MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
    MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
        
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
    MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
    MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
    MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
    MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
    MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
    MX6Q_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
    MX6Q_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
    MX6Q_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
    MX6Q_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,
};
static iomux_v3_cfg_t mx6q_qy_imx6s_uart_pads[] = {
	MX6Q_PAD_CSI0_DAT10__UART1_TXD,
    MX6Q_PAD_CSI0_DAT11__UART1_RXD,

	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	MX6Q_PAD_EIM_D24__UART3_TXD,
	MX6Q_PAD_EIM_D25__UART3_RXD,

	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,
	MX6Q_PAD_CSI0_DAT18__GPIO_6_4, //rs485 ctl

	MX6Q_PAD_KEY_COL1__UART5_TXD,
	MX6Q_PAD_KEY_ROW1__UART5_RXD,
	MX6Q_PAD_CSI0_DAT19__GPIO_6_5, //rs485 ctl
};

static iomux_v3_cfg_t mx6q_qy_imx6s_rgmii_pads[] = {
    MX6Q_PAD_ENET_MDIO__ENET_MDIO,
    MX6Q_PAD_ENET_MDC__ENET_MDC,
    MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
    MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
    MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
    MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
    MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
    MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
    MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
    MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
    MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
    MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
    MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
    MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
    MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
};

static iomux_v3_cfg_t mx6q_qy_imx6s_audio_pads[] = {
	MX6Q_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
	MX6Q_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
	MX6Q_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
	MX6Q_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,
};

static iomux_v3_cfg_t mx6q_qy_imx6s_can_pads[] = {
	MX6Q_PAD_KEY_COL2__CAN1_TXCAN,
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,

    MX6Q_PAD_KEY_COL4__CAN2_TXCAN,
	MX6Q_PAD_KEY_ROW4__CAN2_RXCAN,
};

#endif

