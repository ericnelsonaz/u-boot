/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2015, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <splash.h>
#include <usb/ehci-fsl.h>

DECLARE_GLOBAL_DATA_PTR;

static void do_enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {{
	.bus	= 1,
	.addr	= 0x50,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= 0,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
}}};
size_t display_count = ARRAY_SIZE(displays);

#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define OUTPUT_40OHM	(PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL	(PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLDN_OUTPUT (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP_OUTPUT (PAD_CTL_PUS_100K_UP |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_SLOW)

/*
 *
 */
static const iomux_v3_cfg_t init_pads[] = {
	/* AUDMUX */
	MX6_PAD_CSI0_DAT7__AUD3_RXD | MUX_PAD_CTRL(AUD_PAD_CTRL),
	MX6_PAD_CSI0_DAT4__AUD3_TXC | MUX_PAD_CTRL(AUD_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__AUD3_TXD | MUX_PAD_CTRL(AUD_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__AUD3_TXFS | MUX_PAD_CTRL(AUD_PAD_CTRL),

	/* bt_rfkill */
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(6, 16)
	MX6_PAD_NANDF_CS3__GPIO6_IO16 | MUX_PAD_CTRL(WEAK_PULLDN),

	/* ECSPI1 */
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(3, 19)
	MX6_PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* ENET pads that don't change for PHY reset */
	MX6_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* AR8035 PHY nRST */
#define GP_ENET_PHY_RESET	IMX_GPIO_NR(1, 27)
	MX6_PAD_ENET_RXD0__GPIO1_IO27 | MUX_PAD_CTRL(WEAK_PULLDN),
#define GPIRQ_ENET_PHY		IMX_GPIO_NR(1, 28)
	MX6_PAD_ENET_TX_EN__GPIO1_IO28 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* i2c1_sgtl5000 */
	MX6_PAD_GPIO_0__CCM_CLKO1 | MUX_PAD_CTRL(OUTPUT_40OHM),
#define GPIRQ_MIC_DET		IMX_GPIO_NR(1, 2)
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(WEAK_PULLUP),
#define GPIRQ_HP_DET		IMX_GPIO_NR(1, 3)
	MX6_PAD_GPIO_3__GPIO1_IO03 | MUX_PAD_CTRL(WEAK_PULLUP),

#define GPIRQ_I2C3_J7		IMX_GPIO_NR(1, 9)
	MX6_PAD_GPIO_9__GPIO1_IO09 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* i2c2_rv4172 rtc */
#define GPIRQ_RTC_RV4162	IMX_GPIO_NR(2, 26)
	MX6_PAD_EIM_RW__GPIO2_IO26 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* J25 */
#define GPIRQ_IR		IMX_GPIO_NR(2, 30)
	MX6_PAD_EIM_EB2__GPIO2_IO30 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* PCIe */
#define GP_PCIE_RESET		IMX_GPIO_NR(4, 7)
	MX6_PAD_KEY_ROW0__GPIO4_IO07 | MUX_PAD_CTRL(WEAK_PULLDN),
#define GP_PCIE_DISABLE		IMX_GPIO_NR(4, 6)
	MX6_PAD_KEY_COL0__GPIO4_IO06 | MUX_PAD_CTRL(WEAK_PULLDN),

	/* PWM2 */
#define GP_PWM2			IMX_GPIO_NR(1, 19)
	MX6_PAD_SD1_DAT2__GPIO1_IO19 | MUX_PAD_CTRL(WEAK_PULLDN),

	/* reg_usbotg_vbus */
#define GP_REG_USBOTG		IMX_GPIO_NR(3, 22)
	MX6_PAD_EIM_D22__GPIO3_IO22 | MUX_PAD_CTRL(WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN		IMX_GPIO_NR(6, 15)
	MX6_PAD_NANDF_CS2__GPIO6_IO15 | MUX_PAD_CTRL(WEAK_PULLDN),

	/* TEST POINTS */
#define GP_TP68			IMX_GPIO_NR(2, 0)
	MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(BUTTON_PAD_CTRL),
#define GP_TP69			IMX_GPIO_NR(2, 1)
	MX6_PAD_NANDF_D1__GPIO2_IO01 | MUX_PAD_CTRL(BUTTON_PAD_CTRL),
#define GP_TP70			IMX_GPIO_NR(2, 3)
	MX6_PAD_NANDF_D3__GPIO2_IO03 | MUX_PAD_CTRL(BUTTON_PAD_CTRL),
#define GP_TP71			IMX_GPIO_NR(2, 4)
	MX6_PAD_NANDF_D4__GPIO2_IO04 | MUX_PAD_CTRL(BUTTON_PAD_CTRL),
#define GP_TP77			IMX_GPIO_NR(7, 13)
	MX6_PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(BUTTON_PAD_CTRL),
#define GP_TP78			IMX_GPIO_NR(4, 5)
	MX6_PAD_GPIO_19__GPIO4_IO05 | MUX_PAD_CTRL(BUTTON_PAD_CTRL),

	/* UART1 */
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),

	/* UART2 */
	MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),

	/* UART3 for wl1271 */
	MX6_PAD_EIM_D24__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D25__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D23__UART3_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D31__UART3_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),

	/* USBH1 */
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(WEAK_PULLDN),

	/* USBOTG */
	MX6_PAD_GPIO_1__USB_OTG_ID | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_KEY_COL4__USB_OTG_OC | MUX_PAD_CTRL(WEAK_PULLUP),

	/* USDHC2 - TiWi wl1271 */
	MX6_PAD_SD2_CLK__SD2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
//	MX6_PAD_SD1_CLK__OSC32K_32K_OUT | MUX_PAD_CTRL(OUTPUT_40OHM),  /* slow clock */

	/* USDHC3 - sdcard */
	MX6_PAD_SD3_CLK__SD3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
#define GP_USDHC3_WP		IMX_GPIO_NR(7, 1)
	MX6_PAD_SD3_DAT4__GPIO7_IO01 | MUX_PAD_CTRL(WEAK_PULLUP),
#define GP_USDHC3_CD		IMX_GPIO_NR(7, 0)
	MX6_PAD_SD3_DAT5__GPIO7_IO00 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* USDHC4 - emmc */
	MX6_PAD_SD4_CLK__SD4_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
#define GP_EMMC_RESET		IMX_GPIO_NR(2, 7)
	MX6_PAD_NANDF_D7__GPIO2_IO07 | MUX_PAD_CTRL(WEAK_PULLUP),

	/* wl1271 */
#define GPIRQ_WL1271_WL		IMX_GPIO_NR(6, 14)
	MX6_PAD_NANDF_CS1__GPIO6_IO14 | MUX_PAD_CTRL(WEAK_PULLDN),

	/* Make sure unused LCD pins don't toggle */
	MX6_PAD_DI0_DISP_CLK__GPIO4_IO16 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DI0_PIN15__GPIO4_IO17 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DI0_PIN2__GPIO4_IO18 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DI0_PIN3__GPIO4_IO19 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DI0_PIN4__GPIO4_IO20 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT0__GPIO4_IO21 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT1__GPIO4_IO22 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT2__GPIO4_IO23 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT3__GPIO4_IO24 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT4__GPIO4_IO25 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT5__GPIO4_IO26 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT6__GPIO4_IO27 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT7__GPIO4_IO28 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT8__GPIO4_IO29 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT9__GPIO4_IO30 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT10__GPIO4_IO31 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT11__GPIO5_IO05 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT12__GPIO5_IO06 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT13__GPIO5_IO07 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT14__GPIO5_IO08 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT15__GPIO5_IO09 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT16__GPIO5_IO10 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT17__GPIO5_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT18__GPIO5_IO12 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT19__GPIO5_IO13 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT20__GPIO5_IO14 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT21__GPIO5_IO15 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT22__GPIO5_IO16 | MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_DISP0_DAT23__GPIO5_IO17 | MUX_PAD_CTRL(WEAK_PULLUP),
};

static const iomux_v3_cfg_t enet_gpio_pads[] = {
	/* pin 35 - 1 (PHY_AD2) on reset */
#define GP_PHY_1P8_SEL		IMX_GPIO_NR(6, 30)
	MX6_PAD_RGMII_RXC__GPIO6_IO30 | MUX_PAD_CTRL(WEAK_PULLUP_OUTPUT),
	/* pin 32 - 1 - (MODE0) all */
#define GP_PHY_AD0		IMX_GPIO_NR(6, 25)
	MX6_PAD_RGMII_RD0__GPIO6_IO25 | MUX_PAD_CTRL(WEAK_PULLDN_OUTPUT),
	/* pin 31 - 1 - (MODE1) all */
#define GP_PHY_AD1		IMX_GPIO_NR(6, 27)
	MX6_PAD_RGMII_RD1__GPIO6_IO27 | MUX_PAD_CTRL(WEAK_PULLDN_OUTPUT),
	/* pin 28 - 1 - (MODE2) all */
#define GP_PHY_MODE1		IMX_GPIO_NR(6, 28)
	MX6_PAD_RGMII_RD2__GPIO6_IO28 | MUX_PAD_CTRL(WEAK_PULLDN_OUTPUT),
	/* pin 27 - 1 - (MODE3) all */
#define GP_PHY_MODE3		IMX_GPIO_NR(6, 29)
	MX6_PAD_RGMII_RD3__GPIO6_IO29 | MUX_PAD_CTRL(WEAK_PULLUP_OUTPUT),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
#define GP_PHY_MODE0		IMX_GPIO_NR(6, 24)
	MX6_PAD_RGMII_RX_CTL__GPIO6_IO24 | MUX_PAD_CTRL(WEAK_PULLDN_OUTPUT),
};

static const iomux_v3_cfg_t enet_pads[] = {
	MX6_PAD_RGMII_RXC__RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
};

static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_16__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_16__GPIO7_IO11 | PC,
		.gp = IMX_GPIO_NR(7, 11)
	}
};

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);
	return 0;
}

int board_ehci_hcd_init(int port)
{
	/* Reset USB hub */
	gpio_direction_output(GP_USB_HUB_RESET, 0);
	mdelay(2);
	gpio_set_value(GP_USB_HUB_RESET, 1);
	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	gpio_set_value(GP_REG_USBOTG, on);
	return 0;
}

static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC3_BASE_ADDR, .max_bus_width = 4},
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	if (cfg->esdhc_base == USDHC4_BASE_ADDR)
		return 1;	/* eMMC always present */
	return !gpio_get_value(GP_USDHC3_CD);
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			break;
		case 1:
			gpio_set_value(GP_EMMC_RESET, 1); /* release reset */
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) then supported by the board (%d)\n",
				index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}
	return 0;
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
}

static void setup_iomux_enet(void)
{
	gpio_direction_output(GP_ENET_PHY_RESET, 0); /* PHY rst */
	gpio_direction_output(GP_PHY_AD0, 0);
	gpio_direction_output(GP_PHY_AD1, 0);
	gpio_direction_output(GP_PHY_MODE0, 0);
	gpio_direction_output(GP_PHY_MODE1, 0);
	gpio_direction_output(GP_PHY_MODE3, 1);
	gpio_direction_output(GP_PHY_1P8_SEL, 1);

	SETUP_IOMUX_PADS(enet_gpio_pads);

	/* Need delay 1 ms */
	udelay(1000);
	gpio_set_value(GP_ENET_PHY_RESET, 1); /* PHY reset */
	/* strap hold time, 2 fails, 3 works, so 5 should be safe */
	udelay(5);

	SETUP_IOMUX_PADS(enet_pads);
	udelay(100);	/* Wait 100 us before using mii interface */
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4 */
	phydev = phy_find_by_mask(bus, (1 << 4), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}

	/* For otg ethernet*/
	usb_eth_initialize(bis);

	return 0;
}


int splash_screen_prepare(void)
{
	char *env_loadsplash;

	if (!getenv("splashimage") || !getenv("splashsize")) {
		return -1;
	}

	env_loadsplash = getenv("loadsplash");
	if (env_loadsplash == NULL) {
		printf("Environment variable loadsplash not found!\n");
		return -1;
	}

	if (run_command_list(env_loadsplash, -1, 0)) {
		printf("failed to run loadsplash %s\n\n", env_loadsplash);
		return -1;
	}

	return 0;
}

static const unsigned short gpios_out_low[] = {
	GP_BT_RFKILL_RESET, 	/* disable bluetooth */
	GP_ENET_PHY_RESET,
	GP_PCIE_RESET,
	GP_PWM2,
	GP_REG_USBOTG,		/* disable USB otg power */
	GP_REG_WLAN_EN,		/* disable wireless */
	GP_USB_HUB_RESET,	/* disable hub */
	GP_EMMC_RESET,		/* hold in reset */
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,	/* SS1 of spi nor */
};

static const unsigned short gpios_in[] = {
	GPIRQ_ENET_PHY,
	GPIRQ_RTC_RV4162,
	GPIRQ_MIC_DET,
	GPIRQ_HP_DET,
	GPIRQ_I2C3_J7,
	GPIRQ_IR,
	GP_PCIE_DISABLE,
	GP_TP68,
	GP_TP69,
	GP_TP70,
	GP_TP71,
	GP_TP77,
	GP_TP78,
	GP_USDHC3_WP,
	GP_USDHC3_CD,
	GPIRQ_WL1271_WL,
};

static void set_gpios_in(const unsigned short *p, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_input(*p++);
}

static void set_gpios(const unsigned short *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

int board_early_init_f(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	SETUP_IOMUX_PADS(init_pads);

	enable_ipu_clock();
	imx_setup_hdmi();
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_init(void)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	return 0;
}

int checkboard(void)
{
	puts("Board: RC\n");
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"back",	GP_TP68,	'B'},
};

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
	int i, numpressed = 0;
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!gpio_get_value(buttons[i].gpnum))
			buf[numpressed++] = buttons[i].ident;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

static int do_kbd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	int numpressed = read_keys(envvalue);
	setenv("keybd", envvalue);
	return numpressed == 0;
}

U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);

static char const kbd_magic_prefix[] = "key_magic";
static char const kbd_command_prefix[] = "key_cmd";

static void preboot_keys(void)
{
	int numpressed;
	char keypress[ARRAY_SIZE(buttons)+1];
	numpressed = read_keys(keypress);
	if (numpressed) {
		char *kbd_magic_keys = getenv("magic_keys");
		char *suffix;
		/*
		 * loop over all magic keys
		 */
		for (suffix = kbd_magic_keys; *suffix; ++suffix) {
			char *keys;
			char magic[sizeof(kbd_magic_prefix) + 1];
			sprintf(magic, "%s%c", kbd_magic_prefix, *suffix);
			keys = getenv(magic);
			if (keys) {
				if (!strcmp(keys, keypress))
					break;
			}
		}
		if (*suffix) {
			char cmd_name[sizeof(kbd_command_prefix) + 1];
			char *cmd;
			sprintf(cmd_name, "%s%c", kbd_command_prefix, *suffix);
			cmd = getenv(cmd_name);
			if (cmd) {
				setenv("preboot", cmd);
				return;
			}
		}
	}
}

static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},	/* 8-bit eMMC */
	{NULL,		0},
};

int misc_init_r(void)
{
	preboot_keys();

	add_board_boot_modes(board_boot_modes);

	return 0;
}

int board_late_init(void)
{
	int cpurev = get_cpu_rev();

	setenv("cpu", get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board","rc");
	if (!getenv("uboot_defconfig"))
		setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	return 0;
}
