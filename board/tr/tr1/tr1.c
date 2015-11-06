/*
 * Copyright (C) 2015 Nelson Integration LLC
 *
 * Author: Eric Nelson <eric@nelint.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/spi.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <mmc.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <usb.h>
#include <usb/ehci-fsl.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_22K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
		      PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	\
		      PAD_CTL_DSE_40ohm | PAD_CTL_HYS |		\
		      PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define OTGID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
			PAD_CTL_PUS_47K_UP | PAD_CTL_SPEED_LOW |\
			PAD_CTL_DSE_80ohm | PAD_CTL_HYS |	\
			PAD_CTL_SRE_FAST)

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TXD__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RXD__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	/* 4 bit SD */
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__USDHC1_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__USDHC1_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__USDHC1_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__USDHC1_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* CD */
	MX6_PAD_SD1_DAT5__GPIO_5_9 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#define USDHC1_CD_GPIO	IMX_GPIO_NR(5, 9)

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__USDHC2_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__USDHC2_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__USDHC2_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__USDHC2_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT4__USDHC2_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT5__USDHC2_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT6__USDHC2_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT7__USDHC2_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR},
	{USDHC2_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC2_BASE_ADDR:
		ret = 1; /* eMMC is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 * mmc2                    USDHC3
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_direction_input(USDHC1_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret) {
			printf("Warning: failed to initialize "
				"mmc dev %d\n", i);
			return ret;
		}
	}

	return 0;
#else
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u32 val;
	u32 port;

	val = readl(&src_regs->sbmr1);

	/* Boot from USDHC */
	port = (val >> 11) & 0x3;
	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usdhc1_pads,
						 ARRAY_SIZE(usdhc1_pads));
		gpio_direction_input(USDHC1_CD_GPIO);
		usdhc_cfg[0].esdhc_base = USDHC1_BASE_ADDR;
		usdhc_cfg[0].max_bus_width = 4;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		break;
	case 1:
		imx_iomux_v3_setup_multiple_pads(usdhc2_pads,
						 ARRAY_SIZE(usdhc2_pads));
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].max_bus_width = 8;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		break;
	}

	gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}

#define PC	MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC */
struct i2c_pads_info i2c_pad_info1 = {
	.sda = {
		.i2c_mode = MX6_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_I2C1_SDA__GPIO_3_13 | PC,
		.gp = IMX_GPIO_NR(3, 13),
	},
	.scl = {
		.i2c_mode = MX6_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_I2C1_SCL__GPIO_3_12 | PC,
		.gp = IMX_GPIO_NR(3, 12),
	},
};

#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	/* OTG1 */
	MX6_PAD_KEY_COL4__USB_USBOTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EPDC_PWRCOM__ANATOP_USBOTG1_ID | MUX_PAD_CTRL(OTGID_PAD_CTRL),
	/* OTG2 */
	MX6_PAD_KEY_COL5__USB_USBOTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL)
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_usb();

	return 0;
}

int checkboard(void)
{
	puts("Board: TR1\n");

	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#include <libfdt.h>

const struct mx6sl_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_sdqs0 = 0x00003030,
	.dram_sdqs1 = 0x00003030,
	.dram_sdqs2 = 0x00003030,
	.dram_sdqs3 = 0x00003030,
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_dqm2 = 0x00000030,
	.dram_dqm3 = 0x00000030,
	.dram_cas  = 0x00000030,
	.dram_ras  = 0x00000030,
	.dram_sdclk_0 = 0x00000028,
	.dram_reset = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_odt0 = 0x00000008,
	.dram_odt1 = 0x00000008,
};

const struct mx6sl_iomux_grp_regs mx6_grp_ioregs = {
	.grp_b0ds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_b2ds = 0x00000030,
	.grp_b3ds = 0x00000030,
	.grp_addds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x00080000,
};

const struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpdgctrl0 =  0x20000000,
	.p0_mpdgctrl1 =  0x00000000,
	.p0_mprddlctl =  0x4241444a,
	.p0_mpwrdlctl =  0x3030312b,
	.mpzqlp2ctl = 0x1b4700c7,
};

static struct mx6_lpddr2_cfg mem_ddr = {
	.mem_speed = 800,
	.density = 4,
	.width = 32,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.trcd_lp = 2000,
	.trppb_lp = 2000,
	.trpab_lp = 2250,
	.trasmin = 4200,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);

	writel(0x00260324, &ccm->cbcmr);
}

static void spl_dram_init(void)
{
	struct mx6_ddr_sysinfo sysinfo = {
		.dsize = mem_ddr.width / 32,
		.cs_density = 20,
		.ncs = 2,
		.cs1_mirror = 0,
		.walat = 0,
		.ralat = 2,
		.mif3_mode = 3,
		.bi_on = 1,
		.rtt_wr = 0,        /* LPDDR2 does not need rtt_wr rtt_nom */
		.rtt_nom = 0,
		.sde_to_rst = 0,    /* LPDDR2 does not need this field */
		.rst_to_cke = 0x10, /* JEDEC value for LPDDR2: 200us */
		.ddr_type = DDR_TYPE_LPDDR2,
	};
	mx6sl_dram_iocfg(32, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

void reset_cpu(ulong addr)
{
}
#endif
