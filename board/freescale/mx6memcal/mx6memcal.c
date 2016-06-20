/*
 * Copyright (C) 2014 Gateworks Corporation
 * Author: Tim Harvey <tharvey@gateworks.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/* #define DEBUG */
#include <common.h>
#include <i2c.h>
#include <asm/io.h>
#include <asm/arch/iomux.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/iomux-v3.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const uart_pads[] = {
#ifdef CONFIG_UART2_EIM_D26_27
	IOMUX_PADS(PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),	/* SABRE Lite, Nitrogen6x */
	IOMUX_PADS(PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
#elif defined(CONFIG_UART1_CSI0_DAT10_11)
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)), /* Wand */
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
#elif defined(CONFIG_UART1_SD3_DAT6_7)
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
#elif defined(CONFIG_UART1_UART1)
	MX6_PAD_UART1_TXD__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RXD__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
#else
#error select UART console pads
#endif
};

#ifdef CONFIG_DDR3
#define GRP_DDRTYPE	0x000C0000
#else
#define GRP_DDRTYPE	0x00080000
#endif

#define DDR_PKE		0
#define DDR_MODE	0x00020000

#define DRAM_DRIVE_STRENGTH \
	(CONFIG_DRAM_DRIVE_STRENGTH << 3)

#define mmdc0 ((struct mmdc_p_regs volatile *)MMDC_P0_BASE_ADDR)
#define mmdc1 ((struct mmdc_p_regs volatile *)MMDC_P1_BASE_ADDR)
#define mx6q_ddrmux ((struct mx6dq_iomux_ddr_regs volatile *)MX6DQ_IOM_DDR_BASE)
#define mx6dl_ddrmux ((struct mx6sdl_iomux_ddr_regs volatile *)MX6SDL_IOM_DDR_BASE)

static struct mxc_ccm_reg * const imx_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

/*
 * set DQS gating delays based on Read DQS Gating HW Status (mpdghwst) registers
 */
static void set_read_dqs_delay(u32 volatile *reg_st0, u32 volatile *reg_st1, u32 volatile *reg_ctrl)
{
	u32 dg_tmp_val, dg_dl_abs_offset, dg_hc_del, val_ctrl;
	u32 mask;

	/*
	 * DQS gating absolute offset should be modified from
	 * reflecting (HW_DG_LOWx + HW_DG_UPx)/2
	 * to reflecting (HW_DG_UPx - 0x80)
	 */

	val_ctrl = readl(reg_ctrl);
	val_ctrl &= 0xf0000000;

	dg_tmp_val = ((readl(reg_st0) & 0x07ff0000) >> 16) - 0xc0;
	dg_dl_abs_offset = dg_tmp_val & 0x7f;
	dg_hc_del = (dg_tmp_val & 0x780) << 1;

	/* byte 0 values */
	mask = dg_dl_abs_offset + dg_hc_del;

	dg_tmp_val = ((readl(reg_st1) & 0x07ff0000) >> 16) - 0xc0;
	dg_dl_abs_offset = dg_tmp_val & 0x7f;
	dg_hc_del = (dg_tmp_val & 0x780) << 1;

	/* byte 1 values */
	mask |= (dg_dl_abs_offset + dg_hc_del) << 16;

	clrsetbits_le32(reg_ctrl, 0x0f7f0f7f, mask);
}

static void mmdc_precharge_all(int cs0_enable, int cs1_enable)
{
	/*
	 * Issue the Precharge-All command to the DDR device for both chip selects
	 * Note, CON_REQ bit should also remain set
	 * If only using one chip select, then precharge only the desired chip select
	 */
	if (cs0_enable)
		writel(0x04008050, &mmdc0->mdscr);
	if (cs1_enable)
		writel(0x04008058, &mmdc0->mdscr);
}

static void mmdc_force_delay_measurement(int data_bus_size)
{
	writel(0x800, &mmdc0->mpmur0);

	if (data_bus_size == 2)
		writel(0x800, &mmdc1->mpmur0);
}

static void mmdc_reset_read_data_fifos(void)
{
	/*
	 * Reset the read data FIFOs (two resets);
	 * only need to issue reset to PHY0 since in x64 mode,
	 * the reset will also go to PHY1 read data FIFOs reset1
	 */
	clrsetbits_le32(&mmdc0->mpdgctrl0, 0, 0x80000000);
	while (readl((&mmdc0->mpdgctrl0)) & 0x80000000);

	/* read data FIFOs reset2 */
	clrsetbits_le32(&mmdc0->mpdgctrl0, 0, 0x80000000);
	while (readl((&mmdc0->mpdgctrl0)) & 0x80000000);
}

static u32 volatile *const mx6q_sdqs_pads[] = {
	&mx6q_ddrmux->dram_sdqs0,
	&mx6q_ddrmux->dram_sdqs1,
	&mx6q_ddrmux->dram_sdqs2,
	&mx6q_ddrmux->dram_sdqs3,
	&mx6q_ddrmux->dram_sdqs4,
	&mx6q_ddrmux->dram_sdqs5,
	&mx6q_ddrmux->dram_sdqs6,
	&mx6q_ddrmux->dram_sdqs7,
};

static u32 volatile *mx6dl_sdqs_pads[] = {
	&mx6dl_ddrmux->dram_sdqs0,
	&mx6dl_ddrmux->dram_sdqs1,
	&mx6dl_ddrmux->dram_sdqs2,
	&mx6dl_ddrmux->dram_sdqs3,
	&mx6dl_ddrmux->dram_sdqs4,
	&mx6dl_ddrmux->dram_sdqs5,
	&mx6dl_ddrmux->dram_sdqs6,
	&mx6dl_ddrmux->dram_sdqs7,
};

/*
 * maximum value of write leveling delay control registers
 * is 0x2f.
 */
int write_level_err(u32 val){
	return (0x2f < (val & 0x7f))
		||
		(0x2f < ((val >> 16) & 0x7f));
}

static int xmmdc_do_dqs_calibration
	(struct mx6_ddr_sysinfo const *sysinfo,
	 struct mx6_mmdc_calibration *calib)
{
	u32 mask;
	int write_cal_err = 0;
	int cs1_enable_initial;
	int PDDWord = 0x00FFFF00;
	int errorcount = 0;
	int cs0_enable;
	int cs1_enable;
	u32 volatile *const *pads;
	u32 pad;
	u32 ddr_mr1;

	/* check to see which chip selects are enabled */
	cs1_enable_initial = (sysinfo->ncs == 2);

	/*
	 * disable auto refresh
	 * before proceeding with calibration
	 */
	writel(0x0000C000, &mmdc0->mdref);

	/* disable DDR logic power down timer */
	clrsetbits_le32(&mmdc0->mdpdc, 0xff00, 0);

	/* disable Adopt power down timer */
	clrsetbits_le32(&mmdc0->mapsr, 0, 1);

	/* set RALAT and WALAT to min */
	clrsetbits_le32(&mmdc0->mdmisc,
			(3 << 16) | (7 << 6),
			(sysinfo->walat << 16) | (sysinfo->ralat << 6));

	if(sysinfo->dsize == 2)
		clrsetbits_le32(&mmdc1->mdmisc,
				(3 << 16) | (7 << 6),
				(sysinfo->walat << 16) | (sysinfo->ralat << 6));

	/*
	 * disable ZQ calibration
	 * before proceeding with Write Leveling calibration
	 */
	clrsetbits_le32(&mmdc0->mpzqhwctrl, 0x3, 0);
	writel(0x00008000, &mmdc0->mdscr);

	debug("wait for con_ack\n");
	/*
	 * poll to make sure the con_ack bit was asserted
	 */
	while (!(readl(&mmdc0->mdscr) & 0x00004000))
		;

	/*
	 * Configure the external DDR device to enter write leveling mode
	 * through Load Mode Register command
	 * Register setting:
	 * Bits[31:16] MR1 value (0x0080 write leveling enable)
	 * Bit[9] set WL_EN to enable MMDC DQS output
	 * Bits[6:4] set CMD bits for Load Mode Register programming
	 * Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
	 */
	writel(0x00808231, &mmdc0->mdscr);

	/* Activate automatic calibration by setting MPWLGCR[HW_WL_EN] */
	writel(0x00000001, &mmdc0->mpwlgcr);

	debug("wait for MPWLGCR\n");
	/* MMDC de-asserts MPWLGCR[HW_WL_EN] on completion */
	while (readl(&mmdc0->mpwlgcr) & 0x00000001)
		;

	/* check for any errors: check both PHYs for x64 configuration,
	 * if x32, check only PHY0
	 */
	if ((readl(&mmdc0->mpwlgcr) & 0x00000F00) ||
			((sysinfo->dsize == 2)
			 && (readl(&mmdc1->mpwlgcr) & 0x00000F00))) {
		printf("write leveling error 0x%08x/0x%08x\n",
                       readl(&mmdc0->mpwlgcr),
                       readl(&mmdc1->mpwlgcr));
		return -1;
	}

	debug("no errors in write leveling\n");
	if (write_level_err(readl(&mmdc0->mpwldectrl0))
	    ||
            write_level_err(readl(&mmdc0->mpwldectrl1))
	    || ((sysinfo->dsize == 2)
		&&
		(write_level_err(readl(&mmdc0->mpwldectrl0))
                 ||
                 write_level_err(readl(&mmdc0->mpwldectrl1))))) {
		printf("increase WALAT and retry\n");
		printf( "MMDC_MPWLDECTRL0 ch0: 0x%08x\n", readl(&mmdc0->mpwldectrl0));
		printf( "MMDC_MPWLDECTRL1 ch0: 0x%08x\n", readl(&mmdc0->mpwldectrl1));
		if (sysinfo->dsize == 2) {
                        printf( "MMDC_MPWLDECTRL0 ch1: 0x%08x\n", readl(&mmdc1->mpwldectrl0));
                        printf( "MMDC_MPWLDECTRL1 ch1: 0x%08x\n", readl(&mmdc1->mpwldectrl1));
		}
	}

	/*
	 * User should issue MRS command to exit write leveling mode
	 * through Load Mode Register command
	 * Register setting:
	 * Bits[31:16] MR1 value "ddr_mr1" value from initialization
	 * Bit[9] clear WL_EN to disable MMDC DQS output
	 * Bits[6:4] set CMD bits for Load Mode Register programming
	 * Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
	 */
	/* MR1 */
	ddr_mr1 = ((sysinfo->rtt_nom & 1) ? 1 : 0) << 2 |
	          ((sysinfo->rtt_nom & 2) ? 1 : 0) << 6;
	debug("ddr_mr1 == %x\n", ddr_mr1);
	writel(((ddr_mr1 << 16)+0x8031), &mmdc0->mdscr);

	/* restore ZQ settings */
	clrsetbits_le32(&mmdc0->mpzqhwctrl, 0, 0x3);

	/* clear CON_REQ */
	writel(0, &mmdc0->mdscr);

	if (is_cpu_type(MXC_CPU_MX6Q))
		pads = mx6q_sdqs_pads;
	else
		pads = mx6dl_sdqs_pads;

	// Set DQS pullups
	for (pad=0; pad < ARRAY_SIZE(mx6q_sdqs_pads); pad++)
		clrsetbits_le32(pads[pad], 0xf000, 0x7000);

	/*
	 * per the ref manual, issue one refresh cycle mdscr[CMD]= 0x2,
	 * this also sets the CON_REQ bit.
	 */
	writel(0x00008020, &mmdc0->mdscr);
	if (cs1_enable_initial)
		writel(0x00008028, &mmdc0->mdscr);

	debug("wait for con_ack2\n");
	/* poll to make sure the con_ack bit was asserted */
	while (!(readl(&mmdc0->mdscr) & 0x00004000))
		;

	/*
	 * check mdmisc register CALIB_PER_CS to see which CS calibration is
	 * targeted to (under normal cases, it should be cleared as this is the
	 * default value, indicating calibration is directed to CS0). Disable
	 * the other chip select not being target for calibration to avoid any
	 * potential issues This will get re-enabled at end of calibration.
	 */
	if ((readl(&mmdc0->mdmisc) & 0x00100000) == 0)
		mask = (1 << 30); /* clear SDE_1 */
	else
		mask = (1 << 31); /* clear SDE_0 */
	clrsetbits_le32(&mmdc0->mdctl, mask, 0);

	/*
	 * check to see which chip selects are now enabled for the remainder
	 * of the calibration.
	 */
	cs0_enable = (readl(&mmdc0->mdctl) & 0x80000000) >> 31;
	cs1_enable = (readl(&mmdc0->mdctl) & 0x40000000) >> 30;

	mmdc_precharge_all(cs0_enable, cs1_enable);

	/* Write the pre-defined value into MPPDCMPR1 */
	writel(PDDWord, &mmdc0->mppdcmpr1);

	/*
	 * Issue a write access to the external DDR device by setting the bit SW_DUMMY_WR (bit 0)
	 * in the MPSWDAR0 and then poll this bit until it clears to indicate completion of the
	 * write access.
	 */
	clrsetbits_le32(&mmdc0->mpswdar0, 0, 1);
	debug("wait for mpswdar0\n");
	while (readl(&mmdc0->mpswdar0) & 1)
		;

	/*
	 * Set the RD_DL_ABS# bits to their default values (will be calibrated later in
	 * the read delay-line calibration)
	 * Both PHYs for x64 configuration, if x32, do only PHY0
	 */
	writel(0x40404040, &mmdc0->mprddlctl);
	if (sysinfo->dsize == 0x2)
		writel(0x40404040, &mmdc1->mprddlctl);

	/* Force a measurement, for previous delay setup to take effect */
	mmdc_force_delay_measurement(sysinfo->dsize);

	debug("reset data fifos\n");
	/*
	 * Read DQS Gating calibration
	 */
	mmdc_reset_read_data_fifos();

	/*
	 * Start the automatic read DQS gating calibration process by asserting
	 * mpdgctrl0[HW_DG_EN] and mpdgctrl0[DG_CMP_CYC] and then poll
	 * mpdgctrl0[HW_DG_EN]] until this bit clears to indicate completion.
	 * Also, ensure that mpdgctrl0[HW_DG_ERR] is clear to indicate no errors
	 * were seen during calibration. Set bit 30: chooses option to wait 32
	 * cycles instead of 16 before comparing read data
	 */
	clrsetbits_le32(&mmdc0->mpdgctrl0, 0, 1 << 30);

	/*
	 * Wait for ack
	 */
	debug("wait for ack\n");
	while (!(readl(&mmdc0->mpdgctrl0) & (1<<30)))
		;

	/* Set bit 28 (HW_DG_EN) to start automatic read DQS gating calibration */
	clrsetbits_le32(&mmdc0->mpdgctrl0, 0, 1 << 28);

	/*
	 * Poll for completion
	 * mpdgctrl0[HW_DG_EN] should be 0
	 */
	debug("poll for completion\n");
	while (readl(&mmdc0->mpdgctrl0) & (1 << 28))
		;

	/*
	 * Check to see if any errors were encountered during calibration
	 * (check mpdgctrl0[HW_DG_ERR])
	 * check both PHYs for x64 configuration, if x32, check only PHY0
	 */
	if (sysinfo->dsize == 0x2) {
		if ((readl(&mmdc0->mpdgctrl0) & 0x00001000) ||
				(readl(&mmdc1->mpdgctrl0) & 0x00001000)) {
			printf("read calibration err 0x%08x/0x%08x\n",
                               readl(&mmdc0->mpdgctrl0),
                               readl(&mmdc1->mpdgctrl0));
			errorcount++;
		}
	} else {
		if (readl(&mmdc0->mpdgctrl0) & 0x00001000) {
			printf("read calibration err 0x%08x\n",
                               readl(&mmdc0->mpdgctrl0));
			errorcount++;
		}
	}

	debug("error count %d\n", errorcount);
	if (errorcount) {
                printf("%s: read calibration error count %d, bus size %d\n", __func__, errorcount, sysinfo->dsize);
		return errorcount;
	}

	/* now disable mpdgctrl0[DG_CMP_CYC] */
	clrsetbits_le32(&mmdc0->mpdgctrl0, 1 << 30, 0);

	debug("set dqs delays\n");
	/*
	 * DQS gating absolute offset should be modified from reflecting
	 * (HW_DG_LOWx + HW_DG_UPx)/2 to reflecting (HW_DG_UPx - 0x80)
	 */
	set_read_dqs_delay(&mmdc0->mpdghwst0,
                           &mmdc0->mpdghwst1,
                           &mmdc0->mpdgctrl0);

	set_read_dqs_delay(&mmdc0->mpdghwst2,
                           &mmdc0->mpdghwst3,
                           &mmdc0->mpdgctrl1);

	if (sysinfo->dsize == 0x2) {
		set_read_dqs_delay(&mmdc1->mpdghwst0,
                                   &mmdc1->mpdghwst1,
                                   &mmdc1->mpdgctrl0);
		set_read_dqs_delay(&mmdc1->mpdghwst2,
                                   &mmdc1->mpdghwst3,
                                   &mmdc1->mpdgctrl1);
	}

	/*
	 * Read delay Calibration
	 */
	mmdc_reset_read_data_fifos();

	debug("precharge all\n");
	mmdc_precharge_all(cs0_enable, cs1_enable);

	/*
	 * Read delay-line calibration
	 * Start the automatic read calibration process by asserting mprddlhwctl[HW_RD_DL_EN]
	 */
	writel(0x00000030, &mmdc0->mprddlhwctl);

	/*
	 * poll for completion
	 * MMDC indicates that the write data calibration had finished by setting
	 * mprddlhwctl[HW_RD_DL_EN] = 0
	 * Also, ensure that no error bits were set
	 */
	debug("wait for mprddlhwctl\n");
	while (readl(&mmdc0->mprddlhwctl) & 0x00000010)
		;

	/* check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (sysinfo->dsize == 0x2) {
		if ((readl(&mmdc0->mprddlhwctl) & 0x0000000f) ||
				(readl(&mmdc1->mprddlhwctl) & 0x0000000f)) {
			errorcount++;
		}
	} else {
		if (readl(&mmdc0->mprddlhwctl) & 0x0000000f) {
			errorcount++;
		}
	}

	/*
	 * Write delay Calibration
	 */
	mmdc_reset_read_data_fifos();

	debug("precharge all2\n");
	mmdc_precharge_all(cs0_enable, cs1_enable);

	/*
	 * Set the WR_DL_ABS# bits to their default values
	 * Both PHYs for x64 configuration, if x32, do only PHY0
	 */
	writel(0x40404040, &mmdc0->mpwrdlctl);
	if (sysinfo->dsize == 0x2)
		writel(0x40404040, &mmdc1->mpwrdlctl);

	mmdc_force_delay_measurement(sysinfo->dsize);

	/* Start the automatic write calibration process by asserting mpwrdlhwctl0[HW_WR_DL_EN] */
	writel(0x00000030, &mmdc0->mpwrdlhwctl);

	/*
	 * poll for completion
	 * MMDC indicates that the write data calibration had finished by setting
	 * mpwrdlhwctl[HW_WR_DL_EN] = 0
	 * Also, ensure that no error bits were set
	 */
	debug("poll for completion2\n");
	while (readl(&mmdc0->mpwrdlhwctl) & 0x00000010)
		;

	/* check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (sysinfo->dsize == 0x2) {
		if ((readl(&mmdc0->mpwrdlhwctl) & 0x0000000f) ||
				(readl(&mmdc1->mpwrdlhwctl) & 0x0000000f)) {
			errorcount++;
			write_cal_err = 1;
		}
	} else {
		if (readl(&mmdc0->mpwrdlhwctl) & 0x0000000f) {
			errorcount++;
			write_cal_err = 1;
		}
	}

	if (write_cal_err)
		printf("Write calibration error\n");

	mmdc_reset_read_data_fifos();

	/* enable DDR logic power down timer */
	clrsetbits_le32(&mmdc0->mdpdc, 0, 0x00005500);

	/* enable auto power down timer */
	clrsetbits_le32(&mmdc0->mapsr, 1, 0);

	clrsetbits_le32(&mmdc0->mdmisc,
			(3 << 16) | (7 << 6),
			(sysinfo->walat << 16) | (sysinfo->ralat << 6));

	/* clear DQS pull ups */
	for (pad=0; pad < ARRAY_SIZE(mx6q_sdqs_pads); pad++)
		writel(readl(pads[pad]) & ~0xF000,pads[pad]);

	/* re-enable SDE (chip selects) if they were set initially */
	clrsetbits_le32(&mmdc0->mdctl, 0,
			(1 << 31) | (cs1_enable_initial << 30));

	/* enable to auto refresh */
	writel((0 << 14) | /* REF_SEL: Periodic refresh cycle: 64kHz */
	       (3 << 11)   /* REFR: Refresh Rate - 4 refreshes */,
	       &mmdc0->mdref);

	/* clear the mdscr (including the con_req bit) */
	writel(0x0, &mmdc0->mdscr); /* CS0 */

	debug("skip wait for con ack on completion\n");
	/* poll to make sure the con_ack bit is clear */
	while (readl(&mmdc0->mdscr) & 0x00004000)
		;

	memset(calib, 0xFF, sizeof(*calib));
	calib->p0_mpwldectrl0 = readl(&mmdc0->mpwldectrl0);
	calib->p0_mpwldectrl1 = readl(&mmdc0->mpwldectrl1);
	calib->p0_mpdgctrl0 = readl(&mmdc0->mpdgctrl0);
	calib->p0_mpdgctrl1 = readl(&mmdc0->mpdgctrl1);
	calib->p0_mprddlctl = readl(&mmdc0->mprddlctl);
	calib->p0_mpwrdlctl = readl(&mmdc0->mpwrdlctl);
	if (sysinfo->dsize == 2) {
		calib->p1_mpwldectrl0 = readl(&mmdc1->mpwldectrl0);
		calib->p1_mpwldectrl1 = readl(&mmdc1->mpwldectrl1);
		calib->p1_mpdgctrl0 = readl(&mmdc1->mpdgctrl0);
		calib->p1_mpdgctrl1 = readl(&mmdc1->mpdgctrl1);
		calib->p1_mprddlctl = readl(&mmdc1->mprddlctl);
		calib->p1_mpwrdlctl = readl(&mmdc1->mpwrdlctl);
	}

	debug("returning with %d errors\n", errorcount);
	return errorcount;
}

/* configure MX6Q/DUAL mmdc DDR io registers */
static struct mx6dq_iomux_ddr_regs const mx6dq_ddr_ioregs = {
	/* SDCLK[0:1], CAS, RAS, Reset: Differential input, 40ohm */
	.dram_sdclk_0 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_sdclk_1 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_cas = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_ras = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_reset = DDR_MODE+DRAM_DRIVE_STRENGTH,
	/* SDCKE[0:1]: 100k pull-up */
	.dram_sdcke0 = 0x00003000,
	.dram_sdcke1 = 0x00003000,
	/* SDBA2: pull-up disabled */
	.dram_sdba2 = 0x00000000,
	/* SDODT[0:1]: 100k pull-up, 40 ohm */
	.dram_sdodt0 = 0x00003000+DRAM_DRIVE_STRENGTH,
	.dram_sdodt1 = 0x00003000+DRAM_DRIVE_STRENGTH,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.dram_sdqs0 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs1 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs2 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs3 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs4 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs5 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs6 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs7 = DRAM_DRIVE_STRENGTH,

	/* DQM[0:7]: Differential input, 40 ohm */
	.dram_dqm0 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm1 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm2 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm3 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm4 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm5 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm6 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm7 = DDR_MODE+DRAM_DRIVE_STRENGTH,
};

/* configure MX6Q/DUAL mmdc GRP io registers */
static struct mx6dq_iomux_grp_regs const mx6dq_grp_ioregs = {
	/* DDR3 */
	.grp_ddr_type = GRP_DDRTYPE,
	.grp_ddrmode_ctl = DDR_MODE,
	/* disable DDR pullups */
	.grp_ddrpke = DDR_PKE,
	/* ADDR[00:16], SDBA[0:1]: 40 ohm */
	.grp_addds = DRAM_DRIVE_STRENGTH,
	/* CS0/CS1/SDBA2/CKE0/CKE1/SDWE: 40 ohm */
	.grp_ctlds = DRAM_DRIVE_STRENGTH,
	/* DATA[00:63]: Differential input, 40 ohm */
	.grp_ddrmode = DDR_MODE,
	.grp_b0ds = DRAM_DRIVE_STRENGTH,
	.grp_b1ds = DRAM_DRIVE_STRENGTH,
	.grp_b2ds = DRAM_DRIVE_STRENGTH,
	.grp_b3ds = DRAM_DRIVE_STRENGTH,
	.grp_b4ds = DRAM_DRIVE_STRENGTH,
	.grp_b5ds = DRAM_DRIVE_STRENGTH,
	.grp_b6ds = DRAM_DRIVE_STRENGTH,
	.grp_b7ds = DRAM_DRIVE_STRENGTH,
};

static struct mx6sdl_iomux_ddr_regs const mx6sdl_ddr_ioregs = {
	/* SDCLK[0:1], CAS, RAS, Reset: Differential input, 40ohm */
	.dram_sdclk_0 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_sdclk_1 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_cas = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_ras = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_reset = DDR_MODE+DRAM_DRIVE_STRENGTH,
	/* SDCKE[0:1]: 100k pull-up */
	.dram_sdcke0 = 0x00003000,
	.dram_sdcke1 = 0x00003000,
	/* SDBA2: pull-up disabled */
	.dram_sdba2 = 0x00000000,
	/* SDODT[0:1]: 100k pull-up, 40 ohm */
	.dram_sdodt0 = 0x00003000+DRAM_DRIVE_STRENGTH,
	.dram_sdodt1 = 0x00003000+DRAM_DRIVE_STRENGTH,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.dram_sdqs0 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs1 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs2 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs3 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs4 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs5 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs6 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs7 = DRAM_DRIVE_STRENGTH,

	/* DQM[0:7]: Differential input, 40 ohm */
	.dram_dqm0 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm1 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm2 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm3 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm4 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm5 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm6 = DDR_MODE+DRAM_DRIVE_STRENGTH,
	.dram_dqm7 = DDR_MODE+DRAM_DRIVE_STRENGTH,
};

/* configure MX6SOLO/DUALLITE mmdc GRP io registers */
static struct mx6sdl_iomux_grp_regs const mx6sdl_grp_ioregs = {
	/* DDR3 */
	.grp_ddr_type = GRP_DDRTYPE,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.grp_ddrmode_ctl = DDR_MODE,
	/* disable DDR pullups */
	.grp_ddrpke = DDR_PKE,
	/* ADDR[00:16], SDBA[0:1]: 40 ohm */
	.grp_addds = DRAM_DRIVE_STRENGTH,
	/* CS0/CS1/SDBA2/CKE0/CKE1/SDWE: 40 ohm */
	.grp_ctlds = DRAM_DRIVE_STRENGTH,
	/* DATA[00:63]: Differential input, 40 ohm */
	.grp_ddrmode = DDR_MODE,
	.grp_b0ds = DRAM_DRIVE_STRENGTH,
	.grp_b1ds = DRAM_DRIVE_STRENGTH,
	.grp_b2ds = DRAM_DRIVE_STRENGTH,
	.grp_b3ds = DRAM_DRIVE_STRENGTH,
	.grp_b4ds = DRAM_DRIVE_STRENGTH,
	.grp_b5ds = DRAM_DRIVE_STRENGTH,
	.grp_b6ds = DRAM_DRIVE_STRENGTH,
	.grp_b7ds = DRAM_DRIVE_STRENGTH,
};

const struct mx6sl_iomux_ddr_regs mx6sl_ddr_ioregs = {
	.dram_sdqs0 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs1 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs2 = DRAM_DRIVE_STRENGTH,
	.dram_sdqs3 = DRAM_DRIVE_STRENGTH,
	.dram_dqm0 = DRAM_DRIVE_STRENGTH,
	.dram_dqm1 = DRAM_DRIVE_STRENGTH,
	.dram_dqm2 = DRAM_DRIVE_STRENGTH,
	.dram_dqm3 = DRAM_DRIVE_STRENGTH,
	.dram_cas  = DRAM_DRIVE_STRENGTH,
	.dram_ras  = DRAM_DRIVE_STRENGTH,
	.dram_sdclk_0 = DRAM_DRIVE_STRENGTH,
	.dram_reset = DRAM_DRIVE_STRENGTH,
	.dram_sdba2 = 0x00020000,
	.dram_odt0 = 0x00030000+DRAM_DRIVE_STRENGTH,
	.dram_odt1 = 0x00030000+DRAM_DRIVE_STRENGTH,
};

const struct mx6sl_iomux_grp_regs mx6sl_grp_ioregs = {
	.grp_b0ds = DRAM_DRIVE_STRENGTH,
	.grp_b1ds = DRAM_DRIVE_STRENGTH,
	.grp_b2ds = DRAM_DRIVE_STRENGTH,
	.grp_b3ds = DRAM_DRIVE_STRENGTH,
	.grp_addds = DRAM_DRIVE_STRENGTH,
	.grp_ctlds = DRAM_DRIVE_STRENGTH,
	.grp_ddrmode_ctl = DDR_MODE,
	.grp_ddrpke = DDR_PKE,
	.grp_ddrmode = DDR_MODE,
	.grp_ddr_type = GRP_DDRTYPE,
};

static struct mx6_ddr_sysinfo const sysinfo = {
	/* width of data bus:0=16,1=32,2=64 */
#if CONFIG_DDRWIDTH == 32
	.dsize = 1,
#elif CONFIG_DDRWIDTH == 64
	.dsize = 2,
#else
#error missing CONFIG_DDRWIDTH
#endif
	/* config for full 4GB range so that get_mem_size() works */
	.cs_density = 32, /* 32Gb per CS */

	/* # of chip selects */
	.ncs = CONFIG_DDRCS,
	.cs1_mirror = 0,
	.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
#ifdef RTT_NOM_120OHM
	.rtt_nom = 2 /*DDR3_RTT_120_OHM*/,	/* RTT_Nom = RZQ/2 */
#else
	.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
#endif
	.walat = CONFIG_WALAT,	/* Write additional latency */
	.ralat = CONFIG_RALAT,	/* Read additional latency */
	.mif3_mode = 3,	/* Command prediction working mode */
	.bi_on = 1,	/* Bank interleaving enabled */
#ifdef CONFIG_DDR3
	.sde_to_rst = 0x10,	/* JEDEC value for LPDDR2 - 200us */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.ddr_type = DDR_TYPE_DDR3,
#else
	.sde_to_rst = 0,	/* LPDDR2 does not need this field */
	.rst_to_cke = 0x10,	/* JEDEC value for LPDDR2: 200us */
	.ddr_type = DDR_TYPE_LPDDR2,
#endif
};

#ifdef CONFIG_MT41K512M16TNA
/* MT41K512M16TNA-125 */
static struct mx6_ddr3_cfg const ddrtype = {
	.mem_speed = 1600,
	.density = 8,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 1,
	.trcd = 1375,
	.trcmin = 5062,
	.trasmin = 3750,
};
#elif defined(CONFIG_MT41K128M16JT)
/* MT41K128M16JT-125 */
static struct mx6_ddr3_cfg const ddrtype = {
	.mem_speed = 1600,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};
#elif defined(CONFIG_H5TQ4G63AFR)
static struct mx6_ddr3_cfg const ddrtype = {
	.mem_speed = 1600,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};
#elif defined CONFIG_H5TQ2G63DFR
static struct mx6_ddr3_cfg const ddrtype = {
	.mem_speed = 1333,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1350,
	.trcmin = 4950,
	.trasmin = 3600,
};
#elif defined (CONFIG_MT42L256M32D2LG)
static struct mx6_lpddr2_cfg ddrtype = {
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
#elif defined (CONFIG_MT29PZZZ4D4BKESK)
static struct mx6_lpddr2_cfg ddrtype = {
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
#else
#error please select DDR type using menuconfig
#endif

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void display_calibration(struct mx6_mmdc_calibration *calib)
{
	printf(".p0_mpdgctrl0\t= 0x%08X\n", calib->p0_mpdgctrl0);
	printf(".p0_mpdgctrl1\t= 0x%08X\n", calib->p0_mpdgctrl1);
	printf(".p0_mprddlctl\t= 0x%08X\n", calib->p0_mprddlctl);
	printf(".p0_mpwrdlctl\t= 0x%08X\n", calib->p0_mpwrdlctl);
	printf(".p0_mpwldectrl0\t= 0x%08X\n",calib->p0_mpwldectrl0);
	printf(".p0_mpwldectrl1\t= 0x%08X\n",calib->p0_mpwldectrl1);
	if (sysinfo.dsize == 2) {
		printf(".p1_mpdgctrl0\t= 0x%08X\n", calib->p1_mpdgctrl0);
		printf(".p1_mpdgctrl1\t= 0x%08X\n", calib->p1_mpdgctrl1);
		printf(".p1_mprddlctl\t= 0x%08X\n", calib->p1_mprddlctl);
		printf(".p1_mpwrdlctl\t= 0x%08X\n", calib->p1_mpwrdlctl);
		printf(".p1_mpwldectrl0\t= 0x%08X\n",calib->p1_mpwldectrl0);
		printf(".p1_mpwldectrl1\t= 0x%08X\n",calib->p1_mpwldectrl1);
	}
#ifdef CONFIG_IMXIMAGE_OUTPUT
	printf("DATA 4 MX6_MMDC_P0_MPDGCTRL0\t= 0x%08X\n", calib->p0_mpdgctrl0);
	printf("DATA 4 MX6_MMDC_P0_MPDGCTRL1\t= 0x%08X\n", calib->p0_mpdgctrl1);
	printf("DATA 4 MX6_MMDC_P0_MPRDDLCTL\t= 0x%08X\n", calib->p0_mprddlctl);
	printf("DATA 4 MX6_MMDC_P0_MPWRDLCTL\t= 0x%08X\n", calib->p0_mpwrdlctl);
	printf("DATA 4 MX6_MMDC_P0_MPWLDECTRL0\t= 0x%08X\n",calib->p0_mpwldectrl0);
	printf("DATA 4 MX6_MMDC_P0_MPWLDECTRL1\t= 0x%08X\n",calib->p0_mpwldectrl1);
	if (sysinfo.dsize == 2) {
		printf("DATA 4 MX6_MMDC_P1_mpdgctrl0\t= 0x%08X\n", calib->p1_mpdgctrl0);
		printf("DATA 4 MX6_MMDC_P1_mpdgctrl1\t= 0x%08X\n", calib->p1_mpdgctrl1);
		printf("DATA 4 MX6_MMDC_P1_mprddlctl\t= 0x%08X\n", calib->p1_mprddlctl);
		printf("DATA 4 MX6_MMDC_P1_mpwrdlctl\t= 0x%08X\n", calib->p1_mpwrdlctl);
		printf("DATA 4 MX6_MMDC_P1_mpwldectrl0\t= 0x%08X\n",calib->p1_mpwldectrl0);
		printf("DATA 4 MX6_MMDC_P1_mpwldectrl1\t= 0x%08X\n",calib->p1_mpwldectrl1);
	}
#endif
}

/*
 * called from C runtime startup code (arch/arm/lib/crt0.S:_main)
 * - we have a stack and a place to store GD, both in SRAM
 * - no variable global data is available
 */
void board_init_f(ulong dummy)
{
	int errs;
	struct mx6_mmdc_calibration calibration;
	u32 cpurev = get_cpu_rev();

	memset((void *)gd, 0, sizeof(struct global_data));

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

	SETUP_IOMUX_PADS(uart_pads);

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	if (sysinfo.dsize != 1) {
		if (is_cpu_type(MXC_CPU_MX6SX)
		    || is_cpu_type(MXC_CPU_MX6UL)
		    || is_cpu_type(MXC_CPU_MX6SL)) {
			printf("cpu type 0x%x doesn't support 64-bit bus\n",
			       get_cpu_type());
			reset_cpu(0);
		}
	}
	printf("CPU:   Freescale i.MX%s rev%d.%d at %d MHz\n",
		get_imx_type((cpurev & 0xFF000) >> 12),
		(cpurev & 0x000F0) >> 4,
		(cpurev & 0x0000F) >> 0,
		mxc_get_clock(MXC_ARM_CLK) / 1000000);
#ifdef CONFIG_MX6SL
	mx6sl_dram_iocfg(CONFIG_DDRWIDTH, &mx6sl_ddr_ioregs,
			 &mx6sl_grp_ioregs);
#else
	if (is_cpu_type(MXC_CPU_MX6Q)) {
		mx6dq_dram_iocfg(CONFIG_DDRWIDTH, &mx6dq_ddr_ioregs,
				 &mx6dq_grp_ioregs);
	}
	else {
		mx6sdl_dram_iocfg(CONFIG_DDRWIDTH, &mx6sdl_ddr_ioregs,
				  &mx6sdl_grp_ioregs);
	}
#endif
	mx6_dram_cfg(&sysinfo, NULL, &ddrtype);

	printf("calling nelson's calibration routine\n");
	errs = xmmdc_do_dqs_calibration(&sysinfo, &calibration);
	if (!errs) {
		printf("completed successfully\n");
		display_calibration(&calibration);
	} else
		printf("completed with %d errors\n", errs);

	printf("calling mainline calibration routine\n");
	errs = mmdc_do_write_level_calibration(&sysinfo, &calibration);
	if (errs) {
		printf("error %d from write level calibration\n", errs);
	} else {
		errs = mmdc_do_dqs_calibration(&sysinfo, &calibration);
		if (errs) {
			printf("error %d from write level calibration\n", errs);
		} else {
			printf("completed successfully\n");
			display_calibration(&calibration);
		}
	}

	reset_cpu(0);
}

