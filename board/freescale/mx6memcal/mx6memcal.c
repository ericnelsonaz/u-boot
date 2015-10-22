/*
 * Copyright (C) 2014 Gateworks Corporation
 * Author: Tim Harvey <tharvey@gateworks.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <asm/io.h>
#include <asm/arch/iomux.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const uart_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),	/* SABRE Lite, Nitrogen6x */
	IOMUX_PADS(PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

#define mmdc0 ((struct mmdc_p_regs volatile *)MMDC_P0_BASE_ADDR)
#define mmdc1 ((struct mmdc_p_regs volatile *)MMDC_P1_BASE_ADDR)
#define mx6q_ddrmux ((struct mx6dq_iomux_ddr_regs volatile *)MX6DQ_IOM_DDR_BASE)
#define mx6dl_ddrmux ((struct mx6sdl_iomux_ddr_regs volatile *)MX6SDL_IOM_DDR_BASE)

/* called from SPL board_init_f() */
int board_early_init_f(void)
{
	SETUP_IOMUX_PADS(uart_pads);
	return 0;
}

static struct mxc_ccm_reg * const imx_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
#define DIV2    1
#define DIV1    0

/*
 * For DDR frequencies >= 135MHz and <= 500MHz
 * Two parameters are passed in: 
 * i = increment variable, which the "frac" value is calculated
 * divider = this indicates whether or not to div-by-1 or div-by-2
*/
static void increase_528pfd2(int i, int divider)
{
	u32 frac;

	// switch the periph_clk domain to "pll3" (480MHz USB OTG PLL?)
	// first, set CBCMR:periph_clk2_sel to select "pll3" bits[13:12]=00 for the periph_clk2_clk clock mux
	clrsetbits_le32(&imx_ccm->cbcmr, 0x00003000, 0x00000000);
	// next, set CBCDR:periph_clk_sel to the periph_clk2_sel clock source (that got set up for "pll3")
	// bit[25]=1
	clrsetbits_le32(&imx_ccm->cbcdr, 0x02000000, 0x02000000);

	// ungate PFD2, clear bit 23
	clrsetbits_le32(&imx_ccm->analog_pfd_528, 0x00800000, 0x00000000);

	// also, make sure DIV_SELECT is set to 1 in the CCM_ANALOG_PLL_528 (HW_ANADIG_PLL_528_RW)
	// register to ensure Fout=Fref*22 (24MHz * 22) = 528MHz
	clrsetbits_le32(&imx_ccm->analog_pll_528, 0x00000001, 0x00000001);

	// wait for 528MHz PLL to lock
	while (!(readl((u32)&imx_ccm->analog_pll_528) & 0x80000000)) ;

	// set dividers that are sourced from the 528PLL: AXI_CLK_ROOT and 132M_CLK_ROOT to default 
	// set CBCDR:axi_podf to div-by-1 bits[18:16]=001 and CBCDR:ahb_podf to div-by-4 bits[12:10]=011
	clrsetbits_le32(&imx_ccm->cbcdr, 0x00071C00, 0x00010C00);

	// configure the MMDC divider in the CCM
	// also configure the frac divider
	if (divider == DIV1) {
		// mmdc_ch0_axi_podf is div-by-1
		clrsetbits_le32(&imx_ccm->cbcdr, 0x00380000, 0x00000000);
		// poll the mmdr_ch0_podf_busy bit
		while (readl((u32)&imx_ccm->cdhipr) & 0x00000010) ;

		frac = 53 - i;
	}
	else if (divider == DIV2) {
		// mmdc_ch0_axi_podf is div-by-2
		clrsetbits_le32(&imx_ccm->cbcdr, 0x00380000, 0x00080000);
		// poll the mmdr_ch0_podf_busy bit
		while (readl((u32)&imx_ccm->cdhipr) & 0x00000010) ;

		frac = 35 - i;
	}
	else {
		printf("** Incorrect divder value \n");
		reset_cpu(0);
		frac = 0;
	}

	// write the frac value into PFD2
	frac = frac << 16;
	clrsetbits_le32(&imx_ccm->analog_pfd_528, 0x003F0000, frac);

	// Switch CBCMR pre_periph_clk_sel to the 400M PFD2 source 
	// bits[19:18] = 01
	clrsetbits_le32(&imx_ccm->cbcmr, 0x000C0000, 0x00040000);

	// Now switch CBCDR:periph_clk_sel to the pre_periph_clk_sel mux clock source 
	// bit[25]=0
	clrsetbits_le32(&imx_ccm->cbcdr, 0x02000000, 0x00000000);
}

/* For DDR frequencies >= 528MHz  */
/* PLL denom is set to 1000, numerator is varied from 0, 200, 400, 600, 800, 999  */
/* This yields fractional part as 0, 0.2, 0.4, 0.6, 0.8, and 0.999 */
static u32 increase_pll528(int i)
{
	u32 ddr_freq_above_528;     // pass back the integer value fo the ddr freq
	double ddr_freq_act;        // this variable is used to calculate the actual ddr freq
	int temp;

	// make sure that mmdc_ch0_axi_podf is div-by-1
	clrsetbits_le32(&imx_ccm->cbcdr, 0x00380000, 0x00000000);
	
	// poll the mmdr_ch0_podf_busy bit
	while (readl((u32)&imx_ccm->cdhipr) & 0x00000010) ;
	
	temp = readl((u32)&imx_ccm->cbcdr);
	if ((temp & 0x00380000) != 0) {
		printf("MMDC podf didn't get changed! \n");
		clrsetbits_le32(&imx_ccm->cbcdr, 0x00380000, 0x00000000);
	}

	// switch the periph_clk domain to "pll3" (480MHz USB OTG PLL3) while we are re-programming 528PLL
	// first, set CBCMR:periph_clk2_sel to select "pll3" bits[13:12]=00 for the periph_clk2_clk clock mux
	clrsetbits_le32(&imx_ccm->cbcmr, 0x00003000, 0x00000000);
    
	// next, set CBCDR:periph_clk_slk to the periph_clk2_clk clock source (that got set up for "pll3")
	// bit[25]=1
	clrsetbits_le32(&imx_ccm->cbcdr, 0x02000000, 0x02000000);

	// set div_sel to 1 to have a multiplier of 22 (bit 0); 24*(22 + num/den)
	clrsetbits_le32(&imx_ccm->analog_pll_528, 0x00000001, 0x00000001);

	// MFD is always 1000 (to allow for div-by-1000 in the equation MFN/(MFD)
	writel(1000,(u32)&imx_ccm->analog_pll_528_denom);

	// increase dividers that are sourced from the 528PLL: AXI_CLK_ROOT and 132M_CLK_ROOT
	// set CBCDR:axi_podf to div-by-3 bits[18:16]=010 and CBCDR:ahb_podf to div-by-6 bits[12:10]=101
	clrsetbits_le32(&imx_ccm->cbcdr, 0x00071C00, 0x00021400);

	// 528MHz < freq < 576
	// this chooses the PLL output as source for DDR
	if ((i >= 0) && (i <= 10)) {
		// first, for the pre_periph_clk_sel mux, choose the PLL2 output (not PFD)
		// Ensure pre_periph_clk_sel mux is set to the 528PLL, CBCMR:pre_periph_clk_sel bits[19:18]=00
		clrsetbits_le32(&imx_ccm->cbcmr, 0x000C0000, 0x00000000);
		
		writel(i * (200),(u32)&imx_ccm->analog_pll_528_num);
		// wait for 528MHz PLL to lock
		while (!(readl((u32)&imx_ccm->analog_pll_528) & 0x80000000)) ;
		
		ddr_freq_act = (float)24 *(22 + (float)(i * 200) / 1000);
		ddr_freq_above_528 = ddr_freq_act;  // turn it to an integer
		// next, set CBCDR:periph_clk_slk to the 528PLL
		// bit[25]=0
		clrsetbits_le32(&imx_ccm->cbcdr, 0x02000000, 0x00000000);
		
		return ddr_freq_above_528;
	// 580MHz < freq < 591MHz
	// this chooses the PFD2 output as source for DDR
	} else if ((i >= 11) && (i <= 29)) {
		// first, for the pre_periph_clk_sel mux, choose the PFD2 output
		// ungate PFD2, clear bit 23
		clrsetbits_le32(&imx_ccm->analog_pfd_528, 0x00800000, 0x00000000);
		// Switch CBCMR pre_periph_clk_sel to the PFD2 source 
		// bits[19:18] = 01
		clrsetbits_le32(&imx_ccm->cbcmr, 0x000C0000, 0x00040000);

		// 580MHz < freq < 591MHz
		// use 480Mhz (div_sel=0) and PFD2 with N=16
		if ((i >= 11) && (i <= 13)) {
			// set div_sel to 0 to have a multiplier of 20 (bit 0); 24*(20 + num/den)
			clrsetbits_le32(&imx_ccm->analog_pll_528, 0x00000001, 0x00000000);
			
			// now set PFD2 frac (N) to 16
			clrsetbits_le32(&imx_ccm->analog_pfd_528, 0x003F0000, 0x00100000);
			
			if (i == 11) {
				writel(1500,(u32)&imx_ccm->analog_pll_528_num);
				ddr_freq_act = (float)24 *(20 + (float)1500 / 1000) * (float)18 / 16;
			} else if (i == 12) {
				writel(1700,(u32)&imx_ccm->analog_pll_528_num);
				ddr_freq_act = (float)24 *(20 + (float)1700 / 1000) * (float)18 / 16;
			} else if (i == 13) {
				writel(1900,(u32)&imx_ccm->analog_pll_528_num);
				ddr_freq_act = (float)24 *(20 + (float)1900 / 1000) * (float)18 / 16;
			}
			// wait for 528MHz PLL to lock
			while (!(readl((u32)&imx_ccm->analog_pll_528) & 0x80000000)) ;
	
			ddr_freq_above_528 = ddr_freq_act;  // turn it to an integer
			// next, set CBCDR:periph_clk_slk to the 528PLL mux
			// bit[25]=0
			clrsetbits_le32(&imx_ccm->cbcdr, 0x02000000, 0x00000000);
	
			return ddr_freq_above_528;
	
		/* freq = 594 to 620MHz */
		// use 528Mhz (div_sel=1) and PFD2 with N=16
		} else if ((i >= 14) && (i <= 19)) {
			// set div_sel to 0 to have a multiplier of 22 (bit 1); 24*(22 + num/den)
			clrsetbits_le32(&imx_ccm->analog_pll_528, 0x00000001, 0x00000001);
	
			// program the PLL num/den fractional part to increase PLL output
			writel((i - 14) * 200,(u32)&imx_ccm->analog_pll_528_num);
	
			// wait for 528MHz PLL to lock
			while (!(readl((u32)&imx_ccm->analog_pll_528) & 0x80000000)) ;
	
			// now set PFD2 frac (N) to 16
			clrsetbits_le32(&imx_ccm->analog_pfd_528, 0x003F0000, 0x00100000);
	
			ddr_freq_act = (float)24 *(22 + (float)(i - 14) * 200 / 1000) * (float)18 / 16;
			ddr_freq_above_528 = ddr_freq_act;  // turn it to an integer
			// next, set CBCDR:periph_clk_slk to the 528PLL mux
			// bit[25]=0
			clrsetbits_le32(&imx_ccm->cbcdr, 0x02000000, 0x00000000);
			
			return ddr_freq_above_528;
	
		/* freq = 623 to 678MHz */
		// use 480Mhz (div_sel=0) and PFD2 with N=14        
		} else if ((i >= 20) && (i <= 29)) {
			// set div_sel to 0 to have a multiplier of 20 (bit 0); 24*(20 + num/den)
			clrsetbits_le32(&imx_ccm->analog_pll_528, 0x00000001, 0x00000000);
	
			// program the PLL num/den fractional part to increase PLL output
			writel((i - 19) * 200,(u32)&imx_ccm->analog_pll_528_num);
	
			// now set PFD2 frac (N) to 14
			clrsetbits_le32(&imx_ccm->analog_pfd_528, 0x003F0000, 0x000E0000);
	
			ddr_freq_act = (float)24 *(20 + (float)(i - 19) * 200 / 1000) * (float)18 / 14;
			ddr_freq_above_528 = ddr_freq_act;  // turn it to an integer
			// next, set CBCDR:periph_clk_slk to the 528PLL mux
			// bit[25]=0
			clrsetbits_le32(&imx_ccm->cbcdr, 0x02000000, 0x00000000);
			
			return ddr_freq_above_528;
		}
	} else {                            // fail if invalid freq
		printf("Error, outside of supported frequencies, setting back to 528MHz \n");
		writel(0,(u32)&imx_ccm->analog_pll_528_num);
	}
	return 0;
}

#define DDR_DIV_528MHZ 35
#define DDR_DIV_400MHZ 29

static u32 set_ddr_clock(int i)
{
	u32 ddr_freq;

	// First make sure that the PHY measurement unit is NOT in bypass mode 
	clrsetbits_le32(&mmdc0->mpmur0, 0x00000400, 0x00000000);
	clrsetbits_le32(&mmdc1->mpmur0, 0x00000400, 0x00000000);

	// use PFD2 as clock source, with div-by-2    
	// 135MHz < DDR freq < 264MHz
	if ((i >= 0) && (i <= 17)) {
                increase_528pfd2(i, DIV2);
                ddr_freq = (528 * 18 / (35 - i)) / 2;
                return ddr_freq;

		// use PFD2 as clock source, div-by-1
		// 271MHz < DDR freq < 500MHz
        } else if ((i >= 18) && (i <= 34)) {
		increase_528pfd2(i, DIV1);
		ddr_freq = (528 * 18 / (53 - i));
		return ddr_freq;

		// for freq >=528MHz
		// will either use the PLL2 or PFD2 output depending on freq
	} else if ((i >= 35) && (i <= 59)) {
		// have to normalize the variable 'i' starting back at zero
		ddr_freq = increase_pll528(i - 35);
		return ddr_freq;
	} else {
		printf("**Not a supported freq, stopping \n");
		reset_cpu(0);
	}
        return 0;
}

static int mmdc_do_write_level_calibration(void)
{
	u32 zq_val;
	u32 ddr_mr1 = is_cpu_type(MXC_CPU_MX6Q) ? 0x42 : 0x4;

	/*
	 * disable ZQ calibration
	 * before proceeding with Write Leveling calibration
	 */
	zq_val = readl(&mmdc0->mpzqhwctrl);
	writel(zq_val & ~(0x3), &mmdc0->mpzqhwctrl);
	writel(0x00008000, &mmdc0->mdscr);
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

	/* Upon completion of this process the MMDC de-asserts the MPWLGCR[HW_WL_EN] */
	while (readl(&mmdc0->mpwlgcr) & 0x00000001);

	/* check for any errors: check both PHYs for x64 configuration, if x32, check only PHY0 */
	if ((readl(&mmdc0->mpwlgcr) & 0x00000F00) ||
			(readl(&mmdc1->mpwlgcr) & 0x00000F00)) {
		printf("write leveling error 0x%08x/0x%08x\n", 
                       readl(&mmdc0->mpwlgcr),
                       readl(&mmdc1->mpwlgcr));
		return -1;
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
	writel(((ddr_mr1 << 16)+0x8031), &mmdc0->mdscr);

	writel(zq_val, &mmdc0->mpzqhwctrl);

	/* clear CON_REQ */
	writel(0, &mmdc0->mdscr);

	return 0;
}

static void modify_dg_result(u32 volatile *reg_st0, u32 volatile *reg_st1, u32 volatile *reg_ctrl)
{
	u32 dg_tmp_val, dg_dl_abs_offset, dg_hc_del, val_ctrl;

	/*
	 * DQS gating absolute offset should be modified from reflecting (HW_DG_LOWx + HW_DG_UPx)/2
	 * to reflecting (HW_DG_UPx - 0x80)
	 */

	val_ctrl = readl(reg_ctrl);
	val_ctrl &= 0xf0000000;

	dg_tmp_val = ((readl(reg_st0) & 0x07ff0000) >> 16) - 0xc0;
	dg_dl_abs_offset = dg_tmp_val & 0x7f;
	dg_hc_del = (dg_tmp_val & 0x780) << 1;

	val_ctrl |= dg_dl_abs_offset + dg_hc_del;

	dg_tmp_val = ((readl(reg_st1) & 0x07ff0000) >> 16) - 0xc0;
	dg_dl_abs_offset = dg_tmp_val & 0x7f;
	dg_hc_del = (dg_tmp_val & 0x780) << 1;

	val_ctrl |= (dg_dl_abs_offset + dg_hc_del) << 16;

	writel(val_ctrl, reg_ctrl);
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

	if (data_bus_size == 0x2)
		writel(0x800, &mmdc1->mpmur0);
}

static void mmdc_reset_read_data_fifos(void)
{
	uint32_t v;

	/*
	 * Reset the read data FIFOs (two resets); only need to issue reset to PHY0 since in x64
	 * mode, the reset will also go to PHY1
	 * read data FIFOs reset1
	 */
	v = readl(&mmdc0->mpdgctrl0);
	v |= 0x80000000;
	writel(v, &mmdc0->mpdgctrl0);

	while (readl((&mmdc0->mpdgctrl0)) & 0x80000000);

	/* read data FIFOs reset2 */
	v = readl(&mmdc0->mpdgctrl0);
	v |= 0x80000000;
	writel(v, &mmdc0->mpdgctrl0);

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

static int mmdc_do_dqs_calibration(void)
{
	u32 esdmisc_val, v;
	int write_cal_err = 0;
	int temp_ref;
	int cs0_enable_initial;
	int cs1_enable_initial;
	int PDDWord = 0x00FFFF00;
	int errorcount = 0;
	int cs0_enable;
	int cs1_enable;
	int data_bus_size;
	u32 volatile *const *pads;
	u32 pad;

	/* check to see which chip selects are enabled */
	cs0_enable_initial = (readl(&mmdc0->mdctl) & 0x80000000) >> 31;
	cs1_enable_initial = (readl(&mmdc0->mdctl) & 0x40000000) >> 30;

	/*
	 * disable auto refresh
	 * before proceeding with calibration
	 */
	temp_ref = readl(&mmdc0->mdref);
	writel(0x0000C000, &mmdc0->mdref);

	/* disable DDR logic power down timer */
	v = readl(&mmdc0->mdpdc);
	v &= ~0xff00;
	writel(v, &mmdc0->mdpdc);

	/* disable Adopt power down timer */
	v = readl(&mmdc0->mapsr);
	v |= 0x1;
	writel(v, &mmdc0->mapsr);

	esdmisc_val = readl(&mmdc0->mdmisc);

	/* set RALAT and WALAT to max */
	v = readl(&mmdc0->mdmisc);
	v |= (1 << 6) | (1 << 7) | (1 << 8) | (1 << 16) | (1 << 17);
	writel(v, &mmdc0->mdmisc);

	v = mmdc_do_write_level_calibration();
	if (v) {
		printf("mmdc_do_write_level_calibration: %d\n", v);
		return -1 ;
	}

	if (is_cpu_type(MXC_CPU_MX6Q))
		pads = mx6q_sdqs_pads;
	else
		pads = mx6dl_sdqs_pads;

	// Set DQS pullups
	for (pad=0; pad < ARRAY_SIZE(mx6q_sdqs_pads); pad++)
		writel((readl(pads[pad]) & ~0xf000) | 0x7000,pads[pad]);

	/*
	 * per the ref manual, issue one refresh cycle mdscr[CMD]= 0x2,
	 * this also sets the CON_REQ bit.
	 */
	if (cs0_enable_initial)
		writel(0x00008020, &mmdc0->mdscr);
	if (cs1_enable_initial)
		writel(0x00008028, &mmdc0->mdscr);

	/* poll to make sure the con_ack bit was asserted */
	while (!(readl(&mmdc0->mdscr) & 0x00004000)) ;

	/*
	 * check mdmisc register CALIB_PER_CS to see which CS calibration is
	 * targeted to (under normal cases, it should be cleared as this is the
	 * default value, indicating calibration is directed to CS0). Disable
	 * the other chip select not being target for calibration to avoid any
	 * potential issues This will get re-enabled at end of calibration.
	 */
	v = readl(&mmdc0->mdctl);
	if ((readl(&mmdc0->mdmisc) & 0x00100000) == 0) {
		v &= ~(1 << 30); /* clear SDE_1 */
	}
	else {
		v &= ~(1 << 31); /* clear SDE_0 */
	}
	writel(v, &mmdc0->mdctl);

	/*
	 * check to see which chip selects are now enabled for the remainder
	 * of the calibration.
	 */
	cs0_enable = (readl(&mmdc0->mdctl) & 0x80000000) >> 31;
	cs1_enable = (readl(&mmdc0->mdctl) & 0x40000000) >> 30;

	/* check to see what is the data bus size */
	data_bus_size = (readl(&mmdc0->mdctl) & 0x30000) >> 16;

	mmdc_precharge_all(cs0_enable, cs1_enable);

	/* Write the pre-defined value into MPPDCMPR1 */
	writel(PDDWord, &mmdc0->mppdcmpr1);

	/*
	 * Issue a write access to the external DDR device by setting the bit SW_DUMMY_WR (bit 0)
	 * in the MPSWDAR0 and then poll this bit until it clears to indicate completion of the
	 * write access.
	 */
	v = readl(&mmdc0->mpswdar0);
	v |= 1;
	writel(v, &mmdc0->mpswdar0);

	while (readl(&mmdc0->mpswdar0) & 0x00000001);

	/*
	 * Set the RD_DL_ABS# bits to their default values (will be calibrated later in
	 * the read delay-line calibration)
	 * Both PHYs for x64 configuration, if x32, do only PHY0
	 */
	writel(0x40404040, &mmdc0->mprddlctl);
	if (data_bus_size == 0x2)
		writel(0x40404040, &mmdc1->mprddlctl);

	/* Force a measurement, for previous delay setup to take effect */
	mmdc_force_delay_measurement(data_bus_size);

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
	v = readl(&mmdc0->mpdgctrl0);
	v |= (1 << 30);
	writel(v, &mmdc0->mpdgctrl0);

	/*
	 * Wait for ack
	 */
	while (!(readl(&mmdc0->mpdgctrl0) & (1<<30)));

	/* Set bit 28 to start automatic read DQS gating calibration */
	writel(readl(&mmdc0->mpdgctrl0) | (1 << 28), &mmdc0->mpdgctrl0);

	/*
	 * Poll for completion
	 * mpdgctrl0[HW_DG_EN] should be 0
	 */
	while (readl(&mmdc0->mpdgctrl0) & 0x10000000);

	/*
	 * Check to see if any errors were encountered during calibration
	 * (check mpdgctrl0[HW_DG_ERR])
	 * check both PHYs for x64 configuration, if x32, check only PHY0
	 */
	if (data_bus_size == 0x2) {
		if ((readl(&mmdc0->mpdgctrl0) & 0x00001000) ||
				(readl(&mmdc1->mpdgctrl0) & 0x00001000)) {
			printf("read calibration err 0x%08x/0x%08x\n",
                               readl(&mmdc0->mpdgctrl0),
                               readl(&mmdc1->mpdgctrl0));
			errorcount++;
		}
	} else {
		if (readl(&mmdc0->mpdgctrl0) & 0x00001000)
			errorcount++;
	}

	if (errorcount) {
                printf("%s: read calibration error count %d, bus size %d\n", __func__, errorcount, data_bus_size);
		return -1;
	}

	/*
	 * DQS gating absolute offset should be modified from reflecting
	 * (HW_DG_LOWx + HW_DG_UPx)/2 to reflecting (HW_DG_UPx - 0x80)
	 */
	modify_dg_result(&mmdc0->mpdghwst0,
			&mmdc0->mpdghwst1,
			&mmdc0->mpdgctrl0);

	modify_dg_result(&mmdc0->mpdghwst2,
			&mmdc0->mpdghwst3,
			&mmdc0->mpdgctrl1);

	if (data_bus_size == 0x2) {
		modify_dg_result(&mmdc1->mpdghwst0,
				&mmdc1->mpdghwst1,
				&mmdc1->mpdgctrl0);
		modify_dg_result(&mmdc1->mpdghwst2,
				&mmdc1->mpdghwst3,
				&mmdc1->mpdgctrl1);
	}

	/*
	 * Read delay Calibration
	 */
	mmdc_reset_read_data_fifos();

	mmdc_precharge_all(cs0_enable, cs1_enable);

	/*
	 * Read delay-line calibration
	 * Start the automatic read calibration process by asserting mprddlhwctl[ HW_RD_DL_EN]
	 */
	writel(0x00000030, &mmdc0->mprddlhwctl);

	/*
	 * poll for completion
	 * MMDC indicates that the write data calibration had finished by setting
	 * mprddlhwctl[HW_RD_DL_EN] = 0
	 * Also, ensure that no error bits were set
	 */
	while (readl(&mmdc0->mprddlhwctl) & 0x00000010) ;

	/* check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (data_bus_size == 0x2) {
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

	mmdc_precharge_all(cs0_enable, cs1_enable);

	/*
	 * Set the WR_DL_ABS# bits to their default values
	 * Both PHYs for x64 configuration, if x32, do only PHY0
	 */
	writel(0x40404040, &mmdc0->mpwrdlctl);
	if (data_bus_size == 0x2)
		writel(0x40404040, &mmdc1->mpwrdlctl);

	mmdc_force_delay_measurement(data_bus_size);

	/* Start the automatic write calibration process by asserting mpwrdlhwctl0[HW_WR_DL_EN] */
	writel(0x00000030, &mmdc0->mpwrdlhwctl);

	/*
	 * poll for completion
	 * MMDC indicates that the write data calibration had finished by setting
	 * mpwrdlhwctl[HW_WR_DL_EN] = 0
	 * Also, ensure that no error bits were set
	 */
	while (readl(&mmdc0->mpwrdlhwctl) & 0x00000010) ;

	/* check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (data_bus_size == 0x2) {
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
	v = readl(&mmdc0->mdpdc) | 0x00005500;
	writel(v, &mmdc0->mdpdc);

	/* enable Adopt power down timer */
	v = readl(&mmdc0->mapsr) & 0xfffffff7;
	writel(v, &mmdc0->mapsr);

	/* restore mdmisc value (RALAT, WALAT) */
	writel(esdmisc_val, &mmdc1->mdmisc);

	/* clear DQS pull ups */
	for (pad=0; pad < ARRAY_SIZE(mx6q_sdqs_pads); pad++)
		writel(readl(pads[pad]) & ~0xF000,pads[pad]);

	v = readl(&mmdc0->mdctl);

	/* re-enable SDE (chip selects) if they were set initially */
	if (cs1_enable_initial == 1)
		v |= (1 << 30); /* set SDE_1 */

	if (cs0_enable_initial == 1)
		v |= (1 << 31); /* set SDE_0 */

	writel(v, &mmdc0->mdctl);

	/* re-enable to auto refresh */
	writel(temp_ref, &mmdc0->mdref);

	/* clear the mdscr (including the con_req bit) */
	writel(0x0, &mmdc0->mdscr); /* CS0 */

	/* poll to make sure the con_ack bit is clear */
	while (readl(&mmdc0->mdscr) & 0x00004000) ;

	printf("mmdc0->mpdgctrl0\t= 0x%08X\n", readl(&mmdc0->mpdgctrl0));
	printf("mmdc0->mpdgctrl1\t= 0x%08X\n", readl(&mmdc0->mpdgctrl1));
	printf("mmdc1->mpdgctrl0\t= 0x%08X\n", readl(&mmdc1->mpdgctrl0));
	printf("mmdc1->mpdgctrl1\t= 0x%08X\n", readl(&mmdc1->mpdgctrl1));
	printf("mmdc0->mprddlctl\t= 0x%08X\n", readl(&mmdc0->mprddlctl));
	printf("mmdc1->mprddlctl\t= 0x%08X\n", readl(&mmdc1->mprddlctl));
	printf("mmdc0->mpwrdlctl\t= 0x%08X\n", readl(&mmdc0->mpwrdlctl));
	printf("mmdc1->mpwrdlctl\t= 0x%08X\n", readl(&mmdc1->mpwrdlctl));
	printf("mmdc0->mpwldectrl0\t= 0x%08X\n",readl(&mmdc0->mpwldectrl0));
	printf("mmdc0->mpwldectrl1\t= 0x%08X\n",readl(&mmdc0->mpwldectrl1));
	printf("mmdc1->mpwldectrl0\t= 0x%08X\n",readl(&mmdc1->mpwldectrl0));
	printf("mmdc1->mpwldectrl1\t= 0x%08X\n",readl(&mmdc1->mpwldectrl1));
	/*
	 * registers below are for debugging purposes
	 * these print out the upper and lower boundaries captured during read DQS gating calibration
	 *
	printf("mpdghwst0 PHY0 (0x021b087c) = 0x%08X\n", readl(&mmdc0->mpdghwst0));
	printf("mpdghwst1 PHY0 (0x021b0880) = 0x%08X\n", readl(&mmdc0->mpdghwst1));
	printf("mpdghwst2 PHY0 (0x021b0884) = 0x%08X\n", readl(&mmdc0->mpdghwst2));
	printf("mpdghwst3 PHY0 (0x021b0888) = 0x%08X\n", readl(&mmdc0->mpdghwst3));
	printf("mpdghwst0 PHY1 (0x021b487c) = 0x%08X\n", readl(&mmdc1->mpdghwst0));
	printf("mpdghwst1 PHY1 (0x021b4880) = 0x%08X\n", readl(&mmdc1->mpdghwst1));
	printf("mpdghwst2 PHY1 (0x021b4884) = 0x%08X\n", readl(&mmdc1->mpdghwst2));
	printf("mpdghwst3 PHY1 (0x021b4888) = 0x%08X\n", readl(&mmdc1->mpdghwst3));
	*/
	return errorcount;
}

/* configure MX6Q/DUAL mmdc DDR io registers */
static struct mx6dq_iomux_ddr_regs const mx6dq_ddr_ioregs = {
	/* SDCLK[0:1], CAS, RAS, Reset: Differential input, 40ohm */
	.dram_sdclk_0 = 0x00020030,
	.dram_sdclk_1 = 0x00020030,
	.dram_cas = 0x00020030,
	.dram_ras = 0x00020030,
	.dram_reset = 0x00020030,
	/* SDCKE[0:1]: 100k pull-up */
	.dram_sdcke0 = 0x00003000,
	.dram_sdcke1 = 0x00003000,
	/* SDBA2: pull-up disabled */
	.dram_sdba2 = 0x00000000,
	/* SDODT[0:1]: 100k pull-up, 40 ohm */
	.dram_sdodt0 = 0x00003030,
	.dram_sdodt1 = 0x00003030,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.dram_sdqs0 = 0x00000030,
	.dram_sdqs1 = 0x00000030,
	.dram_sdqs2 = 0x00000030,
	.dram_sdqs3 = 0x00000030,
	.dram_sdqs4 = 0x00000030,
	.dram_sdqs5 = 0x00000030,
	.dram_sdqs6 = 0x00000030,
	.dram_sdqs7 = 0x00000030,

	/* DQM[0:7]: Differential input, 40 ohm */
	.dram_dqm0 = 0x00020030,
	.dram_dqm1 = 0x00020030,
	.dram_dqm2 = 0x00020030,
	.dram_dqm3 = 0x00020030,
	.dram_dqm4 = 0x00020030,
	.dram_dqm5 = 0x00020030,
	.dram_dqm6 = 0x00020030,
	.dram_dqm7 = 0x00020030,
};

/* configure MX6Q/DUAL mmdc GRP io registers */
static struct mx6dq_iomux_grp_regs const mx6dq_grp_ioregs = {
	/* DDR3 */
	.grp_ddr_type = 0x000c0000,
	.grp_ddrmode_ctl = 0x00020000,
	/* disable DDR pullups */
	.grp_ddrpke = 0x00000000,
	/* ADDR[00:16], SDBA[0:1]: 40 ohm */
	.grp_addds = 0x00000030,
	/* CS0/CS1/SDBA2/CKE0/CKE1/SDWE: 40 ohm */
	.grp_ctlds = 0x00000030,
	/* DATA[00:63]: Differential input, 40 ohm */
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_b2ds = 0x00000030,
	.grp_b3ds = 0x00000030,
	.grp_b4ds = 0x00000030,
	.grp_b5ds = 0x00000030,
	.grp_b6ds = 0x00000030,
	.grp_b7ds = 0x00000030,
};

static struct mx6sdl_iomux_ddr_regs const mx6sdl_ddr_ioregs = {
	/* SDCLK[0:1], CAS, RAS, Reset: Differential input, 40ohm */
	.dram_sdclk_0 = 0x00020030,
	.dram_sdclk_1 = 0x00020030,
	.dram_cas = 0x00020030,
	.dram_ras = 0x00020030,
	.dram_reset = 0x00020030,
	/* SDCKE[0:1]: 100k pull-up */
	.dram_sdcke0 = 0x00003000,
	.dram_sdcke1 = 0x00003000,
	/* SDBA2: pull-up disabled */
	.dram_sdba2 = 0x00000000,
	/* SDODT[0:1]: 100k pull-up, 40 ohm */
	.dram_sdodt0 = 0x00003030,
	.dram_sdodt1 = 0x00003030,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.dram_sdqs0 = 0x00000030,
	.dram_sdqs1 = 0x00000030,
	.dram_sdqs2 = 0x00000030,
	.dram_sdqs3 = 0x00000030,
	.dram_sdqs4 = 0x00000030,
	.dram_sdqs5 = 0x00000030,
	.dram_sdqs6 = 0x00000030,
	.dram_sdqs7 = 0x00000030,

	/* DQM[0:7]: Differential input, 40 ohm */
	.dram_dqm0 = 0x00020030,
	.dram_dqm1 = 0x00020030,
	.dram_dqm2 = 0x00020030,
	.dram_dqm3 = 0x00020030,
	.dram_dqm4 = 0x00020030,
	.dram_dqm5 = 0x00020030,
	.dram_dqm6 = 0x00020030,
	.dram_dqm7 = 0x00020030,
};

/* configure MX6SOLO/DUALLITE mmdc GRP io registers */
static struct mx6sdl_iomux_grp_regs const mx6sdl_grp_ioregs = {
	/* DDR3 */
	.grp_ddr_type = 0x000c0000,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.grp_ddrmode_ctl = 0x00020000,
	/* disable DDR pullups */
	.grp_ddrpke = 0x00000000,
	/* ADDR[00:16], SDBA[0:1]: 40 ohm */
	.grp_addds = 0x00000030,
	/* CS0/CS1/SDBA2/CKE0/CKE1/SDWE: 40 ohm */
	.grp_ctlds = 0x00000030,
	/* DATA[00:63]: Differential input, 40 ohm */
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_b2ds = 0x00000030,
	.grp_b3ds = 0x00000030,
	.grp_b4ds = 0x00000030,
	.grp_b5ds = 0x00000030,
	.grp_b6ds = 0x00000030,
	.grp_b7ds = 0x00000030,
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
	.walat = 1,	/* Write additional latency */
	.ralat = 5,	/* Read additional latency */
	.mif3_mode = 3,	/* Command prediction working mode */
	.bi_on = 1,	/* Bank interleaving enabled */
	.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
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
#else
#error undefined memory type
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

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
	writel(0x007F007F, &iomux->gpr[6]);
	writel(0x007F007F, &iomux->gpr[7]);
}

/*
 * called from C runtime startup code (arch/arm/lib/crt0.S:_main)
 * - we have a stack and a place to store GD, both in SRAM
 * - no variable global data is available
 */
void board_init_f(ulong dummy)
{
	int ddr_freq;
	int errs;

	memset((void *)gd, 0, sizeof(struct global_data));

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	if (is_cpu_type(MXC_CPU_MX6Q)) {
		ddr_freq = set_ddr_clock(DDR_DIV_528MHZ);
		mx6dq_dram_iocfg(CONFIG_DDRWIDTH, &mx6dq_ddr_ioregs,
				 &mx6dq_grp_ioregs);
	}
	else {
		ddr_freq = set_ddr_clock(DDR_DIV_400MHZ);
		mx6sdl_dram_iocfg(CONFIG_DDRWIDTH, &mx6sdl_ddr_ioregs,
				  &mx6sdl_grp_ioregs);
	}
	mx6_dram_cfg(&sysinfo, 0, &ddrtype);

	printf("run calibration at %u MHz\n",ddr_freq);
	errs = mmdc_do_dqs_calibration();
	if (errs)
		printf("completed with %d errors\n", errs);
	else
		printf("completed successfully\n");

	reset_cpu(0);
}

