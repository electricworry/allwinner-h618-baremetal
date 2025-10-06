TCON TOP linux/Documentation/devicetree/bindings/display/allwinner,sun8i-r40-tcon-top.yaml

TCON TOP is like a switch. Our DT for H6128 shows that our pipeline is like this:

   mixer0
          \
           TCON-TOP
                   \ TCON-TV
                            \
                             TCON-TOP - HDMI

And so when sun4i_drv_bind() is called, the following bind() functions are called:
- sun8i_mixer_bind
- sun8i_tcon_top_bind
- sun4i_tcon_bind
- sun8i_dw_hdmi_bind

Each of these components are part of the DE, which runs the show.

## HDMI PHY

This IP block is not part of the component framwork, but it is an essential part
of the graphics.

On sun8i_hdmi_phy_init() (which is called by HDMI) various bespoke registers
of the HDMI PHY are poked which will magically cause the HDMI block to start
working.

## Display Engine

The sun4i DE is in (sun4i_drv.c).

sun4i_drv_bind() causes the following sub-components to bind and then creates
the DRM client which ultimately results in the framebuffer being created,
gates being set up correctly in TCON TOP, and the pipeline from Mixer to HDMI
working and outputting graphics.

This working is completely dependant on the bespoke driver functions in each
of the following sub-components.

## Mixer

The Mixer implements various `struct sunxi_engine_ops`, which are pointed to by
the `struct sunxi_engine`.

The mixer implements multiple UI/VI layers. These are implemented in their own
C files.

## TCON TOP

This is the most trivial part of the pipeline.

sun8i_tcon_top_bind() sets up this driver. 

It de-asserts reset(s), enables a clk, and depending on the 'quirks' of the
device, sets up up to three gates which can be controlled. Thereafter there
are two functions that can be called to pull these levers:

* sun8i_tcon_top_set_hdmi_src
* sun8i_tcon_top_de_config

The sun4i TCON will use these functions to control the gates.

## TCON

sun4i_tcon_bind() sets up this driver.

sun8i_r40_tcon_tv_set_mux() will be called to set the TCON TOP gates.

sun4i_tcon_mode_set() will be called by the CRTC to set the TCON up for the
output display mode (which will have been discovered via DDC (EDID)).

## HDMI

sun8i_dw_hdmi_bind() sets up this driver.

This driver has a relationship with HDMI PHY. It invokes the
sun8i_hdmi_phy_init() function, and also links pointers to HDMI PHY's various
control helpers and structs.

It is done in this way because the generic dw-hdmi.c is used (as this is a
variation of the DesignWare HDMI block), and that is the real code that runs
this sub-component. dw-hdmi is a beast with hundreds of functions.

One of the important functions is dw_hdmi_phy_power_on() which is called in the
following path:
- dw_hdmi_bridge_atomic_enable
- dw_hdmi_update_power
- dw_hdmi_poweron
- dw_hdmi_setup
- dw_hdmi_phy_init
- hdmi_phy_configure
- dw_hdmi_phy_power_on

## CRTC

The CRTC is created by the TCON when sun4i_tcon_bind() calls sun4i_crtc_init().

As a CRTC is the Linux concept that obfuscates the driver into a standard
interface for the kernel to create framebuffers, this is 

Various CRTC helper functions are defined here for the kernel to call.

## Clocks

```
We must set the "tmds" <&ccu CLK_HDMI> to the pixelclock (108MHz)
static const char * const hdmi_parents[] = { "pll-video0", "pll-video0-4x",
				     "pll-video2", "pll-video2-4x" };
static SUNXI_CCU_M_WITH_MUX_GATE(hdmi_clk, "hdmi", hdmi_parents, 0xb00,
                0, 4,		/* M */
                24, 2,		/* mux */
                BIT(31),	/* gate */
                0);
These clock registers have the following implementation:
N or Factor N is a multiplying factor. There is no N on this clock.
M or Factor M is a dividing factor. Here bits 3:0 are M.
Bits 25:24 are a mux, between four parent clocks. PLL_VIDEO0(1X), PLL_VIDEO0(4X), PLL_VIDEO2(1X), PLL_VIDEO2(4X)

The ccu_div_ops struct defines the following functions:

const struct clk_ops ccu_div_ops = {
	.disable	= ccu_div_disable,
	.enable		= ccu_div_enable,
	.is_enabled	= ccu_div_is_enabled,

	.get_parent	= ccu_div_get_parent,
	.set_parent	= ccu_div_set_parent,

	.determine_rate	= ccu_div_determine_rate,
	.recalc_rate	= ccu_div_recalc_rate,
	.set_rate	= ccu_div_set_rate,
};

An nm clock defines the following functions:

const struct clk_ops ccu_nm_ops = {
	.disable	= ccu_nm_disable,
	.enable		= ccu_nm_enable,
	.is_enabled	= ccu_nm_is_enabled,

	.recalc_rate	= ccu_nm_recalc_rate,
	.round_rate	= ccu_nm_round_rate,
	.set_rate	= ccu_nm_set_rate,
};

In the Linux clk framework the following calls are made:

determine_rate - This gives the driver the opportunity to change the rate. In a
div(mux) clk 


```

## Debugging clocks

If you set a breakpoint on `clk_set_rate` there will be an opaque clk as param0.
To see what this clock is, print `clk->core->hw`. This will show the actual clock.

## Old guff

```


    // TODO DELETE THE FOLLOWING CRAP
    // Enable the iosc
    // SUN6I_DCXO_CTRL_REG |= (1 << 0);

    // START r_ccu aka prmc
        // SUN50I_H616_CLK_R_APB1_TWD_REG |= (1 << 0); // This clock is marked as critical
    // END r_ccu prmc

    // Various guff I was playing with.
    // // START rtc
    //     // The rtc needs the following r_ccu clock, so we enable it. 
    //     // SUN50I_H616_CLK_R_APB1_RTC_REG |= (1 << 0); 
    //     // The rtc could also ask for <&ccu CLK_PLL_SYSTEM_32K> enabled which isn't a real clock, but its parent is SUN50I_H616_PLL_PERIPH0_REG so we enable it?
    //     SUN50I_H616_PLL_PERIPH0_REG |= (1 << 31);
    // // END rtc - that should satisfy rtc unless something else needs it.
    // SUN50I_H616_GPU_CLK1_REG |= (1 << 31);
    // SUN50I_H616_PLL_DDR0_REG |= (1 << 31);
    // SUN50I_H616_DRAM_BGR_REG |= (1 << 0);
    // SUN50I_H616_DMA_BGR_REG |= (1 << 16);
    // SUN50I_H616_DMA_BGR_REG |= (1 << 0);
    // SUN50I_H616_MBUS_MAT_CLK_GATING_REG |= (1 << 0);
    //     // The rtc needs the following r_ccu clock, so we enable it. 
    //     SUN50I_H616_CLK_R_APB1_RTC_REG |= (1 << 0); 
    // SUN50I_H616_HDMI_BGR_REG |= (1<<0); // Enable CLK_BUS_HDMI
    // SUN50I_H616_HDMI_BGR_REG |= (1<<16) | (1<<17); // De-assert reset of RST_BUS_HDMI and RST_BUS_HDMI_SUB
    // SUN50I_H616_HDMI0_SLOW_CLK_REG    = (1<<31); // Enable HDMI slow clk
    // Need to enable CLK_HDMI
    // static const char * const hdmi_parents[] = { "pll-video0", "pll-video0-4x",
    //                         "pll-video2", "pll-video2-4x" };
    // static SUNXI_CCU_M_WITH_MUX_GATE(hdmi_clk, "hdmi", hdmi_parents, 0xb00,
	// 			 0, 4,		/* M */
	// 			 24, 2,		/* mux */
	// 			 BIT(31),	/* gate */
	// 			 0);
    // SUN50I_H616_HDMI0_CLK_REG |= (1 << 31);
    // The following crap from before...
    // Set up shared and dedicated clocks for HDMI, LCD/TCON and DE2
    // PLL_DE_CTRL      = (1<<31) | (1<<24) | (17<<8) | (0<<0); // 432MHz
    // PLL_VIDEO0_CTRL   = (1<<31) | (1<<25) | (1<<24) | (98<<8) | (7<<0); // 297MHz
    // BUS_CLK_GATING1 |= (1<<12) | (1<<11) | (1<<3); // Enable DE, HDMI, TCON0
    // BUS_SOFT_RST1   |= (1<<12) | (3<<10) | (1<<3); // De-assert reset of DE, HDMI0/1, TCON0
    // DE_CLK           = (1<<31) | (1<<24); // Enable DE clock, set source to PLL_DE
    // HDMI_CLK         = (1<<31); // Enable HDMI clk (use PLL3)
    // TCON0_CLK        = (1<<31) | 3; // Enable TCON0 clk, divide by 4
```


```




    // // START PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_h3_hdmi_phy_config()

    // // It gets intereting... include/drm/drm_modes.h

    // We get the pixel clock from EDID.
    uint32_t clk_rate = e.pixel_clock;

    // In sun8i_h3_hdmi_phy_config() we set up some freq independent vals:

	uint32_t pll_cfg1_init;
	uint32_t pll_cfg2_init;
	uint32_t ana_cfg1_end;
	uint32_t ana_cfg2_init;
	uint32_t ana_cfg3_init;
	uint32_t b_offset = 0;

	// if (phy->variant->has_phy_clk)
	// 	clk_set_rate(phy->clk_phy, clk_rate);
    // We know the h3 variant has a clock, and we can find the code in:
    // START drivers/gpu/drm/sun4i/sun8i_hdmi_phy_clk.c sun8i_phy_clk_set_rate()
    {
        uint32_t parent_rate = 192000000; // Correct, but where does it come from?
        unsigned long best_rate = 0;
        uint8_t best_m = 0, m;

        for (m = 1; m <= 16; m++) {
        	unsigned long tmp_rate = parent_rate / m;

        	if (tmp_rate > clk_rate)
        		continue;

        	if (!best_rate ||
        	    (clk_rate - tmp_rate) < (clk_rate - best_rate)) {
        		best_rate = tmp_rate;
        		best_m = m;
        	}
        }
        printf("RATE: %d\n", best_m);
        reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG2_REG,
        		   SUN8I_HDMI_PHY_PLL_CFG2_PREDIV_MSK,
        		   SUN8I_HDMI_PHY_PLL_CFG2_PREDIV(best_m));
    }

	// sun8i_hdmi_phy_set_polarity(phy, mode)... TODO from the EDID
    val = 0;
    bool hpol_n = true;
    bool vpol_n = true;
	if (hpol_n)
		val |= SUN8I_HDMI_PHY_DBG_CTRL_POL_NHSYNC; // Specifies Active Low Hsync
	if (vpol_n)
		val |= SUN8I_HDMI_PHY_DBG_CTRL_POL_NVSYNC;
	reg_phy_update_bits(SUN8I_HDMI_PHY_DBG_CTRL_REG,
			   SUN8I_HDMI_PHY_DBG_CTRL_POL_MASK, val);

	/* bandwidth / frequency independent settings */

	pll_cfg1_init = SUN8I_HDMI_PHY_PLL_CFG1_LDO2_EN |
			SUN8I_HDMI_PHY_PLL_CFG1_LDO1_EN |
			SUN8I_HDMI_PHY_PLL_CFG1_LDO_VSET(7) |
			SUN8I_HDMI_PHY_PLL_CFG1_UNKNOWN(1) |
			SUN8I_HDMI_PHY_PLL_CFG1_PLLDBEN |
			SUN8I_HDMI_PHY_PLL_CFG1_CS |
			SUN8I_HDMI_PHY_PLL_CFG1_CP_S(2) |
			SUN8I_HDMI_PHY_PLL_CFG1_CNT_INT(63) |
			SUN8I_HDMI_PHY_PLL_CFG1_BWS;

	pll_cfg2_init = SUN8I_HDMI_PHY_PLL_CFG2_SV_H |
			SUN8I_HDMI_PHY_PLL_CFG2_VCOGAIN_EN |
			SUN8I_HDMI_PHY_PLL_CFG2_SDIV2;

	ana_cfg1_end = SUN8I_HDMI_PHY_ANA_CFG1_REG_SVBH(1) |
		       SUN8I_HDMI_PHY_ANA_CFG1_AMP_OPT |
		       SUN8I_HDMI_PHY_ANA_CFG1_EMP_OPT |
		       SUN8I_HDMI_PHY_ANA_CFG1_AMPCK_OPT |
		       SUN8I_HDMI_PHY_ANA_CFG1_EMPCK_OPT |
		       SUN8I_HDMI_PHY_ANA_CFG1_ENRCAL |
		       SUN8I_HDMI_PHY_ANA_CFG1_ENCALOG |
		       SUN8I_HDMI_PHY_ANA_CFG1_REG_SCKTMDS |
		       SUN8I_HDMI_PHY_ANA_CFG1_TMDSCLK_EN |
		       SUN8I_HDMI_PHY_ANA_CFG1_TXEN_MASK |
		       SUN8I_HDMI_PHY_ANA_CFG1_TXEN_ALL |
		       SUN8I_HDMI_PHY_ANA_CFG1_BIASEN_TMDSCLK |
		       SUN8I_HDMI_PHY_ANA_CFG1_BIASEN_TMDS2 |
		       SUN8I_HDMI_PHY_ANA_CFG1_BIASEN_TMDS1 |
		       SUN8I_HDMI_PHY_ANA_CFG1_BIASEN_TMDS0 |
		       SUN8I_HDMI_PHY_ANA_CFG1_ENP2S_TMDS2 |
		       SUN8I_HDMI_PHY_ANA_CFG1_ENP2S_TMDS1 |
		       SUN8I_HDMI_PHY_ANA_CFG1_ENP2S_TMDS0 |
		       SUN8I_HDMI_PHY_ANA_CFG1_CKEN |
		       SUN8I_HDMI_PHY_ANA_CFG1_LDOEN |
		       SUN8I_HDMI_PHY_ANA_CFG1_ENVBS |
		       SUN8I_HDMI_PHY_ANA_CFG1_ENBI;

	ana_cfg2_init = SUN8I_HDMI_PHY_ANA_CFG2_M_EN |
			SUN8I_HDMI_PHY_ANA_CFG2_REG_DENCK |
			SUN8I_HDMI_PHY_ANA_CFG2_REG_DEN |
			SUN8I_HDMI_PHY_ANA_CFG2_REG_CKSS(1) |
			SUN8I_HDMI_PHY_ANA_CFG2_REG_CSMPS(1);

	ana_cfg3_init = SUN8I_HDMI_PHY_ANA_CFG3_REG_WIRE(0x3e0) |
			SUN8I_HDMI_PHY_ANA_CFG3_SDAEN |
			SUN8I_HDMI_PHY_ANA_CFG3_SCLEN;

	/* bandwidth / frequency dependent settings */

	if (clk_rate <= 27000000) {
		pll_cfg1_init |= SUN8I_HDMI_PHY_PLL_CFG1_HV_IS_33 |
				 SUN8I_HDMI_PHY_PLL_CFG1_CNT_INT(32);
		pll_cfg2_init |= SUN8I_HDMI_PHY_PLL_CFG2_VCO_S(4) |
				 SUN8I_HDMI_PHY_PLL_CFG2_S(4);
		ana_cfg1_end |= SUN8I_HDMI_PHY_ANA_CFG1_REG_CALSW;
		ana_cfg2_init |= SUN8I_HDMI_PHY_ANA_CFG2_REG_SLV(4) |
				 SUN8I_HDMI_PHY_ANA_CFG2_REG_RESDI(phy_rcal);
		ana_cfg3_init |= SUN8I_HDMI_PHY_ANA_CFG3_REG_AMPCK(3) |
				 SUN8I_HDMI_PHY_ANA_CFG3_REG_AMP(5);
	} else if (clk_rate <= 74250000) {
		pll_cfg1_init |= SUN8I_HDMI_PHY_PLL_CFG1_HV_IS_33 |
				 SUN8I_HDMI_PHY_PLL_CFG1_CNT_INT(32);
		pll_cfg2_init |= SUN8I_HDMI_PHY_PLL_CFG2_VCO_S(4) |
				 SUN8I_HDMI_PHY_PLL_CFG2_S(5);
		ana_cfg1_end |= SUN8I_HDMI_PHY_ANA_CFG1_REG_CALSW;
		ana_cfg2_init |= SUN8I_HDMI_PHY_ANA_CFG2_REG_SLV(4) |
				 SUN8I_HDMI_PHY_ANA_CFG2_REG_RESDI(phy_rcal);
		ana_cfg3_init |= SUN8I_HDMI_PHY_ANA_CFG3_REG_AMPCK(5) |
				 SUN8I_HDMI_PHY_ANA_CFG3_REG_AMP(7);
	} else if (clk_rate <= 148500000) {
		pll_cfg1_init |= SUN8I_HDMI_PHY_PLL_CFG1_HV_IS_33 |
				 SUN8I_HDMI_PHY_PLL_CFG1_CNT_INT(32);
		pll_cfg2_init |= SUN8I_HDMI_PHY_PLL_CFG2_VCO_S(4) |
				 SUN8I_HDMI_PHY_PLL_CFG2_S(6);
		ana_cfg2_init |= SUN8I_HDMI_PHY_ANA_CFG2_REG_BIGSWCK |
				 SUN8I_HDMI_PHY_ANA_CFG2_REG_BIGSW |
				 SUN8I_HDMI_PHY_ANA_CFG2_REG_SLV(2);
		ana_cfg3_init |= SUN8I_HDMI_PHY_ANA_CFG3_REG_AMPCK(7) |
				 SUN8I_HDMI_PHY_ANA_CFG3_REG_AMP(9);
	} else {
		b_offset = 2;
		pll_cfg1_init |= SUN8I_HDMI_PHY_PLL_CFG1_CNT_INT(63);
		pll_cfg2_init |= SUN8I_HDMI_PHY_PLL_CFG2_VCO_S(6) |
				 SUN8I_HDMI_PHY_PLL_CFG2_S(7);
		ana_cfg2_init |= SUN8I_HDMI_PHY_ANA_CFG2_REG_BIGSWCK |
				 SUN8I_HDMI_PHY_ANA_CFG2_REG_BIGSW |
				 SUN8I_HDMI_PHY_ANA_CFG2_REG_SLV(4);
		ana_cfg3_init |= SUN8I_HDMI_PHY_ANA_CFG3_REG_AMPCK(9) |
				 SUN8I_HDMI_PHY_ANA_CFG3_REG_AMP(13) |
				 SUN8I_HDMI_PHY_ANA_CFG3_REG_EMP(3);
	}

    printf("pll_cfg1_init 0x%x\n", pll_cfg1_init);
    printf("pll_cfg2_init 0x%x\n", pll_cfg2_init);
    printf("ana_cfg1_end 0x%x\n", ana_cfg1_end);
    printf("ana_cfg2_init 0x%x\n", ana_cfg2_init);
    printf("ana_cfg3_init 0x%x\n", ana_cfg3_init);
    printf("b_offset 0x%x\n", b_offset);

    reg_phy_update_bits(SUN8I_HDMI_PHY_ANA_CFG1_REG,
        SUN8I_HDMI_PHY_ANA_CFG1_TXEN_MASK,
        0);
    
	/*
	 * NOTE: We have to be careful not to overwrite PHY parent
	 * clock selection bit and clock divider.
	 */
	reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG1_REG,
			   (uint32_t)~SUN8I_HDMI_PHY_PLL_CFG1_CKIN_SEL_MSK,
			   pll_cfg1_init);
	reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG2_REG,
			   (uint32_t)~SUN8I_HDMI_PHY_PLL_CFG2_PREDIV_MSK,
			   pll_cfg2_init);
	udelay(10000);
	reg_phy_write(SUN8I_HDMI_PHY_PLL_CFG3_REG,
		     SUN8I_HDMI_PHY_PLL_CFG3_SOUT_DIV2);
	reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG1_REG,
			   SUN8I_HDMI_PHY_PLL_CFG1_PLLEN,
			   SUN8I_HDMI_PHY_PLL_CFG1_PLLEN);
	udelay(10000);

	/* get B value */
	reg_phy_read(SUN8I_HDMI_PHY_ANA_STS_REG, &val);
	val = (val & SUN8I_HDMI_PHY_ANA_STS_B_OUT_MSK) >>
		SUN8I_HDMI_PHY_ANA_STS_B_OUT_SHIFT;
	val = MIN(val + b_offset, (uint32_t)0x3f);

	reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG1_REG,
			   SUN8I_HDMI_PHY_PLL_CFG1_REG_OD1 |
			   SUN8I_HDMI_PHY_PLL_CFG1_REG_OD,
			   SUN8I_HDMI_PHY_PLL_CFG1_REG_OD1 |
			   SUN8I_HDMI_PHY_PLL_CFG1_REG_OD);
	reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG1_REG,
			   SUN8I_HDMI_PHY_PLL_CFG1_B_IN_MSK,
			   val << SUN8I_HDMI_PHY_PLL_CFG1_B_IN_SHIFT);
	// udelay(100000);
	reg_phy_write(SUN8I_HDMI_PHY_ANA_CFG1_REG, ana_cfg1_end);
	reg_phy_write(SUN8I_HDMI_PHY_ANA_CFG2_REG, ana_cfg2_init);
	reg_phy_write(SUN8I_HDMI_PHY_ANA_CFG3_REG, ana_cfg3_init);

    // END PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_h3_hdmi_phy_config()

                        // //
                        // // Everything here is wild west. But it does produce flickering. What is happening in here that enables the display? Maybe something in kernel I'm yet to find.
                        // //
                        // // The following is from catphish. Not much more to do!
                        reg_phy_write(SUN8I_HDMI_PHY_PLL_CFG1_REG, 0x39dc5040); // Match
                        reg_phy_write(SUN8I_HDMI_PHY_PLL_CFG2_REG, 0x80084381); // DIFFERENT
                        udelay(10000);
                        reg_phy_write(SUN8I_HDMI_PHY_PLL_CFG3_REG, 1); // Match
                        reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG1_REG,
                              (1<<25),
                              (1<<25));  // Match
                        // udelay(10000);
                        // reg_phy_read(SUN8I_HDMI_PHY_ANA_STS_REG, &val);
                        // val = (val & 0x1f800) >> 11; // match
                        // reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG1_REG,
                        //       (1<<31) | (1<<30) | val,
                        //       (1<<31) | (1<<30) | val);  // Sort of same
                        // reg_phy_write(SUN8I_HDMI_PHY_ANA_CFG1_REG, 0x01FFFF7F); // Similar
                        // reg_phy_write(SUN8I_HDMI_PHY_ANA_CFG2_REG, 0x8063A800); // Similar
                        // reg_phy_write(SUN8I_HDMI_PHY_ANA_CFG3_REG, 0x0F81C485); // Similar

    
    // START drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_bridge_atomic_enable()


    /* HDMI Initialization Step B.1 */
    // START drivers/gpu/drm/bridge/synopsys/dw-hdmi.c hdmi_av_composer()

    // SKIPPING STUFF

	int hblank, vblank, h_de_hs, v_de_vs, hsync_len, vsync_len;
	unsigned int vdisplay, hdisplay;

    // TEST MONITOR RESOLUTION: 800 x 480 
    //   DTD   1:   800x480    65.681445 Hz   5:3     34.483 kHz     32.000000 MHz (108 mm x 68 mm)
    //                  Hfront   40 Hsync  48 Hback   40 Hpol N
    //                  Vfront   13 Vsync   3 Vback   29 Vpol N

    hdisplay = e.horizontal_px;
    vdisplay = e.vertical_px;
    hblank = e.horizontal_blanking_px; // Hfront+Hsync+Hback
    vblank = e.vertical_blanking_px; // Vfront+Vsync+Vback
    h_de_hs = 40; // ???
    v_de_vs = 13; // ???
    hsync_len = e.horizontal_sync_pulse_px; // Hsync
    vsync_len = e.vertical_sync_pulse_px; // Vsync
	/* Set up horizontal active pixel width */
	hdmi_writeb(hdisplay >> 8, HDMI_FC_INHACTV1);
	hdmi_writeb(hdisplay, HDMI_FC_INHACTV0);
	/* Set up vertical active lines */
	hdmi_writeb(vdisplay >> 8, HDMI_FC_INVACTV1);
	hdmi_writeb(vdisplay, HDMI_FC_INVACTV0);
	/* Set up horizontal blanking pixel region width */
	hdmi_writeb(hblank >> 8, HDMI_FC_INHBLANK1);
	hdmi_writeb(hblank, HDMI_FC_INHBLANK0);
	/* Set up vertical blanking pixel region width */
	hdmi_writeb(vblank, HDMI_FC_INVBLANK);
	/* Set up HSYNC active edge delay width (in pixel clks) */
	hdmi_writeb(h_de_hs >> 8, HDMI_FC_HSYNCINDELAY1);
	hdmi_writeb(h_de_hs, HDMI_FC_HSYNCINDELAY0);
	/* Set up VSYNC active edge delay (in lines) */
	hdmi_writeb(v_de_vs, HDMI_FC_VSYNCINDELAY);
	/* Set up HSYNC active pulse width (in pixel clks) */
	hdmi_writeb(hsync_len >> 8, HDMI_FC_HSYNCINWIDTH1);
	hdmi_writeb(hsync_len, HDMI_FC_HSYNCINWIDTH0);
	/* Set up VSYNC active edge delay (in lines) */
	hdmi_writeb(vsync_len, HDMI_FC_VSYNCINWIDTH);
    // END drivers/gpu/drm/bridge/synopsys/dw-hdmi.c hdmi_av_composer()

	/* HDMI Initialization Step B.3 */
    /* HDMI Initialization Step B.4 */

    // START drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_enable_video_path()
    /* Again, following from the driver is similar to what catphish did.
       https://people.freebsd.org/~gonzo/arm/iMX6-HDMI.pdf
    */
	/* control period minimum duration */
	hdmi_writeb(12, HDMI_FC_CTRLDUR);
	hdmi_writeb(32, HDMI_FC_EXCTRLDUR);
	hdmi_writeb(1, HDMI_FC_EXCTRLSPAC);
	/* Set to fill TMDS data channels */
	hdmi_writeb(0x0B, HDMI_FC_CH0PREAM);
	hdmi_writeb(0x16, HDMI_FC_CH1PREAM);
	hdmi_writeb(0x21, HDMI_FC_CH2PREAM);
    uint8_t mc_clkdis = 0x7f; // From dw_hdmi_probe()
	/* Enable pixel clock and tmds data path */
	mc_clkdis |= HDMI_MC_CLKDIS_HDCPCLK_DISABLE |
			   HDMI_MC_CLKDIS_CSCCLK_DISABLE |
			   HDMI_MC_CLKDIS_AUDCLK_DISABLE |
			   HDMI_MC_CLKDIS_PREPCLK_DISABLE |
			   HDMI_MC_CLKDIS_TMDSCLK_DISABLE;
	mc_clkdis &= ~HDMI_MC_CLKDIS_PIXELCLK_DISABLE;
	hdmi_writeb(mc_clkdis, HDMI_MC_CLKDIS);
	mc_clkdis &= ~HDMI_MC_CLKDIS_TMDSCLK_DISABLE;
	hdmi_writeb(mc_clkdis, HDMI_MC_CLKDIS);
	/* Enable csc path */
	if (0) { // if is_csc_needed(hdmi) - Colour Space conversion
		mc_clkdis &= ~HDMI_MC_CLKDIS_CSCCLK_DISABLE;
		hdmi_writeb(mc_clkdis, HDMI_MC_CLKDIS);

		hdmi_writeb(HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_IN_PATH,
			    HDMI_MC_FLOWCTRL);
	} else {
		mc_clkdis |= HDMI_MC_CLKDIS_CSCCLK_DISABLE;
		hdmi_writeb(mc_clkdis, HDMI_MC_CLKDIS);
        printf("AAAA 0x%x\n", mc_clkdis);

		hdmi_writeb(HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS,
			    HDMI_MC_FLOWCTRL);
	}
    // END drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_enable_video_path()

    // Back in dw_hdmi_setup()...

    /* SKIPPED HDMI Initialization Step E - Configure audio */
```