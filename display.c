#include <stdbool.h>
#include <unistd.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdio.h>
#include "ccu.h"
#include "system.h"
#include "display.h"

volatile uint32_t* active_buffer;
volatile uint32_t framebuffer1[512*512] __attribute__ ((section ("UNCACHED")));
volatile uint32_t framebuffer2[512*512] __attribute__ ((section ("UNCACHED")));
volatile uint32_t framebuffer3[512*512] __attribute__ ((section ("UNCACHED")));


#define HDMI_IH_PHY_STAT0_RX_SENSE \
	(HDMI_IH_PHY_STAT0_RX_SENSE0 | HDMI_IH_PHY_STAT0_RX_SENSE1 | \
	 HDMI_IH_PHY_STAT0_RX_SENSE2 | HDMI_IH_PHY_STAT0_RX_SENSE3)

#define HDMI_PHY_RX_SENSE \
	(HDMI_PHY_RX_SENSE0 | HDMI_PHY_RX_SENSE1 | \
	 HDMI_PHY_RX_SENSE2 | HDMI_PHY_RX_SENSE3)

uint32_t ccu_readl(uint32_t reg)
{
    return *(uint32_t *)(CCU_BASE + reg);
}

uint32_t ccu_writel(uint32_t val, uint32_t reg)
{
    return *(uint32_t *)(CCU_BASE + reg);
}

uint8_t hdmi_readb(uint32_t reg)
{
    return *(uint8_t *)(HDMI_BASE + reg);
}

void hdmi_writeb(uint8_t val, uint32_t reg)
{
    *(uint8_t *)(HDMI_BASE + reg) = val;
}

void reg_phy_write(uint32_t reg, uint32_t val)
{
    *(uint32_t *)(HDMI_PHY_BASE + reg) = val;
}

void reg_phy_read(uint32_t reg, uint32_t *val)
{
    *val =  *(uint32_t *)(HDMI_PHY_BASE + reg);
}

void reg_phy_update_bits(uint32_t reg, uint32_t mask, uint32_t val) 
{ 
    uint32_t tmp, orig;
 
    reg_phy_read(reg, &orig);
 
    tmp = orig & ~mask;
    tmp |= val & mask;
 
    if (tmp != orig)
        reg_phy_write(reg, tmp);
}

#define reg_phy_read_poll_timeout(reg, val, cond, sleep_us, timeout_us) \
({ \
    uint32_t slept = 0; \
    for (;;) { \
        reg_phy_read(reg, &val); \
        if (cond) \
            break; \
        udelay(sleep_us); \
        slept += sleep_us; \
        if (timeout_us && slept > timeout_us) \
            break; \
    } \
})

void display_clocks_init()
{
    uint32_t val;
    /* This is somewhat documented in the H616 spec, but it is easier to decipher
       the kernel source for this. It's less painful than the H3 code too.

       Everything that needs to be done here can be inferred from
       drivers/clk/sunxi-ng/ccu-sun50i-h616.c in combination with the device-tree.
    */

    // START drivers/clk/sunxi-ng/ccu-sun50i-h616.c sun50i_h616_ccu_probe()

        /* Enable the lock bits and the output enable bits on all PLLs */
        SUN50I_H616_PLL_CPUX_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_DDR0_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_DDR1_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_PERIPH0_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_PERIPH1_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_GPU_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_VIDEO0_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_VIDEO1_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_VIDEO2_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_VE_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_DE_REG |= (1 << 29) | (1 << 27);
        SUN50I_H616_PLL_AUDIO_REG |= (1 << 29) | (1 << 27);

        /*
        * Force the output divider of video PLLs to 0.
        *
        * See the comment before pll-video0 definition for the reason.
        */
        SUN50I_H616_PLL_VIDEO0_REG &= ~(1 << 0);
        SUN50I_H616_PLL_VIDEO1_REG &= ~(1 << 0);
        SUN50I_H616_PLL_VIDEO2_REG &= ~(1 << 0);

        /*
        * Force OHCI 12M clock sources to 00 (12MHz divided from 48MHz)
        *
        * This clock mux is still mysterious, and the code just enforces
        * it to have a valid clock parent.
        */
        SUN50I_H616_USB0_CLK_REG &= ~GENMASK(25, 24);
        SUN50I_H616_USB1_CLK_REG &= ~GENMASK(25, 24);
        SUN50I_H616_USB2_CLK_REG &= ~GENMASK(25, 24);
        SUN50I_H616_USB3_CLK_REG &= ~GENMASK(25, 24);

        /*
        * Set the output-divider for the pll-audio clocks (M0) to 2 and the
        * input divider (M1) to 1 as recommended by the manual when using
        * SDM.
        */
        val = SUN50I_H616_PLL_AUDIO_REG;
        val &= ~(1 << 1);
        val |= (1 << 0);
        SUN50I_H616_PLL_AUDIO_REG = val;

        /*
        * Set the input-divider for the gpu1 clock to 3, to reach a safe 400 MHz.
        */
        val = SUN50I_H616_GPU_CLK1_REG;
        val &= ~GENMASK(1, 0);
        val |= 2;
        SUN50I_H616_GPU_CLK1_REG = val;

        /*
        * First clock parent (osc32K) is unusable for CEC. But since there
        * is no good way to force parent switch (both run with same frequency),
        * just set second clock parent here.
        */
        val = SUN50I_H616_HDMI_CEC_CLK_REG;
        val |= (1 << 24);
        SUN50I_H616_HDMI_CEC_CLK_REG = val;

    // END drivers/clk/sunxi-ng/ccu-sun50i-h616.c sun50i_h616_ccu_probe()

    // TODO: Enable all of the other things we need.
    // SUN50I_H616_HDMI_BGR_REG |= (1<<0); // Enable CLK_BUS_HDMI
    // SUN50I_H616_HDMI_BGR_REG |= (1<<16) | (1<<17); // De-assert reset of RST_BUS_HDMI and RST_BUS_HDMI_SUB
    // SUN50I_H616_HDMI0_SLOW_CLK_REG    = (1<<31); // Enable HDMI slow clk
    // TODO: Need to enable CLK_HDMI
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
}

void hdmi_init() {

    uint32_t val;
    uint8_t bval;
    uint32_t phy_rcal;
    int ret;
    uint8_t phy_mask;

    // The following is taken from the component sunxi driver. See docs/STACKTRACE.txt for the full horrible story.
    // Everything starts from sun8i_dw_hdmi_probe which due to the component nature causes:
    // sun4i_drv_bind()
    // sun4i_tcon_bind
    // sun8i_dw_hdmi_bind()
    // etc.
    // It's very confusing, and I'd like to understand it better.

    // TODO: Anything before this?

                            SUN6I_LOSC_CTRL = 0x16aa0000;

    // START drivers/clk/sunxi-ng/ccu-sun8i-de2.c sunxi_de2_clk_probe()
        SUN50I_H616_DE_BGR_REG |= (1 << 0);  // Enable CLK_BUS_DE
        SUN50I_H616_PLL_DE_REG |= (1 << 31); // Enable pll-de (parent of next)
        SUN50I_H616_DE_CLK_REG |= (1 << 31); // Enable CLK_DE
        SUN50I_H616_DE_BGR_REG |= (1 << 16); // De-assert RST_BUS_DE
        DE_RANDOM_1_REG = DE_RANDOM_1_VAL;
        DE_RANDOM_2_REG = DE_RANDOM_2_VAL;
    // END drivers/clk/sunxi-ng/ccu-sun8i-de2.c sunxi_de2_clk_probe()

    // START HDMI drivers/gpu/drm/sun4i/sun8i_dw_hdmi.c sun8i_dw_hdmi_bind()
        SUN50I_H616_HDMI_BGR_REG |= (1<<16); // De-assert reset of RST_BUS_HDMI aka rst_ctrl
        SUN50I_H616_PLL_VIDEO0_REG |= (1 << 31); // Enable parent PLL pll-video0
        SUN50I_H616_HDMI0_CLK_REG |= (1 << 31); // Enable clock CLK_HDMI aka clk_tmds
        // START PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_hdmi_phy_init()
            SUN50I_H616_HDMI_BGR_REG |= (1 << 17); // De-assert reset of RST_BUS_HDMI_SUB
            SUN50I_H616_HDMI_BGR_REG |= (1 << 0); // Enable clock CLK_BUS_HDMI
            SUN50I_H616_HDMI0_SLOW_CLK_REG |= (1 << 31); // CLK_HDMI_SLOW
            // START PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun50i_hdmi_phy_init_h6()
                reg_phy_update_bits(SUN8I_HDMI_PHY_REXT_CTRL_REG,
                        SUN8I_HDMI_PHY_REXT_CTRL_REXT_EN,
                        SUN8I_HDMI_PHY_REXT_CTRL_REXT_EN);
                reg_phy_update_bits(SUN8I_HDMI_PHY_REXT_CTRL_REG,
                        0xffff0000, 0x80c00000);
            // END PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun50i_hdmi_phy_init_h6()
        // END PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_hdmi_phy_init()
        // START HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_bind()
            // START HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_probe()
                phy_mask = (uint8_t) ~(HDMI_PHY_HPD | HDMI_PHY_RX_SENSE);
                SUN50I_H616_HDMI_CEC_CLK_REG |= (1 << 31) | (1 << 30); // Enable CLK_HDMI_CEC aka cec
                // Get DesignWare version
                uint16_t version = hdmi_readb(HDMI_DESIGN_ID) << 8 | hdmi_readb(HDMI_REVISION_ID);
                uint8_t id0 = hdmi_readb(HDMI_PRODUCT_ID0); // HDMI_PRODUCT_ID0_HDMI_TX
                uint8_t id1 = hdmi_readb(HDMI_PRODUCT_ID1); // BIT(0) must be set
                // START dw_hdmi_detect_phy()
                    uint8_t config_id2 = hdmi_readb(HDMI_CONFIG2_ID); // DW_HDMI_PHY_DWC_HDMI20_TX_PHY
                    printf("HDMI_CONFIG2_ID: %x\n", config_id2);
                // END dw_hdmi_detect_phy()
                printf("HDMI Version: %x ID: %x %x\n", version, id0, id1);
                // START dw_hdmi_init_hw()
                    // START initialize_hdmi_ih_mutes()
                        uint8_t ih_mute;
                        /*
                        * Boot up defaults are:
                        * HDMI_IH_MUTE   = 0x03 (disabled)
                        * HDMI_IH_MUTE_* = 0x00 (enabled)
                        *
                        * Disable top level interrupt bits in HDMI block
                        */
                        ih_mute = hdmi_readb(HDMI_IH_MUTE) |
                            HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT |
                            HDMI_IH_MUTE_MUTE_ALL_INTERRUPT;

                        hdmi_writeb(ih_mute, HDMI_IH_MUTE);

                        /* by default mask all interrupts */
                        hdmi_writeb(0xff, HDMI_VP_MASK);
                        hdmi_writeb(0xff, HDMI_FC_MASK0);
                        hdmi_writeb(0xff, HDMI_FC_MASK1);
                        hdmi_writeb(0xff, HDMI_FC_MASK2);
                        hdmi_writeb(0xff, HDMI_PHY_MASK0);
                        hdmi_writeb(0xff, HDMI_PHY_I2CM_INT_ADDR);
                        hdmi_writeb(0xff, HDMI_PHY_I2CM_CTLINT_ADDR);
                        hdmi_writeb(0xff, HDMI_AUD_INT);
                        hdmi_writeb(0xff, HDMI_AUD_SPDIFINT);
                        hdmi_writeb(0xff, HDMI_AUD_HBR_MASK);
                        hdmi_writeb(0xff, HDMI_GP_MASK);
                        hdmi_writeb(0xff, HDMI_A_APIINTMSK);
                        hdmi_writeb(0xff, HDMI_I2CM_INT);
                        hdmi_writeb(0xff, HDMI_I2CM_CTLINT);

                        /* Disable interrupts in the IH_MUTE_* registers */
                        hdmi_writeb(0xff, HDMI_IH_MUTE_FC_STAT0);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_FC_STAT1);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_FC_STAT2);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_AS_STAT0);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_PHY_STAT0);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_I2CM_STAT0);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_CEC_STAT0);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_VP_STAT0);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_I2CMPHY_STAT0);
                        hdmi_writeb(0xff, HDMI_IH_MUTE_AHBDMAAUD_STAT0);

                        /* Enable top level interrupt bits in HDMI block */
                        ih_mute &= ~(HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT |
                                HDMI_IH_MUTE_MUTE_ALL_INTERRUPT);
                        hdmi_writeb(ih_mute, HDMI_IH_MUTE);
                    // END initialize_hdmi_ih_mutes()
                    // START dw_hdmi_i2c_init()
                        hdmi_writeb(HDMI_PHY_I2CM_INT_ADDR_DONE_POL,
                                HDMI_PHY_I2CM_INT_ADDR);

                        hdmi_writeb(HDMI_PHY_I2CM_CTLINT_ADDR_NAC_POL |
                                HDMI_PHY_I2CM_CTLINT_ADDR_ARBITRATION_POL,
                                HDMI_PHY_I2CM_CTLINT_ADDR);

                        /* Software reset */
                        hdmi_writeb(0x00, HDMI_I2CM_SOFTRSTZ);

                        /* Set Standard Mode speed (determined to be 100KHz on iMX6) */
                        hdmi_writeb(0x00, HDMI_I2CM_DIV);

                        /* Set done, not acknowledged and arbitration interrupt polarities */
                        hdmi_writeb(HDMI_I2CM_INT_DONE_POL, HDMI_I2CM_INT);
                        hdmi_writeb(HDMI_I2CM_CTLINT_NAC_POL | HDMI_I2CM_CTLINT_ARB_POL,
                                HDMI_I2CM_CTLINT);

                        /* Clear DONE and ERROR interrupts */
                        hdmi_writeb(HDMI_IH_I2CM_STAT0_ERROR | HDMI_IH_I2CM_STAT0_DONE,
                                HDMI_IH_I2CM_STAT0);

                        /* Mute DONE and ERROR interrupts */
                        hdmi_writeb(HDMI_IH_I2CM_STAT0_ERROR | HDMI_IH_I2CM_STAT0_DONE,
                                HDMI_IH_MUTE_I2CM_STAT0);
                    // END dw_hdmi_i2c_init()
                    // START dw_hdmi_phy_setup_hpd()
                        /*
                        * Configure the PHY RX SENSE and HPD interrupts polarities and clear
                        * any pending interrupt.
                        */
                        hdmi_writeb(HDMI_PHY_HPD | HDMI_PHY_RX_SENSE, HDMI_PHY_POL0);
                        hdmi_writeb(HDMI_IH_PHY_STAT0_HPD | HDMI_IH_PHY_STAT0_RX_SENSE,
                                HDMI_IH_PHY_STAT0);
                        /* Enable cable hot plug irq. */
                        hdmi_writeb(phy_mask, HDMI_PHY_MASK0);
                        /* Clear and unmute interrupts. */
                        hdmi_writeb(HDMI_IH_PHY_STAT0_HPD | HDMI_IH_PHY_STAT0_RX_SENSE,
                                HDMI_IH_PHY_STAT0);
                        hdmi_writeb(~(HDMI_IH_PHY_STAT0_HPD | HDMI_IH_PHY_STAT0_RX_SENSE),
                                HDMI_IH_MUTE_PHY_STAT0);
                    // END dw_hdmi_phy_setup_hpd()
                // END dw_hdmi_init_hw()
                // We're back in dw_hdmi_probe()!
                // TODO Skipping IRQ setup
                // SKIP hdmi_init_clk_regenerator() - It called hdmi_set_clk_regenerator() and hdmi_set_cts_n() but had no effect in my testing. It's just audio stuff?
            // END HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_probe()
        // END HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_bind()
    // END HDMI drivers/gpu/drm/sun4i/sun8i_dw_hdmi.c sun8i_dw_hdmi_bind()


    // BEGIN dw_hdmi_i2c_xfer() aka Read the EDID!
    /* This little widget gets called when sun4i_drv_bind is
        called and the whole Linux DRM framebuffer thing runs.
        It's currently not working, probably due to clock stuff
        I've not set up yet.
    */
    char edid[128] = {0};
    char pos;
    // HDMI_IH_MUTE_I2CM_STAT0 = 0
    for (pos = 0; pos < sizeof(edid); pos++)
    {
        hdmi_writeb(0x50, HDMI_I2CM_SLAVE);
        hdmi_writeb(pos, HDMI_I2CM_ADDRESS);
        hdmi_writeb(HDMI_I2CM_OPERATION_READ, HDMI_I2CM_OPERATION);
        //HDMI_IH_I2CM_STAT0 = 0x2 would acknowledge the IRQ if we had it.
        udelay(100);
        edid[pos] = hdmi_readb(HDMI_I2CM_DATAI);
    }
    // HDMI_IH_MUTE_I2CM_STAT0 = 3

    for (pos = 0; pos < sizeof(edid); pos++)
    {
        if (pos && !(pos % 16))
            printf("\n");
        printf("%02x", edid[pos]);
    }
    printf("\n");
    // END dw_hdmi_i2c_xfer() aka Read the EDID!
    

    // // START PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_h3_hdmi_phy_config()

    // // It gets intereting... include/drm/drm_modes.h

    // Assume we've got the clock from EDID, which is 32MHz.
    uint32_t clk_rate = 32000000; // Verified correct.

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
    printf("HERE1 \n");
	reg_phy_write(SUN8I_HDMI_PHY_PLL_CFG3_REG,
		     SUN8I_HDMI_PHY_PLL_CFG3_SOUT_DIV2);
	reg_phy_update_bits(SUN8I_HDMI_PHY_PLL_CFG1_REG,
			   SUN8I_HDMI_PHY_PLL_CFG1_PLLEN,
			   SUN8I_HDMI_PHY_PLL_CFG1_PLLEN);
	udelay(10000);
    printf("HERE2 \n");

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

    printf("HERE3 \n");
    
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

    hdisplay = 800;
    vdisplay = 480;
    hblank = 128; // Hfront+Hsync+Hback
    vblank = 45; // Vfront+Vsync+Vback
    h_de_hs = 40; // ???
    v_de_vs = 13; // ???
    hsync_len = 48; // Hsync
    vsync_len = 3; // Vsync
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
}

void lcd_init() {
    // START drivers/gpu/drm/sun4i/sun4i_tcon.c sun4i_tcon1_mode_set()

    // LCD0 feeds mixer0 to HDMI
    LCD0_GCTL         = (1<<31);
    udelay(5);
    LCD0_GINT0        = 0;
    udelay(5);
    LCD0_TCON1_CTL    = (1<<31) | (30<<4);
    udelay(5);
    LCD0_TCON1_BASIC0 = (((800-1)&0xfff)<<16) | ((480-1)&0xfff);
    udelay(5);
    LCD0_TCON1_BASIC1 = (((800-1)&0xfff)<<16) | ((480-1)&0xfff);
    udelay(5);
    LCD0_TCON1_BASIC2 = (((800-1)&0xfff)<<16) | ((480-1)&0xfff);
    udelay(5);
    LCD0_TCON1_BASIC3 = ((((800+40+48+40) - 1) & 0x1fff) << 16) | // Width+Hfront+Hsync+Hback
                            (((48+40) - 1) & 0xfff);              // Hsync+Hback
    udelay(5);
    LCD0_TCON1_BASIC4 = ((((480+13+3+29)*2) & 0x1fff) << 16) | // (Height+Vfront+Vsync+Vback)*2
                                (((3+29) - 1) & 0xfff);        // Vsync+Vback
    udelay(5);
    LCD0_TCON1_BASIC5 = ((((48) - 1) & 0x3ff) << 16) |   // Hsync
                                    (((3) - 1) & 0x3ff); // Vsync
    udelay(5);
    
    LCD0_GINT1 = 1;
    udelay(5);
    LCD0_GINT0 = (1<<28);
    udelay(5);
}

// This function configured DE2 as follows:
// MIXER0 -> WB -> MIXER1 -> HDMI
void de2_init() {
    // START drivers/gpu/drm/sun4i/sun4i_drv.c
  DE_AHB_RESET |= (1<<0);
  DE_SCLK_GATE |= (1<<0);
  DE_HCLK_GATE |= (1<<0);
  DE_DE2TCON_MUX &= ~(1<<0);

  // Erase the whole of MIXER0. This contains uninitialized data.
  for(uint32_t addr = DE_MIXER0 + 0x0000; addr < DE_MIXER0 + 0xC000; addr += 4)
   *(volatile uint32_t*)(addr) = 0;

  DE_MIXER0_GLB_CTL = 1;
  DE_MIXER0_GLB_SIZE = ((480-1)<<16) | (800-1);

  DE_MIXER0_BLD_FILL_COLOR_CTL = 0x100;
  DE_MIXER0_BLD_CH_RTCTL = 0;
  DE_MIXER0_BLD_SIZE = ((480-1)<<16) | (800-1);
  DE_MIXER0_BLD_CH_ISIZE(0) = ((480-1)<<16) | (800-1);

  // The output takes a 480x270 area from a total 512x302
  // buffer leaving a 16px overscan on all 4 sides.
  DE_MIXER0_OVL_V_ATTCTL(0) = (1<<15) | (1<<0);
  DE_MIXER0_OVL_V_MBSIZE(0) = (269<<16) | 479;
  DE_MIXER0_OVL_V_COOR(0) = 0;
  DE_MIXER0_OVL_V_PITCH0(0) = 512*4; // Scan line in bytes including overscan
  DE_MIXER0_OVL_V_TOP_LADD0(0) = (uint32_t)&framebuffer1[512*16+16]; // Start at y=16

  DE_MIXER0_OVL_V_SIZE = (269<<16) | 479;

  DE_MIXER0_VS_CTRL = 1;
  DE_MIXER0_VS_OUT_SIZE = ((480-1)<<16) | (800-1);
  DE_MIXER0_VS_Y_SIZE = (269<<16) | 479;
  DE_MIXER0_VS_Y_HSTEP = 0x40000;
  DE_MIXER0_VS_Y_VSTEP = 0x40000;
  DE_MIXER0_VS_C_SIZE = (269<<16) | 479;
  DE_MIXER0_VS_C_HSTEP = 0x40000;
  DE_MIXER0_VS_C_VSTEP = 0x40000;
  for(int n=0;n<32;n++) {
    DE_MIXER0_VS_Y_HCOEF0(n) = 0x40000000;
    DE_MIXER0_VS_Y_HCOEF1(n) = 0;
    DE_MIXER0_VS_Y_VCOEF(n)  = 0x00004000;
    DE_MIXER0_VS_C_HCOEF0(n) = 0x40000000;
    DE_MIXER0_VS_C_HCOEF1(n) = 0;
    DE_MIXER0_VS_C_VCOEF(n)  = 0x00004000;
  }
  DE_MIXER0_VS_CTRL = 1 | (1<<4);
  DE_MIXER0_GLB_DBUFFER = 1;
}

// This function initializes the HDMI port and TCON.
// Almost everything here is resolution specific and
// currently hardcoded to 1920x1080@60Hz.
void display_init() {
//   active_buffer = framebuffer1;
  display_clocks_init();
  printf("DONE display_clocks_init\n");
  hdmi_init();
  printf("DONE hdmi_init\n");
  lcd_init();
  printf("DONE lcd_init\n");
  de2_init();
  printf("DONE de2_init\n");
}

void buffer_swap() {
  DE_MIXER0_OVL_V_TOP_LADD0(0) = (uint32_t)(active_buffer + 512*16+16);
  if(active_buffer == framebuffer1) {
      active_buffer = framebuffer2;
  } else if(active_buffer == framebuffer2) {
      active_buffer = framebuffer3;
  } else {
      active_buffer = framebuffer1;
  }
  // Blank visible area
//   for(int n=512*16; n<512*(270+16); n++)
//     active_buffer[n] = 0;
  DE_MIXER0_GLB_DBUFFER = 1;
}