#include <stdbool.h>
#include <unistd.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdio.h>
#include "clk/clk.h"
#include "system.h"
#include "display.h"
#include "edid.h"

volatile uint32_t* active_buffer;
volatile uint32_t framebuffer1[512*512] __attribute__ ((section ("UNCACHED")));
volatile uint32_t framebuffer2[512*512] __attribute__ ((section ("UNCACHED")));
volatile uint32_t framebuffer3[512*512] __attribute__ ((section ("UNCACHED")));
volatile uint32_t waiting_for_irq = 0;

#define HDMI_IH_PHY_STAT0_RX_SENSE \
	(HDMI_IH_PHY_STAT0_RX_SENSE0 | HDMI_IH_PHY_STAT0_RX_SENSE1 | \
	 HDMI_IH_PHY_STAT0_RX_SENSE2 | HDMI_IH_PHY_STAT0_RX_SENSE3)

#define HDMI_PHY_RX_SENSE \
	(HDMI_PHY_RX_SENSE0 | HDMI_PHY_RX_SENSE1 | \
	 HDMI_PHY_RX_SENSE2 | HDMI_PHY_RX_SENSE3)

static void writeb(uint8_t val, uint32_t reg)
{
    *(uint8_t *)(reg) = val;
}

static void writel(uint32_t val, uint32_t reg)
{
    *(uint32_t *)(reg) = val;
}

static uint8_t readb(uint32_t reg)
{
    return *(uint8_t *)(reg);
}

static uint32_t readl(uint32_t reg)
{
    return *(uint32_t *)(reg);
}

static void writeb_mask(uint8_t data, uint32_t reg,
			     uint8_t shift, uint8_t mask)
{
    uint8_t val = *(uint8_t *)(reg);
    val &= ~mask;
    val |= (data << shift);
    *(uint8_t *)(reg) = val;
}

void updatel(uint32_t reg, uint32_t mask, uint32_t val) 
{ 
    uint32_t tmp, orig;
 
    orig = *(uint32_t *)(reg);
 
    tmp = orig & ~mask;
    tmp |= val & mask;
 
    if (tmp != orig)
        writel(tmp, reg);
}

/*  IRQ handling
*/

void complete_irq_hdmi(void)
{
    /*  Acknowlege interrupt. We don't care if it's ERROR or DONE. In error case
        result will be all zeroes EDID which will fail safely (nothing to output
        to).
    */
    writeb(HDMI_IH_I2CM_STAT0_ERROR | HDMI_IH_I2CM_STAT0_DONE, HDMI_IH_I2CM_STAT0);
    waiting_for_irq = 0;
}

void await_irq_completion(void)
{
    waiting_for_irq = 1;
    while (waiting_for_irq) {
        asm("WFI");
    }
}

/*  Our driver
*/

void sun50i_h616_ccu_probe(void)
{
    uint32_t val;
    /* Enable the lock bits and the output enable bits on all PLLs */
    SUN50I_H616_CCU_PLL_CPUX_REG    |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_DDR0_REG    |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_DDR1_REG    |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_PERIPH0_REG |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_PERIPH1_REG |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_GPU_REG     |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_VIDEO0_REG  |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_VIDEO1_REG  |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_VIDEO2_REG  |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_VE_REG      |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_DE_REG      |= BIT(29) | BIT(27);
    SUN50I_H616_CCU_PLL_AUDIO_REG   |= BIT(29) | BIT(27);
    /*
    * Force the output divider of video PLLs to 0.
    *
    * See the comment before pll-video0 definition for the reason.
    */
    SUN50I_H616_CCU_PLL_VIDEO0_REG &= ~BIT(0);
    SUN50I_H616_CCU_PLL_VIDEO1_REG &= ~BIT(0);
    SUN50I_H616_CCU_PLL_VIDEO2_REG &= ~BIT(0);
    /*
    * Force OHCI 12M clock sources to 00 (12MHz divided from 48MHz)
    *
    * This clock mux is still mysterious, and the code just enforces
    * it to have a valid clock parent.
    */
    SUN50I_H616_CCU_USB0_CLK_REG &= ~GENMASK(25, 24);
    SUN50I_H616_CCU_USB1_CLK_REG &= ~GENMASK(25, 24);
    SUN50I_H616_CCU_USB2_CLK_REG &= ~GENMASK(25, 24);
    SUN50I_H616_CCU_USB3_CLK_REG &= ~GENMASK(25, 24);
    /*
    * Set the output-divider for the pll-audio clocks (M0) to 2 and the
    * input divider (M1) to 1 as recommended by the manual when using
    * SDM.
    */
    val = SUN50I_H616_CCU_PLL_AUDIO_REG;
    val &= ~BIT(1);
    val |= BIT(0);
    SUN50I_H616_CCU_PLL_AUDIO_REG = val;
    /*
    * Set the input-divider for the gpu1 clock to 3, to reach a safe 400 MHz.
    */
    val = SUN50I_H616_CCU_GPU_CLK1_REG;
    val &= ~GENMASK(1, 0);
    val |= 2;
    SUN50I_H616_CCU_GPU_CLK1_REG = val;
    /*
    * First clock parent (osc32K) is unusable for CEC. But since there
    * is no good way to force parent switch (both run with same frequency),
    * just set second clock parent here.
    */
    val = SUN50I_H616_CCU_HDMI_CEC_CLK_REG;
    val |= BIT(24);
    SUN50I_H616_CCU_HDMI_CEC_CLK_REG = val;

    // devm_sunxi_ccu_probe() is called - i.e. the sunxi-ng framework.

    // [I]cpux is marked as critical, but no gate
    SUN50I_H616_CCU_PLL_CPUX_REG |= BIT(31); // [I]pll-cpux

    // [I]mbus is marked as critical, so enable
    // Parents: [I]pll-periph0
    SUN50I_H616_CCU_PLL_PERIPH0_REG |= BIT(31); // [I]pll-periph0
    SUN50I_H616_CCU_MBUS_CFG_REG    |= BIT(31); // [I]mbus

    // gpu1 is marked as critical, so enable
    SUN50I_H616_CCU_GPU_CLK1_REG |= BIT(31); // [I]gpu1

    // dram is marked as critical, but no gate
    // Parents: pll-ddr0
    SUN50I_H616_CCU_PLL_DDR0_REG |= BIT(31); // [I]pll-ddr0;

    // bus-dram is marked as critical, so enable
    SUN50I_H616_CCU_DRAM_BGR_REG |= BIT(0); // bus-dram

    // TODO?: Skipping the notifier registration
}

void sun6i_rtc_probe(void)
{
    /*  sun6i_rtc_clk_init()
        This function - before we can debug - sets up some internal data for the
        driver.
        Clk [I]losc is created. It has the following parents:
        0 - [I]rtc-int-osc, a fixed-rate clock with rate 16MHz
        1 - NULL - no external losc.
    */

    // First get clk [E]bus(<&r_ccu CLK_R_APB1_RTC>) and enable.
    SUN50I_H6_R_CCU_CLK_R_APB1_RTC_REG |= SUN50I_H6_R_CCU_CLK_R_APB1_RTC_REG_GATE; 
    // TODO?: IRQ is set up here.
	/* clear the alarm counter value */
    SUN6I_RTC_ALRM_COUNTER = 0;
	/* disable counter alarm */
    SUN6I_RTC_ALRM_EN = 0;
	/* disable counter alarm interrupt */
    SUN6I_RTC_ALRM_IRQ_EN = 0;
	/* disable week alarm */
    SUN6I_RTC_ALRM1_EN = 0;
	/* disable week alarm interrupt */
    SUN6I_RTC_ALRM1_IRQ_EN = 0;
	/* clear counter alarm pending interrupts */
    SUN6I_RTC_ALRM_IRQ_STA = SUN6I_RTC_ALRM_IRQ_STA_CNT_IRQ_PEND;
	/* clear week alarm pending interrupts */
    SUN6I_RTC_ALRM1_IRQ_STA = SUN6I_RTC_ALRM1_IRQ_STA_WEEK_IRQ_PEND;
	/* disable alarm wakeup */
    SUN6I_RTC_ALARM_CONFIG = 0;
    /*  [I]losc (see above) is "enabled" here, but it's got no enable op.
        Parent [I]rtc-int-osc also has no enable op.
    */
    // TODO? I skipped some final RTC setup, and also NVMEM on the registers.
}

/*  Initialise the various clock control units.
    There are multiple clock providers that the SOC relies on. These are:
    * R_CCU
    * CCU
    * RTC
    * display_clocks
    This function does the fundamental non-optional configuration that the
    drivers of these IP blocks do on probe().
*/
void clocks_init(void)
{
    /* START R_CCU allwinner,sun50i-h616-r-ccu linux/drivers/clk/sunxi-ng/ccu-sun50i-h6-r.c:sun50i_h6_r_ccu_probe() */
        // [I]r-apb1-twd is marked as critical, so enabled
        // Parents: have no gates
        SUN50I_H6_R_CCU_CLK_R_APB1_TWD_REG |= SUN50I_H6_R_CCU_CLK_R_APB1_TWD_REG_GATE;
    /* END R_CCU allwinner,sun50i-h616-r-ccu linux/drivers/clk/sunxi-ng/ccu-sun50i-h6-r.c:sun50i_h6_r_ccu_probe() */
    /* START CCU allwinner,sun50i-h616-ccu linux/drivers/clk/sunxi-ng/ccu-sun50i-h616.c:sun50i_h616_ccu_probe() */
        sun50i_h616_ccu_probe();
    /* END CCU allwinner,sun50i-h616-ccu linux/drivers/clk/sunxi-ng/ccu-sun50i-h616.c:sun50i_h616_ccu_probe() */
    /* DMA: TODO? the DMA controller starts here. I've ignored it, but if we want to do DMA, I'll need to come back to it. */
    /* DE allwinner,sun50i-h6-display-engine linux/drivers/gpu/drm/sun4i/sun4i_drv.c:sun4i_drv_probe() */
    /* TCON-TV allwinner,sun8i-r40-tcon-tv linux/drivers/gpu/drm/sun4i/sun4i_tcon.c:sun4i_tcon_probe() */
    /* HDMI-PHY allwinner,sun50i-h616-hdmi-phy linux/drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c:sun8i_hdmi_phy_probe() */
    /* TCON-TOP allwinner,sun50i-h6-tcon-top linux/drivers/gpu/drm/sun4i/sun8i_tcon_top.c:sun8i_tcon_top_probe() */
    /* START RTC allwinner,sun50i-h616-rtc linux/drivers/rtc/rtc-sun6i.c:sun6i_rtc_probe() */
        sun6i_rtc_probe();
    /* END RTC allwinner,sun50i-h616-rtc linux/drivers/rtc/rtc-sun6i.c:sun6i_rtc_probe() */
    /* START display_clocks allwinner,sun50i-h616-de33-clk linux/drivers/clk/sunxi-ng/ccu-sun8i-de2.c:sunxi_de2_clk_probe() */
        // [E]bus(<&ccu CLK_BUS_DE>) aka bus-de enable.
        SUN50I_H616_CCU_DE_BGR_REG |= (1 << 0);  // Enable CLK_BUS_DE
        // [E]mod(<&ccu CLK_DE>) aka de enable. Parents: pll-de
        SUN50I_H616_CCU_PLL_DE_REG |= (1 << 31); // Enable pll-de
        SUN50I_H616_CCU_DE_CLK_REG |= (1 << 31); // Enable CLK_DE
        // De-assert [E]<&ccu RST_BUS_DE>
        SUN50I_H616_CCU_DE_BGR_REG |= (1 << 16); // De-assert RST_BUS_DE
        /*
        * The DE33 requires these additional (unknown) registers set
        * during initialisation.
        */
        DE_RANDOM_1_REG = DE_RANDOM_1_VAL;
        DE_RANDOM_2_REG = DE_RANDOM_2_VAL;
        // devm_sunxi_ccu_probe() is called - i.e. the sunxi-ng framework - nothing happens
    /* END display-clocks allwinner,sun50i-h616-de33-clk linux/drivers/clk/sunxi-ng/ccu-sun8i-de2.c:sunxi_de2_clk_probe() */
    /* MIXER allwinner,sun50i-h616-de33-mixer-0 linux/drivers/gpu/drm/sun4i/sun8i_mixer.c:sun8i_mixer_probe() */
    /* HDMI allwinner,sun50i-h6-dw-hdmi linux/drivers/gpu/drm/sun4i/sun8i_dw_hdmi.c:sun8i_dw_hdmi_probe() */

    /* Thus, the clock setup is all done. Hereafter, we call sun4i_drv_bind()
       which will do the serious work to get the framebuffer going. */
}

/*  Now having set up the clocks, we move onto setting up the graphics IP blocks
    correctly. Everything here happens from sun4i_drv_bind()
*/
void display_configure(void) {

    /*  The following is taken from the component sunxi driver.
        We gloss over sun4i_drv_probe() because all that really does is set up
        the component framework so that bind works.
        Everything starts from sun4i_drv_bind() which due to the component nature causes:
        sun8i_mixer_bind()
        sun8i_tcon_top_bind()
        sun4i_tcon_bind
        sun8i_dw_hdmi_bind()
        etc.
    */

    uint32_t val;
    uint8_t bval;
    uint32_t phy_rcal;
    int ret;
    uint8_t phy_mask;
	int plane_cnt, i;

    /* START MIXER allwinner,sun50i-h616-de33-mixer-0 linux/drivers/gpu/drm/sun4i/sun8i_mixer.c:sun8i_mixer_bind() */
    // Ops are defined.
    // DMA is configured.
    // De-assert [E]<&display_clocks RST_MIXER0>
    DE33_CLK_RST_MIXER0 |= BIT(0);
    // [E]bus(<&display_clocks CLK_BUS_MIXER0>) enable aka bus-mixer0
    DE33_CLK_BUS_MIXER0 |= BIT(0);
	/*
	 * It seems that we need to enforce that rate for whatever
	 * reason for the mixer to be functional. Make sure it's the
	 * case.
	 */
    // TODO! I've skipped setting the clock rate. MUST DO!
    // Set CLK_MIXER0 rate to 600 MHz.
    // [E]mod(<&display_clocks CLK_MIXER0>) enable aka mixer0
    DE33_CLK_MIXER0 |= BIT(0);
        /* START sun8i_mixer_init() */
	    /* Enable the mixer */
        SUN8I_MIXER_GLOBAL_CTL = SUN8I_MIXER_GLOBAL_CTL_RT_EN;
        SUN50I_MIXER_GLOBAL_CLK = 1;
	    /* Set background color to black */
        SUN8I_MIXER_BLEND_BKCOLOR = SUN8I_MIXER_BLEND_COLOR_BLACK;
        /*
        * Set fill color of bottom plane to black. Generally not needed
        * except when VI plane is at bottom (zpos = 0) and enabled.
        */
        SUN8I_MIXER_BLEND_PIPE_CTL = SUN8I_MIXER_BLEND_PIPE_CTL_FC_EN(0);
        SUN8I_MIXER_BLEND_ATTR_FCOLOR(0) = SUN8I_MIXER_BLEND_COLOR_BLACK;

        plane_cnt = 1 /* vi_num */ + 3 /* ui_num */;
        for (i = 0; i < plane_cnt; i++) {
            SUN8I_MIXER_BLEND_MODE(i) = SUN8I_MIXER_BLEND_MODE_DEF;
        }
        SUN8I_MIXER_BLEND_PIPE_CTL &= ~SUN8I_MIXER_BLEND_PIPE_CTL_EN_MSK;
        /* END sun8i_mixer_init() */
    /* END MIXER allwinner,sun50i-h616-de33-mixer-0 linux/drivers/gpu/drm/sun4i/sun8i_mixer.c:sun8i_mixer_bind() */

    /* START TCON_TOP allwinner,sun50i-h6-tcon-top drivers/gpu/drm/sun4i/sun8i_tcon_top.c:sun8i_tcon_top_bind() */
    // De-assert [E]<&ccu RST_BUS_TCON_TOP>
    SUN50I_H616_CCU_DISPLAY_IF_TOP_BGR_REG |= BIT(16);
    // [E]bus(<&ccu CLK_BUS_TCON_TOP>) enable aka bus-tcon-top
    SUN50I_H616_CCU_DISPLAY_IF_TOP_BGR_REG |= BIT(0);
    /*
    * At least on H6, some registers have some bits set by default
    * which may cause issues. Clear them here.
    */
    TCON_TOP_PORT_SEL_REG = 0;
    TCON_TOP_GATE_SRC_REG = 0;
    /*
    * TCON TOP has two muxes, which select parent clock for each TCON TV
    * channel clock. Parent could be either TCON TV or TVE clock. For now
    * we leave this fixed to TCON TV, since TVE driver for R40 is not yet
    * implemented. Once it is, graph needs to be traversed to determine
    * if TVE is active on each TCON TV. If it is, mux should be switched
    * to TVE clock parent.
    */
    // Here up to three gates are registered. Nothing is done with them yet.
    // As this variant has no quirks, just one gate - tcon-tv0 - is registered.
    /* END TCON_TOP allwinner,sun50i-h6-tcon-top drivers/gpu/drm/sun4i/sun8i_tcon_top.c:sun8i_tcon_top_bind() */

    /* START TCON_TV allwinner,sun8i-r40-tcon-tv drivers/gpu/drm/sun4i/sun4i_tcon.c:sun4i_tcon_bind() */
    // QUIRKS: has_channel_1, polarity_in_ch0, .set_mux = sun8i_r40_tcon_tv_set_mux
    // De-assert [E]lcd(<&ccu RST_BUS_TCON_TV0>)
    SUN50I_H616_CCU_TCON_TV_BGR_REG |= BIT(16);
    // can_lvds = false;

        // START sun4i_tcon_init_clocks()
        // [E]ahb(<&ccu CLK_BUS_TCON_TV0>) enable aka bus-tcon-tv0
        SUN50I_H616_CCU_TCON_TV_BGR_REG |= BIT(0);
        // END sun4i_tcon_init_clocks()

        // START sun4i_tcon_init_regmap()
        /* Make sure the TCON is disabled and all IRQs are off */
        SUN4I_TCON_GCTL_REG = 0;
        SUN4I_TCON_GINT0_REG = 0;
        SUN4I_TCON_GINT1_REG = 0;
        /* Disable IO lines and set them to tristate */
        SUN4I_TCON0_IO_TRI_REG = ~0;
        SUN4I_TCON1_IO_TRI_REG = ~0;
        // END sun4i_tcon_init_regmap()

        // TODO? sun4i_tcon_init_irq() - I've skipped IRQ

        // START sun4i_crtc_init()
        // This creates and initialises the CRTC.
        // TODO! This initialises layers (see mixer) and is critical
        // END sun4i_crtc_init()

    SUN4I_TCON_GCTL_REG |= SUN4I_TCON_GCTL_PAD_SEL;
    /* END TCON_TV allwinner,sun8i-r40-tcon-tv drivers/gpu/drm/sun4i/sun4i_tcon.c:sun4i_tcon_bind() */

    /* START HDMI allwinner,sun50i-h6-dw-hdmi drivers/gpu/drm/sun4i/sun8i_dw_hdmi.c:sun8i_dw_hdmi_bind() */
    // Ignore the regulator. Not defined in DT so dummy.
    SUN50I_H616_CCU_HDMI_BGR_REG |= (1<<16); // De-assert reset of RST_BUS_HDMI aka rst_ctrl
    SUN50I_H616_CCU_PLL_VIDEO0_REG |= (1 << 31); // Enable parent PLL pll-video0
    SUN50I_H616_CCU_HDMI0_CLK_REG |= (1 << 31); // Enable clock CLK_HDMI aka tmds

        // START PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_hdmi_phy_init()
            SUN50I_H616_CCU_HDMI_BGR_REG |= (1 << 17); // De-assert reset of RST_BUS_HDMI_SUB
            SUN50I_H616_CCU_HDMI_BGR_REG |= (1 << 0); // Enable clock CLK_BUS_HDMI
            SUN50I_H616_CCU_HDMI0_SLOW_CLK_REG |= (1 << 31); // CLK_HDMI_SLOW
                // START PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun50i_hdmi_phy_init_h6()
                updatel(SUN8I_HDMI_PHY_REXT_CTRL_REG,
                        SUN8I_HDMI_PHY_REXT_CTRL_REXT_EN,
                        SUN8I_HDMI_PHY_REXT_CTRL_REXT_EN);
                updatel(SUN8I_HDMI_PHY_REXT_CTRL_REG,
                        0xffff0000, 0x80c00000);
                // END PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun50i_hdmi_phy_init_h6()
        // END PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_hdmi_phy_init()

        // START HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_bind()
            // START HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_probe()
            phy_mask = (uint8_t) ~(HDMI_PHY_HPD | HDMI_PHY_RX_SENSE);
            // Enable clock isfr
            // Enable clock iahb
            SUN50I_H616_CCU_HDMI_CEC_CLK_REG |= (1 << 31) | (1 << 30); // Enable CLK_HDMI_CEC aka cec
            // Get DesignWare version
            uint16_t version = readb(HDMI_DESIGN_ID) << 8 | readb(HDMI_REVISION_ID);
            uint8_t id0 = readb(HDMI_PRODUCT_ID0); // HDMI_PRODUCT_ID0_HDMI_TX
            uint8_t id1 = readb(HDMI_PRODUCT_ID1); // BIT(0) must be set
                // START dw_hdmi_detect_phy()
                uint8_t config_id2 = readb(HDMI_CONFIG2_ID); // DW_HDMI_PHY_DWC_HDMI20_TX_PHY
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
                    ih_mute = readb(HDMI_IH_MUTE) |
                        HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT |
                        HDMI_IH_MUTE_MUTE_ALL_INTERRUPT;
                    writeb(ih_mute, HDMI_IH_MUTE);
                    /* by default mask all interrupts */
                    writeb(0xff, HDMI_VP_MASK);
                    writeb(0xff, HDMI_FC_MASK0);
                    writeb(0xff, HDMI_FC_MASK1);
                    writeb(0xff, HDMI_FC_MASK2);
                    writeb(0xff, HDMI_PHY_MASK0);
                    writeb(0xff, HDMI_PHY_I2CM_INT_ADDR);
                    writeb(0xff, HDMI_PHY_I2CM_CTLINT_ADDR);
                    writeb(0xff, HDMI_AUD_INT);
                    writeb(0xff, HDMI_AUD_SPDIFINT);
                    writeb(0xff, HDMI_AUD_HBR_MASK);
                    writeb(0xff, HDMI_GP_MASK);
                    writeb(0xff, HDMI_A_APIINTMSK);
                    writeb(0xff, HDMI_I2CM_INT);
                    writeb(0xff, HDMI_I2CM_CTLINT);
                    /* Disable interrupts in the IH_MUTE_* registers */
                    writeb(0xff, HDMI_IH_MUTE_FC_STAT0);
                    writeb(0xff, HDMI_IH_MUTE_FC_STAT1);
                    writeb(0xff, HDMI_IH_MUTE_FC_STAT2);
                    writeb(0xff, HDMI_IH_MUTE_AS_STAT0);
                    writeb(0xff, HDMI_IH_MUTE_PHY_STAT0);
                    writeb(0xff, HDMI_IH_MUTE_I2CM_STAT0);
                    writeb(0xff, HDMI_IH_MUTE_CEC_STAT0);
                    writeb(0xff, HDMI_IH_MUTE_VP_STAT0);
                    writeb(0xff, HDMI_IH_MUTE_I2CMPHY_STAT0);
                    writeb(0xff, HDMI_IH_MUTE_AHBDMAAUD_STAT0);
                    /* Enable top level interrupt bits in HDMI block */
                    ih_mute &= ~(HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT |
                            HDMI_IH_MUTE_MUTE_ALL_INTERRUPT);
                    writeb(ih_mute, HDMI_IH_MUTE);
                    // END initialize_hdmi_ih_mutes()

                    // START dw_hdmi_i2c_init()
                    writeb(HDMI_PHY_I2CM_INT_ADDR_DONE_POL,
                            HDMI_PHY_I2CM_INT_ADDR);
                    writeb(HDMI_PHY_I2CM_CTLINT_ADDR_NAC_POL |
                            HDMI_PHY_I2CM_CTLINT_ADDR_ARBITRATION_POL,
                            HDMI_PHY_I2CM_CTLINT_ADDR);
                    /* Software reset */
                    writeb(0x00, HDMI_I2CM_SOFTRSTZ);
                    /* Set Standard Mode speed (determined to be 100KHz on iMX6) */
                    writeb(0x00, HDMI_I2CM_DIV);
                    /* Set done, not acknowledged and arbitration interrupt polarities */
                    writeb(HDMI_I2CM_INT_DONE_POL, HDMI_I2CM_INT);
                    writeb(HDMI_I2CM_CTLINT_NAC_POL | HDMI_I2CM_CTLINT_ARB_POL,
                            HDMI_I2CM_CTLINT);
                    /* Clear DONE and ERROR interrupts */
                    writeb(HDMI_IH_I2CM_STAT0_ERROR | HDMI_IH_I2CM_STAT0_DONE,
                            HDMI_IH_I2CM_STAT0);
                    /* Mute DONE and ERROR interrupts */
                    writeb(HDMI_IH_I2CM_STAT0_ERROR | HDMI_IH_I2CM_STAT0_DONE,
                            HDMI_IH_MUTE_I2CM_STAT0);
                    // END dw_hdmi_i2c_init()

                    // START dw_hdmi_phy_setup_hpd()
                    // TODO: I disabled this; I'm not interested in hotplug events
                    // /*
                    // * Configure the PHY RX SENSE and HPD interrupts polarities and clear
                    // * any pending interrupt.
                    // */
                    // writeb(HDMI_PHY_HPD | HDMI_PHY_RX_SENSE, HDMI_PHY_POL0);
                    // writeb(HDMI_IH_PHY_STAT0_HPD | HDMI_IH_PHY_STAT0_RX_SENSE,
                    //         HDMI_IH_PHY_STAT0);
                    // /* Enable cable hot plug irq. */
                    // writeb(phy_mask, HDMI_PHY_MASK0);
                    /* Clear interrupts. */
                    // writeb(HDMI_IH_PHY_STAT0_HPD | HDMI_IH_PHY_STAT0_RX_SENSE,
                    //         HDMI_IH_PHY_STAT0);
                    // /* Unmute interrupts. */
                    // writeb(~(HDMI_IH_PHY_STAT0_HPD | HDMI_IH_PHY_STAT0_RX_SENSE),
                    //         HDMI_IH_MUTE_PHY_STAT0);
                    // END dw_hdmi_phy_setup_hpd()

                // END dw_hdmi_init_hw()

            // We're back in dw_hdmi_probe()!
            // TODO Skipping IRQ setup - Although in this, dw_hdmi_update_power is called.
            // SKIP hdmi_init_clk_regenerator() - It called hdmi_set_clk_regenerator() and hdmi_set_cts_n() but had no effect in my testing. It's just audio stuff?

            // Further setup skipped. 2 registers are read to configure the driver capabilities. I don't think we care.

            // END HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_probe()
        // END HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_bind()
    /* END HDMI allwinner,sun50i-h6-dw-hdmi drivers/gpu/drm/sun4i/sun8i_dw_hdmi.c:sun8i_dw_hdmi_bind() */

    /* At this point we're back in sun4i_drv_bind(). The component_bind_all()
       has just completed and the DE continutes setup. */

    // drm_vblank_init() - IGNORE

    // aperture_remove_all_conflicting_devices() - IGNORE

    // sun4i_framebuffer_init - This just adds FPs/helpers to the drm_device.

    // drm_kms_helper_poll_init - Output polling / hot plug detection

    // drm_dev_register - Registers the device

    /* The last thing sun4i_drv_bind() does is call drm_client_setup(). This is
       a generic kernel function, so we are only really interested in the sunxi
       functions that it calls into...
    */

    // dw_hdmi_phy_read_hpd() called to check if connector connected

    // BEGIN dw_hdmi_connector_get_modes()
        // BEGIN dw_hdmi_edid_read()/dw_hdmi_i2c_xfer()
        /* This little widget gets called when sun4i_drv_bind calls into drm_client_setup
            and the whole Linux DRM framebuffer thing runs.
        */
        char edid[128] = {0};
        char pos;
        writeb(0x0, HDMI_IH_MUTE_I2CM_STAT0);
        for (pos = 0; pos < sizeof(edid); pos++)
        {
            writeb(0x50, HDMI_I2CM_SLAVE);
            writeb(pos, HDMI_I2CM_ADDRESS);
            writeb(HDMI_I2CM_OPERATION_READ, HDMI_I2CM_OPERATION);
            await_irq_completion();
            edid[pos] = readb(HDMI_I2CM_DATAI);
        }
        // HDMI_IH_MUTE_I2CM_STAT0 = 3
        struct edid e = {0};
        parse_edid_structure(edid, &e);
        print_edid(&e);
        for (pos = 0; pos < sizeof(edid); pos++)
        {
            if (pos && !(pos % 16))
                printf("\n");
            printf("%02x", edid[pos]);
        }
        printf("\n");
        // END dw_hdmi_edid_read()/dw_hdmi_i2c_xfer()
    // END dw_hdmi_connector_get_modes()
    
    // BEGIN sun4i_tcon_mode_set()
    // DRM_MODE_ENCODER_TMDS
        // BEGIN sun4i_tcon1_mode_set(tcon, mode);
	    /* Configure the dot clock */
        // clk_set_rate(tcon->sclk1, 32000 /* from where? */ * 1000 / 1 ); aka "tcon-ch1" <&tcon_top CLK_TCON_TOP_TV0>
	    /* Adjust clock delay */
        // clk_delay = 30
        val = SUN4I_TCON1_CTL_REG & ~SUN4I_TCON1_CTL_CLK_DELAY_MASK;
        val |= SUN4I_TCON1_CTL_CLK_DELAY(30);
        SUN4I_TCON1_CTL_REG = val;
	    /* Set interlaced mode */
	    /* Set the input resolution */
        // regmap_write(tcon->regs, SUN4I_TCON1_BASIC0_REG,
        //         SUN4I_TCON1_BASIC0_X(mode->crtc_hdisplay / div) | 800
        //         SUN4I_TCON1_BASIC0_Y(mode->crtc_vdisplay)); 480
        SUN4I_TCON1_BASIC0_REG = SUN4I_TCON1_BASIC0_X(800) | SUN4I_TCON1_BASIC0_Y(480);
	    /* Set the upscaling resolution */
        SUN4I_TCON1_BASIC1_REG = SUN4I_TCON1_BASIC1_X(800) | SUN4I_TCON1_BASIC1_Y(480);
        // regmap_write(tcon->regs, SUN4I_TCON1_BASIC1_REG,
        //         SUN4I_TCON1_BASIC1_X(mode->crtc_hdisplay / div) |
        //         SUN4I_TCON1_BASIC1_Y(mode->crtc_vdisplay));
	    /* Set the output resolution */
        SUN4I_TCON1_BASIC2_REG = SUN4I_TCON1_BASIC2_X(800) | SUN4I_TCON1_BASIC2_Y(480);
	    /* Set horizontal display timings */
        // htotal = hfront + hwidth + hsync + hback = 928
        // bp = hsync + hback
        SUN4I_TCON1_BASIC3_REG = SUN4I_TCON1_BASIC3_H_TOTAL(928) | SUN4I_TCON1_BASIC3_H_BACKPORCH(88);
        /*
        * The vertical resolution needs to be doubled in all
        * cases. We could use crtc_vtotal and always multiply by two,
        * but that leads to a rounding error in interlace when vtotal
        * is odd.
        *
        * This happens with TV's PAL for example, where vtotal will
        * be 625, crtc_vtotal 312, and thus crtc_vtotal * 2 will be
        * 624, which apparently confuses the hardware.
        *
        * To work around this, we will always use vtotal, and
        * multiply by two only if we're not in interlace.
        */
        // vtotal = 525 * 2
	    /* Set vertical display timings */
        // bp = vsync + vback
        SUN4I_TCON1_BASIC4_REG = SUN4I_TCON1_BASIC4_V_TOTAL(525*2) | SUN4I_TCON1_BASIC4_V_BACKPORCH(32);
	    /* Set Hsync and Vsync length */
        // vsync = 3
        // hsync = 48
        SUN4I_TCON1_BASIC5_REG = SUN4I_TCON1_BASIC5_V_SYNC(3) | SUN4I_TCON1_BASIC5_H_SYNC(48);
	    /* Setup the polarity of multiple signals */
        // Quirk .polarity_in_ch0
        val = 0;
        // If +ve, change
        SUN4I_TCON0_IO_POL_REG = val;
	    /* Map output pins to channel 1 */
        val = SUN4I_TCON_GCTL_REG & ~SUN4I_TCON_GCTL_IOMAP_MASK;
        val |= SUN4I_TCON_GCTL_IOMAP_TCON1;
        SUN4I_TCON_GCTL_REG = val;
        // END sun4i_tcon1_mode_set(tcon, mode);

        // BEGIN sun4i_tcon_set_mux()/sun8i_r40_tcon_tv_set_mux()
            // BEGIN sun8i_tcon_top_set_hdmi_src()
            val = TCON_TOP_GATE_SRC_REG;
            val &= ~TCON_TOP_HDMI_SRC_MSK;
            val |= (TCON_TOP_HDMI_SRC_MSK, (1 << 28)); // 1
            TCON_TOP_GATE_SRC_REG = val;
            // END sun8i_tcon_top_set_hdmi_src()
            // BEGIN sun8i_tcon_top_de_config()
            val = TCON_TOP_PORT_SEL_REG;
            val &= ~TCON_TOP_PORT_DE0_MSK;
            val |= 2;
            TCON_TOP_PORT_SEL_REG = val;
            // END sun8i_tcon_top_de_config()
        // END sun4i_tcon_set_mux()/sun8i_r40_tcon_tv_set_mux()
    // END sun4i_tcon_mode_set()

    // START sunxi_engine_mode_set()/sun8i_mixer_mode_set()
    SUN50I_MIXER_GLOBAL_SIZE = SUN8I_MIXER_SIZE(800, 480);
    SUN8I_MIXER_BLEND_OUTSIZE = SUN8I_MIXER_SIZE(800, 480);

	if (0) /* interlaced */
		val = SUN8I_MIXER_BLEND_OUTCTL_INTERLACED;
	else
		val = 0;
    val = SUN8I_MIXER_BLEND_OUTCTL;
    val &= ~SUN8I_MIXER_BLEND_OUTCTL_INTERLACED;
    val |= 0;
    SUN8I_MIXER_BLEND_BKCOLOR = SUN8I_MIXER_BLEND_COLOR_BLACK;
    SUN8I_MIXER_BLEND_ATTR_FCOLOR(0) = SUN8I_MIXER_BLEND_COLOR_BLACK;
        // START sun50i_fmt_setup()
        SUN50I_FMT_CTRL = 0;
        SUN50I_FMT_SIZE = SUN8I_MIXER_SIZE(800, 480);
        SUN50I_FMT_SWAP = 0;
        SUN50I_FMT_DEPTH = 0; // assume bit10 false
        SUN50I_FMT_FORMAT = 0; // Assume default colourspace
        SUN50I_FMT_COEF = 0;
        SUN50I_FMT_LMT_Y = (4095 << 16) | 0;
        SUN50I_FMT_LMT_C0 = (4095 << 16) | 0;
        SUN50I_FMT_LMT_C1 = (4095 << 16) | 0;
        SUN50I_FMT_CTRL = 1;
        // END sun50i_fmt_setup()
    // FIN sunxi_engine_mode_set()/sun8i_mixer_mode_set()

    // BEGIN sun4i_tcon_set_status - enable
    // encoder type for HDMI is TMDS - channel = 1
    SUN4I_TCON_GCTL_REG |= SUN4I_TCON_GCTL_TCON_ENABLE;
        // BEGIN sun4i_tcon_channel_set_status
        SUN4I_TCON1_CTL_REG |= SUN4I_TCON1_CTL_TCON_ENABLE;
        TCON_TOP_GATE_SRC_REG |= (1 << 20); // Enable tcon-ch1 <&tcon_top CLK_TCON_TOP_TV0>
        // END sun4i_tcon_channel_set_status
    // END sun4i_tcon_set_status - enable

    // We must set the "tmds" <&ccu CLK_HDMI> to the pixelclock (108MHz)
    // static const char * const hdmi_parents[] = { "pll-video0", "pll-video0-4x",
	// 				     "pll-video2", "pll-video2-4x" };
    // static SUNXI_CCU_M_WITH_MUX_GATE(hdmi_clk, "hdmi", hdmi_parents, 0xb00,
    //                 0, 4,		/* M */
    //                 24, 2,		/* mux */
    //                 BIT(31),	/* gate */
    //                 0);
    // These clock registers have the following implementation:
    // M or Factor M is a dividing factor. Here bits 3:0 are M.
    // Bits 25:24 are a mux, between four parent clocks. PLL_VIDEO0(1X), PLL_VIDEO0(4X), PLL_VIDEO2(1X), PLL_VIDEO2(4X)



    // START dw_hdmi_bridge_atomic_enable
        // START dw_hdmi_update_power
            // START dw_hdmi_poweron
                // START dw_hdmi_setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // Skipping ahead a little...

    // dw_hdmi_phy_init()
        // dw_hdmi_phy_sel_data_en_pol()
        writeb_mask(1, HDMI_PHY_CONF0, HDMI_PHY_CONF0_SELDATAENPOL_OFFSET, HDMI_PHY_CONF0_SELDATAENPOL_MASK);
        // dw_hdmi_phy_sel_interface_control()
        writeb_mask(0, HDMI_PHY_CONF0, HDMI_PHY_CONF0_SELDIPIF_OFFSET, HDMI_PHY_CONF0_SELDIPIF_MASK);
        // hdmi_phy_configure -- TODO

        // dw_hdmi_phy_power_off
        // dw_hdmi_phy_gen2_txpwron
        // dw_hdmi_phy_gen2_pddq
        // dw_hdmi_set_high_tmds_clock_ratio
        // Seriously, it might not be the first run though, but it happens.
}

/*  This function attempts to get graphics working.
*/
void display_init() {
//   active_buffer = framebuffer1;
  clocks_init();
  printf("DONE clocks_init\n");
  display_configure();
  printf("DONE display_configure\n");
}

void buffer_swap() {
//   DE_MIXER0_OVL_V_TOP_LADD0(0) = (uint32_t)(active_buffer + 512*16+16);
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
//   DE_MIXER0_GLB_DBUFFER = 1;
}
