#include <stdbool.h>
#include <unistd.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdio.h>
#include "clk/clk.h"
#include "system.h"
#include "display.h"
#include "edid.h"

/*
Max width: 8192
Max height: 8192
Max clock: ????
*/

volatile uint32_t* active_buffer;
volatile uint32_t framebuffer1[512*512] __attribute__ ((section ("UNCACHED")));
volatile uint32_t framebuffer2[512*512] __attribute__ ((section ("UNCACHED")));
volatile uint32_t framebuffer3[512*512] __attribute__ ((section ("UNCACHED")));
volatile uint32_t framebufferz[1000*1000] __attribute__ ((section ("UNCACHED")));
volatile uint32_t waiting_for_irq = 0;

#define HDMI_IH_PHY_STAT0_RX_SENSE \
	(HDMI_IH_PHY_STAT0_RX_SENSE0 | HDMI_IH_PHY_STAT0_RX_SENSE1 | \
	 HDMI_IH_PHY_STAT0_RX_SENSE2 | HDMI_IH_PHY_STAT0_RX_SENSE3)

#define HDMI_PHY_RX_SENSE \
	(HDMI_PHY_RX_SENSE0 | HDMI_PHY_RX_SENSE1 | \
	 HDMI_PHY_RX_SENSE2 | HDMI_PHY_RX_SENSE3)

static void writeb(uint8_t val, uint64_t reg)
{
    *(volatile uint8_t *)(reg) = val;
}

static void writel(uint32_t val, uint64_t reg)
{
    *(volatile uint32_t *)(reg) = val;
}

static uint8_t readb(uint64_t reg)
{
    return *(volatile uint8_t *)(reg);
}

static uint32_t readl(uint64_t reg)
{
    return *(volatile uint32_t *)(reg);
}

static void writeb_mask(uint8_t data, uint64_t reg,
			     uint8_t shift, uint8_t mask)
{
    uint8_t val = *(volatile uint8_t *)(reg);
    val &= ~mask;
    val |= (mask & (data << shift));
    *(volatile uint8_t *)(reg) = val;
}

void updatel(volatile uint64_t reg, uint32_t mask, uint32_t val) 
{ 
    uint32_t tmp, orig;
 
    orig = *(volatile uint32_t *)(reg);
 
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
    // SUN50I_H616_CCU_XXXXXXXXXXX &= ~BIT(16);
    // SUN50I_H616_CCU_XXXXXXXXXXX |= BIT(16);
    // SUN50I_H616_CCU_XXXXXXXXXXX &= ~BIT(16);
    // SUN50I_H616_CCU_XXXXXXXXXXX &= ~BIT(0);
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
        // devm_sunxi_ccu_probe() is called - i.e. the sunxi-ng framework - nothing happens
    /* END display-clocks allwinner,sun50i-h616-de33-clk linux/drivers/clk/sunxi-ng/ccu-sun8i-de2.c:sunxi_de2_clk_probe() */
    /* MIXER allwinner,sun50i-h616-de33-mixer-0 linux/drivers/gpu/drm/sun4i/sun8i_mixer.c:sun8i_mixer_probe() */
    /* HDMI allwinner,sun50i-h6-dw-hdmi linux/drivers/gpu/drm/sun4i/sun8i_dw_hdmi.c:sun8i_dw_hdmi_probe() */

    /* Thus, the clock setup is all done. Hereafter, we call sun4i_drv_bind()
       which will do the serious work to get the framebuffer going. */
}

void reclaim_sram_c_and_erase(void)
{
    /* This piece of shitty code is me figuring out that I need to
       'reclaim' SRAM_C which was 'borrowed' from DE at startup. Sigh.
       The whole DE (display_clocks + mixer0) will fail to work without this.
       And the dangerous confusion is that everything else works: TCON_TOP,
       TCON_TV, HDMI, HDMI_PHY are all good, but the Mixer won't be working
       without this and a cyan colour will be output unless it's working.
    */
    /* claim sram */
    // ram DT 0x00028000
    uint32_t test = *((uint32_t *)(0x3000000  + 0x4));
    *((uint32_t *)(0x3000000  + 0x4)) = test & ~0x1000000;
    // test = *((uint32_t *)(0x3000000  + 0x4));
    // Erase the whole of DE. It contains uninitialized data.
    for(uint64_t addr = DE_BASE; addr < (DE_BASE + 0x400000); addr += 4)
    {
        *(volatile uint32_t*)(addr) = 0;
    }
}

static bool hdmi_phy_wait_i2c_done(int msec)
{
	uint32_t val;

	while ((val = readb(HDMI_IH_I2CMPHY_STAT0) & 0x3) == 0) {
		if (msec-- == 0)
			return false;
		udelay(1000);
	}
	writeb(val, HDMI_IH_I2CMPHY_STAT0);

	return true;
}

void dw_hdmi_phy_i2c_write(unsigned short data, unsigned char addr)
{
	writeb(0xFF, HDMI_IH_I2CMPHY_STAT0);
	writeb(addr, HDMI_PHY_I2CM_ADDRESS_ADDR);
	writeb((unsigned char)(data >> 8), HDMI_PHY_I2CM_DATAO_1_ADDR);
	writeb((unsigned char)(data >> 0), HDMI_PHY_I2CM_DATAO_0_ADDR);
	writeb(HDMI_PHY_I2CM_OPERATION_ADDR_WRITE, HDMI_PHY_I2CM_OPERATION_ADDR);
	hdmi_phy_wait_i2c_done(1000);
}

/*  Now having set up the clocks, we move onto setting up the graphics IP blocks
    correctly. Everything here happens from sun4i_drv_bind()
*/
void display_configure(void)
{
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


        /*
        * The DE33 requires these additional (unknown) registers set
        * during initialisation.
        */
        DE_RANDOM_1_REG = DE_RANDOM_1_VAL;
        DE_RANDOM_2_REG = DE_RANDOM_2_VAL;

    // *((uint32_t *)0x07010190) = 0;
    // *((uint32_t *)0x07000000) = 0;
    // *((uint32_t *)0x07000014) = 0x429;
    // *((uint32_t *)0x0300170c) = 0;
    // *((uint32_t *)0x03001804) = 5;
    // *((uint32_t *)0x03001830) = 0x8100000b;
    // *((uint32_t *)0x0300184c) = 0x00030003;
    // *((uint32_t *)0x0300197c) = 0x10001;
    // *((uint32_t *)0x030019f0) = 0x10001;
    // *((uint32_t *)0x03001a74) = 0xe0000000;
    // *((uint32_t *)0x03001a8c) = 0x01620122;
    // *((uint32_t *)0x03001ff0) = 0x20001;
    // // *((uint32_t *)0x01008028) = 0xa980; // THIS!!!
    // *((uint32_t *)0x07010190) = 0;
    // *((uint32_t *)0x07010190) = 0;
    // *((uint32_t *)0x07010190) = 0;
    // *((uint32_t *)0x07010190) = 0;
    // *((uint32_t *)0x07010190) = 0;
    // *((uint32_t *)0x07010190) = 0;
    // *((uint32_t *)0x07010190) = 0;
    // *((uint32_t *)0x07010190) = 0;

    uint32_t val;
    uint8_t bval;
    uint32_t phy_rcal;
    int ret;
    uint8_t phy_mask;
	uint64_t plane_cnt, i;

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
    // [E]mod(<&display_clocks CLK_MIXER0>) aka mixer0, set to 600 MHz.
    // Clock has no op for set clk, but need to propagate to parents.
    // Parent: mixer0-div (ccu_div_set_rate)
    // Parent: de (ccu_div_set_rate)
    // Parents: pll-de (ccu_nkmp_set_rate), pll-periph0-2x
                                            // Parents: pll-periph0 (ccu_nkmp_set_rate)
    /* 1: pll-de, set to 600MHz, parent=24MHz
       N=25, PLL_FACTOR_N=24 (bits 15:8)
       M=1, PLL_INPUT_DIV_M1=0 (bits 1)
       P=1, PLL_OUTPUT_DIV_M0=0 (bits 0)
    */
    val = SUN50I_H616_CCU_PLL_DE_REG;
    val &= ~(GENMASK(15, 8) | BIT(1) | BIT(0));
    val |= (24 << 8) | (0 << 1) | (0 << 0);
    SUN50I_H616_CCU_PLL_DE_REG = val;
    /* 2:de set to 600MHz, parent=600MHz
       CLK_SRC_SEL=0(PLL-DE) (bits 24)
       M=1, FACTOR_M=0 (bits 3:0)
    */
    val = SUN50I_H616_CCU_DE_CLK_REG;
    val &= ~(BIT(24) | GENMASK(3, 0));
    val |= (0 << 24) | (0 << 0);
    SUN50I_H616_CCU_DE_CLK_REG = val;
    /* 3:mixer0-div set to 600MHz, parent=600MHz
       M=1, FACTOR_M=0 (bits 3:0)
    */
    val = DE33_CLK_MIXER0_DIV;
    val &= ~(GENMASK(3, 0));
    val |= (0 << 0);
    DE33_CLK_MIXER0_DIV = val;
    /* 4:mixer0_clk NOP
    */
    // [E]mod(<&display_clocks CLK_MIXER0>) enable aka mixer0
    DE33_CLK_MIXER0 |= BIT(0);
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
    // Reset(assert/deassert) [E]lcd(<&ccu RST_BUS_TCON_TV0>)
    SUN50I_H616_CCU_TCON_TV_BGR_REG &= ~BIT(16);
    udelay(10);
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

                // sun8i_layers_init - Creates one video layer and 3 ui layers.

        // END sun4i_crtc_init()
    SUN4I_TCON_GCTL_REG |= SUN4I_TCON_GCTL_PAD_SEL;
    /* END TCON_TV allwinner,sun8i-r40-tcon-tv drivers/gpu/drm/sun4i/sun4i_tcon.c:sun4i_tcon_bind() */

    /* START HDMI allwinner,sun50i-h6-dw-hdmi drivers/gpu/drm/sun4i/sun8i_dw_hdmi.c:sun8i_dw_hdmi_bind() */
    // QUIRKS: use_drm_infoframe, .mode_valid = sun8i_dw_hdmi_mode_valid_h6
    // encoder->possible_crtcs = sun8i_dw_hdmi_find_possible_crtcs(drm, dev->of_node);
    // Ignore the regulator. Not defined in DT so dummy.
    // De-assert [E]ctrl(<&ccu RST_BUS_HDMI>)
    SUN50I_H616_CCU_HDMI_BGR_REG   |= BIT(16);
    // [E]tmds(<&ccu CLK_HDMI>) enable aka hdmi
    // Parents: [I]pll-video0
    SUN50I_H616_CCU_PLL_VIDEO0_REG |= BIT(31);
    SUN50I_H616_CCU_HDMI0_CLK_REG  |= BIT(31);
        /* START PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_hdmi_phy_init() */
        // De-assert [E]phy(<&ccu RST_BUS_HDMI_SUB>)
        SUN50I_H616_CCU_HDMI_BGR_REG |= BIT(17);
        // [E]bus(<&ccu CLK_BUS_HDMI>) enable aka bus-hdmi
        SUN50I_H616_CCU_HDMI_BGR_REG |= BIT(0);
        // [E]mod(<&ccu CLK_HDMI_SLOW>) enable aka hdmi-slow
        SUN50I_H616_CCU_HDMI0_SLOW_CLK_REG |= BIT(31);
            /* START PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun50i_hdmi_phy_init_h6() */
            updatel(SUN8I_HDMI_PHY_REXT_CTRL_REG,
                    SUN8I_HDMI_PHY_REXT_CTRL_REXT_EN,
                    SUN8I_HDMI_PHY_REXT_CTRL_REXT_EN);
            updatel(SUN8I_HDMI_PHY_REXT_CTRL_REG,
                    0xffff0000, 0x80c00000);
            /* END PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun50i_hdmi_phy_init_h6() */
        /* END PHY drivers/gpu/drm/sun4i/sun8i_hdmi_phy.c sun8i_hdmi_phy_init() */

        /* START HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_bind() */
            /* START HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_probe() */
            phy_mask = (uint8_t) ~(HDMI_PHY_HPD | HDMI_PHY_RX_SENSE);
            // [E]isfr(<&ccu CLK_HDMI_SLOW>) enable: already done
            // [E]iahb(<&ccu CLK_BUS_HDMI>) enable: already done
            // [E]cec(<&ccu CLK_HDMI_CEC>) enable aka hdmi-cec
            SUN50I_H616_CCU_HDMI_CEC_CLK_REG |= BIT(31) | BIT(30);
            // Get DesignWare version
            uint16_t version = readb(HDMI_DESIGN_ID) << 8 | readb(HDMI_REVISION_ID);
            uint8_t id0 = readb(HDMI_PRODUCT_ID0); // HDMI_PRODUCT_ID0_HDMI_TX
            uint8_t id1 = readb(HDMI_PRODUCT_ID1); // BIT(0) must be set
                // START dw_hdmi_detect_phy()
                uint8_t config_id2 = readb(HDMI_CONFIG2_ID); // DW_HDMI_PHY_DWC_HDMI20_TX_PHY
                printf("HDMI_CONFIG2_ID: %x\n", config_id2);
                // END dw_hdmi_detect_phy()
            printf("HDMI Version: %x ID: %x %x\n", version, id0, id1);
                /* START dw_hdmi_init_hw() */
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
                    /* SKIP dw_hdmi_phy_setup_hpd() - TODOTODO I'm not interested in hotplug events? */
                /* END dw_hdmi_init_hw() */
            /* TODOTODO Skipping IRQ setup - Although in this, dw_hdmi_update_power is called. */
            /* SKIP hdmi_init_clk_regenerator() - TODOTODO It called hdmi_set_clk_regenerator() and hdmi_set_cts_n() but had no effect in my testing. It's just audio stuff? */
            /* SKIP Further setup skipped. 2 registers are read to configure the driver capabilities. I don't think we care. */
            /* END HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_probe() */
        /* SKIP drm_bridge_add() call causes calls into dw_hdmi_bridge_attach()/dw_hdmi_connector_create() TODO: This might actually do something*/
        /* END HDMI drivers/gpu/drm/bridge/synopsys/dw-hdmi.c dw_hdmi_bind() */
    /* END HDMI allwinner,sun50i-h6-dw-hdmi drivers/gpu/drm/sun4i/sun8i_dw_hdmi.c:sun8i_dw_hdmi_bind() */

    /* At this point we're back in sun4i_drv_bind(). The component_bind_all()
       has just completed and the DE continutes setup. */

    // drm_vblank_init() - Nothing interesting
    // aperture_remove_all_conflicting_devices() - Nothing interesting
    // sun4i_framebuffer_init - This just adds FPs/helpers to the drm_device.
    // drm_kms_helper_poll_init - Output polling / hot plug detection
    // drm_dev_register - Registers the device

    /* The last thing sun4i_drv_bind() does is call drm_client_setup(). This is
       a generic kernel function, so we are only really interested in the sunxi
       functions that it calls into...
    */

    // BEGIN dw_hdmi_connector_get_modes()
        // BEGIN dw_hdmi_edid_read()/dw_hdmi_i2c_xfer()
        // TODOTODO why does this also do a write?
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

    // drm_edid.c - Parse the EDID data into a DRM mode in drm_mode_detailed()
    
    // BEGIN sun4i_tcon_mode_set()
    // DRM_MODE_ENCODER_TMDS
        // BEGIN sun4i_tcon1_mode_set(tcon, mode);
	    /* Configure the dot clock clk_set_rate(tcon->sclk1, 32000 (from EDID * 1000 / 1 ); aka tcon-ch1 <&tcon_top CLK_TCON_TOP_TV0> aka tcon-top-tv0 */
        // Clock has no op for set clk, but need to propagate to parents.
        // Parent: pll-video0 pll-video0-4x pll-video1 pll-video1-4x
        // The algorithm decides to set pll_video0_clk to 96MHz
        /* 1: pll-video0, want 96MHz/108MHz, parent=24MHz
        FIXED_POST_DIV=4, so actually want 384MHz/432MHz
        N=16, PLL_FACTOR_N=15 (bits 15:8) / N=18, FACTOR_N=17
        M=1, PLL_INPUT_DIV_M=0 (bits 1) / Ditto
        */
        val = SUN50I_H616_CCU_PLL_VIDEO0_REG;
        val &= ~(GENMASK(15, 8) | BIT(1));
        val |= (15 << 8) | (0 << 1);
        SUN50I_H616_CCU_PLL_VIDEO0_REG = val;
        /* 2: tve0, set to 96MHz, parent=96MHz
        N=1, FACTOR_N=0 (bits 9:8)
        M=1, FACTOR_M=0 (bits 3:0)
        */
        val = SUN50I_H616_CCU_TVE0_CLK_REG;
        val &= ~(GENMASK(9, 8) | GENMASK(3, 0));
        val |= (0 << 8) | (0 << 0);
        SUN50I_H616_CCU_TVE0_CLK_REG = val;
        /* 3: tcon-tv1, set to 96MHz, parent=96MHz
        N=1, FACTOR_N=0 (bits 9:8)
        M=1, FACTOR_M=0 (bits 3:0)
        */
        val = SUN50I_H616_CCU_TCON_TV1_CLK_REG;
        val &= ~(GENMASK(9, 8) | GENMASK(3, 0));
        val |= (0 << 8) | (0 << 0);
        SUN50I_H616_CCU_TCON_TV1_CLK_REG = val;
        /* 4: tcon-tv0, set to 32MHz, parent=96MHz
        N=1, FACTOR_N=0 (bits 9:8)
        M=3, FACTOR_M=2 (bits 3:0)
        */
        val = SUN50I_H616_CCU_TCON_TV0_CLK_REG;
        val &= ~(GENMASK(9, 8) | GENMASK(3, 0));
        val |= (0 << 8) | (2 << 0); /* (0<<2) was a mistake.*/
                                    /* (2<<0) is what it should be? */
                                    /* Higher values produce some wonderfully janky output */
        SUN50I_H616_CCU_TCON_TV0_CLK_REG = val;
        /* 5: hdmi-clk, set to 96MHz, parent=96MHz
        M=1, FACTOR_M=0 (bits 3:0)
        */
        val = SUN50I_H616_CCU_HDMI0_CLK_REG;
        val &= ~(GENMASK(3, 0));
        val |= (0 << 0);
        SUN50I_H616_CCU_HDMI0_CLK_REG = val;
	    /* Adjust clock delay */
        // clk_delay = 30
        val = SUN4I_TCON1_CTL_REG & ~SUN4I_TCON1_CTL_CLK_DELAY_MASK;
        val |= SUN4I_TCON1_CTL_CLK_DELAY(30);
        SUN4I_TCON1_CTL_REG = val;
	    /* TODOTODO Set interlaced mode */
	    /* Set the input resolution */
        // regmap_write(tcon->regs, SUN4I_TCON1_BASIC0_REG,
        //         SUN4I_TCON1_BASIC0_X(mode->crtc_hdisplay / div) | 800
        //         SUN4I_TCON1_BASIC0_Y(mode->crtc_vdisplay)); 480
        SUN4I_TCON1_BASIC0_REG = SUN4I_TCON1_BASIC0_X(800) | SUN4I_TCON1_BASIC0_Y(480);
	    /* Set the upscaling resolution */
        // NOTE: If these values are reduced, the layer is bordered.
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
		// if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		// 	val |= SUN4I_TCON0_IO_POL_HSYNC_POSITIVE;
		// if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		// 	val |= SUN4I_TCON0_IO_POL_VSYNC_POSITIVE;

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
            val |= (1 << 28); // 1
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


    /* hdmi-clk, set to 32MHz, parent=96MHz
    M=3, FACTOR_M=2 (bits 3:0)
    */
    val = SUN50I_H616_CCU_HDMI0_CLK_REG;
    val &= ~(GENMASK(3, 0));
    val |= (2 << 0);
    SUN50I_H616_CCU_HDMI0_CLK_REG = val;

    // BEGIN sun4i_tcon_set_status - enable
    // encoder type for HDMI is TMDS - channel = 1
    SUN4I_TCON_GCTL_REG |= SUN4I_TCON_GCTL_TCON_ENABLE;
        // BEGIN sun4i_tcon_channel_set_status
        SUN4I_TCON1_CTL_REG |= SUN4I_TCON1_CTL_TCON_ENABLE;
        TCON_TOP_GATE_SRC_REG |= (1 << 20); // Enable [E]tcon-ch1 <&tcon_top CLK_TCON_TOP_TV0> aka tcon-top-tv0
        SUN50I_H616_CCU_TCON_TV0_CLK_REG |= BIT(31); // Parent: [E]tcon-tv0 <&ccu CLK_TCON_TV0> aka tcon-tv0 enable
        // END sun4i_tcon_channel_set_status
    // END sun4i_tcon_set_status - enable

    /* START dw_hdmi_bridge_atomic_enable */
        /* START dw_hdmi_update_power */
            /* START dw_hdmi_poweron */
                /* START dw_hdmi_setup */ /// TODO GET THIS IMPLEMENTED
                    // hdmi_disable_overflow_interrupts
                    writeb(HDMI_IH_MUTE_FC_STAT2_OVERFLOW_MASK, HDMI_IH_MUTE_FC_STAT2);
                	/* HDMI Initialization Step B.1 */
                    /* START hdmi_av_composer() */
                        // clk = 32 MHz
                        // mtmdsclock = 32 MHz
                        // TODO Calc:
                        uint8_t inv_val = 0x10;
                        writeb(inv_val, HDMI_FC_INVIDCONF);
                        /* Set up horizontal active pixel width */
                        uint32_t hdisplay = 800;
                        writeb(hdisplay >> 8, HDMI_FC_INHACTV1);
                        writeb(hdisplay, HDMI_FC_INHACTV0);
                        /* Set up vertical active lines */
                        uint32_t vdisplay = 480;
                        writeb(vdisplay >> 8, HDMI_FC_INVACTV1);
                        writeb(vdisplay, HDMI_FC_INVACTV0);
                        /* Set up horizontal blanking pixel region width */
                        uint32_t hblank = 128;
                        writeb(hblank >> 8, HDMI_FC_INHBLANK1);
                        writeb(hblank, HDMI_FC_INHBLANK0);
                        /* Set up vertical blanking pixel region width */
                        uint32_t vblank = 45;
                        writeb(vblank, HDMI_FC_INVBLANK);
                        /* Set up HSYNC active edge delay width (in pixel clks) */
                        uint32_t h_de_hs = 40;
                        writeb(h_de_hs >> 8, HDMI_FC_HSYNCINDELAY1);
                        writeb(h_de_hs, HDMI_FC_HSYNCINDELAY0);
                        /* Set up VSYNC active edge delay (in lines) */
                        uint32_t v_de_vs = 13;
                        writeb(v_de_vs, HDMI_FC_VSYNCINDELAY);
                        /* Set up HSYNC active pulse width (in pixel clks) */
                        uint32_t hsync_len = 48;
                        writeb(hsync_len >> 8, HDMI_FC_HSYNCINWIDTH1);
                        writeb(hsync_len, HDMI_FC_HSYNCINWIDTH0);
                        /* Set up VSYNC active edge delay (in lines) */
                        uint32_t vsync_len = 3;
                        writeb(vsync_len, HDMI_FC_VSYNCINWIDTH);
                    /* END hdmi_av_composer() */
                    /* HDMI Initializateion Step B.2 */
                    /* START dw_hdmi_phy_init() */
                        /* HDMI Phy spec says to do the phy initialization sequence twice */
                        for (int xxx=0; xxx < 2; xxx++) {
                        // dw_hdmi_phy_sel_data_en_pol()
                        writeb_mask(1, HDMI_PHY_CONF0, HDMI_PHY_CONF0_SELDATAENPOL_OFFSET, HDMI_PHY_CONF0_SELDATAENPOL_MASK);
                        // dw_hdmi_phy_sel_interface_control()
                        writeb_mask(0, HDMI_PHY_CONF0, HDMI_PHY_CONF0_SELDIPIF_OFFSET, HDMI_PHY_CONF0_SELDIPIF_MASK);
                        // hdmi_phy_configure()
                            // dw_hdmi_phy_power_off()
                            writeb_mask(0, HDMI_PHY_CONF0, HDMI_PHY_CONF0_GEN2_TXPWRON_OFFSET, HDMI_PHY_CONF0_GEN2_TXPWRON_MASK);
                            /*
                            * Wait for TX_PHY_LOCK to be deasserted to indicate that the PHY went
                            * to low power mode.
                            */
                            for (i = 0; i < 5; ++i) {
                                printf("LOOP1\n");
                                bval = readb(HDMI_PHY_STAT0);
                                if (!(bval & HDMI_PHY_TX_PHY_LOCK))
                                    break;
                                udelay(2000);
                            }
                            writeb_mask(1, HDMI_PHY_CONF0, HDMI_PHY_CONF0_GEN2_PDDQ_OFFSET, HDMI_PHY_CONF0_GEN2_PDDQ_MASK);
                            // SKIP dw_hdmi_set_high_tmds_clock_ratio() don't care
                            /* Leave low power consumption mode by asserting SVSRET. */
                            writeb_mask(1, HDMI_PHY_CONF0, HDMI_PHY_CONF0_SVSRET_OFFSET, HDMI_PHY_CONF0_SVSRET_MASK);
                            /* PHY reset. The reset signal is active high on Gen2 PHYs. */
                            writeb(HDMI_MC_PHYRSTZ_PHYRSTZ, HDMI_MC_PHYRSTZ);
                            writeb(0, HDMI_MC_PHYRSTZ);
                            writeb(HDMI_MC_HEACPHY_RST_ASSERT, HDMI_MC_HEACPHY_RST);

                            // dw_hdmi_phy_i2c_set_addr
                            writeb_mask(1 << HDMI_PHY_TST0_TSTCLR_OFFSET, HDMI_PHY_TST0, HDMI_PHY_TST0_TSTCLK_OFFSET, HDMI_PHY_TST0_TSTCLR_MASK);
                            writeb(HDMI_PHY_I2CM_SLAVE_ADDR_PHY_GEN2, HDMI_PHY_I2CM_SLAVE_ADDR);
                            writeb_mask(0, HDMI_PHY_TST0, HDMI_PHY_TST0_TSTCLK_OFFSET, HDMI_PHY_TST0_TSTCLR_MASK);

                            /* Write to the PHY as configured by the platform */
                                // hdmi_phy_configure_dwc_hdmi_3d_tx - TODO these need to be set based on the pixelclock
                                // mpixelclock = 32 MHz;
                                dw_hdmi_phy_i2c_write(0x0072, HDMI_3D_TX_PHY_CPCE_CTRL);
                                dw_hdmi_phy_i2c_write(0x0003,
                                            HDMI_3D_TX_PHY_GMPCTRL);
                                dw_hdmi_phy_i2c_write(0x0013,
                                            HDMI_3D_TX_PHY_CURRCTRL);

                                dw_hdmi_phy_i2c_write(0, HDMI_3D_TX_PHY_PLLPHBYCTRL);
                                dw_hdmi_phy_i2c_write(HDMI_3D_TX_PHY_MSM_CTRL_CKO_SEL_FB_CLK,
                                            HDMI_3D_TX_PHY_MSM_CTRL);

                                dw_hdmi_phy_i2c_write(0x0004, HDMI_3D_TX_PHY_TXTERM);
                                dw_hdmi_phy_i2c_write(0x8019,
                                            HDMI_3D_TX_PHY_CKSYMTXCTRL);
                                dw_hdmi_phy_i2c_write(0x0290,
                                            HDMI_3D_TX_PHY_VLEVCTRL);
                                /* Override and disable clock termination. */
                                dw_hdmi_phy_i2c_write(HDMI_3D_TX_PHY_CKCALCTRL_OVERRIDE,
                                            HDMI_3D_TX_PHY_CKCALCTRL);
                            // dw_hdmi_phy_power_on
                            writeb_mask(1, HDMI_PHY_CONF0, HDMI_PHY_CONF0_GEN2_TXPWRON_OFFSET, HDMI_PHY_CONF0_GEN2_TXPWRON_MASK);
                            writeb_mask(0, HDMI_PHY_CONF0, HDMI_PHY_CONF0_GEN2_PDDQ_OFFSET, HDMI_PHY_CONF0_GEN2_PDDQ_MASK);
                            /* Wait for PHY PLL lock */
                            for (i = 0; i < 5; ++i) {
                                bval = readb(HDMI_PHY_STAT0) & HDMI_PHY_TX_PHY_LOCK;
                                if (bval){
                                    break;
                                }
                                printf("LOOP2 %x\n", bval);
                                udelay(2000);
                            }
                            // udelay(2000);
                        }
                    /* END dw_hdmi_phy_init() */  // Screen does deep black here.

	                /* HDMI Initialization Step B.3 */
                    /* START dw_hdmi_enable_video_path */
                        /* control period minimum duration */
                        writeb(12, HDMI_FC_CTRLDUR);
                        writeb(32, HDMI_FC_EXCTRLDUR);
                        writeb(1, HDMI_FC_EXCTRLSPAC);

                        /* Set to fill TMDS data channels */
                        writeb(0x0B, HDMI_FC_CH0PREAM);
                        writeb(0x16, HDMI_FC_CH1PREAM);
                        writeb(0x21, HDMI_FC_CH2PREAM);

                        /* Enable pixel clock and tmds data path */
                        uint8_t mc_clkdis = 0x7f;
                        mc_clkdis |= HDMI_MC_CLKDIS_HDCPCLK_DISABLE |
                                HDMI_MC_CLKDIS_CSCCLK_DISABLE |
                                HDMI_MC_CLKDIS_AUDCLK_DISABLE |
                                HDMI_MC_CLKDIS_PREPCLK_DISABLE |
                                HDMI_MC_CLKDIS_TMDSCLK_DISABLE;
                        mc_clkdis &= ~HDMI_MC_CLKDIS_PIXELCLK_DISABLE;
                        writeb(mc_clkdis, HDMI_MC_CLKDIS); // 0x7e <---- screen turns off here (i.e. signal lost)

                        mc_clkdis &= ~HDMI_MC_CLKDIS_TMDSCLK_DISABLE;
                        writeb(mc_clkdis, HDMI_MC_CLKDIS); // 0x7c <--- screen is a deep red here.

                        /* Enable csc (colour space conversion) path */
                        if (/*is_csc_needed(hdmi)*/ 0) {
                            // hdmi->mc_clkdis &= ~HDMI_MC_CLKDIS_CSCCLK_DISABLE;
                            // hdmi_writeb(hdmi, hdmi->mc_clkdis, HDMI_MC_CLKDIS);

                            // hdmi_writeb(hdmi, HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_IN_PATH,
                            //         HDMI_MC_FLOWCTRL);
                        } else {
                            mc_clkdis |= HDMI_MC_CLKDIS_CSCCLK_DISABLE;
                            // writeb(mc_clkdis, HDMI_MC_CLKDIS);

                            // writeb(HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS,
                            //         HDMI_MC_FLOWCTRL);
                        }
                    /* END dw_hdmi_enable_video_path */

                    /* SKIP HDMI Initialization Step E - Configure audio */

                    /* not for DVI mode */ // TODO: Why does this display appear as DVI?
                    if (0) {
                        // dev_dbg(hdmi->dev, "%s HDMI mode\n", __func__);

                        /* HDMI Initialization Step F - Configure AVI InfoFrame */
                        // hdmi_config_AVI(hdmi, connector, mode);
                        // hdmi_config_vendor_specific_infoframe(hdmi, connector, mode);
                        // hdmi_config_drm_infoframe(hdmi, connector);
                    }

                    /* Final trailing calls: */
                    // START hdmi_video_packetize(hdmi);
                        writeb(64, HDMI_VP_PR_CD);
                        /* HDMI1.4b specification section 6.5.3:
                        * Source shall only send GCPs with non-zero CD to sinks
                        * that indicate support for Deep Color.
                        * GCP only transmit CD and do not handle AVMUTE, PP norDefault_Phase (yet).
                        * Disable Auto GCP when 24-bit color for sinks that not support Deep Color.
                        */
                        bval = readb(HDMI_FC_DATAUTO3);
	                    uint8_t clear_gcp_auto = 1;
                        uint8_t vp_conf;
                        unsigned int output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS;
	                    unsigned int remap_size = HDMI_VP_REMAP_YCC422_16bit;
                        if (clear_gcp_auto == 1)
                            val &= ~HDMI_FC_DATAUTO3_GCP_AUTO;
                        else
                            val |= HDMI_FC_DATAUTO3_GCP_AUTO;
                        writeb(val, HDMI_FC_DATAUTO3);

                        writeb_mask(HDMI_VP_STUFF_PR_STUFFING_STUFFING_MODE, HDMI_VP_STUFF,
                            0, HDMI_VP_STUFF_PR_STUFFING_MASK);

                        /* Data from pixel repeater block */
                        // if (hdmi_data->pix_repet_factor > 1) {
                        //     vp_conf = HDMI_VP_CONF_PR_EN_ENABLE |
                        //         HDMI_VP_CONF_BYPASS_SELECT_PIX_REPEATER;
                        // } else { /* data from packetizer block */
                            vp_conf = HDMI_VP_CONF_PR_EN_DISABLE |
                                HDMI_VP_CONF_BYPASS_SELECT_VID_PACKETIZER;
                        // }

                        writeb_mask(vp_conf, HDMI_VP_CONF, 0,
                            HDMI_VP_CONF_PR_EN_MASK |
                            HDMI_VP_CONF_BYPASS_SELECT_MASK);

                        writeb_mask(1 << HDMI_VP_STUFF_IDEFAULT_PHASE_OFFSET, HDMI_VP_STUFF, 0,
                            HDMI_VP_STUFF_IDEFAULT_PHASE_MASK);

                        writeb(remap_size, HDMI_VP_REMAP);

                        // if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_PP) {
                        //     vp_conf = HDMI_VP_CONF_BYPASS_EN_DISABLE |
                        //         HDMI_VP_CONF_PP_EN_ENABLE |
                        //         HDMI_VP_CONF_YCC422_EN_DISABLE;
                        // } else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_YCC422) {
                        //     vp_conf = HDMI_VP_CONF_BYPASS_EN_DISABLE |
                        //         HDMI_VP_CONF_PP_EN_DISABLE |
                        //         HDMI_VP_CONF_YCC422_EN_ENABLE;
                        // } else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS) {
                            vp_conf = HDMI_VP_CONF_BYPASS_EN_ENABLE |
                                HDMI_VP_CONF_PP_EN_DISABLE |
                                HDMI_VP_CONF_YCC422_EN_DISABLE;
                        // } else {
                        //     return;
                        // }

                        writeb_mask(vp_conf, HDMI_VP_CONF, 0,
                            HDMI_VP_CONF_BYPASS_EN_MASK | HDMI_VP_CONF_PP_EN_ENMASK |
                            HDMI_VP_CONF_YCC422_EN_MASK);

                        writeb_mask(HDMI_VP_STUFF_PP_STUFFING_STUFFING_MODE |
                                HDMI_VP_STUFF_YCC422_STUFFING_STUFFING_MODE, HDMI_VP_STUFF, 0,
                            HDMI_VP_STUFF_PP_STUFFING_MASK |
                            HDMI_VP_STUFF_YCC422_STUFFING_MASK);

                        writeb_mask(output_select, HDMI_VP_CONF, 0,
                            HDMI_VP_CONF_OUTPUT_SELECTOR_MASK);
                    // END hdmi_video_packetize(hdmi);

                    /* START hdmi_video_csc(hdmi); */
                        /* Configure the CSC registers */
                        writeb(0, HDMI_CSC_CFG);
                        writeb_mask(0, HDMI_CSC_SCALE, 0, HDMI_CSC_SCALE_CSC_COLORDE_PTH_MASK);
                        // TODO dw_hdmi_update_csc_coeffs(hdmi);
                    /* END hdmi_video_csc(hdmi); */
                    /* START hdmi_video_sample(hdmi); */
                        bval = HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE |
                            ((1 << HDMI_TX_INVID0_VIDEO_MAPPING_OFFSET) &
                            HDMI_TX_INVID0_VIDEO_MAPPING_MASK);
                        writeb(bval, HDMI_TX_INVID0);

                        /* Enable TX stuffing: When DE is inactive, fix the output data to 0 */
                        bval = HDMI_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE |
                            HDMI_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE |
                            HDMI_TX_INSTUFFING_GYDATA_STUFFING_ENABLE;
                        writeb(bval, HDMI_TX_INSTUFFING);
                        writeb(0x0, HDMI_TX_GYDATA0);
                        writeb(0x0, HDMI_TX_GYDATA1);
                        writeb(0x0, HDMI_TX_RCRDATA0);
                        writeb(0x0, HDMI_TX_RCRDATA1);
                        writeb(0x0, HDMI_TX_BCBDATA0);
                        writeb(0x0, HDMI_TX_BCBDATA1);
                    /* END hdmi_video_sample(hdmi); */
                    /* START hdmi_tx_hdcp_config(hdmi); */
                            uint8_t de = HDMI_A_VIDPOLCFG_DATAENPOL_ACTIVE_HIGH;
                            /* disable rx detect */
                            writeb_mask(HDMI_A_HDCPCFG0_RXDETECT_DISABLE, HDMI_A_HDCPCFG0, 0,
                                HDMI_A_HDCPCFG0_RXDETECT_MASK);

                            writeb_mask(de, HDMI_A_VIDPOLCFG, 0, HDMI_A_VIDPOLCFG_DATAENPOL_MASK);

                            writeb_mask(HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_DISABLE, HDMI_A_HDCPCFG1, 0,
                                HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_MASK);
                    /* END hdmi_tx_hdcp_config(hdmi); */
                    /* START dw_hdmi_clear_overflow(hdmi); */
                        writeb((uint8_t)~HDMI_MC_SWRSTZ_TMDSSWRST_REQ, HDMI_MC_SWRSTZ);
                        bval = readb(HDMI_FC_INVIDCONF);
                        writeb(bval, HDMI_FC_INVIDCONF);
                    /* END dw_hdmi_clear_overflow(hdmi); */

                /* END dw_hdmi_setup */
            /* END dw_hdmi_poweron */
        /* END dw_hdmi_update_power */
        /* SKIP dw_hdmi_update_phy_mask */ // <- With breakpoint here, screen has gone red. Which is good test for baremetal, as I set it as background colour.
    /* END dw_hdmi_bridge_atomic_enable */

/* START sun8i_mixer_init() */
	    /* Enable the mixer */
        SUN8I_MIXER_GLOBAL_CTL = SUN8I_MIXER_GLOBAL_CTL_RT_EN;
        SUN50I_MIXER_GLOBAL_CLK = 1;
	    /* Set background color to black */
        SUN8I_MIXER_BLEND_BKCOLOR = 0xffc0c0c0; // grey
        /*
        * Set fill color of bottom plane to black. Generally not needed
        * except when VI plane is at bottom (zpos = 0) and enabled.
        */
        SUN8I_MIXER_BLEND_PIPE_CTL = SUN8I_MIXER_BLEND_PIPE_CTL_FC_EN(0);
        SUN8I_MIXER_BLEND_ATTR_FCOLOR(0) = 0xffcc00cc; // magenta

        plane_cnt = 1; //1 /* vi_num */ + 3 /* ui_num */;
        for (i = 0; i < plane_cnt; i++) {
            SUN8I_MIXER_BLEND_MODE(i) = SUN8I_MIXER_BLEND_MODE_DEF;
        }
        SUN8I_MIXER_BLEND_PIPE_CTL &= ~SUN8I_MIXER_BLEND_PIPE_CTL_EN_MSK;
/* END sun8i_mixer_init() */

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
    SUN8I_MIXER_BLEND_OUTCTL = val;
    SUN8I_MIXER_BLEND_BKCOLOR = 0xffff0000; // red
    SUN8I_MIXER_BLEND_ATTR_FCOLOR(0) = 0xffff8000; // orange
    
    // HHMMMM = 0xe4;
    // *((uint32_t *)0x01008040) = 0x7;
    // *((uint32_t *)0x01008044) = 0x7;
    // *((uint32_t *)0x01008048) = 0x7;
    // *((uint32_t *)0x0100804c) = 0x7;
    
    // *((uint32_t *)0x01008050) = 0x7;
    // *((uint32_t *)0x01008054) = 0x7;
    // *((uint32_t *)0x01008058) = 0x7;
    // *((uint32_t *)0x0100805c) = 0x7;
    
    // *((uint32_t *)0x01008060) = 0x7;
    // *((uint32_t *)0x01008064) = 0x7;
    // *((uint32_t *)0x01008068) = 0x7;
    // *((uint32_t *)0x0100806c) = 0x7;
    
    // *((uint32_t *)0x01008070) = 0x7;
    // *((uint32_t *)0x01008074) = 0x7;
    // *((uint32_t *)0x01008078) = 0x7;
    // *((uint32_t *)0x0100807c) = 0x7;
    
    // *((uint32_t *)0x01008080) = 0x707;
    // *((uint32_t *)0x01008084) = 0x7;
    
        // // START sun50i_fmt_setup()
        // SUN50I_FMT_CTRL = 0;
        // SUN50I_FMT_SIZE = SUN8I_MIXER_SIZE(800, 480);
        // SUN50I_FMT_SWAP = 0;
        // SUN50I_FMT_DEPTH = 0; // assume bit10 false
        // SUN50I_FMT_FORMAT = 0; // Assume default colourspace
        // SUN50I_FMT_COEF = 0;
        // SUN50I_FMT_LMT_Y = (4095 << 16) | 0;
        // SUN50I_FMT_LMT_C0 = (4095 << 16) | 0;
        // SUN50I_FMT_LMT_C1 = (4095 << 16) | 0;
        // SUN50I_FMT_CTRL = 1;
        // // END sun50i_fmt_setup()
// FIN sunxi_engine_mode_set()/sun8i_mixer_mode_set()

/* BEGIN sun8i_ui_layer_update_coord */
            // channel/layer = 1
            // overlay = 0
            SUN8I_MIXER_CHAN_UI_LAYER_SIZE(0) = SUN8I_MIXER_SIZE(800, 480); // insize
            udelay(200);
            SUN8I_MIXER_CHAN_UI_OVL_SIZE = SUN8I_MIXER_SIZE(800, 480); // insize
            udelay(200);
            // No scaling required
                // sun8i_vi_scaler_disable(mixer, 1)
	            // SUN8I_SCALER_VSU_CTRL = 0;
            SUN8I_MIXER_BLEND_ATTR_COORD(0 /*zpos*/) = SUN8I_MIXER_COORD(0, 0);
            udelay(200);
            SUN8I_MIXER_BLEND_ATTR_INSIZE(0 /*zpos*/) = SUN8I_MIXER_SIZE(800, 480); // outsize  // just this should turn display from orange to black (or red?). It turns red.
            udelay(200);
            // return;
/* END sun8i_ui_layer_update_coord */

/* START sun8i_ui_layer_update_formats */
            // FOURCC = XR24
            // #define DRM_FORMAT_XRGB8888	fourcc_code('X', 'R', '2', '4') /* [31:0] x:R:G:B 8:8:8:8 little endian */
            // DE2 fmt = SUN8I_MIXER_FBFMT_XRGB8888 = 4
            uint32_t hw_fmt = 0x4;
            val = (SUN8I_MIXER_CHAN_UI_LAYER_ATTR(0) &= ~SUN8I_MIXER_CHAN_UI_LAYER_ATTR_FBFMT_MASK) |
                ((4 << SUN8I_MIXER_CHAN_UI_LAYER_ATTR_FBFMT_OFFSET) & SUN8I_MIXER_CHAN_UI_LAYER_ATTR_FBFMT_MASK);
            SUN8I_MIXER_CHAN_UI_LAYER_ATTR(0) = val;
/* END sun8i_ui_layer_update_formats */

/* START sun8i_ui_layer_update_buffer!!! */
            // bpp = 4 (bytes per pixel)
            // Pitches = 3200 (800w x 4)
            SUN8I_MIXER_CHAN_UI_LAYER_PITCH(0) = 800*4; // 800 * 4?
            // Address is just the start of the gem->dma_addr, no offset required
            SUN8I_MIXER_CHAN_UI_LAYER_TOP_LADDR(0) = (uint32_t)(uint64_t)framebufferz;
            // udelay(2000);
/* END sun8i_ui_layer_update_buffer!!! */

/* BEGIN sun8i_mixer_commit */
                    // We only enable channel=1, which is UI0
                    // ch_base: c1000 overlay: 0 channel: 1 zpos: 0
                    // Layer enable
                    SUN8I_MIXER_CHAN_UI_LAYER_ATTR(0) |= SUN8I_MIXER_CHAN_UI_LAYER_ATTR_EN;
			        /* Route layer to pipe based on zpos */
                    uint32_t route = 1 << SUN8I_MIXER_BLEND_ROUTE_PIPE_SHIFT(0 /*zpos*/); // i.e. place ch1 at zpos=0
                    uint32_t pipe_en = SUN8I_MIXER_BLEND_PIPE_CTL_EN(0/*zpos*/); // Enable zpos=0
                    SUN8I_MIXER_BLEND_ROUTE = route;
                    SUN8I_MIXER_BLEND_PIPE_CTL = (pipe_en | SUN8I_MIXER_BLEND_PIPE_CTL_FC_EN(0)); // Orange again. Should be black (i.e. on framebuffer!)
                    SUN50I_MIXER_GLOBAL_DBUFF = SUN8I_MIXER_GLOBAL_DBUFF_ENABLE;
/* END sun8i_mixer_commit */ // <- This should replace red background with the framebuffer

// SUN8I_MIXER_GLOBAL_CTL2 = 0x1d;
SUN8I_MIXER_BLEND_ATTR_FCOLOR(0) = 0xffff8000;
SUN8I_MIXER_GLOBAL_STATUS = ~BIT(8);
    SUN8I_MIXER_BLEND_OUTCTL &= ~SUN8I_MIXER_BLEND_OUTCTL_INTERLACED;

	uint64_t ptr;
	uint32_t val1, val2, val3, val4;

	printf("======= CLK ========\n");
	for (ptr = 0x1008000; ptr < 0x1008100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}

	printf("======= TOP ========\n");
	for (ptr = 0x1008100; ptr < 0x1008140; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}

	printf("======= R_CCU ========\n");
	for (ptr = 0x7010000; ptr < 0x7010210; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}

	printf("======= RTC ========\n");
	for (ptr = 0x7000000; ptr < 0x7000400; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}

	printf("======= CCU ========\n");
	for (ptr = 0x3001000; ptr < 0x3002000; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}







	printf("======= BLENDER ========\n");
	for (ptr = 0x1281000; ptr < 0x1281100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}

	printf("======= FMT ========\n");
	for (ptr = 0x1285000; ptr < 0x1285100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}

	printf("======= ch0 ========\n");
	for (ptr = 0x1101000; ptr < 0x1101100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}

	printf("======= ch1 ========\n");
	for (ptr = 0x1121000; ptr < 0x1121100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}
	printf("======= ch2 ========\n");
	for (ptr = 0x1141000; ptr < 0x1141100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}
	printf("======= ch3 ========\n");
	for (ptr = 0x1161000; ptr < 0x1161100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}
	printf("======= ch4 ========\n");
	for (ptr = 0x1181000; ptr < 0x1181100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}
	printf("======= ch5 ========\n");
	for (ptr = 0x11a1000; ptr < 0x11a1100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}
	printf("======= ch6 ========\n");
	for (ptr = 0x11c1000; ptr < 0x11c1100; ptr += 16)
	{
        val1 = *((uint32_t *)(ptr));
        val2 = *((uint32_t *)(ptr+4));
        val3 = *((uint32_t *)(ptr+8));
        val4 = *((uint32_t *)(ptr+12));
        printf("%08x  %08x %08x  %08x %08x\n", ptr, val1, val2, val3, val4);
	}

    printf("0x%x\n", *(uint32_t*)0x4031aaa0);

    for (uint32_t x = 0; x < 1000*1000; x++) {
        framebufferz[x] = 0xff00ffff;
    }
}

/*  This function attempts to get graphics working.
*/
void display_init()
{
    for (uint32_t x = 0; x < 1000*1000; x++) {
        framebufferz[x] = 0xff00ff00;
    }
    clocks_init();
    reclaim_sram_c_and_erase();
    display_configure();
    printf("SUN8I_MIXER_CHAN_UI_LAYER_TOP_LADDR(0) %x\n", SUN8I_MIXER_CHAN_UI_LAYER_TOP_LADDR(0));
}
