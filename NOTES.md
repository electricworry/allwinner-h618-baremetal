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
