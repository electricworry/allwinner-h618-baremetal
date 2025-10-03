// allwinner,sun50i-a64-de2
// bus@1000000
#define DE_BASE 0x1000000

// allwinner,sun50i-h616-de33-clk
// display_clocks: clock@8000
#define DE33_CLK_BASE              (DE_BASE + 0x8000)
#define DE33_CLK_RST_MIXER0       *(volatile uint32_t *)(DE33_CLK_BASE + 0x0008)
#define DE33_CLK_BUS_MIXER0       *(volatile uint32_t *)(DE33_CLK_BASE + 0x0004)
#define DE33_CLK_MIXER0           *(volatile uint32_t *)(DE33_CLK_BASE + 0x0000)


// From drivers/clk/sunxi-ng/ccu-sun8i-de2.c

#define DE_RANDOM_1_REG       *(volatile uint32_t *)(DE33_CLK_BASE + 0x0024)
#define DE_RANDOM_1_VAL       0x0
#define DE_RANDOM_2_REG       *(volatile uint32_t *)(DE33_CLK_BASE + 0x0028)
#define DE_RANDOM_2_VAL       0x0000a980


// allwinner,sun50i-h616-de33-mixer-0
// mixer0: mixer@100000
#define DE33_MIXER_BASE              (DE_BASE + 0x100000)

#define DE33_MIXER_ENG_REGS_BASE     (DE_BASE + 0x100000)

#define DE33_MIXER_TOP_REGS_BASE     (DE_BASE + 0x8100)
#define SUN8I_MIXER_GLOBAL_CTL      *(volatile uint32_t *)(DE33_MIXER_TOP_REGS_BASE + 0x0000)
#define SUN8I_MIXER_GLOBAL_CTL_RT_EN		BIT(0)
#define SUN50I_MIXER_GLOBAL_CLK      *(volatile uint32_t *)(DE33_MIXER_TOP_REGS_BASE + 0x0000)

#define DE33_MIXER_DISP_REGS_BASE    (DE_BASE + 0x280000)
#define DE2_BLD_BASE 0x1000
#define SUN8I_MIXER_BLEND_PIPE_CTL   	*(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x0)
#define SUN8I_MIXER_BLEND_BKCOLOR		*(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x88)
#define SUN8I_MIXER_BLEND_ATTR_FCOLOR(x)	*(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x4 + 0x10 * (x))
#define SUN8I_MIXER_BLEND_MODE(x)	    	*(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x90 + 0x04 * (x))

#define SUN8I_MIXER_BLEND_MODE_DEF		0x03010301
#define SUN8I_MIXER_BLEND_PIPE_CTL_EN_MSK	GENMASK(12, 8)
#define SUN8I_MIXER_BLEND_PIPE_CTL_FC_EN(pipe)	BIT(pipe)
/* colors are always in AARRGGBB format */
// TODO make black! 0xff00000000
#define SUN8I_MIXER_BLEND_COLOR_BLACK		0xffffffff
/* The following numbers are some still unknown magic numbers */

// allwinner,sun50i-h616-de33-clk
// ccu: clock@3001000
