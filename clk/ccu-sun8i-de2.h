/*  This isn't really a clock file. Improve how this is stored later.
*/

#ifndef CCU_SUN8I_DE2_H
#define CCU_SUN8I_DE2_H

// allwinner,sun50i-a64-de2
// bus@1000000
#define DE_BASE 0x1000000

// allwinner,sun50i-h616-de33-clk
// display_clocks: clock@8000
#define DE33_CLK_BASE              (DE_BASE + 0x8000)
#define DE33_CLK_MIXER0           *(volatile uint32_t *)(DE33_CLK_BASE + 0x0000)
#define DE33_CLK_BUS_MIXER0       *(volatile uint32_t *)(DE33_CLK_BASE + 0x0004)
#define DE33_CLK_RST_MIXER0       *(volatile uint32_t *)(DE33_CLK_BASE + 0x0008)
#define DE_RANDOM_1_REG           *(volatile uint32_t *)(DE33_CLK_BASE + 0x0024)
#define DE_RANDOM_1_VAL           0x0
#define DE_RANDOM_2_REG           *(volatile uint32_t *)(DE33_CLK_BASE + 0x0028)
#define DE_RANDOM_2_VAL           0x0000a980

// allwinner,sun50i-h616-de33-mixer-0
// mixer0: mixer@100000
// Mixer is awkwardly split into three sections
// - 0x100000 BASE
// - 0x008100 TOP
// - 0x280000 DISP

#define SUN8I_MIXER_BLEND_OUTCTL_INTERLACED	BIT(1)

#define DE33_MIXER_BASE              (DE_BASE + 0x100000)

#define DE33_MIXER_ENG_REGS_BASE     (DE_BASE + 0x100000)

#define DE33_MIXER_TOP_REGS_BASE     (DE_BASE + 0x8100)
#define SUN8I_MIXER_GLOBAL_CTL      *(volatile uint32_t *)(DE33_MIXER_TOP_REGS_BASE + 0x0000)
#define SUN50I_MIXER_GLOBAL_SIZE		*(volatile uint32_t *)(DE33_MIXER_TOP_REGS_BASE + 0x8)
#define SUN8I_MIXER_SIZE(w, h)			(((h) - 1) << 16 | ((w) - 1))
#define SUN8I_MIXER_GLOBAL_CTL_RT_EN  BIT(0)
#define SUN50I_MIXER_GLOBAL_CLK      *(volatile uint32_t *)(DE33_MIXER_TOP_REGS_BASE + 0x000c)

#define DE33_MIXER_DISP_REGS_BASE    (DE_BASE + 0x280000)
#define DE2_BLD_BASE 0x1000
#define SUN50I_FMT_DE33 0x5000
#define SUN8I_MIXER_BLEND_PIPE_CTL          *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x0)
#define SUN8I_MIXER_BLEND_ATTR_FCOLOR(x)	*(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x04 + 0x10 * (x))
#define SUN8I_MIXER_BLEND_BKCOLOR	        *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x88)
#define SUN8I_MIXER_BLEND_OUTSIZE		    *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x8c)
#define SUN8I_MIXER_BLEND_MODE(x)	    	*(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0x90 + 0x04 * (x))
#define SUN8I_MIXER_BLEND_OUTCTL	    	*(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + DE2_BLD_BASE + 0xfc)


#define SUN50I_FMT_CTRL                     *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x00)
#define SUN50I_FMT_SIZE                     *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x04)
#define SUN50I_FMT_SWAP                     *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x08)
#define SUN50I_FMT_DEPTH                    *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x0c)
#define SUN50I_FMT_FORMAT                   *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x10)
#define SUN50I_FMT_COEF                     *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x14)
#define SUN50I_FMT_LMT_Y                    *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x20)
#define SUN50I_FMT_LMT_C0                   *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x24)
#define SUN50I_FMT_LMT_C1                   *(volatile uint32_t *)(DE33_MIXER_DISP_REGS_BASE + SUN50I_FMT_DE33 + 0x28)

#define SUN8I_MIXER_BLEND_MODE_DEF		    0x03010301
#define SUN8I_MIXER_BLEND_PIPE_CTL_EN_MSK	GENMASK(12, 8)
#define SUN8I_MIXER_BLEND_PIPE_CTL_FC_EN(pipe)	BIT(pipe)
/* colors are always in AARRGGBB format */
// TODO make black! 0xff00000000
#define SUN8I_MIXER_BLEND_COLOR_BLACK		0xffff0000

#endif
