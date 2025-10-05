// allwinner,sun50i-h616-ccu
// ccu: clock@3001000

#ifndef CCU_SUN50I_H616_H
#define CCU_SUN50I_H616_H

#define CCU_BASE 0x03001000

#define SUN50I_H616_CCU_PLL_CPUX_REG            *(volatile uint32_t *)(CCU_BASE + 0x0000)
#define SUN50I_H616_CCU_PLL_DDR0_REG            *(volatile uint32_t *)(CCU_BASE + 0x0010)
#define SUN50I_H616_CCU_PLL_DDR1_REG            *(volatile uint32_t *)(CCU_BASE + 0x0018)
#define SUN50I_H616_CCU_PLL_PERIPH0_REG         *(volatile uint32_t *)(CCU_BASE + 0x0020)
#define SUN50I_H616_CCU_PLL_PERIPH1_REG         *(volatile uint32_t *)(CCU_BASE + 0x0028)
#define SUN50I_H616_CCU_PLL_GPU_REG             *(volatile uint32_t *)(CCU_BASE + 0x0030)
#define SUN50I_H616_CCU_PLL_VIDEO0_REG          *(volatile uint32_t *)(CCU_BASE + 0x0040)
#define SUN50I_H616_CCU_PLL_VIDEO1_REG          *(volatile uint32_t *)(CCU_BASE + 0x0048)
#define SUN50I_H616_CCU_PLL_VIDEO2_REG          *(volatile uint32_t *)(CCU_BASE + 0x0050)
#define SUN50I_H616_CCU_PLL_VE_REG              *(volatile uint32_t *)(CCU_BASE + 0x0058)
#define SUN50I_H616_CCU_PLL_DE_REG              *(volatile uint32_t *)(CCU_BASE + 0x0060)
#define SUN50I_H616_CCU_PLL_AUDIO_REG           *(volatile uint32_t *)(CCU_BASE + 0x0078)
#define SUN50I_H616_CCU_MBUS_CFG_REG            *(volatile uint32_t *)(CCU_BASE + 0x0540)
#define SUN50I_H616_CCU_DE_CLK_REG              *(volatile uint32_t *)(CCU_BASE + 0x0600)
#define SUN50I_H616_CCU_DE_BGR_REG              *(volatile uint32_t *)(CCU_BASE + 0x060c)
#define SUN50I_H616_CCU_GPU_CLK1_REG            *(volatile uint32_t *)(CCU_BASE + 0x0674)
#define SUN50I_H616_CCU_DMA_BGR_REG             *(volatile uint32_t *)(CCU_BASE + 0x070c)
#define SUN50I_H616_CCU_DRAM_CLK_REG            *(volatile uint32_t *)(CCU_BASE + 0x0800)
#define SUN50I_H616_CCU_MBUS_MAT_CLK_GATING_REG *(volatile uint32_t *)(CCU_BASE + 0x0804)
#define SUN50I_H616_CCU_DRAM_BGR_REG            *(volatile uint32_t *)(CCU_BASE + 0x080c)
#define SUN50I_H616_CCU_USB0_CLK_REG            *(volatile uint32_t *)(CCU_BASE + 0x0a70)
#define SUN50I_H616_CCU_USB1_CLK_REG            *(volatile uint32_t *)(CCU_BASE + 0x0a74)
#define SUN50I_H616_CCU_USB2_CLK_REG            *(volatile uint32_t *)(CCU_BASE + 0x0a78)
#define SUN50I_H616_CCU_USB3_CLK_REG            *(volatile uint32_t *)(CCU_BASE + 0x0a7c)
#define SUN50I_H616_CCU_HDMI0_CLK_REG           *(volatile uint32_t *)(CCU_BASE + 0x0b00)
#define SUN50I_H616_CCU_HDMI0_SLOW_CLK_REG      *(volatile uint32_t *)(CCU_BASE + 0x0b04)
#define SUN50I_H616_CCU_HDMI_CEC_CLK_REG        *(volatile uint32_t *)(CCU_BASE + 0x0b10)
#define SUN50I_H616_CCU_HDMI_BGR_REG            *(volatile uint32_t *)(CCU_BASE + 0x0b1c)
#define SUN50I_H616_CCU_DISPLAY_IF_TOP_BGR_REG  *(volatile uint32_t *)(CCU_BASE + 0x0b5C)
#define SUN50I_H616_CCU_TVE0_CLK_REG            *(volatile uint32_t *)(CCU_BASE + 0x0b80)
#define SUN50I_H616_CCU_TCON_TV_BGR_REG         *(volatile uint32_t *)(CCU_BASE + 0x0b9c)

#endif
