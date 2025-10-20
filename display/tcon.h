// allwinner,sun8i-r40-tcon-tv

#ifndef TCON_H
#define TCON_H

#include "../util.h"

// From TCON_TV

#define SUN4I_TCON_BASE         0x06515000
#define SUN4I_TCON_GCTL_REG       *(volatile uint32_t *)(SUN4I_TCON_BASE + 0x00)
#define SUN4I_TCON_GCTL_PAD_SEL				BIT(1)
#define SUN4I_TCON_GCTL_TCON_ENABLE			BIT(31)
#define SUN4I_TCON_GCTL_IOMAP_MASK			BIT(0)
#define SUN4I_TCON_GCTL_IOMAP_TCON1			(1 << 0)
#define SUN4I_TCON_GINT0_REG       *(volatile uint32_t *)(SUN4I_TCON_BASE + 0x04)
#define SUN4I_TCON_GINT1_REG       *(volatile uint32_t *)(SUN4I_TCON_BASE + 0x08)
#define SUN4I_TCON0_IO_TRI_REG       *(volatile uint32_t *)(SUN4I_TCON_BASE + 0x8c)
#define SUN4I_TCON1_IO_TRI_REG       *(volatile uint32_t *)(SUN4I_TCON_BASE + 0xf4)
#define SUN4I_TCON1_CTL_REG			*(volatile uint32_t *)(SUN4I_TCON_BASE + 0x90)
#define SUN4I_TCON1_CTL_TCON_ENABLE			BIT(31)
#define SUN4I_TCON1_CTL_CLK_DELAY_MASK			GENMASK(8, 4)
#define SUN4I_TCON1_CTL_CLK_DELAY(delay)		((delay << 4) & SUN4I_TCON1_CTL_CLK_DELAY_MASK)
#define SUN4I_TCON0_IO_POL_HSYNC_POSITIVE		BIT(25)
#define SUN4I_TCON0_IO_POL_VSYNC_POSITIVE		BIT(24)

#define SUN4I_TCON1_BASIC0_REG			*(volatile uint32_t *)(SUN4I_TCON_BASE + 0x94)
#define SUN4I_TCON1_BASIC0_X(width)			((((width) - 1) & 0xfff) << 16)
#define SUN4I_TCON1_BASIC0_Y(height)			(((height) - 1) & 0xfff)

#define SUN4I_TCON1_BASIC1_REG			*(volatile uint32_t *)(SUN4I_TCON_BASE + 0x98)
#define SUN4I_TCON1_BASIC1_X(width)			((((width) - 1) & 0xfff) << 16)
#define SUN4I_TCON1_BASIC1_Y(height)			(((height) - 1) & 0xfff)

#define SUN4I_TCON1_BASIC2_REG			*(volatile uint32_t *)(SUN4I_TCON_BASE + 0x9c)
#define SUN4I_TCON1_BASIC2_X(width)			((((width) - 1) & 0xfff) << 16)
#define SUN4I_TCON1_BASIC2_Y(height)			(((height) - 1) & 0xfff)

#define SUN4I_TCON1_BASIC3_REG			*(volatile uint32_t *)(SUN4I_TCON_BASE + 0xa0)
#define SUN4I_TCON1_BASIC3_H_TOTAL(total)		((((total) - 1) & 0x1fff) << 16)
#define SUN4I_TCON1_BASIC3_H_BACKPORCH(bp)		(((bp) - 1) & 0xfff)

#define SUN4I_TCON1_BASIC4_REG			*(volatile uint32_t *)(SUN4I_TCON_BASE + 0xa4)
#define SUN4I_TCON1_BASIC4_V_TOTAL(total)		(((total) & 0x1fff) << 16)
#define SUN4I_TCON1_BASIC4_V_BACKPORCH(bp)		(((bp) - 1) & 0xfff)

#define SUN4I_TCON1_BASIC5_REG			*(volatile uint32_t *)(SUN4I_TCON_BASE + 0xa8)
#define SUN4I_TCON1_BASIC5_H_SYNC(width)		((((width) - 1) & 0x3ff) << 16)
#define SUN4I_TCON1_BASIC5_V_SYNC(height)		(((height) - 1) & 0x3ff)

#define SUN4I_TCON0_IO_POL_REG			*(volatile uint32_t *)(SUN4I_TCON_BASE + 0x88)

#endif
