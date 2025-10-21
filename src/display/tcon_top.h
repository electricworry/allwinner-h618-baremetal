// allwinner,sun50i-h6-tcon-top

#ifndef TCON_TOP_H
#define TCON_TOP_H

#include "../util.h"

#define TCON_TOP_BASE           0x6510000

#define TCON_TOP_PORT_SEL_REG   *(volatile uint32_t *)(TCON_TOP_BASE + 0x1C)
#define TCON_TOP_PORT_DE0_MSK   GENMASK(1, 0)
#define TCON_TOP_GATE_SRC_REG   *(volatile uint32_t *)(TCON_TOP_BASE + 0x20)
#define TCON_TOP_HDMI_SRC_MSK   GENMASK(29, 28)

#endif
