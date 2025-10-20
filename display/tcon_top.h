// allwinner,sun50i-h6-tcon-top

#include "../util.h"

// From TCON_TOP

#define TCON_TOP_BASE               0x6510000 // allwinner,sun50i-h6-tcon-top
#define TCON_TOP_TCON_TV_SETUP_REG	*(volatile uint32_t *)(TCON_TOP_BASE + 0x00)
#define TCON_TOP_PORT_SEL_REG		*(volatile uint32_t *)(TCON_TOP_BASE + 0x1C)
#define TCON_TOP_PORT_DE0_MSK			GENMASK(1, 0)
#define TCON_TOP_PORT_DE1_MSK			GENMASK(5, 4)
#define TCON_TOP_GATE_SRC_REG		*(volatile uint32_t *)(TCON_TOP_BASE + 0x20)
#define TCON_TOP_HDMI_SRC_MSK			GENMASK(29, 28)
#define TCON_TOP_TCON_TV1_GATE			24
#define TCON_TOP_TCON_TV0_GATE			20
#define TCON_TOP_TCON_DSI_GATE			16
#define TCON_TOP_CLK_NUM				3
#define TCON_TOP_
