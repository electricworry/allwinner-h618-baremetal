// allwinner,sun50i-h616-hdmi-phy

#ifndef HDMI_PHY_H
#define HDMI_PHY_H

#include "../util.h"

#define HDMI_PHY_BASE                       0x6010000 

#define SUN8I_HDMI_PHY_REXT_CTRL_REG        (HDMI_PHY_BASE + 0x0004)
#define SUN8I_HDMI_PHY_REXT_CTRL_REXT_EN    BIT(31)

#endif