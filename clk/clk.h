#include "ccu-sun50i-h6-r.h"
#include "ccu-sun50i-h616.h"
#include "rtc-sun6i.h"

#define R_PRCM_BASE 0x01F01400
#define APB0_CLK_GATING       *(volatile uint32_t *)(R_PRCM_BASE + 0x28)
