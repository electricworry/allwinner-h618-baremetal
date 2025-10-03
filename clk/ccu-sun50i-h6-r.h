// allwinner,sun50i-h616-r-ccu
// r_ccu: clock@7010000

#define R_CCU_BASE 0x07010000

#define SUN50I_H616_CLK_R_APB1_TWD_REG      *(volatile uint32_t *)(R_CCU_BASE + 0x012c)
#define SUN50I_H616_CLK_R_APB1_RTC_REG      *(volatile uint32_t *)(R_CCU_BASE + 0x020c)