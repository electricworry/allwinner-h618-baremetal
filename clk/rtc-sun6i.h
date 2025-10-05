// allwinner,sun50i-h616-rtc
// rtc: rtc@7000000

#ifndef RTC_SUN6I_H
#define RTC_SUN6I_H

#define RTC_BASE 0x07000000

#define SUN6I_RTC_LOSC_CTRL                     *(volatile uint32_t *)(RTC_BASE + 0x0000)
#define SUN6I_RTC_DCXO_CTRL_REG                 *(volatile uint32_t *)(RTC_BASE + 0x0160)
#define SUN6I_RTC_ALRM_COUNTER                  *(volatile uint32_t *)(RTC_BASE + 0x0020)
#define SUN6I_RTC_ALRM_EN                       *(volatile uint32_t *)(RTC_BASE + 0x0028)
#define SUN6I_RTC_ALRM_IRQ_EN                   *(volatile uint32_t *)(RTC_BASE + 0x002c)
#define SUN6I_RTC_ALRM1_EN                      *(volatile uint32_t *)(RTC_BASE + 0x0044)
#define SUN6I_RTC_ALRM1_IRQ_EN                  *(volatile uint32_t *)(RTC_BASE + 0x0048)
#define SUN6I_RTC_ALRM_IRQ_STA                  *(volatile uint32_t *)(RTC_BASE + 0x0030)
#define SUN6I_RTC_ALRM1_IRQ_STA                 *(volatile uint32_t *)(RTC_BASE + 0x004c)
#define SUN6I_RTC_ALARM_CONFIG                  *(volatile uint32_t *)(RTC_BASE + 0x0050)


#define SUN6I_RTC_ALRM_IRQ_STA_CNT_IRQ_PEND	    BIT(0)
#define SUN6I_RTC_ALRM1_IRQ_STA_WEEK_IRQ_PEND   BIT(0)

#endif
