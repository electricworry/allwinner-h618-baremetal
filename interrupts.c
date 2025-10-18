#include <stdint.h>
#include <stdio.h>
#include "uart.h"
#include "interrupts.h"
#include "display.h"
#include "util.h"

void complete_irq_hdmi(void);

/*  This is called after the interrupt handler in the vector table has
    acknowledged the IRQ, meaning the state has changed from pending to active.
    A real kernel would have some mechanism for drivers to register their
    interest in different IRQs, but we're just going to hardcode the destination
    of different IRQs.
*/
void /*__attribute__((interrupt("IRQ")))*/ interrupt_active(uint32_t iar_val)
{
    uint32_t irq = GENMASK(9, 0) & iar_val;
    switch (irq) {
        case 95: //HDMI
            complete_irq_hdmi();
            break;
        default:
            printf("Received an unexpected IRQ: %d\n", irq);
    }
}
