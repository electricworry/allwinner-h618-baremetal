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


// extern uint32_t _ivt;
void game_tick_next();

// Called when an interrupt is triggered
// Currently this is always triggered by at new frame at 60Hz
// void __attribute__((interrupt("FIQ"))) interrupt(void) {
//   printf("Received INT\n");
//   game_tick_next();
//   // LCD0_GINT0 &= ~(1<<12);
// }

// Copy the interrupt table from _ivt to 0x0
void install_ivt() {
  // uint32_t* source = &_ivt;
  // uint32_t* destination = (uint32_t*)(0);
  // for(int n=0; n<2*8; n++)
  //   destination[n] = source[n];

  // struct gicd_reg* gicd = (struct gicd_reg*) GICD_BASE;
  // gicd->ctlr = 1;
  // gicd->isenabler[118/32] = 1<<(118%32);
  // gicd->itargetsr[118] = 1;
  // gicd->ipriorityr[118] = 1;
  // struct gicc_reg* gicc = (struct gicc_reg*) GICC_BASE;
  // gicc->ctlr = 1;
  // gicc->pmr = 10;
  // asm("cpsie if;"); // Enable interrupts
}
