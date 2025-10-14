#include <stdio.h>
#include "ports.h"
#include "uart.h"
#include "mmu.h"
#include "system.h"
#include "display.h"
#include "interrupts.h"
#include "clk/clk.h"
#include "usb.h"

uint32_t tick_counter;

void game_tick(uint32_t tick_counter);
void game_start();

int main(int argc, char *argv[])
{
    printf("Booting!\n");

    // Reboot in n seconds using watchdog
    // reboot(0xb); // 0xb == 16 second reset timer
    // reboot(4);

    // Enble all GPIO
    // gpio_init();

    // Configure the UART for debugging
    uart_init();

    printf("Booting!\n");

    // Install interrupts
    install_ivt();


    // Set up MMU and paging configuration
    // mmu_init();

    // Illuminate the red LED - PC12
    set_pin_mode(PORTC, 12, 1); // PORT PL12 output
    set_pin_data(PORTC, 12, 1); // PORT PL12 high
    // Illuminate the green LED - PC13
    set_pin_mode(PORTC, 13, 1); // PORT PL13 output
    set_pin_data(PORTC, 13, 1); // PORT PL13 high

    // Configure display
    display_init();

    // USB
    // usb_init();

    printf("Ready!\n");
    game_start();
    // while(1)
    // {
    //   game_tick_next();
    // }
    printf("DONE!\n");

}

void game_tick_next() {
    buffer_swap();
    game_tick(tick_counter);
    tick_counter++;
}
