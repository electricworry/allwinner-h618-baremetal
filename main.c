#include <stdio.h>
#include "ports.h"
#include "uart.h"
#include "mmu.h"
#include "system.h"
#include "display.h"
#include "interrupts.h"
#include "ccu.h"
#include "usb.h"

uint32_t tick_counter;

void game_tick(uint32_t tick_counter);
void game_start();

int main(int argc, char *argv[])
{
    // Reboot in n seconds using watchdog
    // reboot(0xb); // 0xb == 16 second reset timer
    // reboot(2);


    int v = 5;
    int count = 6000000;
    while(count--)
    {
    }
    printf("Booting!\n");
    printf("bddress: %d\n", 5); // This fails in a memcpy. Need to enable paging.
    printf("TEST2\n");
    uart_putc('b');
    uart_putc('\n');

    // Enble all GPIO
    gpio_init();

    // Configure the UART for debugging
    uart_init();

    printf("Booting!\n");

    // Install interrupts
    install_ivt();


    // Set up MMU and paging configuration
    // mmu_init();

    // Illuminate the power LED
    set_pin_mode(PORTL, 10, 1); // PORT L10 output
    set_pin_data(PORTL, 10, 1); // PORT L10 high

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
