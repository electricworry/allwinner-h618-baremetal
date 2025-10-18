#include <stdio.h>
#include "ports.h"
#include "system.h"
#include "display.h"

int main(int argc, char *argv[])
{
    printf("Booting!\n");

    reboot(8);

    // Illuminate the red LED - PC12
    set_pin_mode(PORTC, 12, 1); // PORT PL12 output
    set_pin_data(PORTC, 12, 1); // PORT PL12 high
    // Illuminate the green LED - PC13
    set_pin_mode(PORTC, 13, 1); // PORT PL13 output
    set_pin_data(PORTC, 13, 1); // PORT PL13 high

    // Configure display
    display_init();

    printf("DONE!\n");
}
