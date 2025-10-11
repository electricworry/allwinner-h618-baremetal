#include <stdint.h>

// Port access struct
struct port_registers {
  uint32_t cfg0;
  uint32_t cfg1;
  uint32_t cfg2;
  uint32_t cfg3;
  uint32_t data;
};

// The PORT registers base address.
#define PIO_BASE          0x0300B000
#define PORTC             PIO_BASE + 2 * 0x24
#define PORTF             PIO_BASE + 5 * 0x24
#define PORTG             PIO_BASE + 6 * 0x24
#define PORTH             PIO_BASE + 7 * 0x24
#define PORTI             PIO_BASE + 9 * 0x24
#define PORTL             0x07022000

void set_pin_mode(uint64_t port, uint32_t pin, uint32_t mode);
void set_pin_data(uint64_t port, uint32_t pin, uint32_t data);
void gpio_init();