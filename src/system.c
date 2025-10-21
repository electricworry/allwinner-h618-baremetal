#include <stdio.h>
#include "system.h"
#include "uart.h"

void init_bss() {
    printf("Init BSS1!\n"); // Yes, UART works before I've initialised it. BROM or SPL possibly responsible.
    // for (char* dst = &__bss_start__; dst < &__bss_end__ - 0xD8 ; dst++) // TODO: This causes problems!!
    //     *dst = 0;
    printf("Init BSS2!\n");
    for (char* dst = &__bss2_start__; dst < &__bss2_end__; dst++)
        *dst = 0;
}

void udelay(uint32_t d) {
    // while (cycles--)
    // {
    //     asm volatile("NOP \n");
    // }
    for(uint32_t n=0;n<d*3;n++) asm("NOP");
}

void reboot(uint32_t seconds) {
    WDOG0_MODE = (seconds << 4) | 1;
}

/* The followiong implements newlib 'syscalls' */

// #include <errno.h>
// #undef errno
// extern int errno;
// int _wait(int *status) {
//   errno = ECHILD;
//   return -1;
// }

#include <sys/stat.h>
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

int _close(int file) {
    return -1;
}

int _write(int file, char *ptr, int len) {
    int todo;
    char c;

    for (todo = 0; todo < len; todo++) {
        c = *ptr++;
        if (c == '\n')
            uart_putc('\r');
        uart_putc(c);
    }
    return len;
}

int _read(int file, char *ptr, int len) {
    return 0;
}

// extern int _end;

#include <unistd.h>
caddr_t _sbrk(int incr) {
    extern char _end;		/* Defined by the linker */
    static char *heap_end;
    char *prev_heap_end; 
    
    if (heap_end == 0) {
        heap_end = &_end;
    }
    prev_heap_end = heap_end;
    // if (heap_end + incr > stack_ptr) {
    //     write (1, "Heap and stack collision\n", 25);
    //     abort ();
    // }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}

void _exit(int status) {
    // __asm("BKPT #0");
    // Go back to sleep
    while(1) asm("wfi");
}

#include <errno.h>
#undef errno
extern int errno;
int _kill(int pid, int sig) {
    errno = EINVAL;
    return -1;
}

int _getpid(void) {
    return 1;
}

int _isatty(int file) {
  return 1;
}
