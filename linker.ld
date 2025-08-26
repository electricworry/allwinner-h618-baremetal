MEMORY
{
    brom    (r)   : ORIGIN = 0x00000000, LENGTH =    0x10000
    sram_a1 (rwx) : ORIGIN = 0x00020000, LENGTH =     0x8000
    sram_c  (rwx) : ORIGIN = 0x00028000, LENGTH =    0x30000
    ram     (rwx) : ORIGIN = 0x40000000, LENGTH = 0x20000000
}

SECTIONS
{
    .text :
    {
        . = ALIGN(16);   /* .text segment must be 4-byte aligned */
        boot.o(.text)   /* put boot.o at start of .text. */
        *(.text)
    } > ram

    .bss (NOLOAD) :
    {
        . = ALIGN(16);
        __bss_start__ = . ;
        *(.bss);
        *(COMMON);
        __bss_end__ = . ;
    } > ram

    .data :
    {
        *(.data)
    } > ram

    .uncached (NOLOAD) : { # was at 0x50000000
        __bss2_start__ = . ;
        *(UNCACHED);
        __bss2_end__ = . ;
    } > ram

    /* End marker for heap start */
    . = ALIGN(16);
    _end = .;    /* Heap starts here and grows upwards */

    /* Stack grows downward from the end of RAM */
    _stack_size = 64K;
    _stack_top = ORIGIN(ram) + LENGTH(ram);
    _stack_bottom = _stack_top - _stack_size;
    
    /* Ensure we don't overlap with heap */
    ASSERT(_end <= _stack_bottom, "Error: Heap collides with stack")
}
