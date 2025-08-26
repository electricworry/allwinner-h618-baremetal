_reset:
// cpsid if

LDR x4, =0x60000000     // TODO: Can this come from linker script?
mov sp, x4

LDR x4, =0x05000000
MOV w5, #0x21
STRB w5, [x4]
MOV w5, #0x0d
STRB w5, [x4]
MOV w5, #0x0a
STRB w5, [x4]
bl main

_aaa:


// MRC p15,0,r0,c1,c0,2    // Read CP Access register
// ORR r0,r0,#0x00f00000   // Enable full access to NEON/VFP (Coprocessors 10 and 11)
// MCR p15,0,r0,c1,c0,2    // Write CP Access register
// ISB
// MOV r0,#0x40000000      // Switch on the VFP and NEON hardware
// VMSR FPEXC,r0           // Set EN bit in FPEXC
// 
// mov sp, #0x60000000     // TODO: Can this come from linker script?
// //blx _start            // This would branch to newlib crt0, but that places the stack at 0x80000. TODO: Investigate how to change this.
// bl init_bss
// bl main
// b _exit
// 
.globl _ivt
_ivt:
// ldr pc, _reset_h
// ldr pc, _undefined_instruction_vector_h
// ldr pc, _software_interrupt_vector_h
// ldr pc, _prefetch_abort_vector_h
// ldr pc, _data_abort_vector_h
// ldr pc, _unused_handler_h
// ldr pc, _interrupt_vector_h
// ldr pc, _fast_interrupt_vector_h
// _reset_h:                           .word   _reset
// _undefined_instruction_vector_h:    .word   _reset
// _software_interrupt_vector_h:       .word   _reset
// _prefetch_abort_vector_h:           .word   _reset
// _data_abort_vector_h:               .word   _reset
// _unused_handler_h:                  .word   _reset
// _interrupt_vector_h:                .word   interrupt
// _fast_interrupt_vector_h:           .word   _reset
