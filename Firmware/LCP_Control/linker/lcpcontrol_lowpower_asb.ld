/******************************************************************************
 *
 * freertos_lowpower.ld - Linker script for applications using startup_gnu.c
 *
 *****************************************************************************/
ENTRY(Reset_Handler)

MEMORY
{
    /* Application code and read-only data */
    /* Starts at 0xC000, ends at 0x7FFFF (End of Flash Instance 0) */
    ROMEM (rx) : ORIGIN = 0x0000C000, LENGTH = 0x00074000 /* 464K */

    /* Non-volatile storage for the data queue */
    /* Starts at 0x80000 (Start of Flash Instance 1), ends at 0xFFFFF (End of Flash Instance 1) */
    NVSTORAGE (rw) : ORIGIN = 0x00080000, LENGTH = 0x00080000 /* 512K */

    /* SRAM */
    RWMEM (rwx) : ORIGIN = 0x10000000, LENGTH = 384K
}

SECTIONS
{
    /* Code and read-only data section */
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Keep interrupt vector table */
        KEEP(*(.patch))      /* Keep patch code */
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .; /* End of text section */
    } > ROMEM

    /* Non-volatile storage section */
    .nvdata :
    {
        . = ALIGN(4);
        *(.nvdata)          /* Non-volatile user data */
        *(.nvdata*)         /* Non-volatile user data */
        . = ALIGN(4);
        _envdata = .;       /* End of NV storage section */
    } > NVSTORAGE

    /* Initialized data section */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;         /* Start of initialized data */
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;         /* End of initialized data */
    } > RWMEM AT>ROMEM

    /* Used by startup to initialize .data */
    _init_data = LOADADDR(.data);

    /* Uninitialized data section */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;          /* Start of BSS */
        *(.bss)
        *(.bss*)
        *(COMMON)           /* Common uninitialized symbols */
        . = ALIGN(4);
        _ebss = .;          /* End of BSS */
    } > RWMEM

    /* User stack section */
    .stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.stack)
        *(.stack*)
        . = ALIGN(8);
    } > RWMEM

    /* End of user data - Heap starts from here */
    end = .;

    /* Stack starts at the end of RAM */
    _estack = ORIGIN(RWMEM) + LENGTH(RWMEM);

    /* ARM attributes */
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
