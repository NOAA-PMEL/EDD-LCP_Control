/******************************************************************************
 *
 * freertos_lowpower.ld - Linker script for applications using startup_gnu.c
 *
 *****************************************************************************/
ENTRY(Reset_Handler)

MEMORY
{
    ROMEM (rx) : ORIGIN = 0x0000C000, LENGTH = 960K
    RWMEM (rwx) : ORIGIN = 0x10000000, LENGTH = 384K
}

/* Define top of stack */
_estack = ORIGIN(RWMEM) + LENGTH(RWMEM);

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        KEEP(*(.patch))
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > ROMEM

    /* User stack section defined in startup code. Put it high in RAM. */
    /* The startup code initializes SP to _estack */
    .stack (NOLOAD):
    {
        . = ALIGN(8);
        _sstack = .; /* Define label for start of stack area (optional) */
        *(.stack)
        *(.stack*)
        . = ALIGN(8);
     } > RWMEM /* Grows down from _estack */


    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RWMEM AT>ROMEM

    /* used by startup to initialize data */
    _init_data = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RWMEM

    /* ---> Define end of BSS and start of Heap <--- */
    . = ALIGN(4);
    _end = .; /* Standard symbol for end of data/bss */
    end = _end; /* Symbol expected by libnosys _sbrk */
    __end__ = _end;
    PROVIDE ( end = _end );
    _heap_start = _end; /* Heap starts immediately after BSS */
    /* --- End of Heap/Start of Stack definition (Optional) --- */
    /* Define heap end relative to stack pointer, leave space for stack */
    /* Example: _heap_end = _estack - 0x800;  */
    /* Ensure _heap_end does not overlap _heap_start */


    .ARM.attributes 0 : { *(.ARM.attributes) }
}