/******************************************************************************
 *
 * ble_freertos_tag.ld - Linker script for applications using startup_gnu.c
 *
 *****************************************************************************/
ENTRY(Reset_Handler)

MEMORY
{
    /* Starts at 0x0C000, ends at 0x65FFF */
    ROMEM (rx) : ORIGIN = 0x0000C000, LENGTH = 360K

    /* Starts immediately after ROMEM at 0x66000, ends at 0xFBFFF */
    NVSTORAGE (rx) : ORIGIN = 0x00066000, LENGTH = 600K

    /* Starts much later, no overlap */
    RWMEM (rwx) : ORIGIN = 0x10000000, LENGTH = 384K
}

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

    /* Non-volatile storage section */
    .nvdata :
    {
        . = ALIGN(4);
        *(.nvdata)          /* Non-volatile user data */
        *(.nvdata*)         /* Non-volatile user data */
        . = ALIGN(4);
        _envdata = .;       /* End of NV storage section */
    } > NVSTORAGE

    /* User stack section initialized by startup code. */
    .stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.stack)
        *(.stack*)
        . = ALIGN(8);
    } > RWMEM

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

	. = ALIGN(4);
    /* End of user data. Heap starts from here */
    end = . ;
  
    /* Stack starts at end of ram */
    _estack = ORIGIN(ROMEM) + LENGTH(ROMEM) - 4;

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
