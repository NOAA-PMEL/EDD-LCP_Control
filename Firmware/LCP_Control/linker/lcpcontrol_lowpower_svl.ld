/******************************************************************************
 *
 * ble_freertos_tag.ld - Linker script for applications using startup_gnu.c
 *
 *****************************************************************************/
ENTRY(Reset_Handler)

MEMORY
{
    ROMEM (rx) : ORIGIN = 0x0000C000, LENGTH = 360K /* Program and constant data */
    NVSTORAGE (rx) : ORIGIN = 0x0004A000, LENGTH = 600K /* Reserved NV storage */
    RWMEM (rwx) : ORIGIN = 0x10000000, LENGTH = 384K /* RAM */
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
