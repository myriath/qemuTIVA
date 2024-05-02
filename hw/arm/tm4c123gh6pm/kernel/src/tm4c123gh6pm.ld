MEMORY {
        FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
        SRAM  (rw) : ORIGIN = 0x20001000, LENGTH = 28K
}

SECTIONS {

    . = 0x00000000;
    .text : {
        KEEP(*(.vectors))
        *(.text)
        *(.rodata)
        . = ALIGN(4);
    } > FLASH
    
    .data : {
        . = ALIGN(4);
        KEEP(*(.vectors_dynamic))
        _sram_sdata = .;
        *(.data);
        . = ALIGN(4);
        _sram_edata = .;
    } > SRAM AT > FLASH

    _flash_sdata = LOADADDR(.data);

    .bss : {
        . = ALIGN(4);
        _sram_sbss = .;
        *(.bss)
        *(COMMON)
        . = ALIGN(4);
        _sram_ebss = .;
    } > SRAM

    _sram_stacktop = ORIGIN(SRAM) + LENGTH(SRAM);
}
