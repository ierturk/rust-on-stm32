/* Linker script for the STM32F429ZIT6 on STM32F429i Discover Kit */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM : ORIGIN = 0x20000000, LENGTH = 192K
  SDRAM    : ORIGIN = 0xd0000000, LENGTH = 8192K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS {
     .frame_buffer (NOLOAD) : {
       . = ALIGN(4);
       *(.frame_buffer);
       . = ALIGN(4);
     } > SDRAM
     .embedded_alloc_heap (NOLOAD) : {
       . = ALIGN(4);
       *(.embedded_alloc_heap);
       . = ALIGN(4);
     } > SDRAM
     .slint_assets (NOLOAD) : {
       . = ALIGN(4);
      __s_slint_assets = .;
       *(.slint_assets);
       . = ALIGN(4);
     } > SDRAM

    __e_slint_assets = .;
    __si_slint_assets = LOADADDR(.slint_assets);
}