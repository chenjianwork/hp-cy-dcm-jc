/* Entry Point */
ENTRY(Reset_Handler)

HEAP_SIZE  = DEFINED(__heap_size__)  ? __heap_size__  : 0x00000400;
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x00004000;

/* Specify the memory areas */
MEMORY
{
    m_romvector     (RX) : ORIGIN = 0x08000000 + TEXT_OFFSET, LENGTH = 1K
    m_romtext       (RX) : ORIGIN = 0x08000400 + TEXT_OFFSET, LENGTH = 2M - 1K
    m_ramdata       (RW) : ORIGIN = 0x20000000, LENGTH = 192K
}

/* Define output sections */
SECTIONS
{
    /* The startup code goes first into internal flash */
    .interrupts :
    {
        __VECTOR_TABLE = .;
        . = ALIGN(4);
        KEEP(*(.isr_vector))     /* Startup code */
        . = ALIGN(4);
    } > m_romvector

    /* The program code and other data goes into internal flash */
    .text :
    {
        . = ALIGN(4);
        *(.text)                 /* .text sections (code) */
        *(.text*)                /* .text* sections (code) */
        *(.rodata)               /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
        *(.glue_7)               /* glue arm to thumb code */
        *(.glue_7t)              /* glue thumb to arm code */
        *(.eh_frame)
        KEEP (*(.init))
        KEEP (*(.fini))
        . = ALIGN(4);
    } > m_romtext

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > m_romtext

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > m_romtext

    .ctors :
    {
        __CTOR_LIST__ = .;
        KEEP (*crtbegin.o(.ctors))
        KEEP (*crtbegin?.o(.ctors))
        KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*(.ctors))
        __CTOR_END__ = .;
    } > m_romtext

    .dtors :
    {
        __DTOR_LIST__ = .;
        KEEP (*crtbegin.o(.dtors))
        KEEP (*crtbegin?.o(.dtors))
        KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*(.dtors))
        __DTOR_END__ = .;
    } > m_romtext

    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } > m_romtext

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } > m_romtext

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } > m_romtext

    __etext = .;
    _sidata = .;
    __DATA_ROM = .;

    .data : AT(__DATA_ROM)
    {
        . = ALIGN(4);
        __DATA_RAM = .;
        _sdata = .;
        __data_start__ = .;      /* create a global symbol at data start */
        *(.data)                 /* .data sections */
        *(.data*)                /* .data* sections */
        KEEP(*(.jcr*))
        . = ALIGN(4);
        _edata = .;
        __data_end__ = .;        /* define a global symbol at data end */
    } > m_ramdata

    /* Symbol is used by startup for data initialization */
    __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);

    /* Uninitialized data section */
    .bss :
    {
        /* This is used by the startup in order to initialize the .bss section */
        . = ALIGN(4);
        __START_BSS = .;
        _sbss = .;
        __bss_start__ = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = .;
        __END_BSS = .;
    } > m_ramdata

    .stack :
    {
        . = ALIGN(8);
        . += STACK_SIZE;
    } > m_ramdata

    _estack = .;
    __StackTop   = .;
    __StackLimit = __StackTop - STACK_SIZE;
    PROVIDE(__stack = __StackTop);

    .heap :
    {
        . = ALIGN(8);
        __end__ = .;
        PROVIDE(end = .);
        __HeapBase = .;
        __heap_start = .;
        . += HEAP_SIZE;
        __HeapLimit = .;
    } > m_ramdata
    
    __romtext_start = ORIGIN(m_romtext);
    __romtext_limit = ORIGIN(m_romtext) + LENGTH(m_romtext);
    
    __heap_limit = ORIGIN(m_ramdata) + LENGTH(m_ramdata);
    
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
