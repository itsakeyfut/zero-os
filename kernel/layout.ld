/*
 * Zero OS Kernel Linker Script
 *
 * This linker script defines the memory layout for the Zero OS kernel.
 * It supports both QEMU VersatilePB and real ARM hardware platforms.
 *
 * Memory Layout:
 * - 0x00000000 - 0x0000FFFF : Exception vectors and boot code
 * - 0x00010000 - 0x007FFFFF : Kernel code and data
 * - 0x00800000 - 0x07FFFFFF : Available RAM for kernel heap and user space
 *
 * Sections:
 * - .text     : Kernel code (read-execute)
 * - .rodata   : Read-only data (constants, strings)
 * - .data     : Initialized data (read-write)
 * - .bss      : Uninitialized data (zero-initialized)
 * - .stack    : Kernel stack space
 * - .heap     : Kernel heap space
 */

/* Entry point */
ENTRY(_start)

/* Memory configuration for QEMU VersatilePB */
{
    /* Exception vectors at start of RAM */
    VECTORS (rx)  : ORIGIN = 0x00000000, LENGTH = 0x1000     /* 4KB for vectors */
    
    /* Kernel code and data */
    KERNEL (rwx)  : ORIGIN = 0x00010000, LENGTH = 0x7F0000   /* ~8MB for kernel */
    
    /* Available RAM for dynamic allocation */
    RAM (rw)      : ORIGIN = 0x00800000, LENGTH = 0x7800000  /* ~120MB available */
}

/* Section definitions */
SECTIONS
{
    /* Exception vector table */
    .vectors : ALIGN(4)
    {
        __vectors_start = .;
        KEEP(*(.vectors))
        KEEP(*(.vectors.*))
        . = ALIGN(4);
        __vectors_end = .;
    } > VECTORS
    
    /* Kernel code section */
    .text : ALIGN(4)
    {
        __text_start = .;
        
        /* Boot code must come first */
        KEEP(*(.boot))
        KEEP(*(.boot.*))
        
        /* Main kernel code */
        *(.text)
        *(.text.*)
        
        /* ARM-specific code sections */
        *(.text.unlikely)
        *(.text.exit)
        *(.text.startup)
        *(.text.hot)
        
        /* Ensure proper alignment */
        . = ALIGN(4);
        __text_end = .;
    } > KERNEL
    
    /* Read-only data section */
    .rodata : ALIGN(4)
    {
        __rodata_start = .;
        
        /* String literals and constants */
        *(.rodata)
        *(.rodata.*)
        
        /* Debug strings (if enabled) */
        *(.rodata.str1.1)
        *(.rodata.str1.4)
        *(.rodata.str1.8)
        
        /* Global offset table (for position-independent code) */
        *(.got)
        *(.got.plt)
        
        /* Exception frame info */
        *(.eh_frame)
        *(.eh_frame_hdr)
        
        /* GNU stack information */
        *(.note.GNU-stack)
        
        . = ALIGN(4);
        __rodata_end = .;
    } > KERNEL
    
    /* Initialized data section */
    .data : ALIGN(4)
    {
        __data_start = .;
        
        /* Global and static variables */
        *(.data)
        *(.data.*)
        
        /* Small data sections (for some ARM variants) */
        *(.sdata)
        *(.sdata.*)
        
        /* Thread-local storage */
        *(.tdata)
        *(.tdata.*)
        
        . = ALIGN(4);
        __data_end = .;
    } > KERNEL
    
    /* Uninitialized data section */
    .bss : ALIGN(4)
    {
        __bss_start = .;
        
        /* Zero-initialized variables */
        *(.bss)
        *(.bss.*)
        
        /* Small BSS sections */
        *(.sbss)
        *(.sbss.*)
        
        /* Thread-local BSS */
        *(.tbss)
        *(.tbss.*)
        
        /* Common symbols */
        *(COMMON)
        
        . = ALIGN(4);
        __bss_end = .;
    } > KERNEL
    
    /* Kernel stack section */
    .stack : ALIGN(4096)
    {
        __stack_start = .;
        
        /* Reserve space for different stack types */
        
        /* Main kernel stack (16KB) */
        . = . + 0x4000;
        __main_stack_top = .;
        
        /* IRQ stack (8KB) */
        . = . + 0x2000;
        __irq_stack_top = .;
        
        /* FIQ stack (4KB) */
        . = . + 0x1000;
        __fiq_stack_top = .;
        
        /* Abort stack (4KB) */
        . = . + 0x1000;
        __abort_stack_top = .;
        
        /* Undefined stack (4KB) */
        . = . + 0x1000;
        __undefined_stack_top = .;
        
        /* Supervisor stack (4KB) */
        . = . + 0x1000;
        __supervisor_stack_top = .;
        
        . = ALIGN(4096);
        __stack_end = .;
    } > KERNEL
    
    /* Kernel heap section */
    .heap : ALIGN(4096)
    {
        __heap_start = .;
        
        /* Reserve initial heap space (1MB) */
        . = . + 0x100000;
        
        __heap_end = .;
    } > KERNEL
    
    /* Symbol table (for debugging) */
    .symtab : { *(.symtab) }
    .strtab : { *(.strtab) }
    .shstrtab : { *(.shstrtab) }
    
    /* Debug information (DWARF) */
    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_ranges   0 : { *(.debug_ranges) }
    .debug_frame    0 : { *(.debug_frame) }
    
    /* ARM-specific debug sections */
    .ARM.attributes 0 : { *(.ARM.attributes) }
    .ARM.exidx      0 : { *(.ARM.exidx) }
    .ARM.extab      0 : { *(.ARM.extab) }
    
    /* Discard unwanted sections */
    /DISCARD/ :
    {
        *(.note.*)
        *(.comment)
        *(.interp)
        *(.dynamic)
        *(.dynsym)
        *(.dynstr)
        *(.hash)
        *(.gnu.hash)
        *(.gnu.version)
        *(.gnu.version_d)
        *(.gnu.version_r)
        *(.plt)
        *(.iplt)
        *(.rel.*)
        *(.rela.*)
    }
}

/* Exported symbols for kernel use */

/* Memory layout symbols */
__kernel_start = ORIGIN(KERNEL);
__kernel_end = __heap_end;
__kernel_size = __kernel_end - __kernel_start;

/* Available RAM for dynamic allocation */
__ram_start = ORIGIN(RAM);
__ram_end = ORIGIN(RAM) + LENGTH(RAM);
__ram_size = LENGTH(RAM);

/* Section size calculations */
__text_size = __text_end - __text_start;
__rodata_size = __rodata_end - __rodata_start;
__data_size = __data_end - __data_start;
__bss_size = __bss_end - __bss_start;
__stack_size = __stack_end - __stack_start;
__heap_size = __heap_end - __heap_start;

/* Page-aligned sections for MMU */
__text_pages = (__text_size + 4095) / 4096;
__rodata_pages = (__rodata_size + 4095) / 4096;
__data_pages = (__data_size + 4095) / 4096;
__bss_pages = (__bss_size + 4095) / 4096;

/* Stack pointers for different ARM modes */
__stack_pointer_main = __main_stack_top;
__stack_pointer_irq = __irq_stack_top;
__stack_pointer_fiq = __fiq_stack_top;
__stack_pointer_abort = __abort_stack_top;
__stack_pointer_undefined = __undefined_stack_top;
__stack_pointer_supervisor = __supervisor_stack_top;

/* Memory management symbols */
__physical_memory_start = 0x00000000;
__physical_memory_end = 0x08000000;    /* 128MB for QEMU VersatilePB */
__physical_memory_size = __physical_memory_end - __physical_memory_start;

/* Kernel heap limits */
__heap_limit = __ram_start;  /* Heap can grow into available RAM */
__heap_max_size = 0x4000000; /* Maximum 64MB heap */

/* Virtual memory layout constants */
__virtual_kernel_base = 0xC0000000;    /* 3GB virtual address for kernel */
__virtual_user_base = 0x00000000;      /* 0GB virtual address for user */
__virtual_user_limit = 0x40000000;     /* 1GB limit for user space */

/* Page table locations (in available RAM) */
__page_table_start = __ram_start;
__page_table_size = 0x4000;            /* 16KB for page tables */
__page_table_end = __page_table_start + __page_table_size;

/* Boot parameters location */
__boot_params = __vectors_start + 0x100;  /* After reset vector */

/* Assetions to ensure memory layout is valid */
ASSERT(__text_start >= __vectors_end, "Text section overlaps with vectors")
ASSERT(__rodata_start >= __text_end, "Read-only data overlaps with text")
ASSERT(__data_start >= __rodata_end, "Data section overlaps with read-only data")
ASSERT(__bss_start >= __data_end, "BSS section overlaps with data")
ASSERT(__stack_start >= __bss_end, "Stack overlaps with BSS")
ASSERT(__heap_start >= __stack_end, "Heap overlaps with stack")
ASSERT(__heap_end <= __ram_start, "Kernel heap extends into available RAM")
