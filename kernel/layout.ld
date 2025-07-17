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