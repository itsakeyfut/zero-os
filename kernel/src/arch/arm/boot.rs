
//! ARM Boot Sequence Implementation
//!
//! This module handles the early boot process for ARM Cortex-A processors.
//! It provides the initial setup required before the main kernel can run,
//! including CPU initialization, memory setup, and exception vector configuration.
//!
//! # Boot Process Overview
//!
//! 1. **Hardware Reset**: CPU starts at reset vector (0x00000000)
//! 2. **Stack Setup**: Initialize stack pointers for all ARM modes
//! 3. **CPU Configuration**: Set up coprocessor registers and features
//! 4. **Memory Initialization**: Clear BSS and set up initial page tables
//! 5. **Exception Vectors**: Install exception handlers
//! 6. **Cache/MMU Setup**: Enable caches and memory management
//! 7. **Kernel Entry**: Transfer control to Rust kernel main
//!
//! # ARM Processor Modes
//!
//! - **User (USR)**: Normal program execution
//! - **System (SYS)**: Privileged user mode  
//! - **Supervisor (SVC)**: OS kernel mode
//! - **Abort (ABT)**: Memory access violations
//! - **Undefined (UND)**: Undefined instruction handling
//! - **IRQ**: Normal interrupt handling
//! - **FIQ**: Fast interrupt handling
//!
//! # Memory Layout During Boot
//!
//! ```text
//! 0x00000000: Exception vectors
//! 0x00000020: Boot code entry point
//! 0x00010000: Kernel code start
//! 0x00800000: Initial page tables
//! ```

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::arch::asm;
use crate::memory::{VirtualAddress, PhysicalAddress};

/// Boot configuration parameters
pub struct BootConfig {
    /// Physical address where kernel is loaded
    pub kernel_physical_start: PhysicalAddress,
    /// Virtual address where kernel should be mapped
    pub kernel_virtual_start: VirtualAddress,
    /// Size of kernel image
    pub kernel_size: usize,
    /// Physical memory start
    pub memory_start: PhysicalAddress,
    /// Total memory size
    pub memory_size: usize,
}

impl Default for BootConfig {
    fn default() -> Self {
        Self {
            kernel_physical_start: PhysicalAddress::new(0x10000),
            kernel_virtual_start: VirtualAddress::new(0xC0010000),
            kernel_size: 8 * 1024 * 1024, // 8MB
            memory_start: PhysicalAddress::new(0x00000000),
            memory_size: 128 * 1024 * 1024, // 128MB
        }
    }
}