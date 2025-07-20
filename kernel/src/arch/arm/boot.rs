
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

/// CPU identification information
#[derive(Debug, Clone, Copy)]
pub struct CpuInfo {
    /// Main ID register value
    pub main_id: u32,
    /// Cache type register
    pub cache_type: u32,
    /// TLB type register
    pub tlb_type: u32,
    /// Processor feature registers
    pub processor_features: [u32; 2],
    /// Memory model features
    pub memory_model_features: [u32; 4],
    /// Instruction set attributes
    pub instruction_set_attributes: [u32; 5],
}

impl CpuInfo {
    /// Read CPU identification from coprocessor registers
    pub fn read() -> Self {
        let mut info = Self {
            main_id: 0,
            cache_type: 0,
            tlb_type: 0,
            processor_features: [0; 2],
            memory_model_features: [0; 4],
            instruction_set_attributes: [0; 5],
        };

        // SAFETY: Reading CPU identification registers is safe
        unsafe {
            // Main ID register
            asm!("mrc p15, 0, {}, c0, c0, 0", out(reg) info.main_id, options(nomem, nostack));
            
            // Cache type register
            asm!("mrc p15, 0, {}, c0, c0, 1", out(reg) info.cache_type, options(nomem, nostack));
            
            // TLB type register
            asm!("mrc p15, 0, {}, c0, c0, 3", out(reg) info.tlb_type, options(nomem, nostack));
            
            // Processor feature registers
            asm!("mrc p15, 0, {}, c0, c1, 0", out(reg) info.processor_features[0], options(nomem, nostack));
            asm!("mrc p15, 0, {}, c0, c1, 1", out(reg) info.processor_features[1], options(nomem, nostack));
            
            // Memory model feature registers
            asm!("mrc p15, 0, {}, c0, c1, 4", out(reg) info.memory_model_features[0], options(nomem, nostack));
            asm!("mrc p15, 0, {}, c0, c1, 5", out(reg) info.memory_model_features[1], options(nomem, nostack));
            asm!("mrc p15, 0, {}, c0, c1, 6", out(reg) info.memory_model_features[2], options(nomem, nostack));
            asm!("mrc p15, 0, {}, c0, c1, 7", out(reg) info.memory_model_features[3], options(nomem, nostack));
        }

        info
    }

    /// Get COU implementer ID
    pub fn implementer(&self) -> u8 {
        ((self.main_id >> 24) & 0xFF) as u8
    }

    /// Get CPU variant
    pub fn variant(&self) -> u8 {
        ((self.main_id >> 20) & 0xF) as u8
    }

    /// Get CPU architecture version
    pub fn architecture(&self) -> u8 {
        ((self.main_id >> 16) & 0xF) as u8
    }

    /// Get CPU part number
    pub fn part_number(&self) -> u16 {
        ((self.main_id >> 4) & 0xFFF) as u16
    }

    /// Get CPU revision
    pub fn revision(&self) -> u8 {
        (self.main_id & 0xF) as u8
    }

    /// Check if MMU is present
    pub fn has_mmu(&self) -> bool {
        (self.memory_model_features[0] & 0xF) != 0
    }

    /// Check if caches are present
    pub fn has_cache(&self) -> bool {
        self.cache_type != 0
    }
}

/// System Control Register (SCTLR) bits
pub mod sctlr_bits {
    /// MMU enable
    pub const MMU_ENABLE: u32 = 1 << 0;
    /// Alignment check enable
    pub const ALIGNMENT_CHECK: u32 = 1 << 1;
    /// Data cache enable
    pub const DATA_CACHE_ENABLE: u32 = 1 << 2;
    /// Write buffer enable
    pub const WRITE_BUFFER_ENABLE: u32 = 1 << 3;
    /// Exception endianness
    pub const EXCEPTION_ENDIAN: u32 = 1 << 25;
    /// Instruction cache enable
    pub const INSTRUCTION_CACHE_ENABLE: u32 = 1 << 12;
    /// High vectors (0xFFFF0000)
    pub const HIGH_VECTORS: u32 = 1 << 13;
    /// Round robin replacement
    pub const ROUND_ROBIN: u32 = 1 << 14;
    /// Disable loading TBIT
    pub const DISABLE_TBIT: u32 = 1 << 15;
    /// Fast interrupt configuration
    pub const FI: u32 = 1 << 21;
    /// Unaligned access enable
    pub const UNALIGNED_ACCESS: u32 = 1 << 22;
    /// Vectored interrupt enable
    pub const VECTORED_INTERRUPT: u32 = 1 << 24;
}

/// Set up stack pointers for all ARM processor modes
unsafe fn setup_stack_pointers() {
    // Get stack addresses from linker script
    unsafe extern "C" {
        static __main_stack_top: u8;
        static __irq_stack_top: u8;
        static __fiq_stack_top: u8;
        static __abort_stack_top: u8;
        static __undefined_stack_top: u8;
        static __supervisor_stack_top: u8;
    }
    
    // SAFETY: We're setting up stacks during boot
    unsafe {
        // IRQ mode stack
        asm!(
            "cps #0x12",  // Switch to IRQ mode
            "ldr sp, ={}",
            sym __irq_stack_top,
            options(nomem, nostack)
        );
        
        // FIQ mode stack  
        asm!(
            "cps #0x11",  // Switch to FIQ mode
            "ldr sp, ={}",
            sym __fiq_stack_top,
            options(nomem, nostack)
        );
        
        // Abort mode stack
        asm!(
            "cps #0x17",  // Switch to Abort mode
            "ldr sp, ={}",
            sym __abort_stack_top,
            options(nomem, nostack)
        );
        
        // Undefined mode stack
        asm!(
            "cps #0x1B",  // Switch to Undefined mode
            "ldr sp, ={}",
            sym __undefined_stack_top,
            options(nomem, nostack)
        );
        
        // Supervisor mode stack (main kernel mode)
        asm!(
            "cps #0x13",  // Switch to Supervisor mode
            "ldr sp, ={}",
            sym __supervisor_stack_top,
            options(nomem, nostack)
        );
    }
}

/// Configure CPU features and coprocessor registers
unsafe fn configure_cpu() {
    let mut sctlr: u32;
    
    // SAFETY: We're configuring CPU during boot
    unsafe {
        // Read current SCTLR
        asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
        
        // Configure basic CPU features
        sctlr |= sctlr_bits::UNALIGNED_ACCESS;  // Allow unaligned access
        sctlr |= sctlr_bits::INSTRUCTION_CACHE_ENABLE; // Enable I-cache
        sctlr &= !sctlr_bits::HIGH_VECTORS;     // Use low vectors (0x00000000)
        
        // Write back SCTLR
        asm!("mcr p15, 0, {}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
        
        // Instruction synchronization barrier
        asm!("isb", options(nomem, nostack));

        // Configure auxiliary control register if needed
        configure_auxiliary_control();
    }
}

/// Configure auxiliary control register for performance
unsafe fn configure_auxiliary_control() {
    let mut actlr: u32;

    // SAFETY: We're configuring CPU during boot
    unsafe {
        // Read auxiliary control register
        asm!("mrc p15, 0, {}, c1, c0, 1", out(reg) actlr, options(nomem, nostack));

        // Enable return stack (if available)
        actlr |= 1 << 2;

        // Enable dynamic branch prediction (if available)
        actlr |= 1 << 1;

        // Write back auxiliary control register
        asm!("mcr p15, 0, {}, c1, c0, 1", in(reg) actlr, options(nomem, nostack));
    }
}