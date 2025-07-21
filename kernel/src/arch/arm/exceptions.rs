
//! ARM Exception Handling
//!
//! This module provides comprehensive exception handling for ARM Cortex-A processors.
//! It manages all types of exceptions including interrupts, data/prefetch aborts,
//! undefined instructions, and system calls.
//!
//! # ARM Exception Vector Table
//!
//! ARM processors use a vector table located at 0x00000000 (or 0xFFFF0000 for high vectors):
//!
//! ```text
//! 0x00: Reset               - System reset
//! 0x04: Undefined           - Undefined instruction
//! 0x08: SWI/SVC            - Software interrupt (system calls)
//! 0x0C: Prefetch Abort     - Instruction fetch memory abort
//! 0x10: Data Abort         - Data access memory abort  
//! 0x14: Reserved           - Not used
//! 0x18: IRQ                - Normal interrupt request
//! 0x1C: FIQ                - Fast interrupt request
//! ```
//!
//! # Exception Handling Process
//!
//! 1. **Hardware Response**: CPU saves state and jumps to vector
//! 2. **Vector Handler**: Assembly stub saves context and calls Rust handler
//! 3. **Rust Handler**: High-level exception processing
//! 4. **Context Restore**: Return to interrupted code or schedule new process
//!
//! # Processor Mode Changes
//!
//! Each exception type causes the processor to enter a specific mode:
//! - IRQ → IRQ mode (0x12)
//! - FIQ → FIQ mode (0x11)  
//! - SWI → Supervisor mode (0x13)
//! - Abort → Abort mode (0x17)
//! - Undefined → Undefined mode (0x1B)

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::arch::asm;
use crate::arch::{CpuContext, InterruptType};
use crate::process::{ProcessId, ProcessManager};
use crate::syscalls::{SystemCall, SystemCallResult, SystemCallArgs, SystemCallNumber};

/// Exception vector offsets
pub mod vectors {
    /// Reset vector offset
    pub const RESET: usize = 0x00;
    /// Undefined instruction vector offset
    pub const UNDEFINED: usize = 0x04;
    /// Software interrupt vector offset
    pub const SWI: usize = 0x08;
    /// Prefetch abort vector offset
    pub const PREFETCH_ABORT: usize = 0x0C;
    /// Data abort vector offset
    pub const DATA_ABORT: usize = 0x10;
    /// Reserved vector offset
    pub const RESERVED: usize = 0x14;
    /// IRQ vector offset
    pub const IRQ: usize = 0x18;
    /// FIQ vector offset
    pub const FIQ: usize = 0x1C;
}

/// ARM processor modes
pub mod modes {
    /// User mode
    pub const USER: u32 = 0x10;
    /// FIQ mode
    pub const FIQ: u32 = 0x11;
    /// IRQ mode
    pub const IRQ: u32 = 0x12;
    /// Supervisor mode
    pub const SUPERVISOR: u32 = 0x13;
    /// Abort mode
    pub const ABORT: u32 = 0x17;
    /// Undefined mode
    pub const UNDEFINED: u32 = 0x1B;
    /// System mode
    pub const SYSTEM: u32 = 0x1F;
}

/// CPSR (Current Program Status Register) flags
pub mod cpsr_flags {
    /// Negative flag
    pub const N: u32 = 1 << 31;
    /// Zero flag
    pub const Z: u32 = 1 << 30;
    /// Carry flag
    pub const C: u32 = 1 << 29;
    /// Overflow flag
    pub const V: u32 = 1 << 28;
    /// Sticky overflow flag
    pub const Q: u32 = 1 << 27;
    /// IRQ disable
    pub const I: u32 = 1 << 7;
    /// FIQ disable
    pub const F: u32 = 1 << 6;
    /// Thumb state
    pub const T: u32 = 1 << 5;
    /// Mode mask
    pub const MODE_MASK: u32 = 0x1F;
}

/// Data Fault Statue Register (DFSR) bits
pub mod dfsr_bits {
    /// External abort on non-linefetch
    pub const EXT_ABORT: u32 = 0x01;
    /// Alignment fault
    pub const ALIGNMENT: u32 = 0x02;
    /// Debug event
    pub const DEBUG: u32 = 0x04;
    /// Access flag fault (level 1)
    pub const ACCESS_FLAG_L1: u32 = 0x05;
    /// Translation fault (level 1)
    pub const TRANSLATION_L1: u32 = 0x06;
    /// Access flag fault (level 2)
    pub const ACCESS_FLAG_L2: u32 = 0x07;
    /// Translation fault (level 2)
    pub const TRANSLATION_L2: u32 = 0x08;
    /// Domain fault (level 1)
    pub const DOMAIN_L1: u32 = 0x09;
    /// Domain fault (level 2)
    pub const DOMAIN_L2: u32 = 0x0B;
    /// Permission fault (level 1)
    pub const PERMISSION_L1: u32 = 0x0D;
    /// Permission fault (level 2)
    pub const PERMISSION_L2: u32 = 0x0F;
}

/// Exception information structure
#[derive(Debug, Clone, Copy)]
pub struct ExceptionInfo {
    /// Exception type
    pub exception_type: ExceptionType,
    /// Fault address (for data/prefetch aborts)
    pub fault_address: Option<u32>,
    /// Fault status (for aborts)
    pub fault_status: Option<u32>,
    /// Instruction that caused the exception
    pub fault_instruction: Option<u32>,
    /// Process ID that caused the exception
    pub process_id: Option<ProcessId>,
}

/// Exception types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExceptionType {
    /// Reset exception
    Reset,
    /// Undefined instruction
    UndefinedInstruction,
    /// Software interrupt (system call)
    SoftwareInterrupt,
    /// Prefetch abort
    PrefetchAbort,
    /// Data abort
    DataAbort,
    /// IRQ interrupt
    Irq,
    /// FIQ interrupt
    Fiq,
}

/// Exception statistics for monitoring
#[derive(Debug, Default, Clone, Copy)]
pub struct ExceptionStats {
    /// Total exceptions handled
    pub total_exceptions: u64,
    /// Undefined instruction count
    pub undefined_instructions: u32,
    /// System call count
    pub system_calls: u64,
    /// Prefetch abort count
    pub prefetch_aborts: u32,
    /// Data abort count
    pub data_aborts: u32,
    /// IRQ count
    pub irq_count: u64,
    /// FIQ count
    pub fiq_count: u64,
    /// Nested interrupt count
    pub nested_interrupts: u32,
    /// Maximum interrupt nesting level
    pub max_nesting_level: u32,
}

/// Exception handler manager
pub struct ExceptionManager {
    /// Exception statistics
    stats: ExceptionStats,
    /// Current interrupt nesting level
    nesting_level: u32,
    /// Initialization state
    initialized: bool,
}

impl ExceptionManager {
    /// Create a new exception manager
    pub const fn new() -> Self {
        Self {
            stats: ExceptionStats {
                total_exceptions: 0,
                undefined_instructions: 0,
                system_calls: 0,
                prefetch_aborts: 0,
                data_aborts: 0,
                irq_count: 0,
                fiq_count: 0,
                nested_interrupts: 0,
                max_nesting_level: 0,
            },
            nesting_level: 0,
            initialized: false,
        }
    }

    /// Install exception vector table
    fn install_vectors(&self) -> Result<(), &'static str> {
        unsafe extern "C" {
            static __vectors_start: u8;
            static __vectors_end: u8;
        }

        // SAFETY: We're installing exception vectors during initialization
        unsafe {
            let vectors_start = &__vectors_start as *const u8;
            let vectors_end = &__vectors_end as *const u8;
            let vector_size = vectors_end as usize - vectors_start as usize;

            // Vectors should be exactly 32 bytes (8 vectors * 4 bytes each)
            if vector_size != 32 {
                return Err("Invalid vector table size");
            }

            // Copy vectors to 0x00000000 (assuming low vectors)
            let dest = 0x00000000 as *mut u8;
            core::ptr::copy_nonoverlapping(vectors_start, dest, vector_size);

            // Ensure vectors are written
            crate::arch::barriers::dsb();
            crate::arch::barriers::isb();
        }

        Ok(())
    }

    /// Configure exception handling features
    fn configure_exception(&self) -> Result<(), &'static str> {
        // SAFETY: We're configuring exception handling
        unsafe {
            let mut sctlr: u32;

            // Read SCTLR
            asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));

            // Ensure low vectors (clear V bit)
            sctlr &= !(1 << 13);

            // Write back SCTLR
            asm!("mcr p15, 0, {}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }

        Ok(())
    }
}