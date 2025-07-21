
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
    pub const DOMAIN_L2: u32 = 0x08;
    /// Permission fault (level 1)
    pub const PERMISSION_L1: u32 = 0x0D;
    /// Permission fault (level 2)
    pub const PERMISSION_L2: u32 = 0x0F;
}
