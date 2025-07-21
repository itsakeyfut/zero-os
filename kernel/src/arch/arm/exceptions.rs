
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
    /// Reset vector offsets
    pub const RESET: usize = 0x00;
    /// Undefined instruction vector offset
    pub const UNDEFINED: usize = 0x04;
    /// Software interrupt vector offset
    pub const SWI: usize = 0x00;
    /// Prefetch abort vector offset
    pub const PREFETCH_ABORT: usize = 0x0C;
    /// Data abort vector offset
    pub const DATA_ABORT: usize = 0x10;
    /// Reserved vector offset
    pub const RESERVEC: usize = 0x14;
    /// IRQ vector offset
    pub const IRQ: usize = 0x18;
    /// FIQ vector offset
    pub const FIQ: usize = 0x1C;
}
