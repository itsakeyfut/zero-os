//! ARM Architecture Implementation
//! 
//! This module provides ARM Cortex-A specific implementations of the architecture
//! abstraction layer. It supports ARMv7-A architecture with features commonly
//! found in embedded systems and development boards.
//! 
//! # Supported Features
//! 
//! - ARM Cortex-A series processors (ARM1176, Cortex-A7, Cortex-A9, etc.)
//! - Memory Management Unit (MMU) with virtual memory support
//! - Generic Interrupt Controller (GIC)
//! - ARM Generic Timer
//! - Cache management (L1 I-cache, D-cache)
//! - TrustZone security extensions (basic support)
//! 
//! # Memory Layout
//! 
//! The ARM implementations assumes a standard embedded system memory layout:
//! - 0x00000000-0x0FFFFFFF: Boot ROM and low memory
//! - 0x10000000-0x1FFFFFFF: I/O peripherals 
//! - 0x20000000-0x3FFFFFFF: RAM
//! - 0x40000000+: High memory and additional peripherals
//! 
//! # Exception Vector Table
//! 
//! The ARM exception vectors are located at 0x00000000 (or high vectors at oxFFFF0000):
//! - 0x00: Reset
//! - 0x04: Undefined Instruction
//! - 0x08: Software Interrupt (SWI/SVC)
//! - 0x0C: Prefetch Abort
//! - 0x10: Data Abort
//! - 0x14: Reserved
//! - 0x18: IRQ
//! - 0x1C: FIQ

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::fmt;
use core::arch::asm;
use super::{ArchError, ArchResult, Architecture, CpuContext, InterruptType, MemoryRegion};

/// ARM-specific debug writer implementation
pub struct DebugWriterImpl {
    /// UART base address for debug output
    uart_base: usize,
}
