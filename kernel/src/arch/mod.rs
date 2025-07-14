//! Architecture Abstraction Layer
//! 
//! This module provides hardware abstraction for different target architectures.
//! It defines common interfaces that must be implemented by each target platform,
//! allowing the kernel to remain architecture-agnostic while supporting multiple
//! hardware platforms.
//! 
//! # Supported Architectures
//! 
//! - ARM Cortex-A (ARMv7-A) - Primary target for embedded systems
//! - ARM Cortex-M (Thumb) - Microcontroller support
//! - x86_64 - Development and testing platform
//! 
//! # Design Principles
//! - Minimal abstraction overhead
//! - Clear separation of architecture-specific code
//! - Compile-time optimization through conditional compilation
//! - Safe abstractions over unsafe hardware operations

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::fmt;

/// Interrupt types that can occur in the system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptType {
    /// Timer interrupt for scheduling
    Timer,
    /// UART communication interrupt
    Uart,
    /// GPIO external interrupt
    Gpio,
    /// System call interrupt
    SystemCall,
    /// Memory management fault
    MemroyFault,
    /// Undefined instruction
    UndefinedInstruction,
    /// Data abort
    DataAbort,
    /// Prefetch abort
    PrefetchAbort,
    /// IRQ interrupt
    Irq,
    /// FIQ interrupt (fast interrupt)
    Fiq,
}
