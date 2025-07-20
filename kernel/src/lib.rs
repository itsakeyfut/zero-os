//! Zero OS Kernel Library
//! 
//! Core kernel functionality and public interfaces for the Zero OS.
//! This library provides the foundational aabstractions for process management,
//! memory management, inter-process communication, and real-time scheduling.
//! 
//! # Architecture
//! 
//! The kernel follows a microkernel architecture inspired by Tock OS:
//! - Minimal kernel space with essential services only
//! - User space applications isolated through memory protection
//! - Capsule-based driver architecture for hardware abstraction
//! - Real-time scheduling with priority inheritance
//! 
//! # Safety
//! 
//! All unsafe operations are carefully documented and justified.
//! The kernel maintains memory safety through:
//! - Rust's type system for compile-time guarantees
//! - Runtime bounds checking where necessary
//! - Process isolation with hardware memory protection
//! - Capability-based access control

#![no_std]
#![no_main]
#![deny(unsafe_op_in_unsafe_fn)]
#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]
#![warn(clippy::panic)]
#![warn(clippy::unwrap_used)]

pub mod arch;
pub mod memory;
pub mod process;
pub mod syscalls;
pub mod platform;
pub mod grants;

/// Core kernel error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KernelError {
    /// Out of memory
    OutOfMemory,
    /// Invalid parameter provided
    InvalidParameter,
    /// Process not found
    ProcessNotFound,
    /// Resource is currently unavailable
    ResourceUnavailable,
    /// Permission denied
    PermissionDenied,
    /// Hardware error occurred
    HardwareError,
    /// IPC communication error
    IpcError,
    /// System in invalid state
    InvalidState,
}

/// Result type for kernel operations
pub type KernelResult<T> = Result<T, KernelError>;