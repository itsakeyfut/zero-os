//! Zero OS Kernel Library
//! 
//! Core kernel functionality and public interfaces for the Zero OS.
//! This library provides the foundational abstractions for process management,
//! memory management, inter-process communication, and real-time scheduling.
//! 
//! # Architecture
//! 
//! The kernel follows a microkernel architecture inspired by Tock OS:
//! - Minimal kernel space with essential services only
//! - User space applications isolated through memory protection
//! - Capsule-based driver architecture for hardware abstraction
//! - Real-time scheduling with priority inheritance
//! - Capability-based security for safe resource access
//! 
//! # Safety
//! 
//! All unsafe operations are carefully documented and justified.
//! The kernel maintains memory safety through:
//! - Rust's type system for compile-time guarantees
//! - Runtime bounds checking where necessary
//! - Process isolation with hardware memory protection
//! - Capability-based access control for system resources

#![no_std]
#![no_main]
#![deny(unsafe_op_in_unsafe_fn)]
#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]
#![warn(clippy::panic)]
#![warn(clippy::unwrap_used)]

// Enable required unstable features
#![feature(panic_info_message)]
#![feature(alloc_error_handler)]
#![feature(core_intrinsics)]

// External crate imports
extern crate alloc;

// Re-export commonly used types and traits
pub use alloc::{boxed::Box, string::String, vec::Vec};

// Module declarations
pub mod macros;
pub mod arch;
pub mod memory;
pub mod process;
pub mod syscalls;
pub mod platform;
pub mod grants;
pub mod drivers;
pub mod safety;

// Public exports for kernel users
pub use arch::{Architecture, CpuContext, InterruptType, MemoryRegion};
pub use memory::{MemoryManager, VirtualAddress, PhysicalAddress, MemoryFlags};
pub use process::{ProcessManager, ProcessId, Process, Priority};
pub use platform::{Platform, HardwareCapabilities};
pub use grants::{Grant, GrantId, GrantPermissions, GrantCapability};

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