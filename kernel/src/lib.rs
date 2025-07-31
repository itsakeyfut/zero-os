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
#[repr(u32)]
pub enum KernelError {
    /// Out of memory
    OutOfMemory = 1,
    /// Invalid parameter provided
    InvalidParameter = 2,
    /// Process not found
    ProcessNotFound = 3,
    /// Resource is currently unavailable
    ResourceUnavailable = 4,
    /// Permission denied
    PermissionDenied = 5,
    /// Hardware error occurred
    HardwareError = 6,
    /// IPC communication error
    IpcError = 7,
    /// System in invalid state
    InvalidState = 8,
    /// Timeout occurred
    Timeout = 9,
    /// Operation interrupted
    Interrupted = 10,
    /// Resource limit exceeded
    ResourceLimitExceeded = 11,
    /// Unsupported operation
    UnsupportedOperation = 12,
    /// System call error
    SystemCallError = 13,
    /// Scheduler error
    SchedulerError = 14,
    /// Memory management error
    MemoryError = 15,
    /// Driver error
    DriverError = 16,
    /// Safety violation
    SafetyViolation = 17,
}

impl KernelError {
    /// Convert kernel error to system call error code
    pub fn to_syscall_error(self) -> u32 {
        self as u32
    }

    /// Check if error is recoverable
    pub fn is_recoverable(self) -> bool {
        match self {
            KernelError::OutOfMemory => false,
            KernelError::HardwareError => false,
            KernelError::SafetyViolation => false,
            KernelError::InvalidState => false,
            _ => true,
        }
    }

    /// Get error severity level
    pub fn severity(self) -> ErrorSeverity {
        match self {
            KernelError::OutOfMemory |
            KernelError::HardwareError |
            KernelError::SafetyViolation => ErrorSeverity::Critical,

            KernelError::ProcessNotFound |
            KernelError::PermissionDenied |
            KernelError::InvalidState => ErrorSeverity::Major,
            
            KernelError::ResourceUnavailable |
            KernelError::Timeout |
            KernelError::Interrupted => ErrorSeverity::Minor,
            
            _ => ErrorSeverity::Warning,
        }
    }
}

/// Error severity levels for kernel errors
#[derive(Debug, Clonem, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ErrorSeverity {
    /// Critical error - system may be unstable
    Critical = 0,
    /// Major error - significant functionality impacted
    Major = 1,
    /// Minor error - limited impact
    Minor = 2,
    /// Warning - potential issue
    Warning = 3,
}

/// Result type for kernel operations
pub type KernelResult<T> = Result<T, KernelError>;