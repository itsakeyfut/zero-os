//! Process Management Subsystem
//! 
//! This module provides comprehensive process management for the Zero OS.
//! It implements a capability-based process system with the following features:
//! 
//! - Process creation and lifecycle management
//! - Memory isolation through virtual address spaces
//! - Inter-process communication (IPC) channels.
//! - Real-time scheduling support
//! - Security through capability-based access control
//! - Resource management and limits
//! 
//! # Design Principles
//! 
//! - Process isolation for security and reliability
//! - Capability-based security model (inspired by Tock OS)
//! - Real-time deterministic process switching
//! - Minimal kernel overhead
//! - Support for both kernel and user processes
//! 
//! # Process States
//! 
//! ```text
//! Created -> Ready -> Running -> Blocked -> Ready
//!     |         |        |         |
//!     |         |        v         |
//!     |         |    Terminated <--+
//!     |         |        ^
//!     |         +--------+
//!     +------------------+
//! ```

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::sync::atomic::{AtomicU32, Ordering};
use heapless::{Vec, FnvIndexMap};
use crate::memory::{MemoryManager, MemoryRegion, VirtualAddress, MemoryType, MemoryFlags};
use crate::arch::{CpuContext, ArchResult};

/// Maximum number of processes in the system
pub const MAX_PROCESSES: usize = 64;

/// Maximum number of memory regions per process
pub const MAX_MEMORY_REGIONS: usize = 16;

/// Maximum number of capabilities per process
pub const MAX_CAPABILITIES: usize = 32;

/// Default user stack size (64KB)
pub const DEFAULT_USER_STACK_SIZE: usize = 64 * 1024;

/// Default user heap size (1MB)
pub const DEFAULT_USER_HEAP_SIZE: usize = 1024 * 1024;

/// Process identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ProcessId(u32);

impl ProcessId {
    /// Create a new process ID
    pub const fn new(id: u32) -> Self {
        Self(id)
    }

    /// Get the raw ID value
    pub const fn as_u32(self) -> u32 {
        self.0
    }

    /// Special process ID for the kernel
    pub const KERNEL: ProcessId = ProcessId(0);

    /// Special process ID for the init process
    pub const INIT: ProcessId = ProcessId(1);
}

/// Process states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessState {
    /// Process has been created but not yet scheduled
    Created,
    /// Process is ready to run
    Ready,
    /// Process is currently running
    Running,
    /// Process is blocked waiting for something
    Blocked,
    /// Process has terminated
    Terminated,
    /// Process encountered a fault
    Faulted,
}

/// Process priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    /// Real-time critical priority
    RealTimeCritical = 0,
    /// Real-time high priority
    RealTimeHigh = 1,
    /// Real-time normal priority
    RealTimeNormal = 2,
    /// Normal priority
    Normal = 3,
    /// Low priority
    Low = 4,
    /// Background priority
    Background = 5,
}

impl Default for Priority {
    fn default() -> Self {
        Priority::Normal
    }
}
