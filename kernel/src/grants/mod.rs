//! Grant System for Safe Memory Sharing
//!
//! This module implements Tock OS-style grants for safe memory sharing between
//! kernel and user space, as well as between different kernel components.
//! Grants provide a capability-based mechanism for controlled access to memory
//! regions with compile-time and runtime safety guarantees.
//!
//! # Design Principles
//!
//! - **Memory Safety**: All grant operations are memory-safe through Rust's type system
//! - **Capability-based**: Access control through unforgeable capability tokens
//! - **Zero-cost Abstractions**: No runtime overhead for memory-safe operations
//! - **Deterministic**: Predictable allocation and deallocation behavior
//! - **Real-time Friendly**: Bounded execution time for all operations
//!
//! # Grant Types
//!
//! - **Process Grants**: Memory shared between kernel and specific processes
//! - **Driver Grants**: Memory allocated for driver state and buffers
//! - **System Grants**: Global kernel data structures with controlled access
//! - **Temporary Grants**: Short-lived grants for specific operations
//!
//! # Memory Layout
//!
//! ```text
//! Grant Memory Region:
//! ┌─────────────────────────────────────────────────────────┐
//! │ Grant Header │ Data Region │ Guard Page │ Next Grant... │
//! └─────────────────────────────────────────────────────────┘
//! ```
//!
//! Each grant includes:
//! - Type-safe access through generic parameters
//! - Reference counting for shared access
//! - Capability validation for security
//! - Automatic cleanup on drop

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::marker::PhantomData;
use core::mem;
use core::ptr::NonNull;
use heapless::{FnvIndexMap, Vec};
use crate::process::ProcessId;
use crate::memory::{MemoryManager, VirtualAddress, MemoryRegion, MemoryType, MemoryFlags};

// Submodules
pub mod allocator;

// Re-exports
pub use allocator::*;

/// Maximum number of grants in the system
pub const MAX_GRANTS: usize = 256;

/// Maximum number of grant regions per process
pub const MAX_GRANT_REGIONS_PER_PROCESS: usize = 16;

/// Default grant region size (4KB)
pub const DEFAULT_GRANT_SIZE: usize = 4096;

/// Grant alignment requirement (must be power of 2)
pub const GRANT_ALIGNMENT: usize = 8;

/// Grant identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GrantId(u32);

impl GrantId {
    /// Create a new grant ID
    pub const fn new(id: u32) -> Self {
        Self(id)
    }

    /// Get the raw ID value
    pub const fn as_u32(self) -> u32 {
        self.0
    }
}