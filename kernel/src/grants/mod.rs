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

    /// Invalid grant ID
    pub const INVALID: GrantId = GrantId(0);
}

/// Grant types for different use cases
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GrantType {
    /// Process-specific grant (isolated per process)
    Process,
    /// Driver grant (shared among driver instances)
    Driver,
    /// System grant (global kernel data)
    System,
    /// Temporary grant (short-lived)
    Temporary,
}

/// Grant access permissions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GrantPermissions {
    /// Read permission
    pub read: bool,
    /// Write permission
    pub write: bool,
    /// Execute permission (rarely used)
    pub execute: bool,
    /// Allow sharing with other processes
    pub shareable: bool,
}

impl GrantPermissions {
    /// Read-only permissions
    pub const fn read_only() -> Self {
        Self {
            read: true,
            write: false,
            execute: false,
            shareable: false,
        }
    }

    /// Read-write permissions
    pub const fn read_write() -> Self {
        Self {
            read: true,
            write: true,
            execute: false,
            shareable: false,
        }
    }

    /// Shareable read-only permissions
    pub const fn shareable_read_only() -> Self {
        Self {
            read: true,
            write: false,
            execute: false,
            shareable: true,
        }
    }

    /// Shareable read-write permissions
    pub const fn shareable_read_write() -> Self {
        Self {
            read: true,
            write: true,
            execute: false,
            shareable: true,
        }
    }
}

/// Grant capability token for access control
pub struct GrantCapability {
    /// Grant ID this capability provides access to
    grant_id: GrantId,
    /// Process that owns this capability
    process_id: ProcessId,
    /// Permissions granted
    permissions: GrantPermissions,
    /// Expiration time (None for permanent)
    expires_at: Option<u64>,
}

impl GrantCapability {
    /// Create a new grant capability
    pub fn new(
        grant_id: GrantId,
        process_id: ProcessId,
        permissions: GrantPermissions,
        expires_at: Option<u64>,
    ) -> Self {
        Self {
            grant_id,
            process_id,
            permissions,
            expires_at,
        }
    }

    /// Check if capability is valid
    pub fn is_valid(&self, current_time: u64) -> bool {
        match self.expires_at {
            Some(expiry) => current_time < expiry,
            None => true,
        }
    }

    /// Check if capability allows read access
    pub fn can_read(&self) -> bool {
        self.permissions.read
    }

    /// Check if capability allows write access
    pub fn can_write(&self) -> bool {
        self.permissions.write
    }

    /// Check if capability allows sharing
    pub fn can_share(&self) -> bool {
        self.permissions.shareable
    }

    /// Get the grant ID
    pub fn grant_id(&self) -> GrantId {
        self.grant_id
    }

    /// Get the process ID
    pub fn process_id(&self) -> ProcessId {
        self.process_id
    }
}

/// Grant region metadata
#[derive(Debug, Clone)]
pub struct GrantRegion {
    /// Grant identifier
    pub grant_id: GrantId,
    /// Grant type
    pub grant_type: GrantType,
    /// Virtual address of the grant
    pub address: VirtualAddress,
    /// Size of the grant in bytes
    pub size: usize,
    /// Process that owns this grant (None for system grants)
    pub owner: Option<ProcessId>,
    /// Current permissions
    pub permissions: GrantPermissions,
    /// Reference count for shared grants
    pub ref_count: u32,
    /// Type ID for type safety
    pub type_id: u64,
    /// Creation timestamp
    pub created_at: u64,
}

impl GrantRegion {
    /// Create a new grant region
    pub fn new(
        grant_id: GrantId,
        grant_type: GrantType,
        address: VirtualAddress,
        size: usize,
        owner: Option<ProcessId>,
        permissions: GrantPermissions,
        type_id: u64,
        created_at: u64,
    ) -> Self {
        Self {
            grant_id,
            grant_type,
            address,
            size,
            owner,
            permissions,
            ref_count: 1,
            type_id,
            created_at,
        }
    }
}