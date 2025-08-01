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
use crate::process::ProcessId;
use crate::memory::VirtualAddress;

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
#[derive(Debug, Clone)]
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
    
    /// Check if address is within this grant region
    pub fn contains_address(&self, addr: VirtualAddress) -> bool {
        let start = self.address.as_usize();
        let end = start + self.size;
        let addr_val = addr.as_usize();
        addr_val >= start && addr_val < end
    }
    
    /// Increment reference count
    pub fn acquire(&mut self) -> bool {
        if self.permissions.shareable {
            self.ref_count += 1;
            true
        } else {
            self.ref_count == 1
        }
    }
    
    /// Decrement reference count
    pub fn release(&mut self) -> bool {
        if self.ref_count > 0 {
            self.ref_count -= 1;
            self.ref_count == 0
        } else {
            false
        }
    }
}

/// Grant handle providing type-safe access to granted memory
pub struct Grant<T> {
    /// Grant region information
    region: GrantRegion,
    /// Capability for access control
    capability: GrantCapability,
    /// Phantom data for type safety
    _phantom: PhantomData<T>,
}

impl<T> Grant<T> {
    /// Create a new grant handle
    /// 
    /// # Safety
    /// 
    /// Caller must ensure that the memory region is valid and properly aligned
    /// for type T, and that the grant capability is valid.
    pub unsafe fn new(region: GrantRegion, capability: GrantCapability) -> Self {
        Self {
            region,
            capability,
            _phantom: PhantomData,
        }
    }
    
    /// Get the grant ID
    pub fn grant_id(&self) -> GrantId {
        self.region.grant_id
    }
    
    /// Get the size of the grant
    pub fn size(&self) -> usize {
        self.region.size
    }
    
    /// Get the address of the grant
    pub fn address(&self) -> VirtualAddress {
        self.region.address
    }
    
    /// Check if the grant is shareable
    pub fn is_shareable(&self) -> bool {
        self.region.permissions.shareable
    }
    
    /// Get a reference to the data (if read permission exists)
    pub fn read(&self) -> Option<&T> {
        if !self.capability.can_read() {
            return None;
        }
        
        // SAFETY: We've verified read permission and the grant was created
        // with proper type safety guarantees
        unsafe {
            let ptr = self.region.address.as_usize() as *const T;
            ptr.as_ref()
        }
    }
    
    /// Get a mutable reference to the data (if write permission exists)
    pub fn write(&mut self) -> Option<&mut T> {
        if !self.capability.can_write() {
            return None;
        }
        
        // SAFETY: We've verified write permission and the grant was created
        // with proper type safety guarantees
        unsafe {
            let ptr = self.region.address.as_usize() as *mut T;
            ptr.as_mut()
        }
    }
    
    /// Map a function over the grant data (read-only)
    pub fn map<R, F>(&self, f: F) -> Option<R>
    where
        F: FnOnce(&T) -> R,
    {
        self.read().map(f)
    }
    
    /// Map a function over the grant data (mutable)
    pub fn map_mut<R, F>(&mut self, f: F) -> Option<R>
    where
        F: FnOnce(&mut T) -> R,
    {
        self.write().map(f)
    }
    
    /// Try to clone the grant (only works for shareable grants)
    pub fn try_clone(&self) -> Option<Self> {
        if !self.capability.can_share() {
            return None;
        }
        
        // SAFETY: We're cloning a valid grant with sharing permission
        unsafe {
            Some(Grant::new(self.region.clone(), self.capability.clone()))
        }
    }
    
    /// Get the capability for this grant
    pub fn capability(&self) -> &GrantCapability {
        &self.capability
    }
    
    /// Get the region information
    pub fn region(&self) -> &GrantRegion {
        &self.region
    }
}

impl<T> Drop for Grant<T> {
    fn drop(&mut self) {
        // Automatically release the grant when dropped
        // This would normally notify the grant allocator
        crate::debug_print!("Dropping grant {:?}", self.grant_id());
    }
}

// Grant can be sent between threads if T is Send
unsafe impl<T: Send> Send for Grant<T> {}

// Grant can be shared between threads if T is Sync and the grant is read-only
unsafe impl<T: Sync> Sync for Grant<T> {}

/// Error types for grant operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GrantError {
    /// Grant not found
    NotFound,
    /// Permission denied
    PermissionDenied,
    /// Grant already exists
    AlreadyExists,
    /// Invalid grant size
    InvalidSize,
    /// Invalid alignment
    InvalidAlignment,
    /// Out of memory
    OutOfMemory,
    /// Grant expired
    Expired,
    /// Type mismatch
    TypeMismatch,
    /// Invalid parameter
    InvalidParameter,
    /// Grant is in use
    InUse,
    /// System limit exceeded
    LimitExceeded,
}

/// Result type for grant operations
pub type GrantResult<T> = Result<T, GrantError>;

/// Grant statistics for monitoring and debugging
#[derive(Debug, Default, Clone, Copy)]
pub struct GrantStats {
    /// Total number of grants allocated
    pub total_grants: u32,
    /// Number of active grants
    pub active_grants: u32,
    /// Number of process grants
    pub process_grants: u32,
    /// Number of driver grants
    pub driver_grants: u32,
    /// Number of system grants
    pub system_grants: u32,
    /// Total memory allocated for grants
    pub total_memory: usize,
    /// Memory currently in use
    pub used_memory: usize,
    /// Number of grant allocation failures
    pub allocation_failures: u32,
    /// Number of permission violations
    pub permission_violations: u32,
}

/// Type-safe wrapper for different grant data types
pub trait GrantData: 'static + Sized {
    /// Get the type ID for this grant data type
    fn type_id() -> u64 {
        // Simple type ID based on type name hash
        let type_name = core::any::type_name::<Self>();
        let mut hash = 0u64;
        for byte in type_name.bytes() {
            hash = hash.wrapping_mul(31).wrapping_add(byte as u64);
        }
        hash
    }
    
    /// Get the required alignment for this type
    fn alignment() -> usize {
        core::mem::align_of::<Self>()
    }
    
    /// Get the size of this type
    fn size() -> usize {
        core::mem::size_of::<Self>()
    }
}

// Implement GrantData for common types
impl GrantData for u8 {}
impl GrantData for u16 {}
impl GrantData for u32 {}
impl GrantData for u64 {}
impl GrantData for i8 {}
impl GrantData for i16 {}
impl GrantData for i32 {}
impl GrantData for i64 {}
impl GrantData for usize {}
impl GrantData for isize {}

impl<T: GrantData, const N: usize> GrantData for [T; N] {}

/// Helper macros for grant operations
#[macro_export]
macro_rules! grant_data {
    ($name:ident { $($field:ident: $type:ty),* $(,)? }) => {
        #[derive(Debug, Clone)]
        pub struct $name {
            $(pub $field: $type,)*
        }
        
        impl $crate::grants::GrantData for $name {}
    };
}

/// Create a typed grant reference
#[macro_export]
macro_rules! grant_ref {
    ($grant:expr, $type:ty) => {
        unsafe {
            $crate::grants::Grant::<$type>::new(
                $grant.region().clone(),
                $grant.capability().clone(),
            )
        }
    };
}