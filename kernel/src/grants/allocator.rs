
//! Grant Allocator Implementation
//!
//! This module provides the core allocator for the grant system, managing
//! allocation and deallocation of grant regions with type safety and
//! capability-based access control.
//!
//! # Design Features
//!
//! - **Deterministic Allocation**: Fixed-time allocation for real-time systems
//! - **Memory Safety**: Type-safe grant creation and access
//! - **Capability Security**: Access control through unforgeable tokens
//! - **Reference Counting**: Safe sharing of grant regions
//! - **Automatic Cleanup**: Resource management with RAII
//!
//! # Allocation Strategy
//!
//! The allocator uses a segregated free list approach:
//! - Small grants (≤ 64 bytes): Slab allocator
//! - Medium grants (≤ 4KB): Buddy allocator
//! - Large grants (> 4KB): First-fit allocator
//!
//! This provides good performance characteristics for different grant sizes
//! while maintaining deterministic behavior.

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::ptr::NonNull;
use core::mem;
use heapless::{FnvIndexMap, Vec, binary_heap::{BinaryHeap, Min}};
use crate::memory::{MemoryManager, VirtualAddress, MemoryRegion, MemoryType, MemoryFlags};
use crate::process::ProcessId;
use super::{
    Grant, GrantId, GrantType, GrantPermissions, GrantCapability, GrantRegion,
    GrantData, GrantError, GrantResult, GrantStats, MAX_GRANTS, GRANT_ALIGNMENT,
    DEFAULT_GRANT_SIZE,
};

/// Maximum size for small grants (slab allocated)
const SMALL_GRANT_MAX_SIZE: usize = 64;

/// Maximum size for medium grants (buddy allocated)
const MEDIUM_GRANT_MAX_SIZE: usize = 4096;

/// Number of size classes for slab allocator
const SLAB_SIZE_CLASSES: usize = 8;

/// Slab size class boundaries
const SLAB_SIZES: [usize; SLAB_SIZE_CLASSES] = [8, 16, 24, 32, 40, 48, 56, 64];

/// Number of buddy allocator orders
const BUDDY_ORDERS: usize = 12; // Up to 4KB

/// Free block header for buddy allocator
#[repr(C)]
struct FreeBlock {
    /// Size of this block
    size: usize,
    /// Pointer to next free block
    next: Option<NonNull<FreeBlock>>,
}

/// Slab allocator for small grants
struct SlabAllocator {
    /// Free lists for each size class
    free_lists: [Option<NonNull<u8>>; SLAB_SIZE_CLASSES],
    /// Total memory allocated for slabs
    total_memory: usize,
    /// Memory currently in use
    used_memory: usize,
}
