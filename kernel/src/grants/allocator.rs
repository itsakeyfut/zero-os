
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

impl SlabAllocator {
    /// Create a new slab allocator
    const fn new() -> Self {
        Self {
            free_lists: [None; SLAB_SIZE_CLASSES],
            total_memory: 0,
            used_memory: 0,
        }
    }

    /// Find size class for allocation size
    fn size_class(size: usize) -> Option<usize> {
        SLAB_SIZES.iter().position(|&s| s >= size)
    }

    /// Allocate memory from slab
    fn allocate(&mut self, size: usize) -> Option<NonNull<u8>> {
        let class = Self::size_class(size)?;
        let actual_size = SLAB_SIZES[class];

        if let Some(ptr) = self.free_lists[class] {
            // SAFETY: We maintain the invariant that free_lists contains valid pointers
            unsafe {
                let next = *(ptr.as_ptr() as *const Option<NonNull<u8>>);
                self.free_lists[class] = next;
                self.used_memory += actual_size;
                Some(ptr)
            }
        } else {
            // Would allocate new slab here in a real implementation
            None
        }
    }

    /// Free memory back to slab
    /// 
    /// # Safety
    /// 
    /// Caller must ensure the pointer was allocated by this allocator
    /// and size matches the original allocation.
    unsafe fn deallocate(&mut self, ptr: NonNull<u8>, size: usize) {
        if let Some(class) = Self::size_class(size) {
            let actual_size = SLAB_SIZES[class];

            // SAFETY: Caller guarantees pointer validity
            unsafe {
                *(ptr.as_ptr() as *mut Option<NonNull<u8>>) = self.free_lists[class];
                self.free_lists[class] = Some(ptr);
                self.used_memory -= actual_size;
            }
        }
    }
}

/// Buddy allocator for medium-sized grants
struct BuddyAllocator {
    /// Free lists for each order
    free_lists: [Option<NonNull<FreeBlock>>; BUDDY_ORDERS],
    /// Base address of allocator memory
    base_address: VirtualAddress,
    /// Total size of allocator memory
    total_size: usize,
    /// Memory currently in use
    used_memory: usize,
}

impl BuddyAllocator {
    /// Create a new buddy allocator
    const fn new() -> Self {
        Self {
            free_lists: [None; BUDDY_ORDERS],
            base_address: VirtualAddress::new(0),
            total_size: 0,
            used_memory: 0,
        }
    }

    /// Initialize buddy allocator with memory region
    fn init(&mut self, base: VirtualAddress, size: usize) {
        self.base_address = base;
        self.total_size = size;

        // Add initial block to largest order
        let order = self.size_to_order(size);
        if order < BUDDY_ORDERS {
            // SAFETY: We're initializing with a valid memory region
            unsafe {
                let block = base.as_usize() as *mut FreeBlock;
                (*block).size = 1 << order;
                (*block).next = None;
                self.free_lists[order] = NonNull::new(block);
            }
        }
    }

    /// Convert size to buddy order
    fn size_to_order(&self, size: usize) -> usize {
        let mut order = 0;
        let mut block_size = 1;
        
        while block_size < size && order < BUDDY_ORDERS {
            block_size <<= 1;
            order += 1;
        }
        
        order
    }

    /// Allocate memory from buddy allocator
    fn allocate(&mut self, size: usize) -> Option<NonNull<u8>> {
        let order = self.size_to_order(size);
        if order >= BUDDY_ORDERS {
            return None;
        }
        
        // Find a free block of sufficient size
        for current_order in order..BUDDY_ORDERS {
            if let Some(block) = self.free_lists[current_order] {
                // SAFETY: We maintain the invariant that free_lists contains valid pointers
                unsafe {
                    let next = (*block.as_ptr()).next;
                    self.free_lists[current_order] = next;
                    
                    // Split block if necessary
                    let current_block = block;
                    for split_order in (order..current_order).rev() {
                        let buddy_addr = current_block.as_ptr() as usize + (1 << split_order);
                        let buddy = buddy_addr as *mut FreeBlock;
                        
                        (*buddy).size = 1 << split_order;
                        (*buddy).next = self.free_lists[split_order];
                        self.free_lists[split_order] = NonNull::new(buddy);
                    }
                    
                    self.used_memory += 1 << order;
                    return Some(current_block.cast());
                }
            }
        }
        
        None
    }
}