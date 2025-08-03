//! Physical Memory Allocator
//! 
//! This module implements a buddy system allocator for managing physical memory.
//! The buddy system provides efficient allocation and deallocation with minimal
//! fragmentation, suitable for real-time systems requiring deterministic behavior.
//! 
//! # Algorithm
//! 
//! The buddy allocator works by:
//! 1. Dividing memory into power-of-2 sized blocks
//! 2. Maintaining free lists for each block size
//! 3. Splitting larger blocks when smaller blocks are needed
//! 4. Coalescing adjacent free blocks when possible
//! 
//! # Time Complexity
//! 
//! - Allocation: O(log n) where n is the number of orders
//! - Deallocation: O(log n)
//! - Worst case: O(1) for pre-allocated pools

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::ptr::NonNull;
use core::mem;
use heapless::{Vec, FnvIndexMap};
use crate::memory::{PhysicalAddress, MemoryResult, MemoryError, PAGE_SIZE, PAGE_SHIFT};
use crate::{KernelResult, KernelError};

/// Maximum allocation order (2^MAX_ORDER pages)
pub const MAX_ORDER: usize = 10; // Up to 4MB allocations

/// Minimum allocation order (single page)
pub const MIN_ORDER: usize = 0;

/// Number of different allocation orders
pub const NUM_ORDERS: usize = MAX_ORDER + 1;

/// Maximum number of memory zones
pub const MAX_ZONES: usize = 4;

/// Maximum number of reserved regions
pub const MAX_RESERVED_REGIONS: usize = 16;

/// Memory zone types for different usage patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MemoryZone {
    /// DMA-accessible memory (typically first 16MB)
    Dma = 0,
    /// Normal memory for general allocation
    Normal = 1,
    /// High memory (above 896MB on 32-bit systems)
    High = 2,
    /// Device memory (memory-mapped I/O)
    Device = 3,
}

/// Free block header in the buddy system
#[repr(C)]
struct FreeBlock {
    /// Next free block in the same order
    next: Option<NonNull<FreeBlock>>,
    /// Previous free block in the same order
    prev: Option<NonNull<FreeBlock>>,
    /// Order of this block
    order: u8,
    /// Zone this block belongs to
    zone: MemoryZone,
}

impl FreeBlock {
    /// Create a new free blcok
    fn new(order: u8, zone: MemoryZone) -> Self {
        Self {
            next: None,
            prev: None,
            order,
            zone,
        }
    }

    /// Get the size of this block in bytes
    fn size(&self) -> usize {
        PAGE_SIZE << self.order
    }

    /// Get the physical address of this block
    fn physical_address(&self) -> PhysicalAddress {
        PhysicalAddress::new(self as *const Self as usize)
    }
}

/// Memory zone descriptor
#[derive(Debug, Clone)]
struct Zone {
    /// Zone type
    zone_type: MemoryZone,
    /// Start address of the zone
    start_addr: PhysicalAddress,
    /// Size of the zone in bytes
    size: usize,
    /// Free lists for each order
    free_lists: [Option<NonNull<FreeBlock>>; NUM_ORDERS],
    /// Total pages in this zone
    total_pages: usize,
    /// Free pages in this zone
    free_pages: usize,
    /// Allocated pages in this zone
    allocated_pages: usize,
}

impl Zone {
    /// Create a new memory zone
    fn new(zone_type: MemoryZone, start_addr: PhysicalAddress, size: usize) -> Self {
        Self {
            zone_type,
            start_addr,
            size,
            free_lists: [None; NUM_ORDERS],
            total_pages: size / PAGE_SIZE,
            free_pages: size / PAGE_SIZE,
            allocated_pages: 0,
        }
    }

    /// Check if an address belongs to this zone
    fn contains(&self, addr: PhysicalAddress) -> bool {
        let start = self.start_addr.as_usize();
        let end = start + self.size;
        let addr_val = addr.as_usize();
        addr_val >= start && addr_val < end
    }
}