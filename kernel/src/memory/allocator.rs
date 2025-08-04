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

    /// Get the buddy address for a given address and order
    fn buddy_address(&self, addr: PhysicalAddress, order: u8) -> PhysicalAddress {
        let block_size = PAGE_SIZE << order;
        let addr_val = addr.as_usize();
        let buddy_addr = addr_val ^ block_size;
        PhysicalAddress::new(buddy_addr)
    }

    /// Check if two blocks are buddies
    fn are_buddies(&self, addr1: PhysicalAddress, addr2: PhysicalAddress, order: u8) -> bool {
        self.buddy_address(addr1, order) == addr2
    }
}

/// Reserved memory region
#[derive(Debug, Clone, Copy)]
pub struct ReservedRegion {
    /// Start address
    pub start: PhysicalAddress,
    /// Size in bytes
    pub size: usize,
    /// Description of what this region is used for
    pub description: &'static str,
}

/// Physical memory allocator using buddy system
pub struct PhysicalAllocator {
    /// Memory zones
    zones: Vec<Zone, MAX_ZONES>,
    /// Reserved memory regions
    reserved_regions: Vec<ReservedRegion, MAX_RESERVED_REGIONS>,
    /// Total system memory
    total_memory: usize,
    /// Total available memory (excluding reserved)
    available_memory: usize,
    /// Total allocated memory
    allocated_memory: usize,
    /// Peak allocated memory
    peak_allocated: usize,
    /// Allocation statistics
    stats: AllocationStats,
    /// Initialization status
    initialized: bool,
}

/// Allocation statistics
#[derive(Debug, Default, Clone, Copy)]
pub struct AllocationStats {
    /// Total allocations
    pub total_allocations: u64,
    /// Total deallocations
    pub total_deallocations: u64,
    /// Current active allocations
    pub active_allocations: u64,
    /// Failed allocations
    pub failed_allocations: u64,
    /// Total bytes allocated
    pub total_bytes_allocated: u64,
    /// Total bytes deallocated
    pub total_bytes_deallocated: u64,
    /// Allocations per order
    pub allocations_per_order: [u64; NUM_ORDERS],
    /// Fragmentation ratio (0-100)
    pub fragmentation_ratio: u8,
}

impl PhysicalAllocator {
    /// Create a new physical memory allocator
    pub const fn new() -> Self {
        Self {
            zones: Vec::new(),
            reserved_regions: Vec::new(),
            total_memory: 0,
            available_memory: 0,
            allocated_memory: 0,
            peak_allocated: 0,
            stats: AllocationStats {
                total_allocations: 0,
                total_deallocations: 0,
                active_allocations: 0,
                failed_allocations: 0,
                total_bytes_allocated: 0,
                total_bytes_deallocated: 0,
                allocations_per_order: [0; NUM_ORDERS],
                fragmentation_ratio: 0,
            },
            initialized: false,
        }
    }
}

/// Memory usage information
#[derive(Debug, Clone)]
pub struct MemoryUsage {
    /// Total system memory
    pub total_memory: usize,
    /// Available memory (excluding reserved)
    pub available_memory: usize,
    /// Currently allocated memory
    pub allocated_memory: usize,
    /// Free memory
    pub free_memory: usize,
    /// Peak allocated memory
    pub peak_allocated: usize,
    /// Per-zone usage
    pub zones: Vec<ZoneUsage, MAX_ZONES>,
}

/// Zone-specific memory usage
#[derive(Debug, Clone, Copy)]
pub struct ZoneUsage {
    /// Zone type
    pub zone_type: MemoryZone,
    /// Total pages in zone
    pub total_pages: usize,
    /// Free pages in zone
    pub free_pages: usize,
    /// Allocated pages in zone
    pub allocated_pages: usize,
}
