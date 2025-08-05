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
use crate::{debug_print, KernelError, KernelResult};

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

    /// Initialize the physical memory allocator
    pub fn init(
        &mut self,
        memory_map: &[(PhysicalAddress, usize, MemoryZone)],
        reserved: &[ReservedRegion],
    ) -> MemoryResult<()> {
        if self.initialized {
            return Err(MemoryError::AlreadyAllocated);
        }

        debug_print!(INFO, "Initializing physical memory allocator");

        // Add reserved regions
        for &region in reserved {
            self.reserved_regions.push(region)
                .map_err(|_| MemoryError::InvalidSize)?;
            debug_print!(DEBUG, "Reserved region: {:?} - {} bytes - {}", 
                        region.start, region.size, region.description);
        }

        // Initialize memory zones
        for &(start_addr, size, zone_type) in memory_map {
            self.add_zone(zone_type, start_addr, size)?;
        }

        // Calculate total and available memory
        self.calculate_memory_stats();

        // Initialize free lists for each zone
        self.initialize_free_lists()?;

        self.initialized = true;

        debug_print!(INFO, "Physical allocator initialized:");
        debug_print!(INFO, "  Total memory: {} MB", self.total_memory / (1024 * 1024));
        debug_print!(INFO, "  Available memory: {} MB", self.available_memory / (1024 * 1024));
        debug_print!(INFO, "  Zones: {}", self.zones.len());
        debug_print!(INFO, "  Reserved regions: {}", self.reserved_regions.len());

        Ok(())
    }

    /// Add a memory zone
    fn add_zone(
        &mut self,
        zone_type: MemoryZone,
        start_addr: PhysicalAddress,
        size: usize,
    ) -> MemoryResult<()> {
        // Align start address and size to page boundaries
        let aligned_start = start_addr.align_up();
        let aligned_size = (size & !(PAGE_SIZE - 1));

        if aligned_size < PAGE_SIZE {
            return Err(MemoryError::InvalidSize);
        }

        // Check for overlaps with existing zones
        for zone in &self.zones {
            let zone_start = zone.start_addr.as_usize();
            let zone_end = zone_start + zone.size;
            let new_start = aligned_start.as_usize();
            let new_end = new_start + aligned_size;

            if (new_start < zone_end) && (new_end > zone_start) {
                return Err(MemoryError::AlreadyAllocated);
            }
        }

        let zone = Zone::new(zone_type, aligned_start, aligned_size);
        self.zone.push(zone)
            .map_err(|_| MemoryError::InvalidSize)?;

        debug_print!(DEBUG, "Added {:?} zone: {:?} - {} bytes",
                    zone_type, aligned_start, aligned_size);

        Ok(())
    }

    /// Calculate memory statistics
    fn calculate_memory_stats(&mut self) {
        self.total_memory = self.zones.iter().map(|z| z.size).sum();

        // Calculate available memory (total minus reserved)
        let reserved_memory: usize = self.reserved_regions.iter().map(|r| r.size).sum();
        self.available_memory = self.total_memory.saturating_sub(reserved_memory);
    }

    /// Initialize free lists for all zones
    fn initialize_free_lists(&mut self) -> MemoryResult<()> {
        for zone in &mut self.zones {
            self.initialize_zone_free_lists(zone)?;
        }
        Ok(())
    }

    /// Initialize free lists for a specific zone
    fn initialize_zone_free_lists(&mut self, zone: &mut Zone) -> MemoryResult<()> {
        let mut current_addr = zone.start_addr;
        let zone_end = PhysicalAddress::new(zone.start_addr.as_usize() + zone.size);

        while current_addr < zone_end {
            // Skip reserved regions
            if self.is_reserved(current_addr) {
                current_addr = PhysicalAddress::new(current_addr.as_usize() + PAGE_SIZE);
                continue;
            }

            // Find the largest possible block size
            let remaining = zone_end.as_usize() - current_addr.as_usize();
            let max_order = self.find_max_order(current_addr, remaining);

            // Add block to appropriate free list
            self.add_to_free_list(zone, current_addr, max_order)?;

            // Move to next block
            let block_size = PAGE_SIZE << max_order;
            current_addr = PhysicalAddress::new(current_addr.as_usize() + block_size);
        }

        Ok(())
    }

    /// Check if an address is in a reserved region
    fn is_reserved(&self, addr: PhysicalAddress) -> bool {
        for region in &self.reserved_regions {
            let region_start = region.start.as_usize();
            let region_end = region_start + region.size;
            let addr_val = addr.as_usize();

            if addr_val >= region_start && addr_val < region_end {
                return true;
            }
        }
        false
    }

    /// Find the maximum order for a block at given address
    fn find_max_order(&self, addr: PhysicalAddress, max_size: usize) -> u8 {
        let addr_val = addr.as_usize();

        for order in (MIN_ORDER..=MAX_ORDER).rev() {
            let block_size = PAGE_SIZE << order;

            // Check if block fits in remaining space
            if block_size > max_size {
                continue;
            }

            // Check if address is properly aligned for this order
            if addr_val & (block_size - 1) == 0 {
                return order as u8;
            }
        }

        MIN_ORDER as u8
    }

    /// Add a block to the appropriate free list
    fn add_to_free_list(
        &mut self,zone: &mut Zone,
        addr: PhysicalAddress,
        order: u8,
    ) -> MemoryResult<()> {
        // SAFETY: We're initializing a new free block at a valid address
        unsafe {
            let block_ptr = addr.as_usize() as *mut FreeBlock;
            let block = &mut *block_ptr;
            *block = FreeBlock::new(order, zone.zone_type);

            // Add to head of free list
            let order_idx = order as usize;
            if let Some(mut current_head) = zone.free_lists[order_idx] {
                current_head.as_mut().prev = NonNull::new(block_ptr);
                block.next = Some(current_head);
            }
            zone.free_lists[order_idx] = NonNull::new(block_ptr);
        }

        Ok(())
    }

    /// Allocate physical memory pages
    pub fn allocate_pages(&mut self, order: u8, zone_type: MemoryZone) -> MemoryResult<PhysicalAddress> {
        if !self.initialized {
            return Err(MemoryError::InvalidAddress);
        }

        if order > MAX_ORDER as u8 {
            return Err(MemoryError::InvalidSize);
        }

        // Find a suitable zone
        let zone_idx = self.find_zone(zone_type)?;

        // Try to allocate from the specified order, or split a larger block
        if let Some(addr) = self.allocate_from_zone(zone_idx, order) {
            // Update statistics
            self.stats.total_allocations += 1;
            self.stats.active_allocations += 1;
            self.stats.allocations_per_order[order as usize] += 1;

            let allocated_bytes = PAGE_SIZE << order;
            self.stats.total_bytes_allocated += allocated_bytes as u64;
            self.allocated_memory += allocated_bytes;

            if self.allocated_memory > self.peak_allocated {
                self.peak_allocated = self.allocated_memory;
            }

            debug_print!(TRACE, "Allocated {} pages (order {}) at {:?}", 
                        1 << order, order, addr);

            Ok(addr)
        } else {
            self.stats.failed_allocations += 1;
            Err(MemoryError::OutOfMemory)
        }
    }

    /// Find a zone of the specified type
    fn find_zone(&self, zone_type: MemoryZone) -> MemoryResult<usize> {
        self.zones.iter().position(|zone| zone.zone_type == zone_type)
            .ok_or(MemoryError::InvalidAddress)
    }

    /// Allocate from a specific zone
    fn allocate_from_zone(&mut self, zone_idx: usize, order: u8) -> Option<PhysicalAddress> {
        // Try to find a block of the requested order
        for current_order in order..=MAX_ORDER as u8 {
            if let Some(block) = self.remove_from_free_list(zone_idx, current_order) {
                // If we got a larger block, split it down to the requested size
                return Some(self.split_block(zone_idx, block, current_order, order));
            }
        }

        None
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
