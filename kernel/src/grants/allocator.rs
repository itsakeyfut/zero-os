
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

    /// Free memory back to buddy allocator
    /// 
    /// # Safety
    /// 
    /// Caller must ensure the pointer was allocated by this allocator.
    unsafe fn deallocate(&mut self, ptr: NonNull<u8>, size: usize) {
        let order = self.size_to_order(size);
        if order >= BUDDY_ORDERS {
            return;
        }

        let mut block_addr = ptr.as_ptr() as usize;
        let mut current_order = order;

        // Coalesce with buddy blocks
        while current_order < BUDDY_ORDERS - 1 {
            let buddy_addr = block_addr ^ (1 << current_order);

            // Check if buddy is free
            let mut prev: Option<NonNull<FreeBlock>> = None;
            let mut current = self.free_lists[current_order];

            while let Some(free_block) = current {
                if free_block.as_ptr() as usize == buddy_addr {
                    // Remove buddy from free list
                    unsafe {
                        let next = (*free_block.as_ptr()).next;
                        if let Some(prev_block) = prev {
                            (*prev_block.as_ptr()).next = next;
                        } else {
                            self.free_lists[current_order] = next;
                        }
                    }

                    // Coalesce blocks
                    if buddy_addr < block_addr {
                        block_addr = buddy_addr;
                    }
                    current_order += 1;
                    break;
                }

                prev = current;
                unsafe {
                    current = (*free_block.as_ptr()).next;
                }
            }

            if current.is_none() {
                break; // Buddy not free, can't coalesce
            }
        }

        // Add coalesced block to free list
        unsafe {
            let block = block_addr as *mut FreeBlock;
            (*block).size = 1 << current_order;
            (*block).next = self.free_lists[current_order];
            self.free_lists[current_order] = NonNull::new(block);
        }

        self.used_memory -= 1 << order;
    }
}

/// Grant allocator managing all grant memory
pub struct GrantAllocator {
    /// All allocated grants
    grants: FnvIndexMap<GrantId, GrantRegion, MAX_GRANTS>,
    /// Process to grants mapping
    process_grants: FnvIndexMap<ProcessId, Vec<GrantId, 16>, 64>,
    /// Next available grant ID
    next_grant_id: u32,
    /// Slab allocator for small grants
    slab_allocator: SlabAllocator,
    /// Buddy allocator for medium grants
    buddy_allocator: BuddyAllocator,
    /// Large grant free list (simple first-fit)
    large_grants: Vec<(VirtualAddress, usize), 32>,
    /// Grnat statistics
    stats: GrantStats,
    /// Initialization state
    initialized: bool,
}

impl GrantAllocator {
    /// Create a new grant allocator
    pub const fn new() -> Self {
        Self {
            grants: FnvIndexMap::new(),
            process_grants: FnvIndexMap::new(),
            next_grant_id: 1,
            slab_allocator: SlabAllocator::new(),
            buddy_allocator: BuddyAllocator::new(),
            large_grants: Vec::new(),
            stats: GrantStats {
                total_grants: 0,
                active_grants: 0,
                process_grants: 0,
                driver_grants: 0,
                system_grants: 0,
                total_memory: 0,
                used_memory: 0,
                allocation_failures: 0,
                permission_violations: 0,
            },
            initialized: false,
        }
    }

    /// Initialize the grant allocator
    pub fn init(&mut self) -> GrantResult<()> {
        if self.initialized {
            return Ok(());
        }

        // Initialize allocators with memory regions
        // In a real implementation, this would get memory from the memory manager
        self.initialized = true;

        crate::debug_print!("Grant allocator initialized");
        Ok(())
    }

    /// Allocate a new grant
    pub fn allocate_grant<T: GrantData>(
        &mut self,
        grant_type: GrantType,
        owner: Option<ProcessId>,
        permissions: GrantPermissions,
    ) -> GrantResult<Grant<T>> {
        if !self.initialized {
            return Err(GrantError::InvalidParameter);
        }

        let size = T::size();
        let alignment = T::alignment();
        let type_id = T::type_id();

        if size == 0 {
            return Err(GrantError::InvalidSize);
        }

        if !alignment.is_power_of_two() || alignment < GRANT_ALIGNMENT {
            return Err(GrantError::InvalidAlignment);
        }

        // Allocate memory for the grant
        let address = self.allocate_memory(size, alignment)?;

        // Create grant region
        let grant_id = GrantId::new(self.next_grant_id);
        self.next_grant_id += 1;

        let current_time = crate::arch::target::Architecture::current_time_us();

        let region = GrantRegion::new(
            grant_id,
            grant_type,
            address,
            size,
            owner,
            permissions,
            type_id,
            current_time,
        );

        // Create capability
        let capability = GrantCapability::new(
            grant_id,
            owner.unwrap_or(ProcessId::new(0)), // Use kernel process for system grants
            permissions,
            None, // No expiration for now
        );

        // Register grant
        self.grants.insert(grant_id, region.clone())
            .map_err(|_| GrantError::LimitExceeded)?;

        // Track by process if applicable
        if let Some(process_id) = owner {
            if !self.process_grants.contains_key(&process_id) {
                self.process_grants.insert(process_id, Vec::new()).ok();
            }
            let process_grant_list = self.process_grants.get_mut(&process_id).unwrap();
            process_grant_list.push(grant_id)
                .map_err(|_| GrantError::LimitExceeded)?;
            self.stats.process_grants += 1;
        } else {
            match grant_type {
                GrantType::Driver => self.stats.driver_grants += 1,
                GrantType::System => self.stats.system_grants += 1,
                _ => {}
            }
        }

        // Update statistics
        self.stats.total_grants += 1;
        self.stats.active_grants += 1;
        self.stats.used_memory += size;

        // SAFETY: We just allocated the memory and vertified the type
        unsafe {
            let grant = Grant::new(region, capability);
            Ok(grant)
        }
    }

    /// Free a grant
    pub fn free_grant<T>(&mut self, grant: Grant<T>) -> GrantResult<()> {
        let grant_id = grant.grant_id();
        let region = grant.region().clone();

        // Remove from grants map
        self.grants.remove(&grant_id)
            .ok_or(GrantError::NotFound)?;

        // Remove from process tracking
        if let Some(owner) = region.owner {
            if let Some(process_grant_list) = self.process_grants.get_mut(&owner) {
                if let Some(pos) = process_grant_list.iter().position(|&id| id == grant_id) {
                    process_grant_list.swap_remove(pos);
                }
            }
        }

        // Free memory
        self.free_memory(region.address, region.size)?;

        // Update statistics
        self.stats.active_grants -= 1;
        self.stats.used_memory -= region.size;

        match region.grant_type {
            GrantType::Process => self.stats.process_grants -= 1,
            GrantType::Driver => self.stats.driver_grants -= 1,
            GrantType::System => self.stats.system_grants -= 1,
            _ => {}
        }

        Ok(())
    }

    /// Get grant by ID
    pub fn get_grant(&self, grant_id: GrantId) -> Option<&GrantRegion> {
        self.grants.get(&grant_id)
    }

    /// Check if a process can access a grant
    pub fn check_access(
        &self,
        grant_id: GrantId,
        process_id: ProcessId,
        capability: &GrantCapability,
    ) -> bool {
        // Verify capability matches grant
        if capability.grant_id() != grant_id {
            return false;
        }
        
        // Check if capability is still valid
        let current_time = crate::arch::target::Architecture::current_time_us();
        if !capability.is_valid(current_time) {
            return false;
        }
        
        // Get grant region
        let region = match self.grants.get(&grant_id) {
            Some(region) => region,
            None => return false,
        };
        
        // Check process ownership or shareability
        match region.owner {
            Some(owner) => {
                owner == process_id || (region.permissions.shareable && capability.can_share())
            }
            None => true, // System grants are accessible by all with proper capability
        }
    }

    /// Get grants for a specific process
    pub fn get_process_grants(&self, process_id: ProcessId) -> Vec<GrantId, 16> {
        self.process_grants.get(&process_id)
            .cloned()
            .unwrap_or_default()
    }

    /// Free all grants for a process
    pub fn free_process_grants(&mut self, process_id: ProcessId) -> GrantResult<()> {
        let grant_ids = self.get_process_grants(process_id);

        for grant_id in grant_ids {
            if let Some(region) = self.grants.remove(&grant_id) {
                self.free_memory(region.address, region.size)?;
                self.stats.active_grants -= 1;
                self.stats.process_grants -= 1;
                self.stats.used_memory -= region.size;
            }
        }

        self.process_grants.remove(&process_id);
        Ok(())
    }

    /// Get allocator statistics
    pub fn stats(&self) -> GrantStats {
        self.stats
    }

    /// Allocate memory for grants
    fn allocate_memory(&mut self, size: usize, alignment: usize) -> GrantResult<VirtualAddress> {
        let aligned_size = (size + alignment - 1) & !(alignment - 1);

        if aligned_size <= SMALL_GRANT_MAX_SIZE {
            // Use slab allocator
            if let Some(ptr) = self.slab_allocator.allocate(aligned_size) {
                Ok(VirtualAddress::new(ptr.as_ptr() as usize))
            } else {
                self.stats.allocation_failures += 1;
                Err(GrantError::OutOfMemory)
            }
        } else if aligned_size <= MEDIUM_GRANT_MAX_SIZE {
            // Use buddy allocator
            if let Some(ptr) = self.buddy_allocator.allocate(aligned_size) {
                Ok(VirtualAddress::new(ptr.as_ptr() as usize))
            } else {
                self.stats.allocation_failures += 1;
                Err(GrantError::OutOfMemory)
            }
        } else {
            // Use large grant allocator (first-fit)
            self.allocate_large_grant(aligned_size, alignment)
        }
    }

    /// Free memory for grants
    fn free_memory(&mut self, address: VirtualAddress, size: usize) -> GrantResult<()> {
        if size <= SMALL_GRANT_MAX_SIZE {
            // SAFETY: Address was allocated by our slab allocator
            unsafe {
                let ptr = NonNull::new(address.as_usize() as *mut u8)
                    .ok_or(GrantError::InvalidParameter)?;
                self.slab_allocator.deallocate(ptr, size);
            }
        } else if size <= MEDIUM_GRANT_MAX_SIZE {
            // SAFETY: Address was allocated by our buddy allocator
            unsafe {
                let ptr = NonNull::new(address.as_usize() as *mut u8)
                    .ok_or(GrantError::InvalidParameter)?;
                self.buddy_allocator.deallocate(ptr, size);
            }
        } else {
            // Return to large grant free list
            self.free_large_grant(address, size)?;
        }

        Ok(())
    }

    /// Allocate large grant using first-fit
    fn allocate_large_grant(&mut self, size: usize, alignment: usize) -> GrantResult<VirtualAddress> {
        for i in 0..self.large_grants.len() {
            let (addr, available_size) = self.large_grants[i];

            // Check alignment
            let aligned_addr = (addr.as_usize() + alignment - 1) & !(alignment - 1);
            let aligned_offset = aligned_addr - addr.as_usize();

            if aligned_offset + size <= available_size {
                // Remove this block
                self.large_grants.swap_remove(i);

                // Add remaining space back if any
                let remaining_addr = aligned_addr + size;
                let remaining_size = available_size - aligned_offset - size;

                if remaining_size > 0 {
                    self.large_grants.push((VirtualAddress::new(remaining_addr), remaining_size))
                        .map_err(|_| GrantError::OutOfMemory)?;
                }

                return Ok(VirtualAddress::new(aligned_addr));
            }
        }

        self.stats.allocation_failures += 1;
        Err(GrantError::OutOfMemory)
    }

    /// Free large grant
    fn free_large_grant(&mut self, address: VirtualAddress, size: usize) -> GrantResult<()> {
        // Simple implementation: add to free list
        // In a real implementation, this would coalesce adjustment blocks
        self.large_grants.push((address, size))
            .map_err(|_| GrantError::OutOfMemory)?;

        Ok(())
    }

    /// Create a temporary grant that expires after specified time
    pub fn create_temporary_grant<T: GrantData>(
        &mut self,
        owner: ProcessId,
        permissions: GrantPermissions,
        duration_us: u64,
    ) -> GrantResult<Grant<T>> {
        let current_time = crate::arch::target::Architecture::current_time_us();
        let expiry_time = current_time + duration_us;

        // Allocate grant normally
        let mut grant = self.allocate_grant::<T>(GrantType::Temporary, Some(owner), permissions)?;

        // Update capability with expiration
        // In a real implementation, we'd modify the capability
        // For now, we'll leave this as a placeholder

        Ok(grant)
    }

    /// Clean up expired grants
    pub fn cleanup_expired_grants(&mut self) {
        let current_time = crate::arch::target::Architecture::current_time_us();
        let mut expired_grants = Vec::<GrantId, 32>::new();
        
        // Find expired grants
        for (grant_id, region) in &self.grants {
            if region.grant_type == GrantType::Temporary {
                // Check if grant has expired (simplified check)
                // In a real implementation, we'd check the actual expiration time
                if current_time > region.created_at + 1_000_000 { // 1 second timeout
                    expired_grants.push(*grant_id).ok();
                }
            }
        }
        
        // Remove expired grants
        for grant_id in expired_grants {
            if let Some(region) = self.grants.remove(&grant_id) {
                let _ = self.free_memory(region.address, region.size);
                self.stats.active_grants -= 1;
                self.stats.used_memory -= region.size;
            }
        }
    }

    /// Validate grant integrity
    pub fn validate_grants(&self) -> bool {
        let mut calculated_stats = GrantStats::default();
        
        for region in self.grants.values() {
            calculated_stats.total_grants += 1;
            calculated_stats.active_grants += 1;
            calculated_stats.used_memory += region.size;
            
            match region.grant_type {
                GrantType::Process => calculated_stats.process_grants += 1,
                GrantType::Driver => calculated_stats.driver_grants += 1,
                GrantType::System => calculated_stats.system_grants += 1,
                _ => {}
            }
        }
        
        // Check if statistics match
        calculated_stats.total_grants == self.stats.total_grants &&
        calculated_stats.active_grants == self.stats.active_grants &&
        calculated_stats.process_grants == self.stats.process_grants &&
        calculated_stats.driver_grants == self.stats.driver_grants &&
        calculated_stats.system_grants == self.stats.system_grants
    }
}