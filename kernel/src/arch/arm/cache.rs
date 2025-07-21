//! ARM Cache Management
//!
//! This module provides comprehensive cache management for ARM Cortex-A processors.
//! It handles both instruction and data caches, including cache maintenance
//! operations required for memory coherency and performance optimization.
//!
//! # Cache Architecture
//!
//! ARM Cortex-A processors typically have:
//! - **L1 Instruction Cache**: Per-core, typically 16-32KB
//! - **L1 Data Cache**: Per-core, typically 16-32KB  
//! - **L2 Unified Cache**: Shared, typically 256KB-1MB (optional)
//! - **Cache Line Size**: Typically 32 or 64 bytes
//!
//! # Cache Coherency
//!
//! Cache maintenance is critical for:
//! - DMA operations (device memory coherency)
//! - Code modification (self-modifying code)
//! - Memory mapping changes (MMU operations)
//! - Multi-core coherency (if applicable)
//!
//! # Cache Operations
//!
//! - **Clean**: Write dirty cache lines back to memory
//! - **Invalidate**: Mark cache lines as invalid
//! - **Clean & Invalidate**: Combine both operations
//! - **Flush**: Implementation-specific complete operation
//!
//! # Performance Considerations
//!
//! - Cache operations can be expensive (100s of cycles)
//! - Range operations are more efficient than global operations
//! - Cache warming strategies for real-time performance
//! - Cache coloring for deterministic behavior

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::arch::asm;
use crate::arch::ArchResult;

/// Cache line size in bytes (ARM Cortex-A typical)
pub const CACHE_LINE_SIZE: usize = 32;

/// Maximum cache size for worst-case calculations
pub const MAX_CACHE_SIZE: usize = 1024 * 1024; // 1MB

/// Cache type information
#[derive(Debug, Clone, Copy)]
pub struct CacheInfo {
    /// Cache type register value
    pub ctr: u32,
    /// Cache size identification register
    pub ccsidr: u32,
    /// Cache level ID register
    pub clidr: u32,
}

impl CacheInfo {
    /// Read cache information from coprocessor registers
    pub fn read() -> Self {
        let mut info = Self {
            ctr: 0,
            ccsidr: 0,
            clidr: 0,
        };

        // SAFETY: Reading cache identification registers is safe
        unsafe {
            // Cache Type Register
            asm!("mrc p15, 0, {}, c0, c0, 1", out(reg) info.ctr, options(nomem, nostack));
            
            // Cache Level ID Register
            asm!("mrc p15, 1, {}, c0, c0, 1", out(reg) info.clidr, options(nomem, nostack));
            
            // Select L1 data cache and read Cache Size ID Register
            asm!("mcr p15, 2, {}, c0, c0, 0", in(reg) 0u32, options(nomem, nostack)); // Select L1 data cache
            asm!("isb", options(nomem, nostack));
            asm!("mrc p15, 1, {}, c0, c0, 0", out(reg) info.ccsidr, options(nomem, nostack));
        }

        info
    }
    
    /// Get instruction cache line size
    pub fn icache_line_size(&self) -> usize {
        let line_size_bits = self.ctr & 0xF;
        4 << line_size_bits
    }
    
    /// Get data cache line size
    pub fn dcache_line_size(&self) -> usize {
        let line_size_bits = (self.ctr >> 16) & 0xF;
        4 << line_size_bits
    }
    
    /// Get minimum cache line size
    pub fn min_cache_line_size(&self) -> usize {
        core::cmp::min(self.icache_line_size(), self.dcache_line_size())
    }
    
    /// Get number of cache sets
    pub fn cache_sets(&self) -> u32 {
        ((self.ccsidr >> 13) & 0x7FFF) + 1
    }
    
    /// Get cache associativity
    pub fn cache_associativity(&self) -> u32 {
        ((self.ccsidr >> 3) & 0x3FF) + 1
    }
    
    /// Get cache size in bytes
    pub fn cache_size(&self) -> usize {
        let line_size = self.dcache_line_size();
        let sets = self.cache_sets();
        let ways = self.cache_associativity();
        line_size * sets as usize * ways as usize
    }
    
    /// Check if instruction and data caches are separate
    pub fn has_separate_caches(&self) -> bool {
        (self.ctr & (1 << 24)) == 0
    }
}

/// Cache operation types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CacheOp {
    /// Clean cache (write back dirty lines)
    Clean,
    /// Invalidate cache (mark lines invalid)
    Invalidate,
    /// Clean and invalidate cache
    CleanInvalidate,
    /// Flush cache (implementation specific)
    Flush,
}

/// Cache types
pub enum CacheType {
    /// Instruction cache
    Instruction,
    /// Data cache
    Data,
    /// Unified cache (both instruction and data)
    Unified,
}

/// Cache maintenance operations
pub struct CacheManager {
    /// Cache information
    info: CacheInfo,
    /// Whether caches are enabled
    enabled: bool,
}

impl CacheManager {
    /// Create a new cache manager
    pub fn new() -> Self {
        Self {
            info: CacheInfo::read(),
            enabled: false,
        }
    }
    
    /// Initialize cache management
    pub fn init(&mut self) -> ArchResult<()> {
        // Read current cache state
        self.info = CacheInfo::read();
        
        crate::debug_print!("Cache Info:");
        crate::debug_print!("  I-cache line size: {} bytes", self.info.icache_line_size());
        crate::debug_print!("  D-cache line size: {} bytes", self.info.dcache_line_size());
        crate::debug_print!("  Cache size: {} KB", self.info.cache_size() / 1024);
        crate::debug_print!("  Separate caches: {}", self.info.has_separate_caches());
        
        Ok(())
    }

    /// Enable all caches
    pub fn enable_caches(&mut self) {
        // SAFETY: Enabling caches is safe
        unsafe {
            // Enable instruction cache
            self.enable_icache();

            // Enable data cache
            self.enable_dcache();

            // Enable branch prediction if available
            self.enable_branch_prediction();
        }

        self.enabled = true;
        crate::debug_print!("Caches enabled");
    }

    /// Disable all caches
    pub fn disable_caches(&mut self) {
        // SAFETY: Disabling caches requires proper cleanup
        unsafe {
            // Clean and disable data cache
            self.clean_dcache_all();
            self.disable_dcache();

            // Disable instruction cache
            self.disable_icache();
        }

        self.enabled = false;
        crate::debug_print!("Caches disabled");
    }

    /// Enable instruction cache
    unsafe fn enable_icache(&self) {
        let mut sctlr: u32;
        
        // SAFETY: We're enabling instruction cache
        unsafe {
            asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
            sctlr |= 1 << 12; // I bit
            asm!("mcr p15, 0, {}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }
    }

    /// Disable instruction cache
    unsafe fn disable_icache(&self) {
        let mut sctlr: u32;
        
        // SAFETY: We're disabling instruction cache
        unsafe {
            // Invalidate instruction cache first
            self.invalidate_icache_all();
            
            asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
            sctlr &= !(1 << 12); // Clear I bit
            asm!("mcr p15, 0, {}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }
    }

    unsafe fn enable_dcache(&self) {
        let mut sctlr: u32;
        
        // SAFETY: We're enabling data cache
        unsafe {
            asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
            sctlr |= 1 << 2; // C bit
            asm!("mcr p15, 0, {}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
            asm!("dsb", options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }
    }

    /// Disable data cache
    unsafe fn disable_dcache(&self) {
        let mut sctlr: u32;
        
        // SAFETY: We're disabling data cache
        unsafe {
            asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
            sctlr &= !(1 << 2); // Clear C bit
            asm!("mcr p15, 0, {}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
            asm!("dsb", options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }
    }

    /// Enable branch prediction
    unsafe fn enable_branch_prediction(&self) {
        let mut sctlr: u32;
        
        // SAFETY: We're enabling branch prediction
        unsafe {
            asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
            sctlr |= 1 << 11; // Z bit
            asm!("mcr p15, 0, {}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }
    }

    /// Invalidate all instruction caches
    pub unsafe fn invalidate_icache_all(&self) {
        // SAFETY: Invalidating instruction cache is safe
        unsafe {
            asm!("mcr p15, 0, {}, c7, c5, 0", in(reg) 0u32, options(nomem, nostack));
            asm!("dsb", options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }
    }

    /// Invalidate instruction cache by address range
    pub unsafe fn invalidate_icache_range(&self, start: usize, size: usize) {
        let line_size = self.info.icache_line_size();
        let end = start + size;
        let mut addr = start & !(line_size - 1); // Align to cache line
        
        // SAFETY: Invalidating instruction cache by address is safe
        unsafe {
            while addr < end {
                asm!("mcr p15, 0, {}, c7, c5, 1", in(reg) addr, options(nomem, nostack)); // ICIMVAU
                addr += line_size;
            }
            asm!("dsb", options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }
    }

    /// Clean all data caches
    pub unsafe fn clean_dcache_all(&self) {
        // SAFETY: Cleaning data cache is safe
        unsafe {
            self.dcache_operation_all(CacheOp::Clean);
        }
    }

    /// Invalidate all data caches
    pub unsafe fn invalidate_dcache_all(&self) {
        // SAFETY: Invalidating data cache is safe but requires care
        unsafe {
            self.dcache_operation_all(CacheOp::Invalidate);
        }
    }

    /// Clean and invalidate all data caches
    pub unsafe fn clean_invalidate_dcache_all(&self) {
        // SAFETY: Clean and invalidate is safe
        unsafe {
            self.dcache_operation_all(CacheOp::CleanInvalidate);
        }
    }

    /// Clean data cache by address range
    pub unsafe fn clean_dcache_range(&self, start: usize, size: usize) {
        unsafe {
            self.dcache_operation_range(start, size, CacheOp::Clean);
        }
    }

    /// Invalidate data cache by address range
    pub unsafe fn invalidate_dcache_range(&self, start: usize, size: usize) {
        unsafe {
            self.dcache_operation_range(start, size, CacheOp::Invalidate);
        }
    }

    /// Clean and invalidate data cache by address range
    pub unsafe fn clean_invalidate_dcache_range(&self, start: usize, size: usize) {
        unsafe {
            self.dcache_operation_range(start, size, CacheOp::CleanInvalidate);
        }
    }

    /// Perform data cache operation on all cache
    unsafe fn dcache_operation_all(&self, op: CacheOp) {
        let sets = self.info.cache_sets();
        let ways = self.info.cache_associativity();
        let line_size_bits = (self.info.ccsidr & 0x7) + 4; // Log2 of line size
        let way_shift = 32 - ways.leading_zeros();
        let set_shift = line_size_bits;
        
        // SAFETY: Performing cache maintenance operations
        unsafe {
            for way in 0..ways {
                for set in 0..sets {
                    let value = (way << way_shift) | (set << set_shift);
                    
                    match op {
                        CacheOp::Clean => {
                            asm!("mcr p15, 0, {}, c7, c10, 2", in(reg) value, options(nomem, nostack)); // DCCSW
                        }
                        CacheOp::Invalidate => {
                            asm!("mcr p15, 0, {}, c7, c6, 2", in(reg) value, options(nomem, nostack)); // DCISW
                        }
                        CacheOp::CleanInvalidate => {
                            asm!("mcr p15, 0, {}, c7, c14, 2", in(reg) value, options(nomem, nostack)); // DCCISW
                        }
                        CacheOp::Flush => {
                            // Flush is equivalent to clean+invalidate
                            asm!("mcr p15, 0, {}, c7, c14, 2", in(reg) value, options(nomem, nostack)); // DCCISW
                        }
                    }
                }
            }
            
            asm!("dsb", options(nomem, nostack));
        }
    }

    /// Perform data cache operation on address range
    unsafe fn dcache_operation_range(&self, start: usize, size: usize, op: CacheOp) {
        let line_size = self.info.dcache_line_size();
        let end = start + size;
        let mut addr = start & !(line_size - 1); // Align to cache line
        
        // SAFETY: Performing cache maintenance operations by address
        unsafe {
            while addr < end {
                match op {
                    CacheOp::Clean => {
                        asm!("mcr p15, 0, {}, c7, c10, 1", in(reg) addr, options(nomem, nostack)); // DCCMVAC
                    }
                    CacheOp::Invalidate => {
                        asm!("mcr p15, 0, {}, c7, c6, 1", in(reg) addr, options(nomem, nostack)); // DCIMVAC
                    }
                    CacheOp::CleanInvalidate => {
                        asm!("mcr p15, 0, {}, c7, c14, 1", in(reg) addr, options(nomem, nostack)); // DCCIMVAC
                    }
                    CacheOp::Flush => {
                        // Flush is equivalent to clean+invalidate
                        asm!("mcr p15, 0, {}, c7, c14, 1", in(reg) addr, options(nomem, nostack)); // DCCIMVAC
                    }
                }
                addr += line_size;
            }
            
            asm!("dsb", options(nomem, nostack));
        }
    }

    /// Flush entire cache hierarchy
    pub unsafe fn flush_all_caches(&self) {
        // SAFETY: Flushing all caches
        unsafe {
            self.clean_invalidate_dcache_all();
            self.invalidate_icache_all();
        }
    }

    /// Prefetch data into cache
    pub fn prefetch(&self, addr: usize) {
        // SAFETY: Prefetching is safe and optional
        unsafe {
            asm!("pld [{}]", in(reg) addr, options(nomem, nostack));
        }
    }

    /// Prefetch instruction into cache
    pub fn prefetch_instruction(&self, addr: usize) {
        // SAFETY: Instruction prefetching is safe
        unsafe {
            asm!("pli [{}]", in(reg) addr, options(nomem, nostack));
        }
    }

    /// Get cache information
    pub fn cache_info(&self) -> &CacheInfo {
        &self.info
    }

    /// Check if caches are enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Perform cache coherency operation for DMA
    pub unsafe fn dma_coherent_range(&self, start: usize, size: usize, to_device: bool) {
        if to_device {
            // Clean cache to ensure data is written to memory
            unsafe {
                self.clean_dcache_range(start, size);
            }
        } else {
            // Invalidate cache to ensure fresh data is read from memory
            unsafe {
                self.invalidate_dcache_range(start, size);
            }
        }
    }

    /// Warm up cache with specific memory range
    pub fn warm_cache(&self, start: usize, size: usize) {
        let line_size = self.info.min_cache_line_size();
        let end = start + size;
        let mut addr = start & !(line_size - 1);
        
        // Touch each cache line to bring it into cache
        while addr < end {
            self.prefetch(addr);
            addr += line_size;
        }
    }
}

/// Global cache manager instance
static mut CACHE_MANAGER: Option<CacheManager> = None;

/// Initialize global cache manager
pub fn init_cache_manager() -> ArchResult<()> {
    // SAFETY: This is called once during system initialization
    unsafe {
        if CACHE_MANAGER.is_some() {
            return Ok(());
        }

        CACHE_MANAGER = Some(CacheManager::new());

        if let Some(manager) = CACHE_MANAGER.as_mut() {
            manager.init()?;
        }
    }

    Ok(())
}

/// Get reference to global cache manager
pub fn cache_manager() -> Option<&'static mut CacheManager> {
    // SAFETY: Cache manager is initialized once
    unsafe { CACHE_MANAGER.as_mut() }
}

/// Enable all caches (convenience function)
pub fn enable_caches() {
    if let Some(manager) = cache_manager() {
        manager.enable_caches();
    }
}

/// Flush all caches (convenience function)
pub fn flush_dcache_all() {
    if let Some(manager) = cache_manager() {
        // SAFETY: Flushing caches is safe
        unsafe {
            manager.clean_invalidate_dcache_all();
        }
    }
}