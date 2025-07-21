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

    /// Invalidate instructino cache by address range
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
}