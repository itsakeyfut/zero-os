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
            asm!("mcr p15, 2, {}, c0, c0, 0", in(reg) 0u32, options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
            asm!("mrc p15, 1, {}, c0, c0, 0", out(reg) info.ccsidr, options(nomem, nostack));
        }

        info
    }
}