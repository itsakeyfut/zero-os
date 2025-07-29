//! ARM Memory Management Unit (MMU) Implementation
//!
//! This module provides comprehensive MMU management for ARM Cortex-A processors.
//! It implements virtual memory management with hardware page table support,
//! memory protection, and address translation services.
//!
//! # MMU Architecture
//!
//! ARM MMU supports:
//! - **Two-level page tables**: L1 (sections/page tables) and L2 (small pages)
//! - **Multiple page sizes**: 1MB sections, 64KB large pages, 4KB small pages
//! - **Memory domains**: Up to 16 domains for access control
//! - **Cache control**: Cacheable, bufferable attributes per page
//! - **Access permissions**: Read/write permissions for kernel and user
//!
//! # Page Table Structure
//!
//! ```text
//! Level 1 (4096 entries, 4KB total):
//! Each entry covers 1MB of virtual address space
//! - Section entry: Direct mapping to 1MB physical section
//! - Page table entry: Pointer to Level 2 page table
//!
//! Level 2 (256 entries, 1KB total):
//! Each entry covers 4KB of virtual address space
//! - Small page entry: Mapping to 4KB physical page
//! ```
//!
//! # Virtual Address Layout
//!
//! ```text
//! 31    20 19    12 11     0
//! |  L1   |   L2   | Offset |
//! | Index | Index  |        |
//! `

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::arch::asm;
use core::ptr;
use crate::memory::{VirtualAddress, PhysicalAddress, MemoryRegion, MemoryFlags, MemoryError, MemoryResult};
use crate::arch::ArchResult;

/// Page size constants
pub const PAGE_SIZE: usize = 4096;
pub const PAGE_SHIFT: usize = 12;
pub const SECTION_SIZE: usize = 1024 * 1024; // 1MB
pub const SECTION_SHIFT: usize = 20;

/// Page table constants
pub const L1_TABLE_SIZE: usize = 4096; // 4KB
pub const L2_TABLE_SIZE: usize = 1024; // 1KB
pub const L1_ENTRIES: usize = 4096;
pub const L2_ENTRIES: usize = 256;

/// Page table entry types
pub const L1_TYPE_FAULT: u32 = 0b00;
pub const L1_TYPE_PAGE_TABLE: u32 = 0b01;
pub const L1_TYPE_SECTION: u32 = 0b10;
pub const L1_TYPE_RESERVED: u32 = 0b11;

pub const L2_TYPE_FAULT: u32 = 0b00;
pub const L2_TYPE_LARGE_PAGE: u32 = 0b01;
pub const L2_TYPE_SMALL_PAGE: u32 = 0b10;
pub const L2_TYPE_RESERVED: u32 = 0b11;

/// Access permission bits (AP[2:0])
pub const AP_NO_ACCESS: u32 = 0b000;
pub const AP_PRIV_RW: u32 = 0b001;      // Privileged read-write
pub const AP_PRIV_RW_USER_RO: u32 = 0b010; // Privileged RW, user read-only
pub const AP_FULL_ACCESS: u32 = 0b011;  // Full access
pub const AP_PRIV_RO: u32 = 0b101;      // Privileged read-only
pub const AP_READ_ONLY: u32 = 0b110;    // Read-only

/// Domain numbers
pub const DOMAIN_KERNEL: u32 = 0;
pub const DOMAIN_USER: u32 = 1;

/// Cache and buffer bits
pub const CACHE_WRITE_THROUGH: u32 = 0b1000;
pub const CACHE_WRITE_BACK: u32 = 0b1100;
pub const BUFFERABLE: u32 = 0b0100;

/// Level 1 page table entry
#[derive(Debug, Clone, Copy)]
#[repr(transparent)]
pub struct L1Entry(pub u32);

impl L1Entry {
    /// Create a fault entry
    pub const fn fault() -> Self {
        Self(L1_TYPE_FAULT)
    }

    /// Create a section entry
    pub fn section(
        physical_addr: PhysicalAddress,
        flags: MemoryFlags,
        domain: u32,
    ) -> Self {
        let mut entry = (physical_addr.as_usize() & 0xFFF00000) as u32;
        entry |= L1_TYPE_SECTION;

        // Set domain
        entry |= (domain & 0xF) << 5;

        // Set access permissions
        let ap = if flags.user {
            if flags.write {
                AP_FULL_ACCESS
            } else {
                AP_READ_ONLY
            }
        } else {
            if flags.write {
                AP_PRIV_RW
            } else {
                AP_PRIV_RO
            }
        };
        entry |= (ap & 0x3) << 10;
        entry |= ((ap >> 2) & 0x1) << 15;

        // Set cache attributes
        if flags.cacheable {
            entry |= CACHE_WRITE_BACK;
        }
        if flags.bufferable {
            entry |= BUFFERABLE;
        }

        Self(entry)
    }

    /// Create a page table entry
    pub fn page_table(
        page_table_addr: PhysicalAddress,
        domain: u32,
    ) -> Self {
        let mut entry = (page_table_addr.as_usize() & 0xFFFFFC00) as u32;
        entry |= L1_TYPE_PAGE_TABLE;
        entry |= (domain & 0xF) << 5;
        Self(entry)
    }

    /// Check if entry is valid (not fault)
    pub fn is_valid(self) -> bool {
        (self.0 & 0x3) != L1_TYPE_FAULT
    }

    /// Check if entry is a section
    pub fn is_section(self) -> bool {
        (self.0 & 0x3) == L1_TYPE_SECTION
    }

    /// Check if entry is a page table
    pub fn is_page_table(self) -> bool {
        (self.0 & 0x3) == L1_TYPE_PAGE_TABLE
    }

    /// Get physical address for section
    pub fn section_address(self) -> Option<PhysicalAddress> {
        if self.is_section() {
            Some(PhysicalAddress::new((self.0 & 0xFFF00000) as usize))
        } else {
            None
        }
    }

    /// Get page table address
    pub fn page_table_address(self) -> Option<PhysicalAddress> {
        if self.is_page_table() {
            Some(PhysicalAddress::new((self.0 & 0xFFFFFC00) as usize))
        } else {
            None
        }
    }
}

/// Level 2 page table entry
#[derive(Debug, Clone, Copy)]
#[repr(transparent)]
pub struct L2Entry(pub u32);

impl L2Entry {
    /// Create a fault entry
    pub const fn fault() -> Self {
        Self(L2_TYPE_FAULT)
    }

    /// Create a small page entry
    pub fn small_page(
        physical_addr: PhysicalAddress,
        flags: MemoryFlags,
    ) -> Self {
        let mut entry = (physical_addr.as_usize() & 0xFFFFF000) as u32;
        entry |= L2_TYPE_SMALL_PAGE;

        // Set access permissions
        let ap = if flags.user {
            if flags.write {
                AP_FULL_ACCESS
            } else {
                AP_READ_ONLY
            }
        } else {
            if flags.write {
                AP_PRIV_RW
            } else {
                AP_PRIV_RO
            }
        };
        entry |= (ap & 0x3) << 4;
        entry |= ((ap >> 2) & 0x1) << 9;

        // Set cache attributes
        if flags.cacheable {
            entry |= 0x8; // C bit
        }
        if flags.bufferable {
            entry |= 0x4; // B bit
        }

        Self(entry)
    }

    /// Check if entry is valid (not fault)
    pub fn is_valid(self) -> bool {
        (self.0 & 0x3) != L2_TYPE_FAULT
    }
}