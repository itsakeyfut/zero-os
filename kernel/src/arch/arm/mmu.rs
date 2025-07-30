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

    /// Check if entry is a small page
    pub fn is_small_page(self) -> bool {
        (self.0 & 0x3) == L2_TYPE_SMALL_PAGE
    }

    /// Get physical address for small page
    pub fn page_address(self) -> Option<PhysicalAddress> {
        if self.is_small_page() {
            Some(PhysicalAddress::new((self.0 & 0xFFFFF000) as usize))
        } else {
            None
        }
    }
}

/// Page table walker for address translation
pub struct PageTableWalker {
    /// L1 page table base address
    l1_table: PhysicalAddress,
}

impl PageTableWalker {
    /// Create a new page table walker
    pub fn new(l1_table: PhysicalAddress) -> Self {
        Self { l1_table }
    }

    /// Translate virtual address to physical address
    pub fn translate(&self, virtual_addr: VirtualAddress) -> Option<PhysicalAddress> {
        let vaddr = virtual_addr.as_usize();

        // Extract L1 index
        let l1_index = (vaddr >> SECTION_SHIFT) & 0xFFF;

        // Read L1 entry
        // SAFETY: We're reading from page table memory
        let l1_entry = unsafe {
            let l1_table_ptr = self.l1_table.as_usize() as *const L1Entry;
            ptr::read_volatile(l1_table_ptr.add(l1_index))
        };

        if !l1_entry.is_valid() {
            return None;
        }

        if l1_entry.is_section() {
            // 1MB section mapping
            let section_base = l1_entry.section_address()?;
            let offset = vaddr & (SECTION_SIZE - 1);
            return Some(PhysicalAddress::new(section_base.as_usize() + offset));
        }

        if l1_entry.is_page_table() {
            // Page table mapping
            let l2_table = l1_entry.page_table_address()?;
            let l2_index = (vaddr >> PAGE_SHIFT) & 0xFF;

            // Read L2 entry
            // SAFETY: We're reading from page table memory
            let l2_entry = unsafe {
                let l2_table_ptr = l2_table.as_usize() as *const L2Entry;
                ptr::read_volatile(l2_table_ptr.add(l2_index))
            };

            if l2_entry.is_valid() && l2_entry.is_small_page() {
                let page_base = l2_entry.page_address()?;
                let offset = vaddr & (PAGE_SIZE - 1);
                return Some(PhysicalAddress::new(page_base.as_usize() + offset));
            }
        }

        None
    }
}

/// MMU manager
pub struct MmuManager {
    /// L1 page table physical address
    l1_table: PhysicalAddress,
    /// L1 page table virtual address
    l1_table_virt: VirtualAddress,
    /// Page table walker
    walker: PageTableWalker,
    /// MMU enabled state
    enabled: bool,
    /// Next available L2 table address
    next_l2_table: PhysicalAddress,
}

impl MmuManager {
    /// Create a new MMU manager
    pub fn new(l1_table_phys: PhysicalAddress, l1_table_virt: VirtualAddress) -> Self {
        Self {
            l1_table: l1_table_phys,
            l1_table_virt,
            walker: PageTableWalker::new(l1_table_phys),
            enabled: false,
            next_l2_table: PhysicalAddress::new(l1_table_phys.as_usize() + L1_TABLE_SIZE),
        }
    }

    /// Initialize MMU with identity mapping
    pub fn init(&mut self) -> ArchResult<()> {
        // Clear L1 page table
        // SAFETY: We're initializing page table memory
        unsafe {
            let l1_table_ptr = self.l1_table_virt.as_usize() as *mut L1Entry;
            for i in 0..L1_ENTRIES {
                ptr::write_volatile(l1_table_ptr.add(i), L1Entry::fault());
            }
        }

        // Create identity mapping for first 128MB (RAM)
        for section in 0..128 {
            let virtual_addr = VirtualAddress::new(section * SECTION_SIZE);
            let physical_addr = PhysicalAddress::new(section * SECTION_SIZE);
            let flags = MemoryFlags::kernel_data();

            self.map_section(virtual_addr, physical_addr, flags, DOMAIN_KERNEL)?;
        }

        // Create kernel mapping at 3GB virtual -> 0GB physical
        for section in 0..128 {
            let virtual_addr = VirtualAddress::new(0xC0000000 + section * SECTION_SIZE);
            let physical_addr = PhysicalAddress::new(section * SECTION_SIZE);
            let flags = MemoryFlags::kernel_code();
            
            self.map_section(virtual_addr, physical_addr, flags, DOMAIN_KERNEL)?;
        }

        crate::debug_print!("MMU page tables initialized");
        Ok(())
    }

    /// Map a 1MB section
    pub fn map_section(
        &mut self,
        virtual_addr: VirtualAddress,
        physical_addr: PhysicalAddress,
        flags: MemoryFlags,
        domain: u32,
    ) -> ArchResult<()> {
        let vaddr = virtual_addr.as_usize();

        if vaddr & (SECTION_SIZE - 1) != 0 {
            return Err(crate::arch::ArchError::AlignmentError);
        }

        let l1_index = (vaddr >> SECTION_SHIFT) & 0xFFF;
        let entry = L1Entry::section(physical_addr, flags, domain);

        // SAFETY: We're writing to page table memory
        unsafe {
            let l1_table_ptr = self.l1_table_virt.as_usize() as *mut L1Entry;
            ptr::write_volatile(l1_table_ptr.add(l1_index), entry);
        }

        Ok(())
    }

    /// Map a 4KB page
    pub fn map_page(
        &mut self,
        virtual_addr: VirtualAddress,
        physical_addr: PhysicalAddress,
        flags: MemoryFlags,
        domain: u32,
    ) -> ArchResult<()> {
        let vaddr = virtual_addr.as_usize();

        if vaddr & (PAGE_SIZE - 1) != 0 {
            return Err(crate::arch::ArchError::AlignmentError);
        }

        let l1_index = (vaddr >> SECTION_SHIFT) & 0xFFF;
        let l2_index = (vaddr >> PAGE_SHIFT) & 0xFF;

        // Get or create L2 page table
        let l2_table = self.get_or_create_l2_table(l1_index, domain)?;

        // Create L2 entry
        let entry = L2Entry::small_page(physical_addr, flags);

        // SAFETY: We're writing to page table memory
        unsafe {
            let l2_table_ptr = l2_table.as_usize() as *mut L2Entry;
            ptr::write_volatile(l2_table_ptr.add(l2_index), entry);
        }

        Ok(())
    }

    /// Get or create L2 page table
    fn get_or_create_l2_table(
        &mut self,
        l1_index: usize,
        domain: u32,
    ) -> ArchResult<PhysicalAddress> {
        // SAFETY: We're reading from page table memory
        let l1_entry = unsafe {
            let l1_table_ptr = self.l1_table_virt.as_usize() as *const L1Entry;
            ptr::read_volatile(l1_table_ptr.add(l1_index))
        };

        if l1_entry.is_page_table() {
            // L2 table already exists
            return Ok(l1_entry.page_table_address().unwrap());
        }

        if l1_entry.is_valid() {
            // Entry is a section, cannot convert to page table
            return Err(crate::arch::ArchError::InvalidAddress);
        }

        // Create new L2 table
        let l2_table_phys = self.next_l2_table;
        self.next_l2_table = PhysicalAddress::new(self.next_l2_table.as_usize() + L2_TABLE_SIZE);

        // Clear L2 table
        // SAFETY: We're initializing new page table memory
        unsafe {
            let l2_table_ptr = l2_table_phys.as_usize() as *mut L2Entry;
            for i in 0..L2_ENTRIES {
                ptr::write_volatile(l2_table_ptr.add(i), L2Entry::fault());
            }
        }

        // Create L1 entry pointing to L2 table
        let l1_entry = L1Entry::page_table(l2_table_phys, domain);

        // SAFETY: We're writing to page table memory
        unsafe {
            let l1_table_ptr = self.l1_table_virt.as_usize() as *mut L1Entry;
            ptr::write_volatile(l1_table_ptr.add(l1_index), l1_entry);
        }

        Ok(l2_table_phys)
    }
}