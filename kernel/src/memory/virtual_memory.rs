//! Virtual Memory Manager
//!
//! This module implements virtual memory management with MMU integration.
//! It provides address space management, page table manipulation, and
//! memory protection for process isolation and safety-critical operation.
//!
//! # Features
//!
//! - Virtual address space management
//! - Page table creation and manipulation
//! - Memory mapping and unmapping
//! - Memory protection and access control
//! - TLB management
//! - Demand paging support
//!
//! # Memory Layout
//!
//! ```text
//! Virtual Address Space (ARM 32-bit):
//! 0x00000000 - 0x3FFFFFFF : User Space (1GB)
//! 0x40000000 - 0x7FFFFFFF : Shared Libraries (1GB)  
//! 0x80000000 - 0xBFFFFFFF : Device Memory (1GB)
//! 0xC0000000 - 0xFFFFFFFF : Kernel Space (1GB)
//! ```

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::ptr::NonNull;
use heapless::{Vec, FnvIndexMap};
use crate::memory::{
    PhysicalAddress, VirtualAddress, MemoryFlags, MemoryType, MemoryRegion,
    MemoryResult, MemoryError, PAGE_SIZE, PAGE_SHIFT,
};
use crate::process::ProcessId;
use crate::arch::{target::Architecture, MemoryProtection};

/// Maximum number of virtual memory areas per address space
pub const MAX_VMAS: usize = 128;

/// Maximum number of address spaces
pub const MAX_ADDRESS_SPACES: usize = 64;
