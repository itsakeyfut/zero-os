//! Memory Management Subsystem
//! 
//! This module provides comprehensive memory management for the Zero OS kernel.
//! It implements a multi-level memory management systen with the following components:
//! 
//! - Physical memory allocator (buddy system for large allocations)
//! - Virtual memory manager (MMU-based virtual addressing)
//! - Kernel heap allocator (slab allocator for kernel objects)
//! - Grant system (Tock-style safe memory sharing)
//! - Memory protection (hardward-enforced isolation)
//! 
//! # Design Principles
//! 
//! - Memory safety through Rust's type system and runtime checks
//! - Real-time deterministic allocation for critical sections
//! - Efficient memory utilization with minimal fragmentation
//! - Hardware memory protection for process isolation
//! - Grant-based capability system for safe memory sharing
//! 
//! # Memory Layout
//! 
//! ```text
//! Virtual Address Space Layout:
//! 0x00000000 - 0x3FFFFFFF : User space (1GB)
//! 0x40000000 - 0x7FFFFFFF : Kernel heap and data (1GB)
//! 0x80000000 - 0xBFFFFFFF : Device memory (1GB) 
//! 0xC0000000 - 0xFFFFFFFF : Kernel code and stack (1GB)
//! ```

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::alloc::{GlobalAlloc, Layout};
use core::ptr::NonNull;
use linked_list_allocator::LockedHeap;
use buddy_system_allocator::FrameAllocator;

/// Memory management errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryError {
    /// Out of physical memory
    OutOfMemory,
    /// Invalid memory address
    InvalidAddress,
    /// Memory alignment error
    AlignmentError,
    /// Invalid memory size (too large or zero)
    InvalidSize,
    /// Permission denied for memory operation
    PermissionDenied,
    /// Memory region already allocated
    AlreadyAllocated,
    /// Memory region not found
    NotFound,
    /// Memory corruption detected
    CorruptionDetected,
    /// Hardware memory management fault
    HardwareFault,
}

/// Result type for memory operations
pub type MemoryResult<T> = Result<T, MemoryError>;
