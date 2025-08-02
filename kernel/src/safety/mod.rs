//! Zero OS Safety Systems
//!
//! This module implements safety-critical functionality for the Zero OS kernel.
//! It provides fault detection, isolation, recovery mechanisms, and system
//! monitoring to ensure safe operation in autonomous systems.
//!
//! # Safety Architecture
//!
//! The safety system follows a layered approach:
//! - **Hardware Safety**: Watchdogs, ECC, lockstep processors
//! - **Software Safety**: Bounds checking, assertions, invariant checking
//! - **System Safety**: Resource monitoring, deadline checking, fault isolation
//! - **Application Safety**: Process isolation, capability checking
//!
//! # Fault Model
//!
//! The system is designed to handle:
//! - Transient faults (cosmic rays, electrical noise)
//! - Permanent faults (hardware failures)
//! - Design faults (software bugs, timing violations)
//! - Interaction faults (resource conflicts, deadlocks)

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use heapless::{Vec, FnvIndexMap};
use crate::{KernelResult, KernelError};

pub mod watchdog;
pub mod fault_detection;
pub mod redundancy;
pub mod recovery;
pub mod monitoring;

// Re-exports
pub use watchdog::*;
pub use fault_detection::*;
pub use redundancy::*;
pub use recovery::*;
pub use monitoring::*;

/// Minimum number of concurrent faults to track
pub const MAX_FAULT_RECORDS: usize = 256;

/// Maximum number of safety monitors
pub const MAX_SAFETY_MONITORS: usize = 64;
