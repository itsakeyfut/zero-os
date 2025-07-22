//! ARM Generic Interrupt Controller (GIC) Driver
//!
//! This module provides support for the ARM Generic Interrupt Controller,
//! which is the standard interrupt controller for ARM Cortex-A processors.
//! The GIC manages both Software Generated Interrupts (SGI), Private Peripheral
//! Interrupts (PPI), and Shared Peripheral Interrupts (SPI).
//!
//! # GIC Architecture
//!
//! The GIC consists of two main components:
//! - **Distributor**: Routes interrupts to target processors
//! - **CPU Interface**: Per-processor interface for interrupt handling
//!
//! # Interrupt Types
//!
//! - **SGI (0-15)**: Software Generated Interrupts (inter-processor communication)
//! - **PPI (16-31)**: Private Peripheral Interrupts (per-processor peripherals)  
//! - **SPI (32-1019)**: Shared Peripheral Interrupts (shared system peripherals)
//!
//! # Priority and Preemption
//!
//! - 8-bit priority values (0 = highest priority)
//! - Priority grouping for preemption/sub-priority
//! - Priority masking for nested interrupt handling
//!
//! # Security Extensions
//!
//! - Secure and Non-secure interrupt handling
//! - Group 0 (secure) and Group 1 (non-secure) interrupts
//! - Banked registers for security separation

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::ptr;
use crate::arch::ArchResult;

/// Maximum number of interrupts supported by GIC
pub const MAX_INTERRUPTS: u32 = 1020;

/// Software Generated Interrupt range
pub const SGI_RANGE: core::ops::Range<u32> = 0..16;

/// Private Peripheral Interrupt range
pub const PPI_RANGE: core::ops::Range<u32> = 16..32;

/// Shared Peripheral Interrupt range
pub const SPI_RANGE: core::ops::Range<u32> = 32..MAX_INTERRUPTS;
