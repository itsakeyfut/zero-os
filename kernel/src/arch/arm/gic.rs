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

/// GIC Distributor registers (memory-mapped)
#[repr(C)]
pub struct GicDistributor {
    /// Distributor Control Register
    pub ctlr: u32,
    /// Interrupt Controller Type Register
    pub typer: u32,
    /// Distributor Implementer Identification Register
    pub iidr: u32,
    /// Reserved
    _reserved1: [u32; 29],
    
    /// Interrupt Group Registers (1 bit per interrupt)
    pub igroupr: [u32; 32],
    
    /// Interrupt Set-Enable Registers (1 bit per interrupt)
    pub isenabler: [u32; 32],
    /// Interrupt Clear-Enable Registers (1 bit per interrupt)
    pub icenabler: [u32; 32],
    /// Interrupt Set-Pending Registers (1 bit per interrupt)
    pub ispendr: [u32; 32],
    /// Interrupt Clear-Pending Registers (1 bit per interrupt)
    pub icpendr: [u32; 32],
    /// Interrupt Set-Active Registers (1 bit per interrupt)
    pub isactiver: [u32; 32],
    /// Interrupt Clear-Active Registers (1 bit per interrupt)
    pub icactiver: [u32; 32],
    
    /// Interrupt Priority Registers (8 bits per interrupt)
    pub ipriorityr: [u32; 255],
    
    /// Reserved
    _reserved2: u32,
    
    /// Interrupt Processor Targets Registers (8 bits per interrupt)
    pub itargetsr: [u32; 255],
    
    /// Reserved
    _reserved3: u32,
    
    /// Interrupt Configuration Registers (2 bits per interrupt)
    pub icfgr: [u32; 64],
    
    /// Reserved
    _reserved4: [u32; 64],
    
    /// Software Generated Interrupt Register
    pub sgir: u32,
    
    /// Reserved
    _reserved5: [u32; 3],
    
    /// SGI Clear-Pending Registers
    pub cpendsgir: [u32; 4],
    /// SGI Set-Pending Registers
    pub spendsgir: [u32; 4],
}
