//! Hardware Driver Subsystem
//! 
//! This module provides a comprehensive hardware driver framework for Zero OS.
//! It implements a layered driver architecture that provides:
//! 
//! - Hardware abstraction layer (HAL) for common peripherals
//! - Device driver interface for specific hardware components
//! - Power management for device state control
//! - Interrupt handling and event management
//! - Resource management for conflict resolution
//! - Hot-plug support for dynamic device management
//! 
//! # Driver Architecture
//! 
//! ```text
//! Application Layer
//!     |
//! Device Interface Layer (DeviceManager)
//!     |
//! Driver Implementation Layer
//!     |
//! Hardware Abstraction Layer (HAL)
//!     |
//! Platform Layer (Registers, Memory-mapped I/O)
//! ```
//! 
//! # Design Principles
//! 
//! - Type-safe hardware access through Rust's type system
//! - Zero-cost abstractions for performance-critical operations
//! - Capability-based security for device access control
//! - Real-time deterministic behavior for critical drivers
//! - Platform independence through trait abstraction
//! - Resource sharing through grant-based mechanisms

#![deny(missing_docs)]
#![deny(clippy::undocumented_unsafe_blocks)]

// Driver modules
pub mod uart;
