
//! Platform Abstraction Layer
//!
//! This module provides platform-specific functionality abstraction for Zero System OS.
//! It defines common interfaces that must be implemented by each target platform,
//! allowing the kernel to remain platform-agnostic while supporting multiple
//! hardware configurations.
//!
//! # Supported Platforms
//!
//! - **QEMU VersatilePB**: Primary development and testing platform
//! - **Raspberry Pi**: Single-board computer deployment
//! - **STM32**: Microcontroller deployment
//! - **Generic ARM**: Fallback for unknown ARM platforms
//!
//! # Platform Services
//!
//! - Hardware initialization and configuration
//! - Interrupt controller management
//! - Timer and clock management
//! - UART and debug output
//! - GPIO and peripheral control
//! - Power management
//! - Hardware-specific optimizations
//!
//! # Design Principles
//!
//! - Minimal abstraction overhead
//! - Runtime platform detection where possible
//! - Compile-time optimization for known platforms
//! - Graceful fallbacks for unsupported features

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::default;

use crate::arch::{InterruptType, ArchResult, ArchError};
use crate::memory::{PhysicalAddress, MemoryError};

// Platform-specific modules
#[cfg(feature = "qemu")]
pub mod qemu;

#[cfg(feature = "raspberry-pi")]
pub mod raspberry_pi;

#[cfg(feature = "stm32")]
pub mod stm32;

pub mod generic_arm;

// Default to the appropriate platform based on features
#[cfg(feature = "qemu")]
pub use qemu as target_platform;

#[cfg(all(feature = "raspberry-pi", not(feature = "qemu")))]
pub use raspberry_pi as target_platform;

#[cfg(all(feature = "stm32", not(any(feature = "qemu", feature = "raspberry-pi"))))]
pub use stm32 as target_platform;

#[cfg(not(any(feature = "qemu", feature = "raspberry-pi", feature = "stm32")))]
pub use generic_arm as target_platform;

/// Platform identification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformType {
    /// QEMU VersatilePB emulation
    QemuVersatilePB,
    /// Respberry Pi (any model)
    RaspberryPi,
    /// STM32 microcontroller
    STM32,
    /// Generic ARM platform
    GenericArm,
    /// Unknown platform
    Unknown,
}

/// Hardware capabilities
#[derive(Debug, Clone, Copy)]
pub struct HardwareCapabilities {
    /// Has memory management unit
    pub has_mmu: bool,
    /// Has floating point unit
    pub has_fpu: bool,
    /// Has cache
    pub has_cache: bool,
    /// Has DMA controller
    pub has_dma: bool,
    /// Has real-time clock
    pub has_rtc: bool,
    /// Has watching timer
    pub has_watchdog: bool,
    /// Number of CPU cores
    pub cpu_cores: u32,
    /// CPU frequency in Hz
    pub cpu_frequency: u32,
    /// Available RAM in bytes
    pub ram_size: usize,
    /// Available flash in bytes
    pub flash_size: usize,
}

impl Default for HardwareCapabilities {
    fn default() -> Self {
        Self {
            has_mmu: true,
            has_fpu: false,
            has_cache: true,
            has_dma: false,
            has_rtc: false,
            has_watchdog: false,
            cpu_cores: 1,
            cpu_frequency: 100_000_000, // 100MHz default
            ram_size: 128 * 1024 * 1024, // 128MB default
            flash_size: 512 * 1024 * 1024, // 512MB default
        }
    }
}

/// Timer configuration
#[derive(Debug, Clone, Copy)]
pub struct TimerConfig {
    /// Timer frequency in Hz
    pub frequency: u32,
    /// Timer resolution in nanoseconds
    pub resolution_ns: u32,
    /// Maximum timer value
    pub max_value: u64,
}

/// UART configuration
#[derive(Debug, Clone, Copy)]
pub struct UartConfig {
    /// Baud rate
    pub baud_rate: u32,
    /// Data bits (5-8)
    pub data_bits: u8,
    /// Stop bits (1-2)
    pub stop_bits: u8,
    /// Parity (0=none, 1=odd, 2=even)
    pub parity: u8,
    /// Flow control enabled
    pub flow_control: bool,
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baud_rate: 115200,
            data_bits: 8,
            stop_bits: 1,
            parity: 0, // No parity
            flow_control: false,
        }
    }
}

/// GPIO pin configuration
#[derive(Debug, Clone, Copy)]
pub struct GpiConfig {
    /// Pin number
    pub pin: u32,
    /// Pin mode (0=input, 1=output, 2=alternate)
    pub mode: u8,
    /// Pull resistor (0=none, 1=pull-up, 2=pull-down)
    pub pull: u8,
    /// Output type (0=push-pull, 1=open-drain)
    pub output_type: u8,
    /// Output speed (0=low, 1=medium, 2=high, 3=very-high)
    pub speed: u8,
}

/// Platform error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformError {
    /// Hardware not found or not responding
    HardwareNotFound,
    /// Unsupported operation on this platform
    UnsupportedOperation,
    /// Hardware configuration error
    ConfigurationError,
    /// Timeout waiting for hardware
    Timeout,
    /// Hardware fault detected
    HardwareFault,
    /// Invalid parameters
    InvalidParameter,
    /// Resource busy
    ResourceBusy,
}
