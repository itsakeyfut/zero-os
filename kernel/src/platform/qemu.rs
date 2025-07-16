//! QEMU VersatilePB Platform Implementation
//! 
//! This module provides platform-specific implementation for QEMU VersatilePB
//! emulation environment. VersatilePB is ARM's development platform that 
//! provides a good balance of features for embedded system development.
//! 
//! # Hardware Features
//! 
//! - ARM1176JZF-S processor (ARMv6 architecture)
//! - 128MB RAM by default (configurable)
//! - PL011 UART for serial communication
//! - PL050 PS/2 keyboard and mouse controllers
//! - PL110 LCD controller
//! - PL181 MultiMedia Card Interface
//! - SP804 dual timer
//! - PL190 Vectored Interrupt Controller (VIC)
//! 
//! # Memory Mpa
//! 
//! ```text
//! 0x00000000 - 0x07FFFFFF : RAM (128MB)
//! 0x10000000 - 0x10000FFF : System registers
//! 0x10001000 - 0x10001FFF : PCI I/O space
//! 0x101F1000 - 0x101F1FFF : UART0 (PL011)
//! 0x101F2000 - 0x101F2FFF : UART1 (PL011)
//! 0x101F3000 - 0x101F3FFF : UART2 (PL011)
//! 0x10120000 - 0x10120FFF : Timer 0 & 1 (SP804)
//! 0x10121000 - 0x10121FFF : Timer 2 & 3 (SP804)
//! 0x10140000 - 0x10140FFF : VIC (PL190)
//! ```

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use crate::arch::InterruptType;
use crate::memory::PhysicalAddress;
use super::{
    PlatformInterface, PlatformType, PlatformResult, PlatformError,
    HardwareCapabilities, UartConfig, TimerConfig, GpioConfig,
};

/// QEMU VersatilePB platform identifier
pub const PLATFORM_TYPE: PlatformType = PlatformType::QemuVersatilePB;

/// Platform name string
pub const PLATFORM_NAME: &str = "QEMU VersatilePB";

/// Platform version string
pub const PLATFORM_VERSION: &str = "1.0.0";

/// Memory layout constants
pub mod memory_layout {
    use crate::memory::PhysicalAddress;

    /// Kernel load address
    pub const KERNEL_LOAD_ADDRESS: PhysicalAddress = PhysicalAddress::new(0x10000);

    /// RAM start address
    pub const RAM_START: PhysicalAddress = PhysicalAddress::new(0x00000000);

    /// RAM size (128MB default for QEMU)
    pub const RAM_SIZE: usize = 128 * 1024 * 1024;

    /// Device memory start
    pub const DEVICE_START: PhysicalAddress = PhysicalAddress::new(0x10000000);

    /// Device memory size
    pub const DEVICE_SIZE: usize = 0x10000000;
}

/// Interrupt numbers for VersatilePB
pub mod interrups {
    /// Timer 0 interrupt
    pub const TIMER_IRQ: u32 = 4;

    /// UART0 interrupt
    pub const UART_IRQ: u32 = 12;

    /// GPIO interrupt (not used on VersatilePB)
    pub const GPIO_IRQ: u32 = 0;

    /// Total number of interrupts
    pub const NUM_INTERRUPTS: u32 = 32;
}

/// Hardware register addresses
pub mod registers {
    /// UART0 base address
    pub const UART_BASE: usize = 0x101F1000;

    /// Timer 0/1 base address
    pub const TIMER_BASE: usize = 0x10120000;

    /// GPIO base address (not applicable for VersatilePB)
    pub const GPIO_BASE: usize = 0x00000000;

    /// VIC (Vectored Interrupt Controller) base address
    pub const INTERRUPT_CONTROLLER_BASE: usize = 0x10140000;

    /// System controller base address
    pub const SYSCON_BASE: usize = 0x10000000;
}