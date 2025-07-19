//! UART Serial Communication Driver
//! 
//! This module provides a comprehensive UART (Universal Asynchronous Receiver/Transmitter)
//! driver for Zero OS. It supports various UART controllers commonly found in
//! ARM-based embedded systems.
//! 
//! # Supported UART Controllers
//! 
//! - PL011 (ARM PrimeCell UART) - Used in VersatilePB, RaspberryPi
//! - 16550 Compatible UARTs - Industry standard
//! - Custom platform-specific UARTs
//! 
//! # Features
//! 
//! - Multiple baud rates (1200 to 3000000 bps)
//! - Data formats: 5, 6, 7, 8 bits
//! - Parity: None, Even, Odd, Mark, Space
//! - Stop bits: 1, 1.5, 2
//! - Flow control: None, RTS/CTS, XON/XOFF
//! - Interrupt-driven and polling modes
//! - DMA support (where available)
//! - Power management integration
//! 
//! # Real-time Characteristics
//! 
//! - Deterministic interrupt latency
//! - Configurable buffer sizes for real-time requirements
//! - Priority-based transmission scheduling
//! - Overrun and error detection

#![deny(missing_docs)]
#![deny(clippy::undocumented_unsafe_blocks)]

/// Maximum number of UART instances
pub const MAX_UART_INSTANCES: usize = 4;

/// Default receive buffer size
pub const DEFAULT_RX_BUFFER_SIZE: usize = 256;

/// Default transmit buffer size
pub const DEFAULT_TX_BUFFER_SIZE: usize = 256;

/// Maximum baud rate supported
pub const MAX_BAUD_RATE: u32 = 3_000_000;

/// UART configuration parameters
#[derive(Debug, Clone, Copy)]
pub struct UartConfig {
    /// Baud rate in bits per second
    pub baud_rate: u32,
    /// Number of data bits (5-8)
    pub data_bits: u8,
    /// Stop bits configuration
    pub stop_bits: StopBits,
    /// Parity configuration
    pub parity: Parity,
    /// Flow control type
    pub flow_control: FlowControl,
    /// Enable receive timeout
    pub rx_timeout: bool,
    /// Receive timeout in milliseconds
    pub rx_timeout_ms: u32,
}

/// Stop bits configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    /// 1 stop bit
    One,
    /// 1.5 stop bits
    OnePointFive,
    /// 2 stop bits
    Two,
}

/// Parity configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    /// No parity
    None,
    /// Even parity
    Even,
    /// Odd parity
    Odd,
    /// Mark parity (always 1)
    Mark,
    /// Space parity (always 0)
    Space,
}

/// Flow control types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowControl {
    /// No flow control
    None,
    /// Hardware flow control (RTS/CTS)
    Hardware,
    /// Software flow control (XON/XOFF)
    Software,
}
