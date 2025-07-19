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

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baud_rate: 115200,
            data_bits: 8,
            stop_bits: StopBits::One,
            parity: Parity::None,
            flow_control: FlowControl::None,
            rx_timeout: false,
            rx_timeout_ms: 100,
        }
    }
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

/// UART error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    /// Frame error (stop bit not detected)
    FrameError,
    /// Parity Error
    ParityError,
    /// Buffer overrun
    Overrun,
    /// Break condition detected
    Break,
    /// Hardware timeout
    Timeout,
    /// FIFO error
    FifoError,
}

/// UART status flags
#[derive(Debug, Clone, Copy)]
pub struct UartStatus {
    /// Transmit FIFO empty
    pub tx_empty: bool,
    /// Transmit FIDO full
    pub tx_full: bool,
    /// Receive FIFO empty
    pub rx_empty: bool,
    /// Receive FIFO full
    pub rx_full: bool,
    /// Clear to send (CTS) status
    pub cts: bool,
    /// Data carrier detect (DCD) status
    pub dcd: bool,
    /// Data set ready (DSR) status
    pub dsr: bool,
    /// Rind indicator (RI) status
    pub ri: bool,
}

/// UART statistics
#[derive(Debug, Default, Clone, Copy)]
pub struct UartStats {
    /// Bytes transmitted
    pub bytes_transmitted: u64,
    /// Bytes received
    pub bytes_received: u64,
    /// Transmission errors
    pub tx_errors: u32,
    /// Reception errors
    pub rx_errors: u32,
    /// Buffer overruns
    pub overruns: u32,
    /// Frame errors
    pub frame_errors: u32,
    /// Parity errors
    pub parity_errors: u32,
}

/// UART register layout for PL011
#[repr(C)]
struct Pl011Registers {
    /// Data register
    dr: u32,
    /// Receive status register / Error clear register
    rsr_ecr: u32,
    /// Reserved
    _reserved1: [u32; 4],
    /// Flag register
    fr: u32,
    /// Reserved
    _reserved2: u32,
    /// Low-power counter register
    ilpr: u32,
    /// Integer baud rate register
    ibrd: u32,
    /// Fractional baud rate register
    fbrd: u32,
    /// Line control register
    lcr_h: u32,
    /// Control register
    cr: u32,
    /// Interrupt FIFO level select register
    ifls: u32,
    /// Interrupt mask set/clear register
    imsc: u32,
    /// Raw interrupt status register
    ris: u32,
    /// Masked interrupt status register
    mis: u32,
    /// Interrupt clear register
    icr: u32,
    /// DMA control register
    dmacr: u32,
}
