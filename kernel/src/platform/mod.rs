
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
pub struct GpioConfig {
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

/// Result type for platform operations
pub type PlatformResult<T> = Result<T, PlatformError>;

/// Platform abstraction trait
pub trait PlatformInterface {
    /// Initialize platform early (before memory management)
    fn early_init(&mut self) -> PlatformResult<()>;
    
    /// Initialize platform late (after memory management)
    fn late_init(&mut self) -> PlatformResult<()>;
    
    /// Get platform type
    fn platform_type(&self) -> PlatformType;
    
    /// Get hardware capabilities
    fn hardware_capabilities(&self) -> HardwareCapabilities;
    
    /// Initialize UART
    fn init_uart(&mut self) -> PlatformResult<()>;
    
    /// Configure UART
    fn configure_uart(&mut self, config: UartConfig) -> PlatformResult<()>;
    
    /// Write data to UART
    fn uart_write(&mut self, data: &[u8]) -> PlatformResult<usize>;
    
    /// Read data from UART
    fn uart_read(&mut self, buffer: &mut [u8]) -> PlatformResult<usize>;
    
    /// Initialize timer
    fn init_timer(&mut self) -> PlatformResult<()>;
    
    /// Configure timer
    fn configure_timer(&mut self, config: TimerConfig) -> PlatformResult<()>;
    
    /// Get current timer value
    fn timer_value(&self) -> u64;
    
    /// Set timer interrupt
    fn set_timer_interrupt(&mut self, us: u64) -> PlatformResult<()>;
    
    /// Initialize GPIO
    fn init_gpio(&mut self) -> PlatformResult<()>;
    
    /// Configure GPIO pin
    fn configure_gpio(&mut self, config: GpioConfig) -> PlatformResult<()>;
    
    /// Set GPIO pin state
    fn gpio_set(&mut self, pin: u32, state: bool) -> PlatformResult<()>;
    
    /// Get GPIO pin state
    fn gpio_get(&self, pin: u32) -> PlatformResult<bool>;
    
    /// Get pending interrupt
    fn get_pending_interrupt(&mut self) -> Option<InterruptType>;
    
    /// Handle UART interrupt
    fn handle_uart_interrupt(&mut self);
    
    /// Handle GPIO interrupt
    fn handle_gpio_interrupt(&mut self);
    
    /// Enter low power mode
    fn enter_low_power_mode(&mut self);
    
    /// Check if should wake up from low power
    fn should_wake_up(&self) -> bool;
    
    /// Load init binary
    fn load_init_binary(&self) -> PlatformResult<&'static [u8]>;
    
    /// Flush I/O operations
    fn flush_io(&mut self);
    
    /// Shutdown platform
    fn shutdown(&mut self);
    
    /// Emergency shutdown
    fn emergency_shutdown(&mut self);
    
    /// System halt
    fn halt(&mut self) -> !;
}

/// Main platform structure
pub struct Platform {
    /// Platform implementation
    inner: target_platform::PlatformImpl,
    /// Platform type
    platform_type: PlatformType,
    /// Hardware capabilities
    capabilities: HardwareCapabilities,
    /// Initialization state
    initialized: bool,
}

impl Platform {
    /// Create a new platofmr instance
    pub const fn new() -> Self {
        Self {
            inner: target_platform::PlatformImpl::new(),
            platform_type: target_platform::PLATFORM_TYPE,
            capabilities: HardwareCapabilities::default(),
            initialized: false,
        }
    }

    /// Detect platform at runtime
    pub fn detect() -> PlatformType {
        target_platform::detect_platform()
    }
}

impl PlatformInterface for Platform {
    fn early_init(&mut self) -> PlatformResult<()> {
        if self.initialized {
            return Ok(());
        }
        
        // Platform-specific early initialization
        self.inner.early_init()?;
        
        // Detect hardware capabilities
        self.capabilities = self.inner.hardware_capabilities();
        
        crate::debug_print!("Platform early init: {:?}", self.platform_type);
        crate::debug_print!("Hardware capabilities: CPU cores={}, RAM={}MB", 
                           self.capabilities.cpu_cores,
                           self.capabilities.ram_size / (1024 * 1024));
        
        Ok(())
    }
    
    fn late_init(&mut self) -> PlatformResult<()> {
        // Platform-specific late initialization
        self.inner.late_init()?;
        
        self.initialized = true;
        
        crate::debug_print!("Platform late init completed");
        Ok(())
    }
    
    fn platform_type(&self) -> PlatformType {
        self.platform_type
    }
    
    fn hardware_capabilities(&self) -> HardwareCapabilities {
        self.capabilities
    }
    
    fn init_uart(&mut self) -> PlatformResult<()> {
        self.inner.init_uart()
    }
    
    fn configure_uart(&mut self, config: UartConfig) -> PlatformResult<()> {
        self.inner.configure_uart(config)
    }
    
    fn uart_write(&mut self, data: &[u8]) -> PlatformResult<usize> {
        self.inner.uart_write(data)
    }
    
    fn uart_read(&mut self, buffer: &mut [u8]) -> PlatformResult<usize> {
        self.inner.uart_read(buffer)
    }
    
    fn init_timer(&mut self) -> PlatformResult<()> {
        self.inner.init_timer()
    }
    
    fn configure_timer(&mut self, config: TimerConfig) -> PlatformResult<()> {
        self.inner.configure_timer(config)
    }
    
    fn timer_value(&self) -> u64 {
        self.inner.timer_value()
    }
    
    fn set_timer_interrupt(&mut self, us: u64) -> PlatformResult<()> {
        self.inner.set_timer_interrupt(us)
    }
    
    fn init_gpio(&mut self) -> PlatformResult<()> {
        self.inner.init_gpio()
    }
    
    fn configure_gpio(&mut self, config: GpioConfig) -> PlatformResult<()> {
        self.inner.configure_gpio(config)
    }
    
    fn gpio_set(&mut self, pin: u32, state: bool) -> PlatformResult<()> {
        self.inner.gpio_set(pin, state)
    }
    
    fn gpio_get(&self, pin: u32) -> PlatformResult<bool> {
        self.inner.gpio_get(pin)
    }
    
    fn get_pending_interrupt(&mut self) -> Option<InterruptType> {
        self.inner.get_pending_interrupt()
    }
    
    fn handle_uart_interrupt(&mut self) {
        self.inner.handle_uart_interrupt()
    }
    
    fn handle_gpio_interrupt(&mut self) {
        self.inner.handle_gpio_interrupt()
    }
    
    fn enter_low_power_mode(&mut self) {
        self.inner.enter_low_power_mode()
    }
    
    fn should_wake_up(&self) -> bool {
        self.inner.should_wake_up()
    }
    
    fn load_init_binary(&self) -> PlatformResult<&'static [u8]> {
        self.inner.load_init_binary()
    }
    
    fn flush_io(&mut self) {
        self.inner.flush_io()
    }
    
    fn shutdown(&mut self) {
        crate::debug_print!("Platform shutdown requested");
        self.inner.shutdown()
    }
    
    fn emergency_shutdown(&mut self) {
        // Emergency shutdown bypasses normal procedures
        self.inner.emergency_shutdown()
    }
    
    fn halt(&mut self) -> ! {
        crate::debug_print!("Platform halt");
        self.inner.halt()
    }
}

/// Get platform name as string
pub fn platform_name() -> &'static str {
    target_platform::PLATFORM_NAME
}

/// Get platform version
pub fn platform_version() -> &'static str {
    target_platform::PLATFORM_VERSION
}

/// Check if feature is supported on this platform
pub fn is_feature_supported(feature: &str) -> bool {
    target_platform::is_feature_supported(feature)
}

/// Get platform-specific configuration value
pub fn get_config_value(key: &str) -> Option<u32> {
    target_platform::get_config_value(key)
}

/// Platform-specific memory layout information
pub mod memory_layout {
    use crate::memory::{VirtualAddress, PhysicalAddress};

    /// Get kernel load address
    pub fn kernel_load_address() -> PhysicalAddress {
        super::target_platform::memory_layout::KERNEL_LOAD_ADDRESS
    }

    /// Get RAM start address
    pub fn ram_start() -> PhysicalAddress {
        super::target_platform::memory_layout::RAM_START
    }

    /// Get Ram size
    pub fn ram_size() -> usize {
        super::target_platform::memory_layout::RAM_SIZE
    }

    /// Get device memory start
    pub fn device_start() -> PhysicalAddress {
        super::target_platform::memory_layout::DEVICE_SIZE
    }
}

/// Platform-specific interrupt information
pub mod interrupts {
    /// Get timer interrupt number
    pub fn timer_irq() -> u32 {
        super::target_platform::interrupts::TIMER_IRQ
    }

    /// Get UART interrupt number
    pub fn uart_irq() -> u32 {
        super::target_platform::interrupts::UART_IRQ
    }

    /// Get GPIO interrupt number
    pub fn gpio_irq() -> u32 {
        super::target_platform::interrupts::GPIO_IRQ
    }

    /// Get total number of interrupts
    pub fn num_interrupts() -> u32 {
        super::target_platform::interrupts::NUM_INTERRUPTS
    }
}

/// Platform-specific register addresses
pub mod registers {
    /// Get UART base address
    pub fn uart_base() -> usize {
        super::target_platform::registers::UART_BASE
    }

    /// Get timer base address
    pub fn timer_base() -> usize {
        super::target_platform::registers::TIMER_BASE
    }

    /// Get GPIO base address
    pub fn gpio_base() -> usize {
        super::target_platform::registers::GPIO_BASE
    }

    /// Get interrupt controller base address
    pub fn interrupt_controller_base() -> usize {
        super::target_platform::registers::INTERRUPT_CONTROLLER_BASE
    }
}