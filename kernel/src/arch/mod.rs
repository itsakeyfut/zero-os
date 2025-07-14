//! Architecture Abstraction Layer
//! 
//! This module provides hardware abstraction for different target architectures.
//! It defines common interfaces that must be implemented by each target platform,
//! allowing the kernel to remain architecture-agnostic while supporting multiple
//! hardware platforms.
//! 
//! # Supported Architectures
//! 
//! - ARM Cortex-A (ARMv7-A) - Primary target for embedded systems
//! - ARM Cortex-M (Thumb) - Microcontroller support
//! - x86_64 - Development and testing platform
//! 
//! # Design Principles
//! - Minimal abstraction overhead
//! - Clear separation of architecture-specific code
//! - Compile-time optimization through conditional compilation
//! - Safe abstractions over unsafe hardware operations

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::fmt;

// Architecture-specific module imports
#[cfg(target_arch = "arm")]
pub mod arm;

#[cfg(any(target_arch = "argm", target_arch = "aarch64"))]
pub use arm as target;


/// Interrupt types that can occur in the system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptType {
    /// Timer interrupt for scheduling
    Timer,
    /// UART communication interrupt
    Uart,
    /// GPIO external interrupt
    Gpio,
    /// System call interrupt
    SystemCall,
    /// Memory management fault
    MemroyFault,
    /// Undefined instruction
    UndefinedInstruction,
    /// Data abort
    DataAbort,
    /// Prefetch abort
    PrefetchAbort,
    /// IRQ interrupt
    Irq,
    /// FIQ interrupt (fast interrupt)
    Fiq,
}

/// CPU execution context for process switching
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct CpuContext {
    /// General purpose registers
    pub registers: [u32; 16],
    /// Program status register
    pub cpsr: u32,
    /// Stack pointer
    pub stack_pointer: u32,
    /// Program counter
    pub program_counter: u32,
}

impl Default for CpuContext {
    fn default() -> Self {
        Self {
            registers: [0; 16],
            cpsr: 0,
            stack_pointer: 0,
            program_counter: 0,
        }
    }
}

/// Memory protection attributes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryProtection {
    /// No access allowed
    None,
    /// Read-only access
    ReadOnly,
    /// Read-writer access
    ReadWrite,
    /// Execute-only access
    ExecuteOnly,
    /// Read-execute access
    ReadExecute,
    /// Full access (read-write-execute)
    Full,
}

/// Memory region descriptor
#[derive(Debug, Clone, Copy)]
pub struct MemoryRegion {
    /// Start address of the region
    pub start: usize,
    /// Size of the region in bytes
    pub size: usize,
    /// Protection attributes
    pub protection: MemoryProtection,
    /// Whether this region is cacheable
    pub cacheable: bool,
    /// Whether this region is bufferable
    pub bufferable: bool,
}

/// Architecture-specific error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArchError {
    /// Invalid memory address
    InvalidAddress,
    /// Memory alignment error
    AlignmentError,
    /// Unsupported operation
    UnsupportedOperation,
    /// Hardware fault
    HardwareFault,
    /// Invalid register access
    InvalidRegister,
    /// Privilege violation
    PrivilegeViolation,
}

/// Result type for architecture operations
pub type ArchResult<T> = Result<T, ArchError>;

/// Architecture abstraction trait
/// 
/// This trait defines the common interface that all supported architectures
/// must implement. It provides abstractions for:
/// - Interrupt handling
/// - Memory management
/// - Context switching
/// - Low-level system operations
pub trait Architecture {
    /// Initialize the architecture-specific components
    fn init() -> ArchResult<()>;

    /// Enable interrupts globally
    fn enable_interrupts();

    /// Disable interrupts globally
    fn disable_interrupts();

    /// Check if interrupts are enabled
    fn interrupts_enabled() -> bool;

    /// Wait for interrupt (low power state)
    fn wait_for_interrupt();

    /// Halt the system
    fn halt_system() -> !;

    /// Get the current CPU context
    fn get_current_context() -> CpuContext;

    /// Set the CPU context (for process switching)
    /// 
    /// # Safety
    /// 
    /// This function directly modifies CPU registers and should only be called
    /// during controlled context switches.
    unsafe fn set_context(context: &CpuContext);

    /// Set up memory protection for a region
    /// 
    /// # Safety
    /// 
    /// This function modifies memory management unit settings and could affect
    /// system stability if used incorrectly.
    unsafe fn setup_memory_protection(region: &MemoryRegion) -> ArchResult<()>;

    /// Flush instruction cache
    fn flush_icache();

    /// Flush data cache
    fn flush_dcache();

    /// Invalidate TLB (Translation Lookaside Buffer)
    fn invalidate_tlb();

    /// Get current time in microseconds since boot
    fn current_time_us() -> u64;

    /// Set up a timer interrupt to fire after the specified microseconds
    fn set_timer_interrupt(us: u64) -> ArchResult<()>;

    /// Handle an interrupt of the specified type
    fn handle_inerrupt(inerrupt_type: InterruptType);
}