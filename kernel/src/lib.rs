//! Zero OS Kernel Library
//! 
//! Core kernel functionality and public interfaces for the Zero OS.
//! This library provides the foundational abstractions for process management,
//! memory management, inter-process communication, and real-time scheduling.
//! 
//! # Architecture
//! 
//! The kernel follows a microkernel architecture inspired by Tock OS:
//! - Minimal kernel space with essential services only
//! - User space applications isolated through memory protection
//! - Capsule-based driver architecture for hardware abstraction
//! - Real-time scheduling with priority inheritance
//! - Capability-based security for safe resource access
//! 
//! # Safety
//! 
//! All unsafe operations are carefully documented and justified.
//! The kernel maintains memory safety through:
//! - Rust's type system for compile-time guarantees
//! - Runtime bounds checking where necessary
//! - Process isolation with hardware memory protection
//! - Capability-based access control for system resources

#![no_std]
#![no_main]
#![deny(unsafe_op_in_unsafe_fn)]
#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]
#![warn(clippy::panic)]
#![warn(clippy::unwrap_used)]

// Enable required unstable features
#![feature(panic_info_message)]
#![feature(alloc_error_handler)]
#![feature(core_intrinsics)]

// External crate imports
extern crate alloc;

// Re-export commonly used types and traits
pub use alloc::{boxed::Box, string::String, vec::Vec};

// Module declarations
pub mod macros;
pub mod arch;
pub mod memory;
pub mod process;
pub mod syscalls;
pub mod platform;
pub mod grants;
pub mod drivers;
pub mod safety;

// Public exports for kernel users
pub use arch::{Architecture, CpuContext, InterruptType, MemoryRegion};
pub use memory::{MemoryManager, VirtualAddress, PhysicalAddress, MemoryFlags};
pub use process::{ProcessManager, ProcessId, Process, Priority};
pub use platform::{Platform, HardwareCapabilities};
pub use grants::{Grant, GrantId, GrantPermissions, GrantCapability};

/// Core kernel error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum KernelError {
    /// Out of memory
    OutOfMemory = 1,
    /// Invalid parameter provided
    InvalidParameter = 2,
    /// Process not found
    ProcessNotFound = 3,
    /// Resource is currently unavailable
    ResourceUnavailable = 4,
    /// Permission denied
    PermissionDenied = 5,
    /// Hardware error occurred
    HardwareError = 6,
    /// IPC communication error
    IpcError = 7,
    /// System in invalid state
    InvalidState = 8,
    /// Timeout occurred
    Timeout = 9,
    /// Operation interrupted
    Interrupted = 10,
    /// Resource limit exceeded
    ResourceLimitExceeded = 11,
    /// Unsupported operation
    UnsupportedOperation = 12,
    /// System call error
    SystemCallError = 13,
    /// Scheduler error
    SchedulerError = 14,
    /// Memory management error
    MemoryError = 15,
    /// Driver error
    DriverError = 16,
    /// Safety violation
    SafetyViolation = 17,
}

impl KernelError {
    /// Convert kernel error to system call error code
    pub fn to_syscall_error(self) -> u32 {
        self as u32
    }

    /// Check if error is recoverable
    pub fn is_recoverable(self) -> bool {
        match self {
            KernelError::OutOfMemory => false,
            KernelError::HardwareError => false,
            KernelError::SafetyViolation => false,
            KernelError::InvalidState => false,
            _ => true,
        }
    }

    /// Get error severity level
    pub fn severity(self) -> ErrorSeverity {
        match self {
            KernelError::OutOfMemory |
            KernelError::HardwareError |
            KernelError::SafetyViolation => ErrorSeverity::Critical,

            KernelError::ProcessNotFound |
            KernelError::PermissionDenied |
            KernelError::InvalidState => ErrorSeverity::Major,
            
            KernelError::ResourceUnavailable |
            KernelError::Timeout |
            KernelError::Interrupted => ErrorSeverity::Minor,
            
            _ => ErrorSeverity::Warning,
        }
    }
}

/// Error severity levels for kernel errors
#[derive(Debug, Clonem, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ErrorSeverity {
    /// Critical error - system may be unstable
    Critical = 0,
    /// Major error - significant functionality impacted
    Major = 1,
    /// Minor error - limited impact
    Minor = 2,
    /// Warning - potential issue
    Warning = 3,
}

/// Result type for kernel operations
pub type KernelResult<T> = Result<T, KernelError>;

/// Kernel configuration parameters
#[derive(Debug, Clone)]
pub struct KernelConfig {
    /// Maximum number of processes
    pub max_processes: usize,
    /// Kernel heap size
    pub kernel_heap_size: usize,
    /// Timer frequency in Hz
    pub timer_frequency: u32,
    /// Enable debug output
    pub debug_enabled: bool,
    /// Enable safety checks
    pub safety_checks_enabled: bool,
    /// Stack size for kernel threads
    pub kernel_stack_size: usize,
    /// Priority levels
    pub priority_levels: u8,
    /// Scheduler quantum in microseconds
    pub scheduler_quantum_us: u64,
}

impl Default for KernelConfig {
    fn default() -> Self {
        Self {
            max_processes: 64,
            kernel_heap_size: 64 * 1024 * 1024, // 64MB
            timer_frequency: 1000, // 1kHz
            debug_enabled: cfg!(debug_assertions),
            safety_checks_enabled: true,
            kernel_stack_size: 64 * 1024, // 64KB
            priority_levels: 32,
            scheduler_quantum_us: 10_000, // 10ms
        }
    }
}

/// Kernel statistics for monitoring and debugging
#[derive(Debug, Default, Clone, Copy)]
pub struct KernelStats {
    /// Total system calls handled
    pub total_syscalls: u64,
    /// Total interrupts handled
    pub total_interrupts: u64,
    /// Total context switches
    pub total_context_switches: u64,
    /// Total page faults
    pub total_page_faults: u64,
    /// Current memory usage
    pub memory_used_bytes: usize,
    /// Peak memory usage
    pub memory_peak_bytes: usize,
    /// Number of active processes
    pub active_processes: u32,
    /// CPU utilization (0-100)
    pub cpu_utilization: u8,
    /// System uptime in microseconds
    pub uptime_us: u64,
    /// Number of kernel errors
    pub kernel_errors: u32,
    /// Number of safety violations
    pub safety_violations: u32,
}

impl KernelStats {
    /// Create a new kernel statistics
    pub const fn new() -> Self {
        Self {
            total_syscalls: 0,
            total_interrupts: 0,
            total_context_switches: 0,
            total_page_faults: 0,
            memory_used_bytes: 0,
            memory_peak_bytes: 0,
            active_processes: 0,
            cpu_utilization: 0,
            uptime_us: 0,
            kernel_errors: 0,
            safety_violations: 0,
        }
    }

    /// Update memory usage statistics
    pub fn update_memory_usage(&mut self, used_bytes: usize) {
        self.memory_used_bytes = used_bytes;
        if used_bytes > self.memory_peak_bytes {
            self.memory_peak_bytes = used_bytes;
        }
    }

    /// Record a system call
    pub fn record_syscall(&mut self) {
        self.total_syscalls = self.total_syscalls.saturating_add(1);
    }

    /// Record an interrupt
    pub fn record_interrupt(&mut self) {
        self.total_interrupts = self.total_interrupts.saturating_add(1);
    }

    /// Record a context switch
    pub fn record_context_switch(&mut self) {
        self.total_context_switches = self.total_context_switches.saturating_add(1);
    }

    /// Record a page fault
    pub fn record_page_fault(&mut self) {
        self.total_page_faults = self.total_page_faults.saturating_add(1);
    }

    /// Record a kernel error
    pub fn record_error(&mut self, error: KernelError) {
        self.kernel_errors = self.kernel_errors.saturating_add(1);

        if error == KernelError::SafetyViolation {
            self.safety_violations = self.safety_violations.saturating_add(1);
        }
    }

    /// Update CPU utilization
    pub fn update_cpu_utilization(&mut self, utilization: u8) {
        self.cpu_utilization = utilization.min(100);
    }

    /// Update uptime
    pub fn update_uptime(&mut self, uptime_us: u64) {
        self.uptime_us = uptime_us;
    }
}

/// Global kernel statistics
static mut KERNEL_STATS: KernelStats = KernelStats::new();

/// Get reference to global kernel statistics
/// 
/// # Safety
/// 
/// This function should only be called from kernel code with appropriate
/// synchrinization.
pub unsafe fn kernel_stats() -> &'static mut KernelStats {
    // SAFETY: Caller guarantees proper synchronization
    unsafe { &mut KERNEL_STATS }
}

/// Kernel version information
pub mod version {
    /// Major version number
    pub const MAJOR: u32 = 0;
    /// Minor version number
    pub const MINOR: u32 = 1;
    /// Patch version number
    pub const PATCH: u32 = 0;
    /// Pre-release identifier
    pub const PRE_RELEASE: Option<&str> = Some("alpha");
    /// Build metadata
    pub const BUILD_METADATA: Option<&str> = Some(env!("GIT_HASH"));

    /// Get a version string
    pub fn version_string() -> &'static str {
        env!("CARGO_PKG_VERSION")
    }

    /// Get full version with metadata
    pub fn full_version() -> alloc::string::String {
        let mut version = alloc::format!("{}.{}.{}", MAJOR, MINOR, PATCH);

        if let Some(pre) = PRE_RELEASE {
            version.push('-');
            version.push_str(pre);
        }

        if let Some(build) = BUILD_METADATA {
            version.push('+');
            version.push_str(build);
        }

        version
    }
}

/// Kernel assertions for safety-critical code
pub mod assertions {
    use super::KernelError;

    /// Assert condition with kernel error
    #[macro_export]
    macro_rules! kernel_assert {
        ($cond:expr) => {
            if !($cond) {
                panic!("Kernel assertion failed: {}", stringify!($cond));
            }
        };
        ($cond:expr, $msg:expr) => {
            if !($cond) {
                panic!("Kernel assertion failed: {}: {}", stringify!($cond), $msg);
            }
        };
    }

    /// Assert condition and return error if false
    #[macro_export]
    macro_rules! kernel_ensure {
        ($cond:expr. $error:expr) => {
            if !($cond) {
                return Err($error);
            }
        };
    }

    /// Bounds checking for array access
    pub fn check_bounds(index: usize, len: usize) -> Result<(), KernelError> {
        if index >= len {
            Err(KernelError::InvalidParameter)
        } else {
            Ok(())
        }
    }

    /// Alignment checking
    pub fn check_alignment(addr: usize, align: usize) -> Result<(), KernelError> {
        if align == 0 || !align.is_power_of_two() {
            return Err(KernelError::InvalidParameter);
        }

        if addr & align != 0 {
            Err(KernelError::InvalidParameter)
        } else {
            Ok(())
        }
    }

    /// Null pointer checking
    pub fn check_null_ptr<T>(ptr: *const T) -> Result<(), KernelError> {
        if ptr.is_null() {
            Err(KernelError::InvalidParameter)
        } else {
            Ok(())
        }
    }
}

/// Utility functions for kernel development
pub mod utils {
    /// Align value up to the next multiple of alignment
    pub const fn align_up(value: usize, alignment: usize) -> usize {
        (value + alignment - 1) & !(alignment - 1)
    }

    /// Align value down to the previous multiple of alignment
    pub const fn align_down(value: usize, alignment: usize) -> usize {
        value & !(alignment - 1)
    }

    /// Check if value is aligned to alignment
    pub const fn is_aligned(value: usize, alignment: usize) -> bool {
        value & (alignment - 1) == 0
    }

    /// Convert bytes to kilobytes
    pub const fn bytes_to_kb(bytes: usize) -> usize {
        bytes / 1024
    }

    /// Convert bytes to megabytes
    pub const fn bytes_to_mb(bytes: usize) -> usize {
        bytes / (1024 * 1024)
    }

    /// Convert microseconds to milliseconds
    pub const fn us_to_ms(us: u64) -> u64 {
        us / 1000
    }

    /// Convert microseconds to seconds
    pub const fn us_to_s(us: u64) -> u64 {
        us / 1_000_000
    }
}

/// Kernel initialization and main entry point
pub use crate::main::{kernel_main, get_kernel_info, KernelInfo};

/// Export main module for kernel entry point
pub mod main;

// Static assertions for kernel configuration
static_assertions::const_assert!(core::mem::size_of::<KernelError>() == 4);
static_assertions::const_assert!(core::mem::align_of::<KernelError>() == 4);

// Compile-time feature checks
#[cfg(not(target_pointer_width = "32"))]
compile_error!("Zero OS currently only supports 32-bit architectures");

#[cfg(not(target_endian = "little"))]
compile_error!("Zero OS currently only supports little-endian architectures");

// Ensure we're building for a supported architecture
#[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
compile_error!("Unsupported target architecture. Zero OS supports ARM only.");

// Ensure no_std environment
#[cfg(feature = "std")]
compile_error!("Zero OS kernel must be built in no_std environment");
