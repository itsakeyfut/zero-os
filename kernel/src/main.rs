//! Zero OS - Safety-Critical Real-time Kernel
//! 
//! Main kernel entry point and initialization sequence.
//! This module handles the complete system initialization from hardware
//! reset to a fully operational kernel state.

#![no_std]
#![no_main]
#![feature(panic_info_message)]
#![deny(unsafe_op_in_unsafe_fn)]
#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::panic::PanicInfo;
use crate::{
    arch,
    memory::MemoryManager,
    platform::Platform,
    process::ProcessManager,
    syscalls,
    KernelResult,
    KernelError,
};

/// Kernel version information
pub const KERNEL_VERSION: &str = env!("CARGO_PKG_VERSION");
pub const KERNEL_NAME: &str = "Zero OS";
pub const BUILD_TARGET: &str = env!("TARGET");

/// Kernel initialization phases
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InitPhase {
    /// Hardware initialization
    Hardware,
    /// Memory management setup
    Memory,
    /// Process management setup
    Process,
    /// System services startup
    Services,
    /// User space initialization
    UserSpace,
    /// System ready
    Ready,
}

/// Kernel state management
#[derive(Debug)]
struct KernelState {
    /// Current initialization phase
    current_phase: InitPhase,
    /// Boot timestamp
    boot_time: u64,
    /// Memory manager
    memory_manager: Option<MemoryManager>,
    /// Process manager
    process_manager: Option<ProcessManager>,
    /// Platform abstraction
    platform: Option<Platform>,
    /// Initialization completed successfully
    initialized: bool,
}

impl KernelState {
    /// Create a new kernel state
    const fn new() -> Self {
        Self {
            current_phase: InitPhase::Hardware,
            boot_time: 0,
            memory_manager: None,
            process_manager: None,
            platform: None,
            initialized: false,
        }
    }

    /// Get current initialization phase
    pub fn current_phase(&self) -> InitPhase {
        self.current_phase
    }

    /// Check if kernel is fully initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get boot time
    pub fn boot_time(&self) -> u64 {
        self.boot_time
    }
}

/// Global kernel state
static mut KERNEL_STATE: KernelState = KernelState::new();

/// Get reference to global kernel state
/// 
/// # Safety
/// 
/// This function should only be called from kernel code in single-threaded
/// context during initialization, or from interrupt handlers.
pub unsafe fn kernel_state() -> &'static mut KernelState {
    // SAFETY: Caller guarantees single-threaded access
    unsafe { &mut KERNEL_STATE }
}

/// Kernel main entry point
/// 
/// This is called from the architecture-specific boot code after basic
/// hardware initialization is complete.
#[no_mangle]
pub extern "C" fn kernel_main() -> ! {
    // Initialize early debug output
    arch::early_debug_init();

    // Print boot banner
    print_boot_banner();

    // Initialize kernel
    match initialize_kernel() {
        Ok(()) => {
            debug_print!("Kernel initialization completed successfully");
            run_kernel()
        }
        Err(error) => {
            debug_print!("FATAL: Kernel initialization failed: {:?}", error);
            panic!("Kernel initialization failed: {:?}", error);
        }
    }
}
