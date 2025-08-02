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

pub struct Kernel {
    process_manager: ProcessManager,
    memory_manager: MemoryManager,
    platform: Platform,
}

impl Kernel {
    pub fn new() -> Self {
        Self {
            process_manager: ProcessManager::new(),
            memory_manager: MemoryManager::new(),
            platform: Platform::new(),
        }
    }
}

#[no_mangle]
pub extern "C" fn kernel_main() -> ! {
    arch::early_debug_init();

    debug_print!("Zero OS - Tactical Support Kernel");
    debug_print!("Kernel starting...");

    let mut kernel = Kernel::new();

    // TODO: implment boot method
    // kernel.boot();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    if let Some(message) = info.message() {
        debug_print!("KERNEL PANIC: {}", message);
    }

    if let Some(location) = info.location() {
        debug_print!("Location: {}:{}:{}",
                    location.file(),
                    location.line(),
                    location.column());
    }
}

#[derive(Debug, Clone, Copy)]
pub enum KernelError {
    OutOfMemory,
    InvalidParameter,
    ProcessNotFound,
    ResourceUnavailable,
    PermissionDenied,
}

pub type KernelResult<T> = Result<T, KernelError>;
