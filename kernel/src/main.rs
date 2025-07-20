//! Zero OS - Tactical Support Kernel
//! 
//! Kernel entry point for Zero OS, a tactical support operating system
//! Microkernel design inspired by Tock OS architecture

#![no_std]
#![no_main]
#![feature(panic_info_message)]

use core::panic::PanicInfo;

mod arch;
mod memory;
mod platform;
mod process;
mod syscalls;

use platform::Platform;
use process::ProcessManager;
use memory::MemoryManager;

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

#[macro_export]
macro_rules! debug_print {
    ($($arg:tt)*) => {
        #[cfg(debug_assertions)]
        {
            use core::fmt::Write;
            let mut debug_writer = arch::DebugWriter::new();
            let _ = write!(debug_writer, "[KERNEL] ");
            let _ = writeln!(debug_writer, $($arg)*);
        }
    };
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
