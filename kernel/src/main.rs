//! Zero OS - Tactical Support Kernel
//! 
//! Kernel entry point for Zero OS, a tactical support operating system
//! Microkernel design inspired by Tock OS architecture

#![no_std]
#![no_main]
#![feature(panic_info_message)]

use core::panic::PanicInfo;

mod arch;

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