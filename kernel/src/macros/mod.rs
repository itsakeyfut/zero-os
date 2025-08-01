//! Zero OS Kernel Macros
//! 
//! This module provides various macros for kernel development including
//! debug output, assertions, logging, and safety checks. All macros are
//! designed to be zero-cost in release builds when not needed.

#![deny(missing_docs)]

use core::fmt::Write;

#[macro_export]
macro_rules! debug_print {
    ($($arg:tt)*) => {
        #[cfg(debug_assertions)]
        {
            use core::fmt::Write;
            if let Ok(mut debug_writer) = $crate::arch::DebugWriter::new() {
                let _ = write!(debug_writer, "[KERNEL] ");
                let _ = writeln!(debug_writer, $($arg)*);
            }
        }
    };
}
