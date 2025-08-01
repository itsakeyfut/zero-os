//! Zero OS Kernel Macros
//! 
//! This module provides various macros for kernel development including
//! debug output, assertions, logging, and safety checks. All macros are
//! designed to be zero-cost in release builds when not needed.

#![deny(missing_docs)]

use core::fmt::Write;

/// Debug output levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum DebugLevel {
    /// Error messages - always shown
    Error = 0,
    /// Warning messages
    Warning = 1,
    /// Information messages
    Info = 2,
    /// Debug messages
    Debug = 3,
    /// Trace messages - most verbose
    Trace = 4,
}

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
