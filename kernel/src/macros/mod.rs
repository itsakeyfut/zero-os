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

/// Current debug level (can be configured at compile time)
#[cfg(debug_assertions)]
pub const DEBUG_LEVEL: DebugLevel = DebugLevel::Debug;

#[cfg(not(debug_assertions))]
pub const DEBUG_LEVEL: DebugLevel = DebugLevel::Warning;

/// Enhanced debug print macro with levels and timestamps
macro_rules! debug_print {
    // Error level (always printed)
    (ERROR, $($arg:tt)*) => {
        $crate::macros::debug_print_impl($crate::macros::DebugLevel::Error, format_args!($($arg)*))
    };
    
    // Warning level
    (WARN, $($arg:tt)*) => {
        $crate::macros::debug_print_impl($crate::macros::DebugLevel::Warning, format_args!($($arg)*))
    };
    
    // Info level
    (INFO, $($arg:tt)*) => {
        $crate::macros::debug_print_impl($crate::macros::DebugLevel::Info, format_args!($($arg)*))
    };
    
    // Debug level
    (DEBUG, $($arg:tt)*) => {
        $crate::macros::debug_print_impl($crate::macros::DebugLevel::Debug, format_args!($($arg)*))
    };
    
    // Trace level
    (TRACE, $($arg:tt)*) => {
        $crate::macros::debug_print_impl($crate::macros::DebugLevel::Trace, format_args!($($arg)*))
    };
    
    // Default to info level if no level specified
    ($($arg:tt)*) => {
        $crate::macros::debug_print_impl($crate::macros::DebugLevel::Info, format_args!($($arg)*))
    };
}

/// Implementation of debug print with level checking
pub fn debug_print_impl(level: DebugLevel, arg: core::fmt::Arguments) {
    // Only print if the message level is at or above the current debug level
    if level <= DEBUG_LEVEL {
        print_with_metadata(level, args);
    }
}

/// Print message with metadata (timestamp, level, etc.)
fn print_with_metadata(level: DebugLevel, args: core::fmt::Arguments) {
    #[cfg(debug_assertions)]
    {
        // Get current time for timestamp
        let timestamp = crate::arch::target::Architecture::current_time_us();

        // Get level string
        let level_str = match level {
            DebugLevel::Error => "ERROR",
            DebugLevel::Warning => "WARNING",
            DebugLevel::Info => "INFO",
            DebugLevel::Debug => "DEBUG",
            DebugLevel::Trace => "TRACE",
        };

        // Try to get debug writer
        if let Ok(mut debug_writer) = crate::arch::DebugWriter::new() {
            // Print with timestamp and level
            let _ = write!(debug_writer, "[{:>10}.{:03}] [{}] ",
                        timestamp / 1000,
                        timestamp % 1000,
                        level_str);
            let _ = writeln!(debug_writer, "{}", args);
        }
    }

    #[cfg(not(debug_assertions))]
    {
        // In release builds, only show errors and warnings
        if level <= DebugLevel::Warning {
            if let Ok(mut debug_writer) = crate::arch::DebugWriter::new() {
                let level_str = match level {
                    DebugLevel::Error => "ERROR",
                    DebugLevel::Warning => "WARN ",
                    _ => "INFO ",
                };
                let _ = write!(debug_writer, "[{}] ", level_str);
                let _ = writeln!(debug_writer, "{}", args);
            }
        }
    }
}

/// Panic with formatted message and location info
#[macro_export]
macro_rules! kernel_panic {
    ($($arg:tt)*) => {
        panic!("KERNEL PANIC at {}:{}: {}", file!(), line!(), format_args!($($arg)*))
    };
}

/// Assert macro with custom panic message
#[macro_export]
macro_rules! kernel_assert {
    ($cond:expr) => {
        if !($cond) {
            $crate::kernel_panic!("Assertion failed: {}", stringify!($cond));
        }
    };
    ($cond:expr, $($arg:tt)*) => {
        if !($cond) {
            $crate::kernel_panic!("Assertion failed: {}: {}", stringify!($cond), format_args!($($arg)*));
        }
    };
}

/// Debug assertion that's only active in debug builds
#[macro_export]
macro_rules! debug_assert_kernel {
    ($($arg:tt)*) => {
        #[cfg(debug_assertions)]
        $crate::kernel_assert!($($arg)*);
    };
}

/// Ensure condition or return error
#[macro_export]
macro_rules! kernel_ensure {
    ($cond:expr, $error:expr) => {
        if !($cond) {
            $crate::debug_print!(ERROR, "Ensure failed: {} at {}:{}", 
                                stringify!($cond), file!(), line!());
            return Err($error);
        }
    };
    ($cond:expr, $error:expr, $($arg:tt)*) => {
        if !($cond) {
            $crate::debug_print!(ERROR, "Ensure failed: {}: {} at {}:{}", 
                                stringify!($cond), format_args!($($arg)*), file!(), line!());
            return Err($error);
        }
    };
}

/// Memory barrier macro for synchronization
#[macro_export]
macro_rules! memory_barrier {
    () => {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    };
    (acquire) => {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
    };
    (release) => {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
    };
}
