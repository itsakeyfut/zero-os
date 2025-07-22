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
