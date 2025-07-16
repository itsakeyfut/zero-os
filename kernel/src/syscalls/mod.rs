//! System call Interface
//! 
//! This module provides the system call interface for Zero OS.
//! It implements a capability-based system call mechanism that provides
//! secure communication between user processes and the kernel.
//! 
//! # Design Principles
//! 
//! - Capability-based security for all operations
//! - Minimal overhead for real-time performance
//! - Type-safe system call arguments
//! - Non-blocking operations where possible
//! - Comprehensive error handling
//! 
//! # System Call Categories
//! 
//! - **Process Management**: spawn, exit, wait, signal
//! - **Memory Management**: map, munmap, mprotect, brk
//! - **Inter-Process Communication**: send, receive, create_channel
//! - **I/O Operations**: read, write, open, close
//! - **Time management*: sleep, timer_create, timer_set
//! - **Tactical Support**: anlyze_position, predict_move, evaluate_threat
//! 
//! # Sytem Call Convention (ARM)
//! 
//! ```text
//! - System call number in r7
//! - Arguments in r0-r5 (up to 6 arguments)
//! - Return value in r0
//! - Error code in r1 (if r0 indicates error)
//! - Invoke via SWI/SVC instruction
//! ```

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::mem;
use heapless::Vec;
use crate::process::{ProcessId, ProcessManager, ProcessError};
use crate::memory::MemoryManager;
use crate::ipc::{IpcManager, IpcChannelId, Message};
use crate::platform::Platform;

/// Maximum number of system call arguments
pub const MAX_SYSCALL_ARGS: usize = 6;
