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

/// System call numbers
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum SystemCallNumber {
    // Process management (0-19)
    /// Exit current process
    Exit = 0,
    /// Fork a new process
    Fork = 1,
    /// Execute a new program
    Exec = 2,
    /// Wait for child process
    Wait = 3,
    /// Get process ID
    GetPid = 4,
    /// Get parent process ID
    GetPPid = 5,
    /// Send signal to process
    Kill = 6,
    /// Set process priority
    SetPriority = 7,
    /// Get process priority
    GetPriority = 8,

    // Memory management (20-39)
    /// Map memory region
    Mmap = 20,
    /// Unmap memory region
    Munmap = 21,
    /// Change memory protection
    Mrpotect = 22,
    /// Change data segment size
    Brk = 23,
    /// Allocate memory
    Malloc = 24,
    /// Free memory
    Free = 25,

    // I/O operations (40-59)
    /// Read from file descriptor
    Read = 40,
    /// Write to file descriptor
    Write = 41,
    /// Open file
    Open = 42,
    /// Close file descriptor
    Close = 43,
    /// Control I/O device

    // IPC operations (60-79)
    /// Send PIC message
    Send = 60,
    /// Receive IPC message
    Receive = 61,
    /// Create IPC channel
    CreateChannel = 62,
    /// Close IPC channel
    CloseChannel = 63,
    /// Connect to IPC channel
    Connect = 64,
    /// Accept IPC connection
    Accept = 65,

    // Time operations (80-99)
    Sleep = 80,
    /// Get current time
    GetTime = 81,
    /// Create timer
    TimerCreate = 82,
    /// Set timer
    TimerSet = 83,
    /// Delete timer
    TimerDelete = 84,

    // Tactical support operations (100-119)
    /// Analyze game position
    AnalyzePosition = 100,
    /// Predict next move
    PredictMove = 101,
    /// Evaluate threat level
    EvaluateThreat = 102,
    /// Generate move suggestions
    GenerateMoves = 103,
    /// Calcualte evaluation score
    CalculateScore = 104,

    // Sensor operations (120-139) - for future robotics
    /// Read sensor data
    ReadSensor = 120,
    /// Configure sensor
    ConfigureSensor = 121,
    /// Calibrate sensor
    CalibrateSensor = 122,

    // Actuator operations (140-159) - for future robotics
    /// Control motor
    ControlMotor = 140,
    /// Set servo position
    SetServo = 141,
    /// Emergency stop
    EmergencyStop = 142,

    // System information (200-219)
    /// Get system information
    GetSystemInfo = 200,
    /// Get process information
    GetProcessInfo = 201,
    /// Get memory information
    GetMemoryInfo = 202,
}
