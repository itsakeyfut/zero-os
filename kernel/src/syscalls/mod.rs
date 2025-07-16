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
    Mprotect = 22,
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
    Ioctl = 44,

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

impl TryFrom<u32> for SystemCallNumber {
    type Error = SystemCallError;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(SystemCallNumber::Exit),
            1 => Ok(SystemCallNumber::Fork),
            2 => Ok(SystemCallNumber::Exec),
            3 => Ok(SystemCallNumber::Wait),
            4 => Ok(SystemCallNumber::GetPid),
            5 => Ok(SystemCallNumber::GetPPid),
            6 => Ok(SystemCallNumber::Kill),
            7 => Ok(SystemCallNumber::SetPriority),
            8 => Ok(SystemCallNumber::GetPriority),

            20 => Ok(SystemCallNumber::Mmap),
            21 => Ok(SystemCallNumber::Munmap),
            22 => Ok(SystemCallNumber::Mprotect),
            23 => Ok(SystemCallNumber::Brk),
            24 => Ok(SystemCallNumber::Malloc),
            25 => Ok(SystemCallNumber::Free),

            40 => Ok(SystemCallNumber::Read),
            41 => Ok(SystemCallNumber::Write),
            42 => Ok(SystemCallNumber::Open),
            43 => Ok(SystemCallNumber::Close),
            44 => Ok(SystemCallNumber::Ioctl),

            60 => Ok(SystemCallNumber::Send),
            61 => Ok(SystemCallNumber::Receive),
            62 => Ok(SystemCallNumber::CreateChannel),
            63 => Ok(SystemCallNumber::CloseChannel),
            64 => Ok(SystemCallNumber::Connect),
            65 => Ok(SystemCallNumber::Accept),

            80 => Ok(SystemCallNumber::Sleep),
            81 => Ok(SystemCallNumber::GetTime),
            82 => Ok(SystemCallNumber::TimerCreate),
            83 => Ok(SystemCallNumber::TimerSet),
            84 => Ok(SystemCallNumber::TimerDelete),

            100 => Ok(SystemCallNumber::AnalyzePosition),
            101 => Ok(SystemCallNumber::PredictMove),
            102 => Ok(SystemCallNumber::EvaluateThreat),
            103 => Ok(SystemCallNumber::GenerateMoves),
            104 => Ok(SystemCallNumber::CalculateScore),

            120 => Ok(SystemCallNumber::ReadSensor),
            121 => Ok(SystemCallNumber::ConfigureSensor),
            122 => Ok(SystemCallNumber::CalibrateSensor),

            140 => Ok(SystemCallNumber::ControlMotor),
            141 => Ok(SystemCallNumber::SetServo),
            142 => Ok(SystemCallNumber::EmergencyStop),

            200 => Ok(SystemCallNumber::GetSystemInfo),
            201 => Ok(SystemCallNumber::GetProcessInfo),
            202 => Ok(SystemCallNumber::GetMemoryInfo),

            _ => Err(SystemCallError::InvalidSystemCall),
        }
    }
}

/// System call arguments
#[derive(Debug, Clone, Copy)]
pub struct SystemCallArgs {
    /// Arguments array (up to 6 arguments)
    pub args: [usize; MAX_SYSCALL_ARGS],
}

impl SystemCallArgs {
    /// Create new system call arguments
    pub const fn new() -> Self {
        Self {
            args: [0; MAX_SYSCALL_ARGS],
        }
    }

    /// Create from argument array
    pub const fn from_array(args: [usize; MAX_SYSCALL_ARGS]) -> Self {
        Self { args }
    }

    /// Get argument at index
    pub fn get(&self, index: usize) -> Option<usize> {
        self.args.get(index).copied()
    }

    /// Get argument as pointer
    /// 
    /// # Safety
    /// 
    /// Caller must ensure the argument actualy contains a valid pointer
    pub unsafe fn get_ptr<T>(&self, index: usize) -> Option<*const T> {
        self.get(index).map(|addr| addr as *const T)
    }
}

/// System call errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum SystemCallError {
    /// Invalid system call number
    InvalidSystemCall = 1,
    /// Invalid argument
    InvalidArgument = 2,
    /// Permission denied
    PermissionDenied = 3,
    /// Process not found
    ProcessNotFound = 4,
    /// Out of memory
    OutOfMemory = 5,
    /// Resource busy
    ResourceBusy = 6,
    /// Resource not found
    NotFound = 7,
    /// Operation not supported
    NotSupported = 8,
    /// Invalid state
    InvalidState = 9,
    /// Timeout occurred
    Timeout = 10,
    /// Operation interrupted
    Interrupted = 11,
    /// Invalid file descriptor
    InvalidDescriptor = 12,
    /// End of file
    EndOfFile = 13,
    /// I/O error
    IoError = 14,
    /// Network error
    NetworkErorr = 15,
    /// Invalid address
    InvalidAddress = 16,
    /// Operation would block
    WouldBlock = 17,
    /// Resource limit exceeded
    ResourceLimitExceeded = 18,
    /// Invalid data format
    InvalidFormat = 19,
    /// Hardware error
    HardwareError = 20,
}

impl From<ProcessError> for SystemCallError {
    fn from(error: ProcessError) -> Self {
        match error {
            ProcessError::NotFound => SystemCallError::ProcessNotFound,
            ProcessError::PermissionDenied => SystemCallError::PermissionDenied,
            ProcessError::OutOfMemory => SystemCallError::OutOfMemory,
            ProcessError::InvalidState => SystemCallError::InvalidState,
            ProcessError::TooManyProcesses => SystemCallError::ResourceLimitExceeded,
            ProcessError::ResourceLimitExceeded  => SystemCallError::ResourceLimitExceeded,
            _ => SystemCallError::InvalidArgument,
        }
    }
}
