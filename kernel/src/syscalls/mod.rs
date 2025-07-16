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

    /// Get argument as mutable pointer
    /// 
    /// # Safety
    /// 
    /// Caller must ensure the argument actually contains a valid mutable pointer
    pub unsafe fn get_mut_ptr<T>(&self, index: usize) -> Option<*mut T> {
        self.get(index).map(|addr| addr as *mut T)
    }
}

/// System call structure
#[derive(Debug, Clone, Copy)]
pub struct SystemCall {
    /// System call number
    pub number: SystemCallNumber,
    /// Arguments
    pub args: SystemCallArgs,
    /// Calling process ID
    pub caller: ProcessId,
}

impl SystemCall {
    /// Create a new system call
    pub fn new(number: SystemCallNumber, args: SystemCallArgs, caller: ProcessId) -> Self {
        Self {
            number,
            args,
            caller,
        }
    }
}

/// System call result
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SystemCallResult {
    /// Return value (0 for success, error code for failure)
    pub value: isize,
    /// Optional secondary return value
    pub value2: usize,
}

impl SystemCallResult {
    /// Create a successful result
    pub const fn ok(value: usize) -> Self {
        Self {
            value: value as isize,
            value2: 0,
        }
    }

    /// Create a successful result with two values
    pub const fn ok2(value1: usize, value2: usize) -> Self {
        Self {
            value: value1 as isize,
            value2,
        }
    }

    /// Create an error result
    pub const fn err(error: SystemCallError) -> Self {
        Self {
            value: -(error as isize),
            value2: 0,
        }
    }

    /// Check if result is successful
    pub fn is_ok(&self) -> bool {
        self.value >= 0
    }

    /// Check if result is an error
    pub fn is_err(&self) -> bool {
        self.value < 0
    }

    /// Get error code if this is an error
    pub fn error(&self) -> Option<SystemCallError> {
        if self.is_err() {
            SystemCallError::try_from(-self.value as u32).ok()
        } else {
            None
        }
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
    InvalidFileDescriptor = 12,
    /// End of file
    EndOfFile = 13,
    /// I/O error
    IoError = 14,
    /// Network error
    NetworkError = 15,
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

impl TryFrom<u32> for SystemCallError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(SystemCallError::InvalidSystemCall),
            2 => Ok(SystemCallError::InvalidArgument),
            3 => Ok(SystemCallError::PermissionDenied),
            4 => Ok(SystemCallError::ProcessNotFound),
            5 => Ok(SystemCallError::OutOfMemory),
            6 => Ok(SystemCallError::ResourceBusy),
            7 => Ok(SystemCallError::NotFound),
            8 => Ok(SystemCallError::NotSupported),
            9 => Ok(SystemCallError::InvalidState),
            10 => Ok(SystemCallError::Timeout),
            11 => Ok(SystemCallError::Interrupted),
            12 => Ok(SystemCallError::InvalidFileDescriptor),
            13 => Ok(SystemCallError::EndOfFile),
            14 => Ok(SystemCallError::IoError),
            15 => Ok(SystemCallError::NetworkError),
            16 => Ok(SystemCallError::InvalidAddress),
            17 => Ok(SystemCallError::WouldBlock),
            18 => Ok(SystemCallError::ResourceLimitExceeded),
            19 => Ok(SystemCallError::InvalidFormat),
            20 => Ok(SystemCallError::HardwareError),
            _ => Err(()),
        }
    }
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

/// Main system call handler
/// 
/// This function dispatches system calls to appropriate handlers based on
/// the system call number.
pub fn handle_syscall(
    syscall: SystemCall,
    process_manager: &mut ProcessManager,
    memory_manager: &mut MemoryManager,
    ipc_manager: &mut IpcManager,
    platform: &mut Platform,
) -> SystemCallResult {
    // Verify calling process exists and is in valid state
    let caller = match process_manager.get_process(syscall.caller) {
        Some(process) => process,
        None => return SystemCallResult::err(SystemCallError::ProcessNotFound),
    };
    
    // Dispatch to appropriate handler
    match syscall.number {
        // Process management
        SystemCallNumber::Exit => process::handle_exit(syscall, process_manager),
        SystemCallNumber::Fork => process::handle_fork(syscall, process_manager, memory_manager),
        SystemCallNumber::Exec => process::handle_exec(syscall, process_manager, memory_manager),
        SystemCallNumber::Wait => process::handle_wait(syscall, process_manager),
        SystemCallNumber::GetPid => process::handle_getpid(syscall, process_manager),
        SystemCallNumber::GetPPid => process::handle_getppid(syscall, process_manager),
        SystemCallNumber::Kill => process::handle_kill(syscall, process_manager),
        SystemCallNumber::SetPriority => process::handle_set_priority(syscall, process_manager),
        SystemCallNumber::GetPriority => process::handle_get_priority(syscall, process_manager),
        
        // Memory management
        SystemCallNumber::Mmap => memory::handle_mmap(syscall, process_manager, memory_manager),
        SystemCallNumber::Munmap => memory::handle_munmap(syscall, process_manager, memory_manager),
        SystemCallNumber::Mprotect => memory::handle_mprotect(syscall, process_manager, memory_manager),
        SystemCallNumber::Brk => memory::handle_brk(syscall, process_manager, memory_manager),
        SystemCallNumber::Malloc => memory::handle_malloc(syscall, process_manager, memory_manager),
        SystemCallNumber::Free => memory::handle_free(syscall, process_manager, memory_manager),
        
        // I/O operations
        SystemCallNumber::Read => fs::handle_read(syscall, process_manager, platform),
        SystemCallNumber::Write => fs::handle_write(syscall, process_manager, platform),
        SystemCallNumber::Open => fs::handle_open(syscall, process_manager, platform),
        SystemCallNumber::Close => fs::handle_close(syscall, process_manager, platform),
        SystemCallNumber::Ioctl => fs::handle_ioctl(syscall, process_manager, platform),
        
        // IPC operations
        SystemCallNumber::Send => ipc::handle_send(syscall, process_manager, ipc_manager),
        SystemCallNumber::Receive => ipc::handle_receive(syscall, process_manager, ipc_manager),
        SystemCallNumber::CreateChannel => ipc::handle_create_channel(syscall, process_manager, ipc_manager),
        SystemCallNumber::CloseChannel => ipc::handle_close_channel(syscall, process_manager, ipc_manager),
        SystemCallNumber::Connect => ipc::handle_connect(syscall, process_manager, ipc_manager),
        SystemCallNumber::Accept => ipc::handle_accept(syscall, process_manager, ipc_manager),
        
        // Time operations
        SystemCallNumber::Sleep => time::handle_sleep(syscall, process_manager, platform),
        SystemCallNumber::GetTime => time::handle_get_time(syscall, process_manager, platform),
        SystemCallNumber::TimerCreate => time::handle_timer_create(syscall, process_manager, platform),
        SystemCallNumber::TimerSet => time::handle_timer_set(syscall, process_manager, platform),
        SystemCallNumber::TimerDelete => time::handle_timer_delete(syscall, process_manager, platform),
        
        // Tactical support operations
        SystemCallNumber::AnalyzePosition => tactical::handle_analyze_position(syscall, process_manager),
        SystemCallNumber::PredictMove => tactical::handle_predict_move(syscall, process_manager),
        SystemCallNumber::EvaluateThreat => tactical::handle_evaluate_threat(syscall, process_manager),
        SystemCallNumber::GenerateMoves => tactical::handle_generate_moves(syscall, process_manager),
        SystemCallNumber::CalculateScore => tactical::handle_calculate_score(syscall, process_manager),
        
        // Future robotics operations (not implemented yet)
        SystemCallNumber::ReadSensor => SystemCallResult::err(SystemCallError::NotSupported),
        SystemCallNumber::ConfigureSensor => SystemCallResult::err(SystemCallError::NotSupported),
        SystemCallNumber::CalibrateSensor => SystemCallResult::err(SystemCallError::NotSupported),
        SystemCallNumber::ControlMotor => SystemCallResult::err(SystemCallError::NotSupported),
        SystemCallNumber::SetServo => SystemCallResult::err(SystemCallError::NotSupported),
        SystemCallNumber::EmergencyStop => SystemCallResult::err(SystemCallError::NotSupported),
        
        // System information
        SystemCallNumber::GetSystemInfo => handle_get_system_info(syscall, process_manager),
        SystemCallNumber::GetProcessInfo => handle_get_process_info(syscall, process_manager),
        SystemCallNumber::GetMemoryInfo => handle_get_memory_info(syscall, process_manager, memory_manager),
    }
}

/// Handle get system information system call
fn handle_get_system_info(
    syscall: SystemCall,
    _process_manager: &mut ProcessManager,
) -> SystemCallResult {
    // Return basic system information
    // In a real implementation, this would return comprehensive system stats
    let system_info = 0x12345678; // Placeholder system info
    SystemCallResult::ok(system_info)
}

/// Handle get process information system call
fn handle_get_process_info(
    syscall: SystemCall,
    process_manager: &mut ProcessManager,
) -> SystemCallResult {
    let target_pid = match ProcessId::try_from(syscall.args.get(0).unwrap_or(0) as u32) {
        Ok(pid) => pid,
        Err(_) => return SystemCallResult::err(SystemCallError::InvalidArgument),
    };

    if let Some(process) = process_manager.get_process(target_pid) {
        // Return process information (simplified)
        SystemCallResult::ok2(process.id.as_u32() as usize, process.memory_usage())
    } else {
        SystemCallResult::err(SystemCallError::ProcessNotFound)
    }
}

/// Handle get memory information system call
fn handle_get_memory_info(
    syscall: SystemCall,
    _process_manager: &mut ProcessManager,
    memory_manager: &mut MemoryManager,
) -> SystemCallResult {
    let stats = memory_manager.usage_stats();
    SystemCallResult::ok2(stats.total_memory, stats.free_memory)
}

/// System call point from assembly
/// 
/// This function is called from the ARM SWI/SVC exception handler.
/// 
/// # Arguments
/// 
/// * `system_number` - System call number from r7
/// * `args` - Arguments from r0-r5
/// * `caller` - Calling process ID
/// 
/// # Safety
/// 
/// This function is called from exception context and must be careful
/// with memory access and system state.
#[no_mangle]
pub extern "C" fn syscall_entry(
    syscall_number: u32,
    args: [usize; MAX_SYSCALL_ARGS],
    caller: u32,
) -> SystemCallResult {
    // Convert arguments to system call structure
    let number = match SystemCallNumber::try_from(syscall_number) {
        Ok(num) => num,
        Err(_) => return SystemCallResult::err(SystemCallError::InvalidSystemCall),
    };

    let syscall_args = SystemCallArgs::from_array(args);
    let caller_pid = ProcessId::new(caller);

    let syscall = SystemCall::new(number, syscall_args, caller_pid);

    // Get kernel instance and handle system call
    let kernel = crate::get_kernel();
    handle_syscall(
        syscall,
        &mut kernel.process_manager,
        &mut kernel.memory_manager,
        &mut kernel.ipc_router,
        &mut kernel.platform,
    )
}
