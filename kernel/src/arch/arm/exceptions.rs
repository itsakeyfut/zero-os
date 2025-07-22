
//! ARM Exception Handling
//!
//! This module provides comprehensive exception handling for ARM Cortex-A processors.
//! It manages all types of exceptions including interrupts, data/prefetch aborts,
//! undefined instructions, and system calls.
//!
//! # ARM Exception Vector Table
//!
//! ARM processors use a vector table located at 0x00000000 (or 0xFFFF0000 for high vectors):
//!
//! ```text
//! 0x00: Reset               - System reset
//! 0x04: Undefined           - Undefined instruction
//! 0x08: SWI/SVC            - Software interrupt (system calls)
//! 0x0C: Prefetch Abort     - Instruction fetch memory abort
//! 0x10: Data Abort         - Data access memory abort  
//! 0x14: Reserved           - Not used
//! 0x18: IRQ                - Normal interrupt request
//! 0x1C: FIQ                - Fast interrupt request
//! ```
//!
//! # Exception Handling Process
//!
//! 1. **Hardware Response**: CPU saves state and jumps to vector
//! 2. **Vector Handler**: Assembly stub saves context and calls Rust handler
//! 3. **Rust Handler**: High-level exception processing
//! 4. **Context Restore**: Return to interrupted code or schedule new process
//!
//! # Processor Mode Changes
//!
//! Each exception type causes the processor to enter a specific mode:
//! - IRQ → IRQ mode (0x12)
//! - FIQ → FIQ mode (0x11)  
//! - SWI → Supervisor mode (0x13)
//! - Abort → Abort mode (0x17)
//! - Undefined → Undefined mode (0x1B)

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::arch::asm;
use crate::arch::{CpuContext, InterruptType};
use crate::process::{ProcessId, ProcessManager};
use crate::syscalls::{SystemCall, SystemCallResult, SystemCallArgs, SystemCallNumber};

/// Exception vector offsets
pub mod vectors {
    /// Reset vector offset
    pub const RESET: usize = 0x00;
    /// Undefined instruction vector offset
    pub const UNDEFINED: usize = 0x04;
    /// Software interrupt vector offset
    pub const SWI: usize = 0x08;
    /// Prefetch abort vector offset
    pub const PREFETCH_ABORT: usize = 0x0C;
    /// Data abort vector offset
    pub const DATA_ABORT: usize = 0x10;
    /// Reserved vector offset
    pub const RESERVED: usize = 0x14;
    /// IRQ vector offset
    pub const IRQ: usize = 0x18;
    /// FIQ vector offset
    pub const FIQ: usize = 0x1C;
}

/// ARM processor modes
pub mod modes {
    /// User mode
    pub const USER: u32 = 0x10;
    /// FIQ mode
    pub const FIQ: u32 = 0x11;
    /// IRQ mode
    pub const IRQ: u32 = 0x12;
    /// Supervisor mode
    pub const SUPERVISOR: u32 = 0x13;
    /// Abort mode
    pub const ABORT: u32 = 0x17;
    /// Undefined mode
    pub const UNDEFINED: u32 = 0x1B;
    /// System mode
    pub const SYSTEM: u32 = 0x1F;
}

/// CPSR (Current Program Status Register) flags
pub mod cpsr_flags {
    /// Negative flag
    pub const N: u32 = 1 << 31;
    /// Zero flag
    pub const Z: u32 = 1 << 30;
    /// Carry flag
    pub const C: u32 = 1 << 29;
    /// Overflow flag
    pub const V: u32 = 1 << 28;
    /// Sticky overflow flag
    pub const Q: u32 = 1 << 27;
    /// IRQ disable
    pub const I: u32 = 1 << 7;
    /// FIQ disable
    pub const F: u32 = 1 << 6;
    /// Thumb state
    pub const T: u32 = 1 << 5;
    /// Mode mask
    pub const MODE_MASK: u32 = 0x1F;
}

/// Data Fault Statue Register (DFSR) bits
pub mod dfsr_bits {
    /// External abort on non-linefetch
    pub const EXT_ABORT: u32 = 0x01;
    /// Alignment fault
    pub const ALIGNMENT: u32 = 0x02;
    /// Debug event
    pub const DEBUG: u32 = 0x04;
    /// Access flag fault (level 1)
    pub const ACCESS_FLAG_L1: u32 = 0x05;
    /// Translation fault (level 1)
    pub const TRANSLATION_L1: u32 = 0x06;
    /// Access flag fault (level 2)
    pub const ACCESS_FLAG_L2: u32 = 0x07;
    /// Translation fault (level 2)
    pub const TRANSLATION_L2: u32 = 0x08;
    /// Domain fault (level 1)
    pub const DOMAIN_L1: u32 = 0x09;
    /// Domain fault (level 2)
    pub const DOMAIN_L2: u32 = 0x0B;
    /// Permission fault (level 1)
    pub const PERMISSION_L1: u32 = 0x0D;
    /// Permission fault (level 2)
    pub const PERMISSION_L2: u32 = 0x0F;
}

/// Exception information structure
#[derive(Debug, Clone, Copy)]
pub struct ExceptionInfo {
    /// Exception type
    pub exception_type: ExceptionType,
    /// Fault address (for data/prefetch aborts)
    pub fault_address: Option<u32>,
    /// Fault status (for aborts)
    pub fault_status: Option<u32>,
    /// Instruction that caused the exception
    pub fault_instruction: Option<u32>,
    /// Process ID that caused the exception
    pub process_id: Option<ProcessId>,
}

/// Exception types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExceptionType {
    /// Reset exception
    Reset,
    /// Undefined instruction
    UndefinedInstruction,
    /// Software interrupt (system call)
    SoftwareInterrupt,
    /// Prefetch abort
    PrefetchAbort,
    /// Data abort
    DataAbort,
    /// IRQ interrupt
    Irq,
    /// FIQ interrupt
    Fiq,
}

/// Exception statistics for monitoring
#[derive(Debug, Default, Clone, Copy)]
pub struct ExceptionStats {
    /// Total exceptions handled
    pub total_exceptions: u64,
    /// Undefined instruction count
    pub undefined_instructions: u32,
    /// System call count
    pub system_calls: u64,
    /// Prefetch abort count
    pub prefetch_aborts: u32,
    /// Data abort count
    pub data_aborts: u32,
    /// IRQ count
    pub irq_count: u64,
    /// FIQ count
    pub fiq_count: u64,
    /// Nested interrupt count
    pub nested_interrupts: u32,
    /// Maximum interrupt nesting level
    pub max_nesting_level: u32,
}

/// Exception handler manager
pub struct ExceptionManager {
    /// Exception statistics
    stats: ExceptionStats,
    /// Current interrupt nesting level
    nesting_level: u32,
    /// Initialization state
    initialized: bool,
}

impl ExceptionManager {
    /// Create a new exception manager
    pub const fn new() -> Self {
        Self {
            stats: ExceptionStats {
                total_exceptions: 0,
                undefined_instructions: 0,
                system_calls: 0,
                prefetch_aborts: 0,
                data_aborts: 0,
                irq_count: 0,
                fiq_count: 0,
                nested_interrupts: 0,
                max_nesting_level: 0,
            },
            nesting_level: 0,
            initialized: false,
        }
    }

    /// Initialize exception handling
    pub fn init(&mut self) -> Result<(), &'static str> {
        if self.initialized {
            return Ok(());
        }

        // Install exception vectors
        self.install_vectors()?;

        // Configure exception handling
        self.configure_exceptions()?;

        self.initialized = true;
        crate::debug_print!("Exception handling initialized");

        Ok(())
    }

    /// Install exception vector table
    fn install_vectors(&self) -> Result<(), &'static str> {
        unsafe extern "C" {
            static __vectors_start: u8;
            static __vectors_end: u8;
        }

        // SAFETY: We're installing exception vectors during initialization
        unsafe {
            let vectors_start = &__vectors_start as *const u8;
            let vectors_end = &__vectors_end as *const u8;
            let vector_size = vectors_end as usize - vectors_start as usize;

            // Vectors should be exactly 32 bytes (8 vectors * 4 bytes each)
            if vector_size != 32 {
                return Err("Invalid vector table size");
            }

            // Copy vectors to 0x00000000 (assuming low vectors)
            let dest = 0x00000000 as *mut u8;
            core::ptr::copy_nonoverlapping(vectors_start, dest, vector_size);

            // Ensure vectors are written
            crate::arch::barriers::dsb();
            crate::arch::barriers::isb();
        }

        Ok(())
    }

    /// Configure exception handling features
    fn configure_exceptions(&self) -> Result<(), &'static str> {
        // SAFETY: We're configuring exception handling
        unsafe {
            let mut sctlr: u32;
            
            // Read SCTLR
            asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
            
            // Ensure low vectors (clear V bit)
            sctlr &= !(1 << 13);
            
            // Write back SCTLR
            asm!("mcr p15, 0, {}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
            asm!("isb", options(nomem, nostack));
        }
        
        Ok(())
    }

    /// Handle undefined instruction exception
    pub fn handle_undefined_instruction(&mut self, context: &mut CpuContext) -> bool {
        self.stats.total_exceptions += 1;
        self.stats.undefined_instructions += 1;

        crate::debug_print!("Undefined instruction at PC: 0x{:08X}", context.program_counter);

        // In a real implementation, this might:
        // 1. Check if it's an emulation trap
        // 2. Handle floating point emulation
        // 3. Deliver signal to process
        // 4. Terminate process if unhandleable
        
        // For now, just terminate the process
        false // Return false to indicate the process should be terminated
    }

    /// Handle software interrupt (system call)
    pub fn handle_software_interrupt(&mut self, context: &mut CpuContext) -> SystemCallResult {
        self.stats.total_exceptions += 1;
        self.stats.system_calls += 1;

        // Extract system call information from registers
        let syscall_number = context.registers[7]; // r7 contains syscall number
        let args = SystemCallArgs::from_array([
            context.registers[0] as usize, // r0
            context.registers[1] as usize, // r1
            context.registers[2] as usize, // r2
            context.registers[3] as usize, // r3
            context.registers[4] as usize, // r4
            context.registers[5] as usize, // r5
        ]);

        // Get current process ID (simplified)
        let caller = ProcessId::new(1); // In real implementation, get from current context

        // Convert syscall number
        let syscall_num = match SystemCallNumber::try_from(syscall_number) {
            Ok(num) => num,
            Err(_) => {
                return SystemCallResult::err(crate::syscalls::SystemCallError::InvalidSystemCall);
            }
        };

        // Create system call structure
        let syscall = SystemCall::new(syscall_num, args, caller);

        // Handle the system call
        // In a real implementation, that would call the system call dispatcher
        crate::debug_print!("System call: {:?} from process {:?}", syscall_num, caller);

        // Return dummy result for now
        SystemCallResult::ok(0)
    }

    /// Handle prefetch abort exception
    pub fn handle_prefetch_abort(&mut self, context: &mut CpuContext) -> bool {
        self.stats.total_exceptions += 1;
        self.stats.prefetch_aborts += 1;

        // Read fault address and status
        let (fault_address, fault_status) = self.read_prefetch_fault_info();

        crate::debug_print!("Prefetch abort at PC: 0x{:08X}, fault addr: 0x{:08X}, status: 0x{:08X}",
                            context.program_counter, fault_address, fault_status);

        // Handle the fault
        self.handle_memory_fault(fault_address, fault_status, false)
    }

    /// Handle data abort exception
    pub fn handle_data_abort(&mut self, context: &mut CpuContext) -> bool {
        self.stats.total_exceptions += 1;
        self.stats.data_aborts += 1;
        
        // Read fault address and status
        let (fault_address, fault_status) = self.read_data_fault_info();
        
        crate::debug_print!("Data abort at PC: 0x{:08X}, fault addr: 0x{:08X}, status: 0x{:08X}",
                           context.program_counter, fault_address, fault_status);
        
        // Handle the fault
        self.handle_memory_fault(fault_address, fault_status, true)
    }

    /// Handle IRQ interrupt
    pub fn handle_irq(&mut self) {
        self.stats.total_exceptions += 1;
        self.stats.irq_count += 1;

        // Track interrupt nesting
        self.nesting_level += 1;
        if self.nesting_level > 1 {
            self.stats.nested_interrupts += 1;
        }
        if self.nesting_level > self.stats.max_nesting_level {
            self.stats.max_nesting_level = self.nesting_level;
        }

        // Handle the interrupt
        // In a real implementations, this would:
        // 1. Read interrupt controller status
        // 2. Dispatch to appropriate driver
        // 3. Acknowledge interrupt

        crate::debug_print!("IRQ interrupt (nesting level: {})", self.nesting_level);

        self.nesting_level -= 1;
    }

    /// Handle FIQ interrupt
    pub fn handle_fiq(&mut self) {
        self.stats.total_exceptions += 1;
        self.stats.fiq_count += 1;

        crate::debug_print!("FIQ interrupt");

        // FIQ should be handled very quickly
        // In a real implementation, this would handle time-critical interrupts
    }

    /// Read refetch fault information
    fn read_prefetch_fault_info(&self) -> (u32, u32) {
        let mut ifar: u32;
        let mut ifsr: u32;

        // SAFETY: Reading fault status registers is safe
        unsafe {
            // Instruction Fault Address Register
            asm!("mrc p15, 0, {}, c6, c0, 2", out(reg) ifar, options(nomem, nostack));
            // Instruction Fault Status Register
            asm!("mrc p15, 0, {}, c5, c0, 1", out(reg) ifsr, options(nomem, nostack));
        }

        (ifar, ifsr)
    }

    /// Read data fault information
    fn read_data_fault_info(&self) -> (u32, u32) {
        let mut dfar: u32;
        let mut dfsr: u32;

        // SAFETY: Reading fault status registers is safe
        unsafe {
            // Data Fault Address Register
            asm!("mrc p15, 0, {}, c6, c0, 0", out(reg) dfar, options(nomem, nostack));
            // Data Fault Status Register
            asm!("mrc p15, 0, {}, c5, c0, 0", out(reg) dfsr, options(nomem, nostack));
        }

        (dfar, dfsr)
    }

    /// Handle memory fault (common for both data and prefetch aborts)
    fn handle_memory_fault(&self, fault_address: u32, fault_status: u32, is_data_fault: bool) -> bool {
        let fault_type = fault_status & 0x0F;
        
        match fault_type {
            dfsr_bits::TRANSLATION_L1 | dfsr_bits::TRANSLATION_L2 => {
                crate::debug_print!("Translation fault at 0x{:08X}", fault_address);
                // Handle page fault - could trigger demand paging
                self.handle_page_fault(fault_address, fault_status)
            }
            dfsr_bits::PERMISSION_L1 | dfsr_bits::PERMISSION_L2 => {
                crate::debug_print!("Permission fault at 0x{:08X}", fault_address);
                // Permission violation - likely a security violation
                false
            }
            dfsr_bits::ALIGNMENT => {
                crate::debug_print!("Alignment fault at 0x{:08X}", fault_address);
                // Alignment fault - could be handled by emulation
                false
            }
            dfsr_bits::DOMAIN_L1 | dfsr_bits::DOMAIN_L2 => {
                crate::debug_print!("Domain fault at 0x{:08X}", fault_address);
                // Domain fault
                false
            }
            _ => {
                crate::debug_print!("Unknown fault type 0x{:02X} at 0x{:08X}", fault_type, fault_address);
                false
            }
        }
    }

    /// Handle page fault
    fn handle_page_fault(&self, fault_address: u32, fault_status: u32) -> bool {
        // In a real implementation, this would:
        // 1. Check if the address is in a valid VMA
        // 2. Allocate physical memory if needed
        // 3. Update page tables
        // 4. Return true if handled, false if process should be terminated

        crate::debug_print!("Page fault at 0x{:08X} - not implemented", fault_address);
        false
    }

    /// Get exception statistics
    pub fn stats(&self) -> ExceptionStats {
        self.stats
    }

    /// Reset exception statistics
    pub fn reset_stats(&mut self) {
        self.stats = ExceptionStats::default();
    }

    /// Check if exceptions are properly initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}

/// Global exception manager instance
static mut EXCEPTION_MANAGER: Option<ExceptionManager> = None;

/// Initialize global exception manager
pub fn init_exception_vectors() -> Result<(), &'static str> {
    // SAFETY: This is called once during system initialization
    unsafe {
        if EXCEPTION_MANAGER.is_some() {
            return Ok(());
        }

        EXCEPTION_MANAGER = Some(ExceptionManager::new());

        if let Some(manager) = EXCEPTION_MANAGER.as_mut() {
            manager.init()?;
        }
    }

    Ok(())
}

/// Get reference to global exception manager
pub fn exception_manager() -> Option<&'static mut ExceptionManager> {
    // SAFETY: Exception manager is initialized once
    unsafe { EXCEPTION_MANAGER.as_mut() }
}

// Exception handler entry points called from assembly
// These are called from the vector table assembly stubs

/// Undefined instruction handler entry point
#[no_mangle]
pub extern "C" fn exception_undefined_instruction(context: &mut CpuContext) -> u32 {
    if let Some(manager) = exception_manager() {
        if manager.handle_undefined_instruction(context) {
            1 // Continue execution
        } else {
            0 // Terminate process
        }
    } else {
        0 // No manager, terminate
    }
}

/// Software interrupt handler entry point
#[no_mangle]
pub extern "C" fn exception_software_interrupt(context: &mut CpuContext) -> u32 {
    if let Some(manager) = exception_manager() {
        let result = manager.handle_software_interrupt(context);

        // Store result in context registers
        context.registers[0] = result.value as u32;
        context.registers[1] = result.value2 as u32;

        1 // Always continue after system call
    } else {
        0 // No manager
    }
}

/// Prefetch abort handler entry point
#[no_mangle]
pub extern "C" fn exception_prefetch_abort(context: &mut CpuContext) -> u32 {
    if let Some(manager) = exception_manager() {
        if manager.handle_prefetch_abort(context) {
            1 // Continue execution
        } else {
            0 // Terminate process
        }
    } else {
        0 // No manager, terminate
    }
}

/// Data abort handler entry point
#[no_mangle]
pub extern "C" fn exception_data_abort(context: &mut CpuContext) -> u32 {
    if let Some(manager) = exception_manager() {
        if manager.handle_data_abort(context) {
            1 // Continue execution
        } else {
            0 // Terminate process
        }
    } else {
        0 // No manager, terminate
    }
}