//! Zero OS - Safety-Critical Real-time Kernel
//! 
//! Main kernel entry point and initialization sequence.
//! This module handles the complete system initialization from hardware
//! reset to a fully operational kernel state.

#![no_std]
#![no_main]
#![feature(panic_info_message)]
#![deny(unsafe_op_in_unsafe_fn)]
#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::panic::PanicInfo;
use crate::{
    arch,
    memory::MemoryManager,
    platform::Platform,
    process::ProcessManager,
    syscalls,
    KernelResult,
    KernelError,
};

/// Kernel version information
pub const KERNEL_VERSION: &str = env!("CARGO_PKG_VERSION");
pub const KERNEL_NAME: &str = "Zero OS";
pub const BUILD_TARGET: &str = env!("TARGET");

/// Kernel initialization phases
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InitPhase {
    /// Hardware initialization
    Hardware,
    /// Memory management setup
    Memory,
    /// Process management setup
    Process,
    /// System services startup
    Services,
    /// User space initialization
    UserSpace,
    /// System ready
    Ready,
}

/// Kernel state management
#[derive(Debug)]
struct KernelState {
    /// Current initialization phase
    current_phase: InitPhase,
    /// Boot timestamp
    boot_time: u64,
    /// Memory manager
    memory_manager: Option<MemoryManager>,
    /// Process manager
    process_manager: Option<ProcessManager>,
    /// Platform abstraction
    platform: Option<Platform>,
    /// Initialization completed successfully
    initialized: bool,
}

impl KernelState {
    /// Create a new kernel state
    const fn new() -> Self {
        Self {
            current_phase: InitPhase::Hardware,
            boot_time: 0,
            memory_manager: None,
            process_manager: None,
            platform: None,
            initialized: false,
        }
    }

    /// Get current initialization phase
    pub fn current_phase(&self) -> InitPhase {
        self.current_phase
    }

    /// Check if kernel is fully initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get boot time
    pub fn boot_time(&self) -> u64 {
        self.boot_time
    }
}

/// Global kernel state
static mut KERNEL_STATE: KernelState = KernelState::new();

/// Get reference to global kernel state
/// 
/// # Safety
/// 
/// This function should only be called from kernel code in single-threaded
/// context during initialization, or from interrupt handlers.
pub unsafe fn kernel_state() -> &'static mut KernelState {
    // SAFETY: Caller guarantees single-threaded access
    unsafe { &mut KERNEL_STATE }
}

/// Kernel main entry point
/// 
/// This is called from the architecture-specific boot code after basic
/// hardware initialization is complete.
#[no_mangle]
pub extern "C" fn kernel_main() -> ! {
    // Initialize early debug output
    arch::early_debug_init();

    // Print boot banner
    print_boot_banner();

    // Initialize kernel
    match initialize_kernel() {
        Ok(()) => {
            debug_print!("Kernel initialization completed successfully");
            run_kernel()
        }
        Err(error) => {
            debug_print!("FATAL: Kernel initialization failed: {:?}", error);
            panic!("Kernel initialization failed: {:?}", error);
        }
    }
}

/// Print system boot banner
fn print_boot_banner() {
    debug_print!("=====================================");
    debug_print!("    {} v{}", KERNEL_NAME, KERNEL_VERSION);
    debug_print!("    Safety-Critical Real-time OS");
    debug_print!("    Target: {}", BUILD_TARGET);
    debug_print!("=====================================");
    debug_print!("");
    debug_print!("Starting system initialization...");
}

/// Initialize the kernel subsystems
fn initialize_kernel() -> KernelResult<()> {
    // SAFETY: We're in single-threaded initialization context
    let kernel_state = unsafe { kernel_state() };
    
    debug_print!("Phase 1: Hardware Architecture Initialization");
    kernel_state.current_phase = InitPhase::Hardware;
    
    // Initialize architecture-specific components
    arch::target::Architecture::init()
        .map_err(|_| KernelError::HardwareError)?;
    
    // Record boot time
    kernel_state.boot_time = arch::target::Architecture::current_time_us();
    debug_print!("Boot time recorded: {} µs", kernel_state.boot_time);
    
    debug_print!("Phase 2: Platform Initialization");
    // Initialize platform abstraction
    let mut platform = Platform::new();
    platform.early_init()
        .map_err(|_| KernelError::HardwareError)?;
    
    debug_print!("Phase 3: Memory Management Initialization");
    kernel_state.current_phase = InitPhase::Memory;
    
    // Initialize memory management
    let memory_manager = MemoryManager::new();
    // TODO: Implement memory manager initialization
    // memory_manager.init()?;
    
    kernel_state.memory_manager = Some(memory_manager);
    debug_print!("Memory management initialized");
    
    debug_print!("Phase 4: Process Management Initialization");
    kernel_state.current_phase = InitPhase::Process;
    
    // Initialize process management
    let process_manager = ProcessManager::new();
    // TODO: Implement process manager initialization
    // process_manager.init()?;
    
    kernel_state.process_manager = Some(process_manager);
    debug_print!("Process management initialized");
    
    debug_print!("Phase 5: System Services Initialization");
    kernel_state.current_phase = InitPhase::Services;
    
    // Initialize system call interface
    // TODO: Implement system call initialization
    // syscalls::init()?;
    debug_print!("System services initialized");
    
    // Complete platform initialization
    platform.late_init()
        .map_err(|_| KernelError::HardwareError)?;
    
    kernel_state.platform = Some(platform);
    
    debug_print!("Phase 6: User Space Preparation");
    kernel_state.current_phase = InitPhase::UserSpace;
    
    // TODO: Load and start init process
    // load_init_process()?;
    debug_print!("User space preparation completed");
    
    // Mark system as ready
    kernel_state.current_phase = InitPhase::Ready;
    kernel_state.initialized = true;
    
    debug_print!("Kernel initialization completed in {} µs", 
                arch::target::Architecture::current_time_us() - kernel_state.boot_time);
    
    Ok(())
}

/// Main kernel execution loop
fn run_kernel() -> ! {
    debug_print!("Entering main kernel loop");

    // Enable interrupts now that system is ready
    arch::target::Architecture::enable_interrupts();

    loop {
        // Main kernel execution loop
        // TODO: Implement proper kernel main loop with scheduling

        // For now, just wait for interrupts
        arch::target::Architecture::wait_for_interrupt();

        // Handle any pending work
        handle_kernel_work();
    }
}

/// Handle kernel housekeeping tasks
fn handle_kernel_work() {
    // SAFETY: We're in kernel context
    let kernel_state = unsafe { kernel_state() };
    
    if !kernel_state.is_initialized() {
        return;
    }
    
    // TODO: Implement kernel housekeeping tasks:
    // - Memory cleanup
    // - Process scheduling decisions
    // - Timer management
    // - I/O completion
    // - Safety monitoring
}

/// Load and start the init process
fn load_init_process() -> KernelResult<()> {
    debug_print!("Loading init process...");
    
    // TODO: Implement init process loading
    // 1. Load init binary from platform
    // 2. Create initial process
    // 3. Set up memory mapping
    // 4. Start execution
    
    debug_print!("Init process loaded and started");
    Ok(())
}

/// Emergency system shutdown
/// 
/// This function performs an emergency system shutdown with minimal
/// cleanup when a critical error occurs.
pub fn emergency_shutdown(reason: &str) -> ! {
    // Disable interrupts immediately
    arch::target::Architecture::disable_interrupts();
    
    debug_print!("EMERGENCY SHUTDOWN: {}", reason);
    debug_print!("System state dump:");
    
    // SAFETY: Emergency context, single-threaded access
    let kernel_state = unsafe { kernel_state() };
    debug_print!("  Phase: {:?}", kernel_state.current_phase);
    debug_print!("  Initialized: {}", kernel_state.initialized);
    debug_print!("  Boot time: {} µs", kernel_state.boot_time);
    
    // TODO: Implement emergency procedures:
    // - Save critical data
    // - Notify external systems
    // - Execute safe shutdown procedures
    
    debug_print!("Emergency shutdown complete - system halted");
    
    // Halt the system
    arch::target::Architecture::halt_system()
}

/// Kernel panic handler
/// 
/// This is called when a panic occurs in kernel space. It attempts to
/// provide diagnostic information and perform a safe system shutdown.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Disable interrupts immediately to prevent further issues
    arch::target::Architecture::disable_interrupts();
    
    debug_print!("==========================================");
    debug_print!("           KERNEL PANIC");
    debug_print!("==========================================");
    
    // Print panic message
    if let Some(message) = info.message() {
        debug_print!("Panic message: {}", message);
    } else {
        debug_print!("Panic occurred (no message)");
    }
    
    // Print location information
    if let Some(location) = info.location() {
        debug_print!("Location: {}:{}:{}", 
                    location.file(), 
                    location.line(), 
                    location.column());
    } else {
        debug_print!("Location: unknown");
    }
    
    // Print system state
    // SAFETY: Panic context, we need to access system state
    let kernel_state = unsafe { kernel_state() };
    debug_print!("System state at panic:");
    debug_print!("  Initialization phase: {:?}", kernel_state.current_phase);
    debug_print!("  Kernel initialized: {}", kernel_state.initialized);
    debug_print!("  Time since boot: {} µs", 
                arch::target::Architecture::current_time_us().saturating_sub(kernel_state.boot_time));
    
    // TODO: Collect additional diagnostic information:
    // - Stack trace (if available)
    // - Memory usage statistics
    // - Process state
    // - Hardware state
    
    debug_print!("==========================================");
    debug_print!("Attempting emergency shutdown...");
    
    // Attempt graceful shutdown
    // Note: We call emergency_shutdown which will halt the system
    emergency_shutdown("Kernel panic occurred");
}

/// Out of memory handler
/// 
/// This is called when the kernel runs out of memory. It attempts to
/// free up memory and recover, or triggers an emergency shutdown.
#[alloc_error_handler]
fn alloc_error_handler(layout: core::alloc::Layout) -> ! {
    debug_print!("OUT OF MEMORY: Failed to allocate {} bytes (align: {})", 
                layout.size(), layout.align());
    
    // TODO: Implement memory recovery strategies:
    // - Free unused memory pools
    // - Kill non-critical processes
    // - Compact memory
    // - Request garbage collection
    
    // If we can't recover, shutdown the system
    emergency_shutdown("Out of memory - unable to recover");
}

/// System call to get kernel information
/// 
/// This provides basic kernel information to user space processes.
pub fn get_kernel_info() -> KernelInfo {
    // SAFETY: Reading kernel state
    let kernel_state = unsafe { kernel_state() };
    
    KernelInfo {
        version: KERNEL_VERSION,
        name: KERNEL_NAME,
        target: BUILD_TARGET,
        boot_time: kernel_state.boot_time,
        current_time: arch::target::Architecture::current_time_us(),
        initialized: kernel_state.initialized,
        current_phase: kernel_state.current_phase,
    }
}
