//! ARM Generic Interrupt Controller (GIC) Driver
//!
//! This module provides support for the ARM Generic Interrupt Controller,
//! which is the standard interrupt controller for ARM Cortex-A processors.
//! The GIC manages both Software Generated Interrupts (SGI), Private Peripheral
//! Interrupts (PPI), and Shared Peripheral Interrupts (SPI).
//!
//! # GIC Architecture
//!
//! The GIC consists of two main components:
//! - **Distributor**: Routes interrupts to target processors
//! - **CPU Interface**: Per-processor interface for interrupt handling
//!
//! # Interrupt Types
//!
//! - **SGI (0-15)**: Software Generated Interrupts (inter-processor communication)
//! - **PPI (16-31)**: Private Peripheral Interrupts (per-processor peripherals)  
//! - **SPI (32-1019)**: Shared Peripheral Interrupts (shared system peripherals)
//!
//! # Priority and Preemption
//!
//! - 8-bit priority values (0 = highest priority)
//! - Priority grouping for preemption/sub-priority
//! - Priority masking for nested interrupt handling
//!
//! # Security Extensions
//!
//! - Secure and Non-secure interrupt handling
//! - Group 0 (secure) and Group 1 (non-secure) interrupts
//! - Banked registers for security separation

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::ptr;
use crate::arch::ArchResult;

/// Maximum number of interrupts supported by GIC
pub const MAX_INTERRUPTS: u32 = 1020;

/// Software Generated Interrupt range
pub const SGI_RANGE: core::ops::Range<u32> = 0..16;

/// Private Peripheral Interrupt range
pub const PPI_RANGE: core::ops::Range<u32> = 16..32;

/// Shared Peripheral Interrupt range
pub const SPI_RANGE: core::ops::Range<u32> = 32..MAX_INTERRUPTS;

/// GIC Distributor registers (memory-mapped)
#[repr(C)]
pub struct GicDistributor {
    /// Distributor Control Register
    pub ctlr: u32,
    /// Interrupt Controller Type Register
    pub typer: u32,
    /// Distributor Implementer Identification Register
    pub iidr: u32,
    /// Reserved
    _reserved1: [u32; 29],
    
    /// Interrupt Group Registers (1 bit per interrupt)
    pub igroupr: [u32; 32],
    
    /// Interrupt Set-Enable Registers (1 bit per interrupt)
    pub isenabler: [u32; 32],
    /// Interrupt Clear-Enable Registers (1 bit per interrupt)
    pub icenabler: [u32; 32],
    /// Interrupt Set-Pending Registers (1 bit per interrupt)
    pub ispendr: [u32; 32],
    /// Interrupt Clear-Pending Registers (1 bit per interrupt)
    pub icpendr: [u32; 32],
    /// Interrupt Set-Active Registers (1 bit per interrupt)
    pub isactiver: [u32; 32],
    /// Interrupt Clear-Active Registers (1 bit per interrupt)
    pub icactiver: [u32; 32],
    
    /// Interrupt Priority Registers (8 bits per interrupt)
    pub ipriorityr: [u32; 255],
    
    /// Reserved
    _reserved2: u32,
    
    /// Interrupt Processor Targets Registers (8 bits per interrupt)
    pub itargetsr: [u32; 255],
    
    /// Reserved
    _reserved3: u32,
    
    /// Interrupt Configuration Registers (2 bits per interrupt)
    pub icfgr: [u32; 64],
    
    /// Reserved
    _reserved4: [u32; 64],
    
    /// Software Generated Interrupt Register
    pub sgir: u32,
    
    /// Reserved
    _reserved5: [u32; 3],
    
    /// SGI Clear-Pending Registers
    pub cpendsgir: [u32; 4],
    /// SGI Set-Pending Registers
    pub spendsgir: [u32; 4],
}

/// GIC CPU Interface registers (memory-mapped)
#[repr(C)]
pub struct GicCpuInterface {
    /// CPU Interface Control Register
    pub ctlr: u32,
    /// Interrupt Priority Mask Register
    pub pmr: u32,
    /// Binary Point Register
    pub bpr: u32,
    /// Interrupt Acknowledge Register
    pub iar: u32,
    /// End of Interrupt Register
    pub eoir: u32,
    /// Running Priority Register
    pub rpr: u32,
    /// Highest Priority Pending Interrupt Register
    pub hppir: u32,
    /// Aliased Binary Point Register
    pub abpr: u32,
    /// Aliased Interrupt Acknowledge Register
    pub aiar: u32,
    /// Aliased End of Interrupt Register
    pub aeoir: u32,
    /// Aliased Highest Priority Pending Interrupt Register
    pub ahppir: u32,
    
    /// Reserved
    _reserved1: [u32; 41],
    
    /// Active Priorities Registers
    pub apr: [u32; 4],
    /// Non-secure Active Priorities Registers
    pub nsapr: [u32; 4],
    
    /// Reserved
    _reserved2: [u32; 3],
    
    /// CPU Interface Identification Register
    pub iidr: u32,
    
    /// Reserved
    _reserved3: [u32; 960],
    
    /// Deactivate Interrupt Register
    pub dir: u32,
}

/// GIC interrupt configuration
#[derive(Debug, Clone, Copy)]
pub struct InterruptConfig {
    /// Interrupt number
    pub id: u32,
    /// Priority (0 = highest, 255 = lowest)
    pub priority: u8,
    /// Target CPU mask (but per CPU)
    pub target_cpu: u8,
    /// Trigger type (true = edge, false = level)
    pub edge_triggered: bool,
    /// Security group (true = Group 1, false = Group 0)
    pub group1: bool,
}

/// GIC interrupt types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptType {
    /// Software Generated Interrupt
    Sgi,
    /// Private Peripheral Interrupt
    Ppi,
    /// Shared Peripheral Interrupt
    Spi,
}

/// GIC interrupt state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptState {
    /// Interrupt is inactive
    Inactive,
    /// Interrupt is pending
    Pending,
    /// Interrupt is active
    Active,
    /// Interrupt is active and pending
    ActivePending,
}

/// GIC manager
pub struct GicManager {
    /// Distributor base address
    distributor: *mut GicDistributor,
    /// CPU interface base address
    cpu_interface: *mut GicCpuInterface,
    /// Number of supported interrupts
    num_interrupts: u32,
    /// Number of CPUs
    num_cpus: u32,
    /// Initialization state
    initialized: bool,
}

impl GicManager {
    /// Create a new GIC manager
    pub const fn new() -> Self {
        Self {
            distributor: ptr::null_mut(),
            cpu_interface: ptr::null_mut(),
            num_interrupts: 0,
            num_cpus: 0,
            initialized: false,
        }
    }

    /// Initialize the GIC
    pub fn init(&mut self, distributor_base: usize, cpu_interface_base: usize) -> ArchResult<()> {
        if self.initialized {
            return Ok(());
        }
        
        self.distributor = distributor_base as *mut GicDistributor;
        self.cpu_interface = cpu_interface_base as *mut GicCpuInterface;
        
        // Read GIC configuration
        self.probe_gic_config()?;
        
        // Initialize distributor
        self.init_distributor()?;
        
        // Initialize CPU interface
        self.init_cpu_interface()?;
        
        self.initialized = true;
        
        crate::debug_print!("GIC initialized: {} interrupts, {} CPUs", 
                           self.num_interrupts, self.num_cpus);
        
        Ok(())
    }

    /// Probe GIC configuration
    fn probe_gic_config(&mut self) -> ArchResult<()> {
        // SAFETY: We're reading from memory-mapped GIC registers
        unsafe {
            let distributor = &*self.distributor;

            // Read interrupt controller type register
            let typer = ptr::read_volatile(&distributor.typer);

            // Extract number of interrupt lines
            self.num_interrupts = ((typer & 0x1F) + 1) * 32;

            // Extract number of CPUs
            self.num_cpus = ((typer >> 5) & 0x7) + 1;

            // Ensure reasonable limits
            if self.num_interrupts > MAX_INTERRUPTS {
                self.num_interrupts = MAX_INTERRUPTS;
            }

            if self.num_cpus > 8 {
                self.num_cpus = 8;
            }
        }

        Ok(())
    }

    /// Initialize GIC distributor
    fn init_distributor(&mut self) -> ArchResult<()> {
        // SAFETY: We're initializing GIC distributor registers
        unsafe {
            let distributor = &mut *self.distributor;
            
            // Disable distributor
            ptr::write_volatile(&mut distributor.ctlr, 0);
            
            // Disable all interrupts
            for i in 0..(self.num_interrupts / 32) {
                ptr::write_volatile(&mut distributor.icenabler[i as usize], 0xFFFFFFFF);
            }
            
            // Clear all pending interrupts
            for i in 0..(self.num_interrupts / 32) {
                ptr::write_volatile(&mut distributor.icpendr[i as usize], 0xFFFFFFFF);
            }
            
            // Set all interrupts to Group 1 (non-secure)
            for i in 0..(self.num_interrupts / 32) {
                ptr::write_volatile(&mut distributor.igroupr[i as usize], 0xFFFFFFFF);
            }
            
            // Set default priorities (lower priority for all)
            for i in 0..(self.num_interrupts / 4) {
                ptr::write_volatile(&mut distributor.ipriorityr[i as usize], 0xA0A0A0A0);
            }
            
            // Set all SPI interrupts to target CPU 0
            for i in 8..(self.num_interrupts / 4) {
                ptr::write_volatile(&mut distributor.itargetsr[i as usize], 0x01010101);
            }
            
            // Configure all interrupts as level-triggered
            for i in 2..(self.num_interrupts / 16) {
                ptr::write_volatile(&mut distributor.icfgr[i as usize], 0x00000000);
            }
            
            // Enable distributor
            ptr::write_volatile(&mut distributor.ctlr, 1);
        }
        
        Ok(())
    }

    /// Initialize GIC CPU interface
    fn init_cpu_interface(&mut self) -> ArchResult<()> {
        // SAFETY: We're initializing GIC CPU interface registers
        unsafe {
            let cpu_interface = &mut *self.cpu_interface;

            // Set priority mask to allow all interrupts
            ptr::write_volatile(&mut cpu_interface.pmr, 0xF0);

            // Set binary point to 0 (no preemption grouping)
            ptr::write_volatile(&mut cpu_interface.bpr, 0);

            // Enable CPU interface
            ptr::write_volatile(&mut cpu_interface.ctlr, 1);
        }

        Ok(())
    }

    /// Enable an interrupt
    pub fn enable_interrupt(&mut self, irq: u32) -> ArchResult<()> {
        if irq >= self.num_interrupts {
            return Err(crate::arch::ArchError::InvalidParameter);
        }

        // SAFETY: We're enabling a valid interrupt
        unsafe {
            let distributor = &mut *self.distributor;
            let reg_index = (irq / 32) as usize;
            let bit_index = irq % 32;

            ptr::write_volatile(&mut distributor.isenabler[reg_index], 1 << bit_index);
        }

        Ok(())
    }

    /// Disable an interrupt
    pub fn disable_interrupt(&mut self, irq: u32) -> ArchResult<()> {
        if irq >= self.num_interrupts {
            return Err(crate::arch::ArchError::InvalidParameter);
        }

        // SAFETY: We're disabling a valid interrupt
        unsafe {
            let distributor = &mut *self.distributor;
            let reg_index = (irq / 32) as usize;
            let bit_index = irq % 32;

            ptr::write_volatile(&mut distributor.icenabler[reg_index], 1 << bit_index);
        }

        Ok(())
    }

    /// Set interrupt priority
    pub fn set_interrupt_priority(&mut self, irq: u32, priority: u8) -> ArchResult<()> {
        if irq >= self.num_interrupts {
            return Err(crate::arch::ArchError::InvalidParameter);
        }
        
        // SAFETY: We're setting priority for a valid interrupt
        unsafe {
            let distributor = &mut *self.distributor;
            let reg_index = (irq / 4) as usize;
            let byte_index = (irq % 4) * 8;
            
            let mut reg_value = ptr::read_volatile(&distributor.ipriorityr[reg_index]);
            reg_value &= !(0xFF << byte_index);
            reg_value |= (priority as u32) << byte_index;
            ptr::write_volatile(&mut distributor.ipriorityr[reg_index], reg_value);
        }
        
        Ok(())
    }

    /// Set interrupt target CPU
    pub fn set_interrupt_target(&mut self, irq: u32, cpu_mask: u8) -> ArchResult<()> {
        if irq >= self.num_interrupts || irq < 32 {
            return Err(crate::arch::ArchError::InvalidParameter);
        }
        
        // SAFETY: We're setting target for a valid SPI interrupt
        unsafe {
            let distributor = &mut *self.distributor;
            let reg_index = (irq / 4) as usize;
            let byte_index = (irq % 4) * 8;
            
            let mut reg_value = ptr::read_volatile(&distributor.itargetsr[reg_index]);
            reg_value &= !(0xFF << byte_index);
            reg_value |= (cpu_mask as u32) << byte_index;
            ptr::write_volatile(&mut distributor.itargetsr[reg_index], reg_value);
        }
        
        Ok(())
    }

    /// Configure interrupt trigger type
    pub fn configure_interrupt(&mut self, irq: u32, edge_triggered: bool) -> ArchResult<()> {
        if irq >= self.num_interrupts || irq < 16 {
            return Err(crate::arch::ArchError::InvalidParameter);
        }
        
        // SAFETY: We're configuring a valid PPI/SPI interrupt
        unsafe {
            let distributor = &mut *self.distributor;
            let reg_index = (irq / 16) as usize;
            let bit_index = ((irq % 16) * 2) + 1;
            
            let mut reg_value = ptr::read_volatile(&distributor.icfgr[reg_index]);
            if edge_triggered {
                reg_value |= 1 << bit_index;
            } else {
                reg_value &= !(1 << bit_index);
            }
            ptr::write_volatile(&mut distributor.icfgr[reg_index], reg_value);
        }
        
        Ok(())
    }

    /// Set interrupt as pending
    pub fn set_pending(&mut self, irq: u32) -> ArchResult<()> {
        if irq >= self.num_interrupts {
            return Err(crate::arch::ArchError::InvalidParameter);
        }

        // SAFETY: We're setting pending for a valid interrupt
        unsafe {
            let distributor = &mut *self.distributor;
            let reg_index = (irq / 32) as usize;
            let bit_index = irq % 32;

            ptr::write_volatile(&mut distributor.ispendr[reg_index], 1 << bit_index);
        }

        Ok(())
    }

    /// Clear interrupt pending state
    pub fn clear_pending(&mut self, irq: u32) -> ArchResult<()> {
        if irq >= self.num_interrupts {
            return Err(crate::arch::ArchError::InvalidParameter);
        }

        // SAFETY: We're setting pending for a valid interrupt
        unsafe {
            let distributor = &mut *self.distributor;
            let reg_index = (irq / 32) as usize;
            let bit_index = irq % 32;

            ptr::write_volatile(&mut distributor.icpendr[reg_index], 1 << bit_index);
        }

        Ok(())
    }

    /// Acknowledge interrupt and get interrupt ID
    pub fn acknowledge_interrupt(&mut self) -> Option<u32> {
        // SAFETY: We're reading interrupt acknowledge register
        unsafe {
            let cpu_interface = &*self.cpu_interface;
            let iar = ptr::read_volatile(&cpu_interface.iar);
            let irq_id = iar & 0x3FF;
            
            // Check for spurious interrupt
            if irq_id >= 1020 {
                None
            } else {
                Some(irq_id)
            }
        }
    }

    /// End of interrupt - signal completion of interrupt handling
    pub fn end_of_interrupt(&mut self, irq: u32) {
        // SAFETY: We're writing to end of interrupt register
        unsafe {
            let cpu_interface = &mut *self.cpu_interface;
            ptr::write_volatile(&mut cpu_interface.eoir, irq);
        }
    }

    /// Send Software Generated Interrupt
    pub fn send_sgi(&mut self, irq: u32, target_cpu_mask: u8) -> ArchResult<()> {
        if !SGI_RANGE.contains(&irq) {
            return Err(crate::arch::ArchError::InvalidParameter);
        }
        
        // SAFETY: We're sending a valid SGI
        unsafe {
            let distributor = &mut *self.distributor;
            let sgir_value = (target_cpu_mask as u32) << 16 | irq;
            ptr::write_volatile(&mut distributor.sgir, sgir_value);
        }
        
        Ok(())
    }

    /// Get interrupt state
    pub fn get_interrupt_state(&self, irq: u32) -> ArchResult<InterruptState> {
        if irq >= self.num_interrupts {
            return Err(crate::arch::ArchError::InvalidParameter);
        }
        
        // SAFETY: We're reading interrupt state for a valid interrupt
        unsafe {
            let distributor = &*self.distributor;
            let reg_index = (irq / 32) as usize;
            let bit_index = irq % 32;
            let bit_mask = 1 << bit_index;
            
            let is_pending = (ptr::read_volatile(&distributor.ispendr[reg_index]) & bit_mask) != 0;
            let is_active = (ptr::read_volatile(&distributor.isactiver[reg_index]) & bit_mask) != 0;
            
            Ok(match (is_active, is_pending) {
                (false, false) => InterruptState::Inactive,
                (false, true) => InterruptState::Pending,
                (true, false) => InterruptState::Active,
                (true, true) => InterruptState::ActivePending,
            })
        }
    }

    /// Check if interrupt is enabled
    pub fn is_interrupt_enabled(&self, irq: u32) -> bool {
        if irq >= self.num_interrupts {
            return false;
        }
        
        // SAFETY: We're reading interrupt enable state for a valid interrupt
        unsafe {
            let distributor = &*self.distributor;
            let reg_index = (irq / 32) as usize;
            let bit_index = irq % 32;
            
            (ptr::read_volatile(&distributor.isenabler[reg_index]) & (1 << bit_index)) != 0
        }
    }

    /// Get interrupt type
    pub fn get_interrupt_type(irq: u32) -> InterruptType {
        if SGI_RANGE.contains(&irq) {
            InterruptType::Sgi
        } else if PPI_RANGE.contains(&irq) {
            InterruptType::Ppi
        } else {
            InterruptType::Spi
        }
    }

    /// Set priority mask
    pub fn set_priority_mask(&mut self, mask: u8) {
        // SAFETY: We're setting priority mask
        unsafe {
            let cpu_interface = &mut *self.cpu_interface;
            ptr::write_volatile(&mut cpu_interface.pmr, mask as u32);
        }
    }

    /// Get number of supported interrupts
    pub fn num_interrupts(&self) -> u32 {
        self.num_interrupts
    }

    /// Get number of CPUs
    pub fn num_cpus(&self) -> u32 {
        self.num_cpus
    }

    /// Check if GIC is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}

// SAFETY: GIC manager can be safely sent between threads as it manages hardware registers
unsafe impl Send for GicManager {}

// SAFETY: GIC manager can be safely shared between threads with proper synchronization
unsafe impl Sync for GicManager {}

/// Global GIC manager instance
static mut GIC_MANAGER: Option<GicManager> = None;

/// Initialize global GIC manager
pub fn init(distributor_base: usize, cpu_interface_base: usize) -> ArchResult<()> {
    // SAFETY: This is called once during system initialization
    unsafe {
        if GIC_MANAGER.is_some() {
            return Ok(());
        }
        
        GIC_MANAGER = Some(GicManager::new());
        
        if let Some(manager) = GIC_MANAGER.as_mut() {
            manager.init(distributor_base, cpu_interface_base)?;
        }
    }
    
    Ok(())
}

/// Get reference to global GIC manager
pub fn gic_manager() -> Option<&'static mut GicManager> {
    // SAFETY: GIC manager is initialized once
    unsafe { GIC_MANAGER.as_mut() }
}

/// Enable an interrupt (convenience function)
pub fn enable_interrupt(irq: u32) -> ArchResult<()> {
    gic_manager()
        .ok_or(crate::arch::ArchError::InvalidState)?
        .enable_interrupt(irq)
}

/// Disable an interrupt (convenience function)
pub fn disable_interrupt(irq: u32) -> ArchResult<()> {
    gic_manager()
        .ok_or(crate::arch::ArchError::InvalidState)?
        .disable_interrupt(irq)
}

/// acknowledge interrupt (convenience function)
pub fn acknowledge_interrupt() -> Option<u32> {
    gic_manager()?.acknowledge_interrupt()
}

/// End of interrupt (convenience function)
pub fn end_of_interrupt(irq: u32) {
    if let Some(manager) = gic_manager() {
        manager.end_of_interrupt(irq);
    }
}