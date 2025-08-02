//! Zero OS Safety Systems
//!
//! This module implements safety-critical functionality for the Zero OS kernel.
//! It provides fault detection, isolation, recovery mechanisms, and system
//! monitoring to ensure safe operation in autonomous systems.
//!
//! # Safety Architecture
//!
//! The safety system follows a layered approach:
//! - **Hardware Safety**: Watchdogs, ECC, lockstep processors
//! - **Software Safety**: Bounds checking, assertions, invariant checking
//! - **System Safety**: Resource monitoring, deadline checking, fault isolation
//! - **Application Safety**: Process isolation, capability checking
//!
//! # Fault Model
//!
//! The system is designed to handle:
//! - Transient faults (cosmic rays, electrical noise)
//! - Permanent faults (hardware failures)
//! - Design faults (software bugs, timing violations)
//! - Interaction faults (resource conflicts, deadlocks)

#![deny(missing_docs)]
#![warn(clippy::undocumented_unsafe_blocks)]

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use heapless::{Vec, FnvIndexMap};
use crate::{KernelResult, KernelError};

pub mod watchdog;
pub mod fault_detection;
pub mod redundancy;
pub mod recovery;
pub mod monitoring;

// Re-exports
pub use watchdog::*;
pub use fault_detection::*;
pub use redundancy::*;
pub use recovery::*;
pub use monitoring::*;

/// Minimum number of concurrent faults to track
pub const MAX_FAULT_RECORDS: usize = 256;

/// Maximum number of safety monitors
pub const MAX_SAFETY_MONITORS: usize = 64;

/// Safety integrity levels (SIL) as defined by IEC 61508
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum SafetyIntegrityLevel {
    /// SIL 1 - Low safety integrity
    Sil1 = 1,
    /// SIL 2 - Medium safety integrity
    Sil2 = 2,
    /// SIL 3 - High safety integrity
    Sil3 = 3,
    /// SIL 4 - Very high safety integrity
    Sil4 = 4,
}

/// ASIL (Automotive Safety Integrity Level) as defined by ISO 26262
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum AutomotiveSafetyLevel {
    /// ASIL A - Lowest safety level
    AsilA = 1,
    /// ASIL B - Low safety level
    AsilB = 2,
    /// ASIL C - Medium safety level
    AsilC = 3,
    /// ASIL D - Highest safety level
    AsilD = 4,
}

/// Fault severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum FaultSeverity {
    /// Informational - no immediate impact
    Info = 0,
    /// Warning - potential future impact
    Warning = 1,
    /// Minor - limited functionality impact
    Minor = 2,
    /// Major - significant functionality impact
    Major = 3,
    /// Critical - system safety at risk
    Critical = 4,
    /// Catastrophic - immediate danger
    Catastrophic = 5,
}

/// Fault categories for classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FaultCategory {
    /// Hardware fault (sensor, actuator, processor)
    Hardware = 1,
    /// Software fault (bug, assertion failure, exception)
    Software = 2,
    /// Timing fault (deadline miss, jitter violation)
    Timing = 3,
    /// Resource fault (memory exhaustion, overload)
    Resource = 4,
    /// Communication fault (message loss, corruption)
    Communication = 5,
    /// Environmental fault (temperature, vibration, EMI)
    Environmental = 6,
    /// Human factor fault (operator error, maintenance)
    Human = 7,
}

/// Fault record for tracking system failures
#[derive(Debug, Clone)]
pub struct FaultRecord {
    /// Unique fault identifier
    pub id: u32,
    /// Fault category
    pub category: FaultCategory,
    /// Fault severity
    pub severity: FaultSeverity,
    /// Timestamp when fault occurred
    pub timestamp: u64,
    /// Source component or subsystem
    pub source: &'static str,
    /// Detailed fault description
    pub description: heapless::String<128>,
    /// Recovery action taken
    pub recovery_action: Option<RecoveryAction>,
    /// Whether fault has been resolved
    pub resolved: bool,
    /// Resolution timestamp
    pub resolved_at: Option<u64>,
}

impl FaultRecord {
    /// Create a new fault record
    pub fn new(
        id: u32,
        category: FaultCategory,
        severity: FaultSeverity,
        source: &'static str,
        description: &str,
    ) -> Self {
        let mut desc = heapless::String::new();
        let _ = desc.push_str(description);

        Self {
            id,
            category,
            severity,
            timestamp: crate::arch::target::Architecture::current_time_us(),
            source,
            description: desc,
            recovery_action: None,
            resolved: false,
            resolved_at: None,
        }
    }

    /// Mark fault as resolved
    pub fn resolve(&mut self, recovery_action: RecoveryAction) {
        self.resolved = true;
        self.resolved_at = Some(crate::arch::target::Architecture::current_time_us());
        self.recovery_action = Some(recovery_action);
    }

    /// Get fault duration in microseconds
    pub fn duration_us(&self) -> Option<u64> {
        self.resolved_at.map(|resolved| resolved - self.timestamp)
    }

    /// Check if fault is critical or higher
    pub fn is_critical(&self) -> bool {
        self.severity >= FaultSeverity::Critical
    }

    /// Check if fault requires immediate action
    pub fn requires_immediate_action(&self) -> bool {
        matches!(self.severity, FaultSeverity::Critical | FaultSeverity::Catastrophic)
    }
}

/// Recovery actions that can be taken for faults
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RecoveryAction {
    /// No action required
    None = 0,
    /// Retry the operation
    Retry = 1,
    /// Reset the component
    Reset = 2,
    /// Switch to backup/redundant component
    Switchover = 3,
    /// Degrade system functionality
    Degrade = 4,
    /// Isolate faulty component
    Isolate = 5,
    /// Emergency stop
    EmergencyStop = 6,
    /// System restart
    Restart = 7,
    /// Manual intervention required
    Manual = 8,
}

/// Safety manager for the kernel
pub struct SafetyManager {
    /// Fault records
    fault_records: Vec<FaultRecord, MAX_FAULT_RECORDS>,
    /// Safety monitors
    monitors: FnvIndexMap<u32, Box<dyn SafetyMonitor>, MAX_SAFETY_MONITORS>,
    /// Next fault ID
    next_fault_id: AtomicU32,
    /// Total faults detected
    total_faults: AtomicU64,
    /// Critical faults count
    critical_faults: AtomicU32,
    /// System safety state
    safety_state: SafetyState,
    /// Emergency stop flag
    emergency_stop: AtomicU32,
    /// Initialization status
    initialized: bool,
}

/// System safety state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SafetyState {
    /// Normal operation
    Normal = 0,
    /// Warning state - monitoring increased
    Warning = 1,
    /// Degraded state - reduced functionality
    Degraded = 2,
    /// Emergency state - safety-critical operation only
    Emergency = 3,
    /// Failed state - system shutdown required
    Failed = 4,
}

/// Safety monitor trait for implementing custom monitors
pub trait SafetyMonitor {
    /// Monitor name
    fn name(&self) -> &'static str;
    
    /// Check safety conditions
    fn check(&mut self) -> Result<(), FaultRecord>;
    
    /// Get monitor priority (lower values = higher priority)
    fn priority(&self) -> u8;
    
    /// Check if monitor is enabled
    fn is_enabled(&self) -> bool;
    
    /// Enable/disable monitor
    fn set_enabled(&mut self, enabled: bool);
}
