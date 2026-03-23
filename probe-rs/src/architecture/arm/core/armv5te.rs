//! ARM926EJ-S (ARMv5TEJ) core state and interface.
//!
//! This module provides the cached state for ARM9 cores that use the
//! pre-CoreSight EmbeddedICE debug architecture.

use crate::CoreStatus;

// ---------------------------------------------------------------------------
// Per-core state
// ---------------------------------------------------------------------------

/// Cached state for an ARMv5TEJ (ARM926EJ-S) core.
///
/// Fields are read by the [`CoreInterface`] implementation added in a
/// companion module; suppress dead-code warnings for the bare definition.
#[derive(Debug)]
#[allow(dead_code)]
pub struct Armv5teState {
    /// Whether the core driver has been initialised.
    pub(crate) initialized: bool,
    /// Current known core status.
    pub(crate) current_state: CoreStatus,
    /// Are breakpoints (watchpoints) currently enabled?
    pub(crate) hw_breakpoints_enabled: bool,
    /// Cached CPSR value (to detect Thumb state).
    pub(crate) cached_cpsr: Option<u32>,
}

impl Default for Armv5teState {
    fn default() -> Self {
        Self::new()
    }
}

impl Armv5teState {
    /// Create new uninitialized state.
    pub fn new() -> Self {
        Self {
            initialized: false,
            current_state: CoreStatus::Unknown,
            hw_breakpoints_enabled: false,
            cached_cpsr: None,
        }
    }
}
