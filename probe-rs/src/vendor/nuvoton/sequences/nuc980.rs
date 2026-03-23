//! Debug sequences for Nuvoton NUC980 series SoCs.
//!
//! The NUC980 uses an ARM926EJ-S core (ARMv5TEJ) with EmbeddedICE debug.
//! There is no CoreSight DAP; this sequence stub exists for the `ArmDebugSequence`
//! trait interface, but actual debug operations are performed directly via
//! [`EmbeddedIce`](crate::architecture::arm::embedded_ice::EmbeddedIce).
//!
//! ## Reset
//! The NUC980 soft-reset is triggered by setting bit 2 (CPURST) of the
//! `SYS_AHBIPRST` register at address `0xB000_0060`.  The bit auto-clears
//! after ~6 system clocks.  Full-chip reset via RESETN pin is preferred in
//! practice.

use std::sync::Arc;

use crate::architecture::arm::sequences::ArmDebugSequence;

/// Nuvoton NUC980 debug sequence.
#[derive(Debug)]
pub struct Nuc980;

impl Nuc980 {
    /// Base address of the `SYS_AHBIPRST` register (CPU soft-reset).
    pub const SYS_AHBIPRST: u64 = 0xB000_0060;
    /// Bit 2: CPU reset request.
    pub const CPURST_BIT: u32 = 1 << 2;

    /// PDID (Product ID) register base address.
    pub const PDID_ADDR: u64 = 0xB000_0000;
    /// NUC980 PDID value.
    pub const PDID_VALUE: u32 = 0x1030_D016;

    /// Create a new NUC980 debug sequence.
    pub fn create() -> Arc<Self> {
        Arc::new(Self)
    }
}

/// The ArmDebugSequence impl for NUC980 is intentionally minimal.
///
/// All real debug operations go through the EmbeddedICE path; the
/// `ArmDebugSequence` trait is used here only so we can register a
/// `DebugSequence::Arm` that the session machinery can store.
impl ArmDebugSequence for Nuc980 {}
