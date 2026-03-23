//! ARM9 EmbeddedICE TAP layer.
//!
//! This module handles the JTAG IR/DR framing for ARM9 cores that use the
//! EmbeddedICE debug architecture.  The ARM9EJ-S scan chain layout is:
//!
//! - IR is 4 bits wide (irlen=4, ircapture=0x1, irmask=0xF).
//! - Selecting a scan chain is done by shifting the chain number into IR
//!   with `SCAN_N`, then writing the chain's DR.
//! - `INTEST` enters internal test mode which makes DR shifts actually talk
//!   to the selected scan chain inside the chip.
//! - `RESTART` exits debug state and lets the CPU run.
//!
//! Reference: ARM9EJ-S Technical Reference Manual, chapter 5.

use crate::probe::{DebugProbeError, JtagAccess};
use bitvec::prelude::*;

// ---------------------------------------------------------------------------
// Instruction register constants (IR width = 4 bits)
// ---------------------------------------------------------------------------

/// EXTEST – boundary-scan
pub const IR_EXTEST: u32 = 0x0;
/// SCAN_N – select a numbered scan chain for subsequent INTEST accesses
pub const IR_SCAN_N: u32 = 0x2;
/// SAMPLED – boundary-scan with sampling
pub const IR_SAMPLED: u32 = 0x3;
/// RESTART – release the CPU from debug state
pub const IR_RESTART: u32 = 0x4;
/// CLAMP – force outputs to values in boundary-scan register
pub const IR_CLAMP: u32 = 0x5;
/// INTEST – shift data through the currently selected scan chain
pub const IR_INTEST: u32 = 0xC;
/// IDCODE – read 32-bit ID code
pub const IR_IDCODE: u32 = 0xE;
/// BYPASS – single-bit bypass
pub const IR_BYPASS: u32 = 0xF;

// ---------------------------------------------------------------------------
// Scan chain numbers
// ---------------------------------------------------------------------------

/// Scan chain 0: full 67-bit core pipeline (used for register access,
/// single-stepping via instruction injection).
pub const SCAN_CHAIN_0: u8 = 0;
/// Scan chain 1: 33-bit JTAG-ICE data (deprecated in ARM9EJ-S; use SC2).
pub const SCAN_CHAIN_1: u8 = 1;
/// Scan chain 2: 38-bit EmbeddedICE register access (primary debug registers).
pub const SCAN_CHAIN_2: u8 = 2;
/// Scan chain 15: 48-bit CP15 coprocessor access (MMU/cache control).
pub const SCAN_CHAIN_15: u8 = 15;

/// Width of the SCAN_N DR in bits (ARM9EJ-S uses a 5-bit chain-select DR).
pub const SCAN_N_DR_WIDTH: u32 = 5;

/// Width of the IR register in bits.
pub const IR_WIDTH: u32 = 4;

/// Writes `ir` to the JTAG IR and then shifts `data` through the DR.
///
/// Returns the bits clocked out of TDO during the DR shift.
pub fn write_ir_dr(
    probe: &mut dyn JtagAccess,
    ir: u32,
    data: &[u8],
    dr_len: u32,
) -> Result<BitVec, DebugProbeError> {
    let bits = probe.write_register(ir, data, dr_len)?;
    Ok(bits)
}

/// Selects a scan chain by writing `chain` into IR=SCAN_N DR (5 bits wide),
/// then switches IR to INTEST so subsequent `write_dr` calls access that chain.
pub fn select_scan_chain(probe: &mut dyn JtagAccess, chain: u8) -> Result<(), DebugProbeError> {
    // Write chain number into the SCAN_N DR (5-bit field, LSB-first).
    let payload = [chain & 0x1F];
    probe.write_register(IR_SCAN_N, &payload, SCAN_N_DR_WIDTH)?;

    // Switch to INTEST so the selected chain is active on subsequent DR shifts.
    // A zero-length DR shift just updates the IR.
    probe.write_register(IR_INTEST, &[], 0)?;

    Ok(())
}
