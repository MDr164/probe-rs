//! ARM9 CP15 access via scan chain 15 (SC15).
//!
//! SC15 is a 48-bit shift register that provides direct access to the
//! coprocessor 15 (system control coprocessor) registers without having to
//! inject instructions into the pipeline:
//!
//! ```text
//! Bit 47..16 : data[31..0]       – 32 data bits
//! Bit 15     : access (write=1)
//! Bit 14..1  : cp15_addr[13..0]  – coprocessor register address
//! Bit 0      : nW (0=write host→cp15, 1=read cp15→host)
//! ```
//!
//! The CP15 address encoding (14 bits) for `MRC/MCR p15, Op1, Rd, CRn, CRm, Op2`:
//! ```text
//! [13..12] = Op1[1..0]
//! [11..8]  = CRn[3..0]
//! [7..4]   = CRm[3..0]
//! [3..1]   = Op2[2..0]
//! [0]      = Op2[2..0] (shared LSB of CRm / Op2 encoding, architecture-defined)
//! ```
//!
//! Reference: ARM926EJ-S Technical Reference Manual, §7.3.

use bitvec::prelude::*;

use crate::probe::{DebugProbeError, JtagAccess};

use super::tap::{IR_INTEST, IR_SCAN_N, SCAN_CHAIN_15, SCAN_N_DR_WIDTH};

/// Width of the SC15 data register in bits.
const SC15_DR_WIDTH: u32 = 48;

/// Encode a CP15 register address from MRC/MCR fields.
///
/// `op1` is usually 0 for standard system control registers.
pub fn cp15_addr(op1: u8, crn: u8, crm: u8, op2: u8) -> u16 {
    let op1 = (op1 as u16) & 0x3;
    let crn = (crn as u16) & 0xF;
    let crm = (crm as u16) & 0xF;
    let op2 = (op2 as u16) & 0x7;
    (op1 << 12) | (crn << 8) | (crm << 4) | (op2 << 1)
}

// ---------------------------------------------------------------------------
// Commonly used CP15 register addresses
// ---------------------------------------------------------------------------

/// CP15 c0, c0, 0 – Main ID Register (MIDR): op1=0, crn=0, crm=0, op2=0.
pub const CP15_MIDR: u16 = 0x0000;
/// CP15 c1, c0, 0 – System Control Register (SCTLR): op1=0, crn=1, crm=0, op2=0.
pub const CP15_SCTLR: u16 = 0x0100;
/// CP15 c7, c5, 0 – Invalidate entire instruction cache (ICIALLU).
pub const CP15_ICIALLU: u16 = 0x0750;
/// CP15 c7, c10, 4 – Data Synchronisation Barrier (DSB).
pub const CP15_DSB: u16 = 0x07A8;

// ---------------------------------------------------------------------------
// SC15 payload helpers
// ---------------------------------------------------------------------------

/// Build a 48-bit SC15 payload packed into 6 bytes (little-endian, LSB first).
///
/// `nw=true`  → read (cp15 → host)
/// `nw=false` → write (host → cp15)
fn build_sc15_payload(addr: u16, nw: bool, access: bool, data: u32) -> [u8; 6] {
    // Layout: [47..16]=data [15]=access [14..1]=cp15_addr [0]=nW
    let raw: u64 = ((data as u64) << 16)
        | ((access as u64) << 15)
        | (((addr & 0x3FFF) as u64) << 1)
        | (nw as u64);
    [
        (raw & 0xFF) as u8,
        ((raw >> 8) & 0xFF) as u8,
        ((raw >> 16) & 0xFF) as u8,
        ((raw >> 24) & 0xFF) as u8,
        ((raw >> 32) & 0xFF) as u8,
        ((raw >> 40) & 0xFF) as u8,
    ]
}

/// Extract the 32-bit data field from a 48-bit SC15 TDO response.
fn extract_sc15_data(bits: &BitSlice<usize, Lsb0>) -> u32 {
    let mut val: u32 = 0;
    for i in 0..32usize {
        if bits[16 + i] {
            val |= 1 << i;
        }
    }
    val
}

/// CP15 access driver.
pub struct Cp15Access<'probe> {
    probe: &'probe mut dyn JtagAccess,
    sc15_selected: bool,
}

impl<'probe> Cp15Access<'probe> {
    /// Create a new CP15 access driver.
    pub fn new(probe: &'probe mut dyn JtagAccess) -> Self {
        Self {
            probe,
            sc15_selected: false,
        }
    }

    /// Ensure SC15 is selected.
    fn ensure_sc15(&mut self) -> Result<(), DebugProbeError> {
        if !self.sc15_selected {
            let payload = [SCAN_CHAIN_15 & 0x1F];
            self.probe
                .write_register(IR_SCAN_N, &payload, SCAN_N_DR_WIDTH)?;
            self.probe.write_register(IR_INTEST, &[], 0)?;
            self.sc15_selected = true;
        }
        Ok(())
    }

    /// Invalidate cached state.
    pub fn invalidate(&mut self) {
        self.sc15_selected = false;
    }

    /// Write a value to a CP15 register.
    pub fn write_cp15(&mut self, addr: u16, value: u32) -> Result<(), DebugProbeError> {
        self.ensure_sc15()?;
        // nw=false (write), access=true
        let payload = build_sc15_payload(addr, false, true, value);
        self.probe.write_dr(&payload, SC15_DR_WIDTH)?;
        Ok(())
    }

    /// Read a value from a CP15 register (two-shift protocol).
    pub fn read_cp15(&mut self, addr: u16) -> Result<u32, DebugProbeError> {
        self.ensure_sc15()?;
        // First shift: nw=true (read), access=true, data=0 (address the register)
        let payload = build_sc15_payload(addr, true, true, 0);
        let _ = self.probe.write_dr(&payload, SC15_DR_WIDTH)?;
        // Second shift: capture the result
        let bits = self.probe.write_dr(&payload, SC15_DR_WIDTH)?;
        Ok(extract_sc15_data(&bits))
    }

    /// Convenience: write `SCTLR` (System Control Register), c1, c0, 0.
    pub fn write_sctlr(&mut self, value: u32) -> Result<(), DebugProbeError> {
        self.write_cp15(cp15_addr(0, 1, 0, 0), value)
    }

    /// Convenience: read `SCTLR`, c1, c0, 0.
    pub fn read_sctlr(&mut self) -> Result<u32, DebugProbeError> {
        self.read_cp15(cp15_addr(0, 1, 0, 0))
    }

    /// Invalidate entire instruction cache (c7, c5, 0).
    pub fn invalidate_icache(&mut self) -> Result<(), DebugProbeError> {
        self.write_cp15(cp15_addr(0, 7, 5, 0), 0)
    }

    /// Data synchronisation barrier (c7, c10, 4).
    pub fn dsb(&mut self) -> Result<(), DebugProbeError> {
        self.write_cp15(cp15_addr(0, 7, 10, 4), 0)
    }
}
