//! ARM9 EmbeddedICE register access via scan chain 2 (SC2).
//!
//! SC2 is a 38-bit shift register:
//!
//! ```text
//! [37..6] data[31..0]   – 32 data bits (LSB first)
//! [5..1]  addr[4..0]    – 5-bit register address
//! [0]     rw            – 0 = read, 1 = write
//! ```
//!
//! To **write** a register: shift `{data, addr, rw=1}` into SC2 while INTEST.
//! To **read** a register:  shift `{0, addr, rw=0}` into SC2 (the current
//!   value of the addressed register is shifted out on TDO during the *next*
//!   shift).  The typical pattern is therefore two sequential shifts: one to
//!   address the register, one to capture the output.
//!
//! Reference: ARM Embedded ICE macrocell 6 Technical Reference Manual,
//!            §3.3 "Scan Chain 2".

use bitvec::prelude::*;

use crate::probe::{DebugProbeError, JtagAccess};

use super::tap::{IR_INTEST, IR_SCAN_N, SCAN_CHAIN_2, SCAN_N_DR_WIDTH};

/// Width of the SC2 data register in bits.
const SC2_DR_WIDTH: u32 = 38;

// ---------------------------------------------------------------------------
// Well-known EmbeddedICE (version 6) register addresses
// ---------------------------------------------------------------------------

/// Debug Control register (read/write, 6 bits in use).
pub const REG_DBG_CTRL: u8 = 0;
/// Debug Status register (read-only, 10 bits in use).
pub const REG_DBG_STAT: u8 = 1;
/// Vector Catch register.
pub const REG_VEC_CATCH: u8 = 2;
/// Debug Comms Control register.
pub const REG_COMMS_CTRL: u8 = 4;
/// Debug Comms Data register.
pub const REG_COMMS_DATA: u8 = 5;
/// Watchpoint 0 address value.
pub const REG_W0_ADDR_VALUE: u8 = 8;
/// Watchpoint 0 address mask.
pub const REG_W0_ADDR_MASK: u8 = 9;
/// Watchpoint 0 data value.
pub const REG_W0_DATA_VALUE: u8 = 10;
/// Watchpoint 0 data mask.
pub const REG_W0_DATA_MASK: u8 = 11;
/// Watchpoint 0 control value.
pub const REG_W0_CTRL_VALUE: u8 = 12;
/// Watchpoint 0 control mask.
pub const REG_W0_CTRL_MASK: u8 = 13;
/// Watchpoint 1 address value.
pub const REG_W1_ADDR_VALUE: u8 = 16;
/// Watchpoint 1 address mask.
pub const REG_W1_ADDR_MASK: u8 = 17;
/// Watchpoint 1 data value.
pub const REG_W1_DATA_VALUE: u8 = 18;
/// Watchpoint 1 data mask.
pub const REG_W1_DATA_MASK: u8 = 19;
/// Watchpoint 1 control value.
pub const REG_W1_CTRL_VALUE: u8 = 20;
/// Watchpoint 1 control mask.
pub const REG_W1_CTRL_MASK: u8 = 21;

// ---------------------------------------------------------------------------
// DBG_CTRL bit positions (6-bit register)
// ---------------------------------------------------------------------------

/// DBG_CTRL: DBGACK – debug acknowledge / halting mode enable.
pub const CTRL_DBGACK: u32 = 1 << 0;
/// DBG_CTRL: DBGRQ – debug request (forces halt).
pub const CTRL_DBGRQ: u32 = 1 << 1;
/// DBG_CTRL: INTDIS – interrupt disable while in debug state.
pub const CTRL_INTDIS: u32 = 1 << 2;
/// DBG_CTRL: MON_EN – monitor mode enable.
pub const CTRL_MON_EN: u32 = 1 << 4;
/// DBG_CTRL: IFEN – instruction fetch enable.
pub const CTRL_IFEN: u32 = 1 << 5;

// ---------------------------------------------------------------------------
// DBG_STAT bit positions (10-bit register for EmbeddedICE v6)
// ---------------------------------------------------------------------------

/// DBG_STAT: DBGACK – the core has entered debug state.
pub const STAT_DBGACK: u32 = 1 << 0;
/// DBG_STAT: DBGRQ – a debug request is pending.
pub const STAT_DBGRQ: u32 = 1 << 1;
/// DBG_STAT: IFEN – instruction fetch occurred.
pub const STAT_IFEN: u32 = 1 << 2;
/// DBG_STAT: SYSCOMP – system speed access completed.
pub const STAT_SYSCOMP: u32 = 1 << 3;
/// DBG_STAT: MON_EN – monitor mode is enabled.
pub const STAT_MON_EN: u32 = 1 << 4;
/// DBG_STAT: MOE[2:0] – method of entry into debug state (bits 9:7 in the
/// 10-bit register; here shifted to bits 7:5 of the 32-bit read value).
pub const STAT_MOE_MASK: u32 = 0b111 << 5;
/// DBG_STAT: TBIT – Thumb state bit.
pub const STAT_TBIT: u32 = 1 << 8;
/// DBG_STAT: ITBIT – IT state bit.
pub const STAT_ITBIT: u32 = 1 << 9;

// ---------------------------------------------------------------------------
// SC2 access helpers
// ---------------------------------------------------------------------------

/// Build a 38-bit SC2 payload and pack it into a 5-byte little-endian buffer.
///
/// Layout: `[37..6]=data, [5..1]=addr, [0]=rw`
fn build_sc2_payload(addr: u8, rw: bool, data: u32) -> [u8; 5] {
    let raw: u64 = ((data as u64) << 6) | (((addr & 0x1F) as u64) << 1) | (rw as u64);
    // Pack 38 bits into 5 bytes, little-endian
    [
        (raw & 0xFF) as u8,
        ((raw >> 8) & 0xFF) as u8,
        ((raw >> 16) & 0xFF) as u8,
        ((raw >> 24) & 0xFF) as u8,
        ((raw >> 32) & 0x3F) as u8,
    ]
}

/// Extract the 32-bit data field from a 38-bit SC2 TDO response.
fn extract_sc2_data(bits: &BitSlice<usize, Lsb0>) -> u32 {
    // Bits [37..6] are the data field; bits [5..0] are addr+rw (previous cycle).
    let mut val: u32 = 0;
    for i in 0..32u32 {
        if bits[6 + i as usize] {
            val |= 1 << i;
        }
    }
    val
}

/// Low-level EmbeddedICE access driver.
pub struct EmbeddedIceAccess<'probe> {
    probe: &'probe mut dyn JtagAccess,
    /// Whether SC2 is currently selected (IR=INTEST, SCAN_N=2).
    sc2_selected: bool,
}

impl<'probe> EmbeddedIceAccess<'probe> {
    /// Create a new access driver.  The scan chain is selected on first use.
    pub fn new(probe: &'probe mut dyn JtagAccess) -> Self {
        Self {
            probe,
            sc2_selected: false,
        }
    }

    /// Ensure SC2 is selected (IR=SCAN_N, DR=2, then IR=INTEST).
    fn ensure_sc2(&mut self) -> Result<(), DebugProbeError> {
        if !self.sc2_selected {
            let payload = [SCAN_CHAIN_2 & 0x1F];
            self.probe
                .write_register(IR_SCAN_N, &payload, SCAN_N_DR_WIDTH)?;
            // Switch to INTEST to activate the selected chain.
            self.probe.write_register(IR_INTEST, &[], 0)?;
            self.sc2_selected = true;
        }
        Ok(())
    }

    /// Invalidate the cached SC2 state (call when switching scan chains).
    pub fn invalidate(&mut self) {
        self.sc2_selected = false;
    }

    /// Write a 32-bit value to an EmbeddedICE register.
    pub fn write_reg(&mut self, addr: u8, value: u32) -> Result<(), DebugProbeError> {
        self.ensure_sc2()?;
        let payload = build_sc2_payload(addr, true, value);
        self.probe.write_dr(&payload, SC2_DR_WIDTH)?;
        Ok(())
    }

    /// Read a 32-bit value from an EmbeddedICE register.
    ///
    /// Requires two DR shifts: the first selects the address, the second
    /// captures the result (the core has one scan-chain latency).
    pub fn read_reg(&mut self, addr: u8) -> Result<u32, DebugProbeError> {
        self.ensure_sc2()?;
        // First shift: address with rw=0, data=0.
        let payload = build_sc2_payload(addr, false, 0);
        let bits = self.probe.write_dr(&payload, SC2_DR_WIDTH)?;
        // The value of the addressed register is captured from the *previous*
        // shift's TDO, which for a cold read is meaningless.  We need a second
        // shift to capture the actual value.
        let _ = bits;
        // Second shift: keep same address, dummy data.
        let bits2 = self.probe.write_dr(&payload, SC2_DR_WIDTH)?;
        Ok(extract_sc2_data(&bits2))
    }

    /// Read the Debug Status register (SC2, addr=1).
    pub fn read_dbg_stat(&mut self) -> Result<u32, DebugProbeError> {
        self.read_reg(REG_DBG_STAT)
    }

    /// Write the Debug Control register (SC2, addr=0).
    pub fn write_dbg_ctrl(&mut self, value: u32) -> Result<(), DebugProbeError> {
        self.write_reg(REG_DBG_CTRL, value)
    }

    /// Read the Debug Control register (SC2, addr=0).
    pub fn read_dbg_ctrl(&mut self) -> Result<u32, DebugProbeError> {
        self.read_reg(REG_DBG_CTRL)
    }

    /// Return `true` if the core has acknowledged debug state (DBGACK=1).
    pub fn is_halted(&mut self) -> Result<bool, DebugProbeError> {
        Ok(self.read_dbg_stat()? & STAT_DBGACK != 0)
    }

    /// Request a halt by setting DBGRQ in DBG_CTRL.
    pub fn request_halt(&mut self) -> Result<(), DebugProbeError> {
        let ctrl = self.read_dbg_ctrl()?;
        self.write_dbg_ctrl(ctrl | CTRL_DBGRQ)
    }

    /// Clear the halt request.
    pub fn clear_halt_request(&mut self) -> Result<(), DebugProbeError> {
        let ctrl = self.read_dbg_ctrl()?;
        self.write_dbg_ctrl(ctrl & !CTRL_DBGRQ)
    }
}
