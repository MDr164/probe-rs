//! ARM9 pipeline debug access via scan chain 0 (SC0).
//!
//! SC0 is a 67-bit shift register that gives direct access to the ARM9
//! instruction pipeline and data bus:
//!
//! ```text
//! Bit 66..35 : instruction bus [31..0]  (note: bit-reversed on shift-in)
//! Bit 34     : sysspeed
//! Bit 33..32 : reserved
//! Bit 31..0  : data bus [31..0]
//! ```
//!
//! In debug state the CPU repeatedly executes "NOP + drain" cycles driven by
//! the JTAG interface.  To read/write registers we inject ARM instructions
//! (MCR/MRC, STR, etc.) into the pipeline via SC0.
//!
//! The key rule: *instr_bus bits are bit-reversed* when shifted.  That is,
//! bit 31 of the instruction word goes into bit 66 of the DR (the MSB), and
//! bit 0 goes into bit 35.
//!
//! Reference: ARM9TDMI Technical Reference Manual, §4 "Debug Interface".

use bitvec::prelude::*;

use crate::probe::{DebugProbeError, JtagAccess};

use super::tap::{IR_INTEST, IR_RESTART, IR_SCAN_N, SCAN_CHAIN_0, SCAN_N_DR_WIDTH};

/// Width of the SC0 data register in bits.
const SC0_DR_WIDTH: u32 = 67;

/// ARM NOP instruction in ARM (A32) encoding.
pub const ARM_NOP: u32 = 0xE320F000;

/// ARM BKPT #0 in A32 encoding (used as flash-algo header sentinel).
pub const ARM_BKPT: u32 = 0xE1200070;

// ---------------------------------------------------------------------------
// Instruction encoding helpers (all A32 unless noted)
// ---------------------------------------------------------------------------

/// Build `STR Rd, [Rn]` (A32, offset=0, no writeback).
pub fn arm_str(rd: u8, rn: u8) -> u32 {
    0xE5800000 | ((rn as u32 & 0xF) << 16) | ((rd as u32 & 0xF) << 12)
}

/// Build `LDR Rd, [Rn]` (A32, offset=0, no writeback).
pub fn arm_ldr(rd: u8, rn: u8) -> u32 {
    0xE5900000 | ((rn as u32 & 0xF) << 16) | ((rd as u32 & 0xF) << 12)
}

/// Build `MRC p14, 0, Rd, c0, c5, 0` – read DCC data register → Rd.
///
/// ARM9EJ-S DCC data register: CRn=0, CRm=5, op1=0, op2=0.
pub fn arm_mrc_dcc(rd: u8) -> u32 {
    // MRC p14, 0, Rd, c0, c5, 0  →  0xEE10_0E15 | (Rd << 12)
    0xEE100E15 | ((rd as u32 & 0xF) << 12)
}

/// Build `MCR p14, 0, Rd, c0, c5, 0` – write Rd → DCC data register.
///
/// ARM9EJ-S DCC data register: CRn=0, CRm=5, op1=0, op2=0.
pub fn arm_mcr_dcc(rd: u8) -> u32 {
    // MCR p14, 0, Rd, c0, c5, 0  →  0xEE00_0E15 | (Rd << 12)
    0xEE000E15 | ((rd as u32 & 0xF) << 12)
}

/// Build `MRS Rd, CPSR` (A32).
pub fn arm_mrs_cpsr(rd: u8) -> u32 {
    0xE10F0000 | ((rd as u32 & 0xF) << 12)
}

/// Build `MSR CPSR_cxsf, Rn` (A32).
pub fn arm_msr_cpsr(rn: u8) -> u32 {
    0xE12FF000 | (rn as u32 & 0xF)
}

// ---------------------------------------------------------------------------
// SC0 access helpers
// ---------------------------------------------------------------------------

/// Pack a 67-bit SC0 payload into a 9-byte buffer (little-endian, LSB first).
///
/// Layout:
/// - `bits[31..0]`  = data_bus
/// - `bits[33..32]` = reserved (0)
/// - `bits[34]`     = sysspeed
/// - `bits[66..35]` = instr_bus (bit-reversed: bit35=instr[31], bit66=instr[0])
fn build_sc0_payload(instr: u32, sysspeed: bool, data: u32) -> [u8; 9] {
    let mut bv: BitVec<u8, Lsb0> = BitVec::repeat(false, 67);

    // data_bus: bits 0..=31
    for i in 0..32usize {
        bv.set(i, (data >> i) & 1 != 0);
    }

    // sysspeed: bit 34
    bv.set(34, sysspeed);

    // instr_bus: bits 35..=66, bit-reversed
    // bit 35 of payload = bit 31 of instruction
    for i in 0..32usize {
        let instr_bit = (instr >> (31 - i)) & 1 != 0;
        bv.set(35 + i, instr_bit);
    }

    let mut out = [0u8; 9];
    out[..bv.as_raw_slice().len()].copy_from_slice(bv.as_raw_slice());
    out
}

/// Extract the 32-bit data bus field from a 67-bit SC0 TDO response.
fn extract_sc0_data(bits: &BitSlice<usize, Lsb0>) -> u32 {
    let mut val: u32 = 0;
    for i in 0..32usize {
        if bits[i] {
            val |= 1 << i;
        }
    }
    val
}

/// ARM9 pipeline debug driver.
pub struct PipelineAccess<'probe> {
    probe: &'probe mut dyn JtagAccess,
    sc0_selected: bool,
}

impl<'probe> PipelineAccess<'probe> {
    /// Create a new pipeline access driver.
    pub fn new(probe: &'probe mut dyn JtagAccess) -> Self {
        Self {
            probe,
            sc0_selected: false,
        }
    }

    /// Ensure SC0 is selected.
    fn ensure_sc0(&mut self) -> Result<(), DebugProbeError> {
        if !self.sc0_selected {
            let payload = [SCAN_CHAIN_0 & 0x1F];
            self.probe
                .write_register(IR_SCAN_N, &payload, SCAN_N_DR_WIDTH)?;
            self.probe.write_register(IR_INTEST, &[], 0)?;
            self.sc0_selected = true;
        }
        Ok(())
    }

    /// Invalidate the cached state.
    pub fn invalidate(&mut self) {
        self.sc0_selected = false;
    }

    /// Clock one instruction into the pipeline; return the data bus value from TDO.
    ///
    /// `sysspeed=false` means the instruction is executed under debug control
    /// (debug speed, one clock per JTAG shift).
    pub fn clock_out(
        &mut self,
        instr: u32,
        data_in: u32,
        sysspeed: bool,
    ) -> Result<u32, DebugProbeError> {
        self.ensure_sc0()?;
        let payload = build_sc0_payload(instr, sysspeed, data_in);
        let bits = self.probe.write_dr(&payload, SC0_DR_WIDTH)?;
        Ok(extract_sc0_data(&bits))
    }

    /// Clock a NOP (no side effects).
    pub fn nop(&mut self) -> Result<(), DebugProbeError> {
        self.clock_out(ARM_NOP, 0, false)?;
        Ok(())
    }

    /// Restart the CPU and exit debug state (IR=RESTART).
    pub fn restart(&mut self) -> Result<(), DebugProbeError> {
        self.sc0_selected = false;
        self.probe.write_register(IR_RESTART, &[], 0)?;
        Ok(())
    }
}
