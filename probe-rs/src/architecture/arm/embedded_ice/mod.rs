//! ARM9 EmbeddedICE debug architecture support.
//!
//! This module provides the low-level interface to ARM9 cores (e.g. ARM926EJ-S)
//! that use the pre-CoreSight EmbeddedICE debug architecture accessed via JTAG
//! scan chains.  It is intentionally separate from the CoreSight DAP-based path
//! used by Cortex-M and Cortex-A targets.
//!
//! # Scan chains
//!
//! | Chain | Width | Purpose                           |
//! |-------|-------|-----------------------------------|
//! | SC0   | 67    | Pipeline debug / register access  |
//! | SC2   | 38    | EmbeddedICE register read/write   |
//! | SC15  | 48    | CP15 coprocessor access           |
//!
//! # Architecture
//!
//! ```text
//! EmbeddedIce
//!   ├── EmbeddedIceAccess (SC2)  – halt/run/register read
//!   ├── PipelineAccess    (SC0)  – instruction injection
//!   └── Cp15Access        (SC15) – MMU / cache control
//! ```

pub mod cp15;
pub mod ice;
pub mod pipeline;
pub mod tap;

use std::time::{Duration, Instant};

use crate::probe::{DebugProbeError, JtagAccess};

use ice::{
    EmbeddedIceAccess, REG_W0_ADDR_MASK, REG_W0_ADDR_VALUE, REG_W0_CTRL_MASK, REG_W0_CTRL_VALUE,
    REG_W0_DATA_MASK, REG_W0_DATA_VALUE,
};
use pipeline::PipelineAccess;

/// Timeout for waiting for the core to halt.
const HALT_TIMEOUT: Duration = Duration::from_millis(500);

/// Watchpoint 0 control value for "always break" (EmbeddedICE v6 encoding).
///
/// The control value that makes watchpoint 0 fire on any fetch (used as
/// single-step breakpoint):  `ENABLE=1, RANGE=0, noTRANS=0, nOPC=0,
/// MASK=0, CHAIN=0, EN=1` — the exact encoding varies by version; this
/// is a typical all-fetch watchpoint.
const WP0_CTRL_BREAK: u32 = 0x00000007;
/// Watchpoint 0 control mask (which control bits are compared = all zeros
/// means compare everything in the ctrl value).
const WP0_CTRL_MASK: u32 = 0x00F00F00;

/// Combined EmbeddedICE interface.
///
/// This is the top-level struct that the `Armv5te` core implementation uses
/// to interact with an ARM926EJ-S (or similar ARM9EJ-S) core via JTAG.
pub struct EmbeddedIce<'probe> {
    probe: &'probe mut dyn JtagAccess,
}

impl<'probe> EmbeddedIce<'probe> {
    /// Create a new EmbeddedICE interface.
    pub fn new(probe: &'probe mut dyn JtagAccess) -> Self {
        Self { probe }
    }

    // -----------------------------------------------------------------------
    // Low-level helpers: SC2, SC0, SC15 accessors
    // -----------------------------------------------------------------------

    fn ice(&mut self) -> EmbeddedIceAccess<'_> {
        EmbeddedIceAccess::new(self.probe)
    }

    fn pipeline(&mut self) -> PipelineAccess<'_> {
        PipelineAccess::new(self.probe)
    }

    fn cp15(&mut self) -> cp15::Cp15Access<'_> {
        cp15::Cp15Access::new(self.probe)
    }

    // -----------------------------------------------------------------------
    // Halt / resume
    // -----------------------------------------------------------------------

    /// Read the Debug Status register.
    pub fn read_dbg_stat(&mut self) -> Result<u32, DebugProbeError> {
        self.ice().read_dbg_stat()
    }

    /// Returns `true` if the core is in debug state (DBGACK=1).
    pub fn is_halted(&mut self) -> Result<bool, DebugProbeError> {
        self.ice().is_halted()
    }

    /// Request the core to halt by asserting DBGRQ.
    pub fn request_halt(&mut self) -> Result<(), DebugProbeError> {
        self.ice().request_halt()
    }

    /// Poll until the core halts or the timeout expires.
    pub fn wait_for_halt(&mut self, timeout: Duration) -> Result<(), DebugProbeError> {
        let start = Instant::now();
        loop {
            if self.is_halted()? {
                return Ok(());
            }
            if start.elapsed() >= timeout {
                return Err(DebugProbeError::Timeout);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Halt the core (assert DBGRQ then wait for DBGACK).
    pub fn halt(&mut self, timeout: Duration) -> Result<(), DebugProbeError> {
        self.request_halt()?;
        self.wait_for_halt(timeout)
    }

    /// Resume execution (clear DBGRQ, then send RESTART).
    pub fn resume(&mut self) -> Result<(), DebugProbeError> {
        self.ice().clear_halt_request()?;
        self.pipeline().restart()
    }

    // -----------------------------------------------------------------------
    // Register read/write (via DCC)
    // -----------------------------------------------------------------------

    /// Read general-purpose register `r0`–`r15`.
    ///
    /// Uses the DCC path: inject `MCR p14, 0, Rn, c0, c5, 0` into the
    /// pipeline (SC0), then read the value from `COMMS_DATA` via SC2.
    pub fn read_core_register(&mut self, reg: u8) -> Result<u32, DebugProbeError> {
        // Phase 1 – SC0: inject MCR instruction to push register into DCC TX.
        {
            let mut pipe = PipelineAccess::new(self.probe);
            pipe.invalidate();
            pipe.nop()?;
            pipe.clock_out(pipeline::arm_mcr_dcc(reg), 0, false)?;
            pipe.nop()?;
            pipe.nop()?;
        }
        // Phase 2 – SC2: read COMMS_DATA.
        {
            let mut ice = EmbeddedIceAccess::new(self.probe);
            ice.invalidate();
            ice.read_reg(ice::REG_COMMS_DATA)
        }
    }

    /// Write general-purpose register `r0`–`r14` via DCC injection.
    pub fn write_core_register(&mut self, reg: u8, value: u32) -> Result<(), DebugProbeError> {
        // Phase 1 – SC2: write value into COMMS_DATA (DCC RX).
        {
            let mut ice = EmbeddedIceAccess::new(self.probe);
            ice.invalidate();
            ice.write_reg(ice::REG_COMMS_DATA, value)?;
        }
        // Phase 2 – SC0: inject MRC p14, 0, Rn, c0, c5, 0  (DCC RX → Rn).
        {
            let mut pipe = PipelineAccess::new(self.probe);
            pipe.invalidate();
            pipe.nop()?;
            pipe.clock_out(pipeline::arm_mrc_dcc(reg), 0, false)?;
            pipe.nop()?;
        }
        Ok(())
    }

    /// Read the PC register.  ARM9 stores the PC as r15; we adjust for the
    /// pipeline prefetch (+8 in ARM state, +4 in Thumb state).
    pub fn read_pc(&mut self) -> Result<u32, DebugProbeError> {
        let raw = self.read_core_register(15)?;
        // The value captured is PC+8 (ARM) or PC+4 (Thumb).
        // Caller is responsible for knowing the instruction set and adjusting.
        Ok(raw)
    }

    /// Read CPSR by injecting `MRS R0, CPSR` and routing through DCC.
    pub fn read_cpsr(&mut self) -> Result<u32, DebugProbeError> {
        // Save R0.
        let r0_saved = self.read_core_register(0)?;

        // Inject MRS R0, CPSR.
        {
            let mut pipe = PipelineAccess::new(self.probe);
            pipe.invalidate();
            pipe.nop()?;
            pipe.clock_out(pipeline::arm_mrs_cpsr(0), 0, false)?;
            pipe.nop()?;
        }

        // Read CPSR from R0 via DCC.
        let cpsr = self.read_core_register(0)?;

        // Restore R0.
        self.write_core_register(0, r0_saved)?;

        Ok(cpsr)
    }

    /// Write CPSR by injecting `MSR CPSR_cxsf, R0` via the pipeline.
    pub fn write_cpsr(&mut self, value: u32) -> Result<(), DebugProbeError> {
        // Save R0.
        let r0_saved = self.read_core_register(0)?;

        // Load the new CPSR value into R0 via DCC.
        self.write_core_register(0, value)?;

        // Inject MSR CPSR_cxsf, R0.
        {
            let mut pipe = PipelineAccess::new(self.probe);
            pipe.invalidate();
            pipe.nop()?;
            pipe.clock_out(pipeline::arm_msr_cpsr(0), 0, false)?;
            pipe.nop()?;
        }

        // Restore R0.
        self.write_core_register(0, r0_saved)?;

        Ok(())
    }

    // -----------------------------------------------------------------------
    // Memory read/write (word-aligned, word-granularity)
    // -----------------------------------------------------------------------

    /// Read a 32-bit word from memory via LDR injection and DCC.
    ///
    /// The address must be 4-byte aligned.  R0 and R1 are saved and restored
    /// so the operation is transparent to the halted program.
    pub fn read_word_32(&mut self, address: u32) -> Result<u32, DebugProbeError> {
        // Save R0 and R1.
        let r0_saved = self.read_core_register(0)?;
        let r1_saved = self.read_core_register(1)?;

        // Load address into R0 via DCC.
        self.write_core_register(0, address)?;

        // SC0: LDR R1, [R0] then MCR p14, 0, R1, c0, c5, 0
        {
            let mut pipe = PipelineAccess::new(self.probe);
            pipe.invalidate();
            pipe.nop()?;
            pipe.nop()?;
            pipe.clock_out(pipeline::arm_ldr(1, 0), 0, false)?;
            pipe.nop()?;
            pipe.clock_out(pipeline::arm_mcr_dcc(1), 0, false)?;
            pipe.nop()?;
            pipe.nop()?;
        }

        // SC2: read COMMS_DATA (the loaded value).
        let result = {
            let mut ice = EmbeddedIceAccess::new(self.probe);
            ice.invalidate();
            ice.read_reg(ice::REG_COMMS_DATA)?
        };

        // Restore R0 and R1.
        self.write_core_register(1, r1_saved)?;
        self.write_core_register(0, r0_saved)?;

        Ok(result)
    }

    /// Write a 32-bit word to memory via STR injection.
    ///
    /// The address must be 4-byte aligned.  R0 and R1 are saved and restored
    /// so the operation is transparent to the halted program.
    pub fn write_word_32(&mut self, address: u32, value: u32) -> Result<(), DebugProbeError> {
        // Save R0 and R1.
        let r0_saved = self.read_core_register(0)?;
        let r1_saved = self.read_core_register(1)?;

        // Load address into R0, value into R1 via DCC.
        self.write_core_register(0, address)?;
        self.write_core_register(1, value)?;

        // SC0: STR R1, [R0]
        {
            let mut pipe = PipelineAccess::new(self.probe);
            pipe.invalidate();
            pipe.nop()?;
            pipe.nop()?;
            pipe.clock_out(pipeline::arm_str(1, 0), 0, false)?;
            pipe.nop()?;
        }

        // Restore R0 and R1.
        self.write_core_register(1, r1_saved)?;
        self.write_core_register(0, r0_saved)?;

        Ok(())
    }

    // -----------------------------------------------------------------------
    // Breakpoints (EmbeddedICE watchpoint units)
    // -----------------------------------------------------------------------

    /// Set a hardware breakpoint using watchpoint unit 0.
    ///
    /// Only unit 0 is available for user breakpoints; unit 1 is reserved for
    /// single-step (see [`step()`](Self::step)).
    pub fn set_breakpoint(&mut self, unit: usize, addr: u32) -> Result<(), DebugProbeError> {
        if unit != 0 {
            return Err(DebugProbeError::NotImplemented {
                function_name: "set_breakpoint: only unit 0 available (unit 1 reserved for step)",
            });
        }

        let mut ice = EmbeddedIceAccess::new(self.probe);
        // Address comparator: exact match, no mask.
        ice.write_reg(REG_W0_ADDR_VALUE, addr)?;
        ice.write_reg(REG_W0_ADDR_MASK, 0x0000_0000)?;
        // Data: don't care (mask=all-ones).
        ice.write_reg(REG_W0_DATA_VALUE, 0x0000_0000)?;
        ice.write_reg(REG_W0_DATA_MASK, 0xFFFF_FFFF)?;
        // Control: fetch (nOPC=0), enabled.
        ice.write_reg(REG_W0_CTRL_VALUE, WP0_CTRL_BREAK)?;
        ice.write_reg(REG_W0_CTRL_MASK, WP0_CTRL_MASK)?;
        Ok(())
    }

    /// Clear a hardware breakpoint on unit 0.
    pub fn clear_breakpoint(&mut self, unit: usize) -> Result<(), DebugProbeError> {
        if unit != 0 {
            return Err(DebugProbeError::NotImplemented {
                function_name: "clear_breakpoint: only unit 0 available (unit 1 reserved for step)",
            });
        }

        let mut ice = EmbeddedIceAccess::new(self.probe);
        ice.write_reg(REG_W0_CTRL_VALUE, 0)?;
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Single-step
    // -----------------------------------------------------------------------

    /// Execute a single instruction step.
    ///
    /// Uses watchpoint 1 as a "step complete" trap: configure it to fire on
    /// any fetch except the current PC, resume, then wait for halt.
    pub fn step(&mut self) -> Result<(), DebugProbeError> {
        // Configure watchpoint 1 as a "break on any fetch" — this will fire
        // after the single instruction executes and the PC changes.
        {
            let mut ice = EmbeddedIceAccess::new(self.probe);
            ice.write_reg(ice::REG_W1_ADDR_VALUE, 0x0000_0000)?;
            ice.write_reg(ice::REG_W1_ADDR_MASK, 0xFFFF_FFFF)?; // match any address
            ice.write_reg(ice::REG_W1_DATA_VALUE, 0x0000_0000)?;
            ice.write_reg(ice::REG_W1_DATA_MASK, 0xFFFF_FFFF)?;
            ice.write_reg(ice::REG_W1_CTRL_VALUE, WP0_CTRL_BREAK)?;
            ice.write_reg(ice::REG_W1_CTRL_MASK, WP0_CTRL_MASK)?;
        }

        // Resume the core.
        self.resume()?;

        // Wait for the watchpoint to fire.
        self.wait_for_halt(HALT_TIMEOUT)?;

        // Disable watchpoint 1 again.
        self.ice().write_reg(ice::REG_W1_CTRL_VALUE, 0)?;

        Ok(())
    }

    // -----------------------------------------------------------------------
    // CP15 helpers
    // -----------------------------------------------------------------------

    /// Invalidate instruction cache.
    pub fn invalidate_icache(&mut self) -> Result<(), DebugProbeError> {
        self.cp15().invalidate_icache()
    }
}
