//! ARM926EJ-S (ARMv5TEJ) core interface via EmbeddedICE.
//!
//! This implements [`CoreInterface`] for ARM9 cores that use the pre-CoreSight
//! EmbeddedICE debug architecture, accessed via raw JTAG scan chains rather
//! than the CoreSight DAP/MEM-AP path.

use std::time::{Duration, Instant};

use crate::{
    Architecture, CoreInformation, CoreInterface, CoreRegister, CoreRegisters, CoreStatus,
    CoreType, Endian, Error, InstructionSet, MemoryInterface, RegisterId, RegisterValue,
    core::HaltReason, probe::DebugProbeError,
};

use super::armv5te_regs::{ARMV5TE_CORE_REGISTERS, FP, PC, RA, SP};

use crate::architecture::arm::embedded_ice::EmbeddedIce;

/// Number of user-available hardware breakpoint units.
///
/// ARM926EJ-S has two EmbeddedICE watchpoint units, but unit 1 is reserved
/// for single-step, leaving only unit 0 for user breakpoints.
const NUM_HW_BREAKPOINTS: usize = 1;

/// CPSR bit 5: Thumb state bit.
const CPSR_T_BIT: u32 = 1 << 5;

// ---------------------------------------------------------------------------
// Per-core state
// ---------------------------------------------------------------------------

/// Cached state for an ARMv5TEJ (ARM926EJ-S) core.
#[derive(Debug)]
pub struct Armv5teState {
    /// Whether the core driver has been initialised.
    initialized: bool,
    /// Current known core status.
    current_state: CoreStatus,
    /// Are breakpoints (watchpoints) currently enabled?
    hw_breakpoints_enabled: bool,
    /// Cached CPSR value (to detect Thumb state).
    // Retained for future use when Thumb-state detection is fully implemented.
    #[allow(dead_code)]
    cached_cpsr: Option<u32>,
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

    fn initialize(&mut self) {
        self.initialized = true;
    }

    fn initialized(&self) -> bool {
        self.initialized
    }
}

// ---------------------------------------------------------------------------
// Armv5te core struct
// ---------------------------------------------------------------------------

/// ARM926EJ-S (ARMv5TEJ) core driver.
pub struct Armv5te<'probe> {
    ice: EmbeddedIce<'probe>,
    state: &'probe mut Armv5teState,
}

impl<'probe> Armv5te<'probe> {
    /// Create and initialise an `Armv5te` core driver.
    pub(crate) fn new(
        ice: EmbeddedIce<'probe>,
        state: &'probe mut Armv5teState,
    ) -> Result<Self, Error> {
        let mut core = Self { ice, state };

        if !core.state.initialized() {
            let status = core.status()?;
            core.state.current_state = status;
            core.state.initialize();
        }

        Ok(core)
    }

    /// Map a `RegisterId` to an ARM register number (0–15, plus CPSR=25).
    fn reg_id_to_num(id: RegisterId) -> Result<u8, Error> {
        let n = id.0;
        if n <= 15 {
            return Ok(n as u8);
        }
        if n == 25 {
            // CPSR handled specially
            return Ok(16); // sentinel for CPSR
        }
        Err(Error::Probe(DebugProbeError::Other(format!(
            "ARMv5TEJ: unknown register ID {n}"
        ))))
    }

    /// Return `true` if the core is currently in Thumb state.
    fn is_thumb(&mut self) -> Result<bool, Error> {
        let cpsr = self.ice.read_cpsr().map_err(Error::Probe)?;
        Ok(cpsr & CPSR_T_BIT != 0)
    }
}

// ---------------------------------------------------------------------------
// MemoryInterface — delegate to EmbeddedIce memory primitives
// ---------------------------------------------------------------------------

impl MemoryInterface<Error> for Armv5te<'_> {
    fn supports_native_64bit_access(&mut self) -> bool {
        false
    }

    fn read_word_64(&mut self, address: u64) -> Result<u64, Error> {
        let lo = self
            .ice
            .read_word_32(address as u32)
            .map_err(Error::Probe)?;
        let hi = self
            .ice
            .read_word_32((address + 4) as u32)
            .map_err(Error::Probe)?;
        Ok(lo as u64 | ((hi as u64) << 32))
    }

    fn read_word_32(&mut self, address: u64) -> Result<u32, Error> {
        self.ice.read_word_32(address as u32).map_err(Error::Probe)
    }

    fn read_word_16(&mut self, address: u64) -> Result<u16, Error> {
        // ARM926 has no halfword debug bus; read 32 bits and extract the halfword.
        let word = self
            .ice
            .read_word_32(address as u32 & !3)
            .map_err(Error::Probe)?;
        let shift = (address & 2) * 8;
        Ok(((word >> shift) & 0xFFFF) as u16)
    }

    fn read_word_8(&mut self, address: u64) -> Result<u8, Error> {
        let word = self
            .ice
            .read_word_32(address as u32 & !3)
            .map_err(Error::Probe)?;
        let shift = (address & 3) * 8;
        Ok(((word >> shift) & 0xFF) as u8)
    }

    fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), Error> {
        for (i, d) in data.iter_mut().enumerate() {
            *d = self.read_word_64(address + (i as u64) * 8)?;
        }
        Ok(())
    }

    fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
        for (i, d) in data.iter_mut().enumerate() {
            *d = self.read_word_32(address + (i as u64) * 4)?;
        }
        Ok(())
    }

    fn read_16(&mut self, address: u64, data: &mut [u16]) -> Result<(), Error> {
        for (i, d) in data.iter_mut().enumerate() {
            *d = self.read_word_16(address + (i as u64) * 2)?;
        }
        Ok(())
    }

    fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        for (i, d) in data.iter_mut().enumerate() {
            *d = self.read_word_8(address + i as u64)?;
        }
        Ok(())
    }

    fn write_word_64(&mut self, address: u64, data: u64) -> Result<(), Error> {
        self.ice
            .write_word_32(address as u32, (data & 0xFFFF_FFFF) as u32)
            .map_err(Error::Probe)?;
        self.ice
            .write_word_32((address + 4) as u32, ((data >> 32) & 0xFFFF_FFFF) as u32)
            .map_err(Error::Probe)
    }

    fn write_word_32(&mut self, address: u64, data: u32) -> Result<(), Error> {
        self.ice
            .write_word_32(address as u32, data)
            .map_err(Error::Probe)
    }

    fn write_word_16(&mut self, _address: u64, _data: u16) -> Result<(), Error> {
        Err(Error::NotImplemented("16-bit write via EmbeddedICE"))
    }

    fn write_word_8(&mut self, _address: u64, _data: u8) -> Result<(), Error> {
        Err(Error::NotImplemented("8-bit write via EmbeddedICE"))
    }

    fn write_64(&mut self, address: u64, data: &[u64]) -> Result<(), Error> {
        for (i, d) in data.iter().enumerate() {
            self.write_word_64(address + (i as u64) * 8, *d)?;
        }
        Ok(())
    }

    fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), Error> {
        for (i, d) in data.iter().enumerate() {
            self.write_word_32(address + (i as u64) * 4, *d)?;
        }
        Ok(())
    }

    fn write_16(&mut self, _address: u64, _data: &[u16]) -> Result<(), Error> {
        Err(Error::NotImplemented("16-bit write slice via EmbeddedICE"))
    }

    fn write_8(&mut self, _address: u64, _data: &[u8]) -> Result<(), Error> {
        Err(Error::NotImplemented("8-bit write slice via EmbeddedICE"))
    }

    fn supports_8bit_transfers(&self) -> Result<bool, Error> {
        Ok(false)
    }

    fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// CoreInterface
// ---------------------------------------------------------------------------

impl CoreInterface for Armv5te<'_> {
    fn wait_for_core_halted(&mut self, timeout: Duration) -> Result<(), Error> {
        let start = Instant::now();
        loop {
            if self.core_halted()? {
                return Ok(());
            }
            if start.elapsed() >= timeout {
                return Err(Error::Probe(DebugProbeError::Timeout));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn core_halted(&mut self) -> Result<bool, Error> {
        self.ice.is_halted().map_err(Error::Probe)
    }

    fn status(&mut self) -> Result<CoreStatus, Error> {
        let halted = self.ice.is_halted().map_err(Error::Probe)?;
        let status = if halted {
            CoreStatus::Halted(HaltReason::Request)
        } else {
            CoreStatus::Running
        };
        self.state.current_state = status;
        Ok(status)
    }

    fn halt(&mut self, timeout: Duration) -> Result<CoreInformation, Error> {
        self.ice.halt(timeout).map_err(Error::Probe)?;
        let pc = self.read_core_reg(PC.id)?;
        self.state.current_state = CoreStatus::Halted(HaltReason::Request);
        Ok(CoreInformation {
            pc: pc.try_into().expect("PC fits in u64"),
        })
    }

    fn run(&mut self) -> Result<(), Error> {
        self.ice.resume().map_err(Error::Probe)?;
        self.state.current_state = CoreStatus::Running;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), Error> {
        // ARM926EJ-S: set CPURST in SYS_AHBIPRST, bit auto-clears.
        // We write through the memory interface, but without knowing the
        // system reset register address we can only provide a no-op here.
        // Vendor-specific sequences (NUC980) override this.
        Err(Error::NotImplemented(
            "reset: use a vendor-specific debug sequence",
        ))
    }

    fn reset_and_halt(&mut self, _timeout: Duration) -> Result<CoreInformation, Error> {
        Err(Error::NotImplemented(
            "reset_and_halt: use a vendor-specific debug sequence",
        ))
    }

    fn step(&mut self) -> Result<CoreInformation, Error> {
        self.ice.step().map_err(Error::Probe)?;
        let pc = self.read_core_reg(PC.id)?;
        Ok(CoreInformation {
            pc: pc.try_into().expect("PC fits in u64"),
        })
    }

    fn read_core_reg(&mut self, address: RegisterId) -> Result<RegisterValue, Error> {
        let n = Self::reg_id_to_num(address)?;
        let val = if n == 16 {
            // CPSR
            self.ice.read_cpsr().map_err(Error::Probe)?
        } else {
            self.ice.read_core_register(n).map_err(Error::Probe)?
        };
        Ok(RegisterValue::U32(val))
    }

    fn write_core_reg(&mut self, address: RegisterId, value: RegisterValue) -> Result<(), Error> {
        let n = Self::reg_id_to_num(address)?;
        let val: u32 = value.try_into().map_err(|_| {
            Error::Probe(DebugProbeError::Other(
                "ARMv5TEJ register value must fit in 32 bits".to_string(),
            ))
        })?;

        if n == 16 {
            // CPSR — inject MSR CPSR_cxsf, R0 via pipeline.
            self.ice.write_cpsr(val).map_err(Error::Probe)?;
            Ok(())
        } else {
            self.ice.write_core_register(n, val).map_err(Error::Probe)
        }
    }

    fn available_breakpoint_units(&mut self) -> Result<u32, Error> {
        Ok(NUM_HW_BREAKPOINTS as u32)
    }

    fn hw_breakpoints(&mut self) -> Result<Vec<Option<u64>>, Error> {
        // We don't cache breakpoints, report as not set.
        Ok(vec![None; NUM_HW_BREAKPOINTS])
    }

    fn enable_breakpoints(&mut self, state: bool) -> Result<(), Error> {
        self.state.hw_breakpoints_enabled = state;
        Ok(())
    }

    fn set_hw_breakpoint(&mut self, unit_index: usize, addr: u64) -> Result<(), Error> {
        self.ice
            .set_breakpoint(unit_index, addr as u32)
            .map_err(Error::Probe)
    }

    fn clear_hw_breakpoint(&mut self, unit_index: usize) -> Result<(), Error> {
        self.ice.clear_breakpoint(unit_index).map_err(Error::Probe)
    }

    fn registers(&self) -> &'static CoreRegisters {
        &ARMV5TE_CORE_REGISTERS
    }

    fn program_counter(&self) -> &'static CoreRegister {
        &PC
    }

    fn frame_pointer(&self) -> &'static CoreRegister {
        &FP
    }

    fn stack_pointer(&self) -> &'static CoreRegister {
        &SP
    }

    fn return_address(&self) -> &'static CoreRegister {
        &RA
    }

    fn hw_breakpoints_enabled(&self) -> bool {
        self.state.hw_breakpoints_enabled
    }

    fn architecture(&self) -> Architecture {
        Architecture::Arm
    }

    fn core_type(&self) -> CoreType {
        CoreType::Armv5te
    }

    fn instruction_set(&mut self) -> Result<InstructionSet, Error> {
        if self.is_thumb()? {
            // ARM926EJ-S supports Thumb but not Thumb-2.
            // The minimum instruction size is 2 bytes for Thumb.
            Ok(InstructionSet::Thumb2)
        } else {
            Ok(InstructionSet::A32)
        }
    }

    fn endianness(&mut self) -> Result<Endian, Error> {
        // NUC980 is always little-endian.
        Ok(Endian::Little)
    }

    fn fpu_support(&mut self) -> Result<bool, Error> {
        // ARM926EJ-S has no FPU (it has Jazelle for Java but no VFP).
        Ok(false)
    }

    fn floating_point_register_count(&mut self) -> Result<usize, Error> {
        Ok(0)
    }

    fn reset_catch_set(&mut self) -> Result<(), Error> {
        // TODO: expose reset catch (EmbeddedICE vector catch, REG_VEC_CATCH bit 0)
        // properly through EmbeddedIce.
        Ok(())
    }

    fn reset_catch_clear(&mut self) -> Result<(), Error> {
        Ok(())
    }

    fn debug_core_stop(&mut self) -> Result<(), Error> {
        // Nothing special to do.
        Ok(())
    }
}
