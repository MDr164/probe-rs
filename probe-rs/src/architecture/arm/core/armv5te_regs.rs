//! Register definitions for ARMv5TEJ (ARM926EJ-S).
//!
//! The GDB register layout for ARM is:
//!   r0–r15, CPSR   (17 registers)
//!
//! Register IDs 0–15 map directly to ARM general-purpose registers;
//! register ID 25 (0b1_1001) is used for CPSR to match the ARM/AArch32
//! convention used elsewhere in this codebase.
//!
//! Frame pointer: ARM convention uses R11 as FP (AAPCS).
//! Stack pointer: R13 (SP).
//! Link register: R14 (LR).
//! Program counter: R15 (PC).

use std::sync::LazyLock;

use crate::{
    CoreRegister, CoreRegisters, RegisterId,
    core::{RegisterDataType, RegisterRole, UnwindRule},
};

// ---------------------------------------------------------------------------
// Named register constants
// ---------------------------------------------------------------------------

/// Program counter.
pub const PC: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("R15"), RegisterRole::ProgramCounter],
    id: RegisterId(15),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::SpecialRule,
};

/// Frame pointer (R11 per AAPCS).
pub const FP: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("R11"), RegisterRole::FramePointer],
    id: RegisterId(11),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Preserve,
};

/// Stack pointer (R13).
pub const SP: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("R13"), RegisterRole::StackPointer],
    id: RegisterId(13),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Preserve,
};

/// Link register / return address (R14).
pub const RA: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("R14"), RegisterRole::ReturnAddress],
    id: RegisterId(14),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::SpecialRule,
};

/// Current Program Status Register.
///
/// ID 25 (0b1_1001) is the standard ARM GDB protocol register number for CPSR.
// Exported for potential use by callers; not currently referenced within this crate.
#[allow(dead_code)]
pub const CPSR: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("CPSR"), RegisterRole::ProcessorStatus],
    id: RegisterId(25),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};

// ---------------------------------------------------------------------------
// Full register set
// ---------------------------------------------------------------------------

/// All ARMv5TEJ core registers (r0–r15, CPSR).
pub static ARMV5TE_CORE_REGISTERS: LazyLock<CoreRegisters> =
    LazyLock::new(|| CoreRegisters::new(ARMV5TE_REGS_SET.iter().collect()));

static ARMV5TE_REGS_SET: &[CoreRegister] = &[
    CoreRegister {
        roles: &[
            RegisterRole::Core("R0"),
            RegisterRole::Argument("a1"),
            RegisterRole::Return("r1"),
        ],
        id: RegisterId(0),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[
            RegisterRole::Core("R1"),
            RegisterRole::Argument("a2"),
            RegisterRole::Return("r2"),
        ],
        id: RegisterId(1),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R2"), RegisterRole::Argument("a3")],
        id: RegisterId(2),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R3"), RegisterRole::Argument("a4")],
        id: RegisterId(3),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R4"), RegisterRole::Other("v1")],
        id: RegisterId(4),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R5"), RegisterRole::Other("v2")],
        id: RegisterId(5),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R6"), RegisterRole::Other("v3")],
        id: RegisterId(6),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R7"), RegisterRole::Other("v4")],
        id: RegisterId(7),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R8"), RegisterRole::Other("v5")],
        id: RegisterId(8),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R9"), RegisterRole::Other("v6")],
        id: RegisterId(9),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R10"), RegisterRole::Other("v7")],
        id: RegisterId(10),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R11"), RegisterRole::FramePointer],
        id: RegisterId(11),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R12"), RegisterRole::Other("ip")],
        id: RegisterId(12),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R13"), RegisterRole::StackPointer],
        id: RegisterId(13),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Preserve,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R14"), RegisterRole::ReturnAddress],
        id: RegisterId(14),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::SpecialRule,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("R15"), RegisterRole::ProgramCounter],
        id: RegisterId(15),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::SpecialRule,
    },
    CoreRegister {
        roles: &[RegisterRole::Core("CPSR"), RegisterRole::ProcessorStatus],
        id: RegisterId(25),
        data_type: RegisterDataType::UnsignedInteger(32),
        unwind_rule: UnwindRule::Clear,
    },
];
