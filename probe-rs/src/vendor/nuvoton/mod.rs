//! Nuvoton SoC support.
//!
//! Supports the NUC980 series (ARM926EJ-S, EmbeddedICE debug).

use crate::{config::DebugSequence, vendor::Vendor};
use probe_rs_target::Chip;
use sequences::nuc980::Nuc980;

pub mod sequences;

/// Nuvoton
#[derive(docsplay::Display)]
pub struct Nuvoton;

impl Vendor for Nuvoton {
    fn try_create_debug_sequence(&self, chip: &Chip) -> Option<DebugSequence> {
        if chip.name.starts_with("NUC980") {
            Some(DebugSequence::Arm(Nuc980::create()))
        } else {
            None
        }
    }
}
