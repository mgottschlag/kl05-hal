//! Hardware abstraction layer for NXP KL05 microcontrollers
#![no_std]

extern crate embedded_hal as hal;
pub extern crate kl05_pac as pac;

pub mod gpio;

#[derive(Debug, PartialEq, Eq)]
pub enum NoError {}
