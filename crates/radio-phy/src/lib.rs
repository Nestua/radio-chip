//! Golden reference model of the PHY.
//!
//! This crate is the bit-level specification of the hardware: every RTL block in `hw/` must
//! reproduce its output bit for bit (gate G3, see `docs/testing.md`). Models use fixed-point or
//! integer arithmetic wherever the hardware does, so that "bit-exact" has a meaning.
//!
//! Bits are carried as `u8` holding 0 or 1, in transmission order.

pub mod convolutional;
pub mod fixed;
pub mod interleaver;
pub mod mapper;
pub mod modulation;
pub mod ofdm;
pub mod ppdu;
pub mod scrambler;
