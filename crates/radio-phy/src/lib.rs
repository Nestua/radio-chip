//! Golden reference model of the PHY.
//!
//! This crate is the bit-level specification of the hardware: every RTL block in `hw/` must
//! reproduce its output bit for bit (gate G3, see `docs/testing.md`). Models use fixed-point or
//! integer arithmetic wherever the hardware does, so that "bit-exact" has a meaning.
//!
//! Bits are carried as `u8` holding 0 or 1, in transmission order.

pub mod channel;
pub mod convolutional;
pub mod fixed;
pub mod interleaver;
pub mod ldpc;
pub mod ldpc_decoder;
mod ldpc_tables;
pub mod mapper;
pub mod modulation;
pub mod ofdm;
pub mod ppdu;
pub mod rx_decision;
pub mod rx_front;
pub mod rx_front_fixed;
pub mod rx_ofdm;
pub mod rx_ofdm_fixed;
pub mod scrambler;
pub mod viterbi;
