//! Modulation orders of the 802.11a/g OFDM PHY and the per-symbol bit counts they imply.

/// Modulation order expressed as coded bits per subcarrier (N_BPSC).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modulation {
    /// BPSK, N_BPSC = 1.
    Bpsk,
    /// QPSK, N_BPSC = 2.
    Qpsk,
    /// 16-QAM, N_BPSC = 4.
    Qam16,
    /// 64-QAM, N_BPSC = 6.
    Qam64,
}

impl Modulation {
    /// Coded bits per subcarrier, N_BPSC.
    pub fn bits_per_subcarrier(self) -> usize {
        match self {
            Modulation::Bpsk => 1,
            Modulation::Qpsk => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
        }
    }

    /// Coded bits per OFDM symbol, N_CBPS = 48 data subcarriers × N_BPSC.
    pub fn bits_per_symbol(self) -> usize {
        DATA_SUBCARRIERS * self.bits_per_subcarrier()
    }

    /// The modulation with `n_bpsc` coded bits per subcarrier, if 802.11a/g defines one.
    pub fn from_bits_per_subcarrier(n_bpsc: usize) -> Option<Self> {
        match n_bpsc {
            1 => Some(Modulation::Bpsk),
            2 => Some(Modulation::Qpsk),
            4 => Some(Modulation::Qam16),
            6 => Some(Modulation::Qam64),
            _ => None,
        }
    }
}

/// Data subcarriers per 20 MHz OFDM symbol. IEEE 802.11-2020 §17.3.5.10.
pub const DATA_SUBCARRIERS: usize = 48;
