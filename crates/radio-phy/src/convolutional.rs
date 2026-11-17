//! 802.11 OFDM convolutional encoder and puncturing (IEEE 802.11-2020 §17.3.5.6).
//!
//! Rate-1/2 mother code, constraint length K = 7, generators g0 = 133₈ (output A) and
//! g1 = 171₈ (output B). Octal generators are read MSB first, the MSB tapping the current input
//! (Figure 17-8). Output order is A then B for every input bit. Higher rates steal bits from the
//! serialised stream A0 B0 A1 B1 … with the patterns of Figure 17-9.

/// Generator for output A, 133₈, MSB = current input. IEEE 802.11-2020 §17.3.5.6.
pub const G0: u8 = 0o133;
/// Generator for output B, 171₈, MSB = current input. IEEE 802.11-2020 §17.3.5.6.
pub const G1: u8 = 0o171;

/// Coding rates of the 802.11 OFDM PHY.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CodeRate {
    /// Rate 1/2: nothing stolen.
    R1_2,
    /// Rate 2/3: from A0 B0 A1 B1 transmit A0 B0 A1.
    R2_3,
    /// Rate 3/4: from A0 B0 A1 B1 A2 B2 transmit A0 B0 A1 B2.
    R3_4,
}

impl CodeRate {
    /// Puncturing pattern over the serialised rate-1/2 stream: 1 = transmitted, 0 = stolen.
    pub fn pattern(self) -> &'static [u8] {
        // IEEE 802.11-2020 §17.3.5.6, Figure 17-9.
        match self {
            CodeRate::R1_2 => &[1, 1],
            CodeRate::R2_3 => &[1, 1, 1, 0],
            CodeRate::R3_4 => &[1, 1, 1, 0, 0, 1],
        }
    }
}

/// The rate-1/2 stream does not end on a whole puncturing period.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartialPeriod {
    /// Length of the rate-1/2 stream that was offered.
    pub len: usize,
    /// Pattern length it must be a multiple of.
    pub period: usize,
}

impl std::fmt::Display for PartialPeriod {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} coded bits is not a whole number of {}-bit puncturing periods",
            self.len, self.period
        )
    }
}

impl std::error::Error for PartialPeriod {}

/// K = 7 rate-1/2 convolutional encoder. Starts in the all-zero state.
#[derive(Clone, Debug, Default)]
pub struct ConvolutionalEncoder {
    /// The six delay cells: bit 5 holds the previous input (D1), bit 0 the oldest (D6). This lines
    /// up with the generators once the current input is placed in bit 6.
    state: u8,
}

impl ConvolutionalEncoder {
    /// Encoder in the all-zero state, as at the start of the SERVICE field.
    pub fn new() -> Self {
        Self::default()
    }

    /// Encodes one input bit and returns `(a, b)`.
    pub fn encode_bit(&mut self, bit: u8) -> (u8, u8) {
        let window = ((bit & 1) << 6) | self.state;
        let parity = |g: u8| ((window & g).count_ones() & 1) as u8;
        self.state = window >> 1;
        (parity(G0), parity(G1))
    }

    /// Encodes `bits` at rate 1/2; the output is A0 B0 A1 B1 …
    pub fn encode(&mut self, bits: &[u8]) -> Vec<u8> {
        bits.iter()
            .flat_map(|&bit| {
                let (a, b) = self.encode_bit(bit);
                [a, b]
            })
            .collect()
    }
}

/// Steals bits from a rate-1/2 stream according to `rate`.
pub fn puncture(coded: &[u8], rate: CodeRate) -> Result<Vec<u8>, PartialPeriod> {
    let pattern = rate.pattern();
    if !coded.len().is_multiple_of(pattern.len()) {
        return Err(PartialPeriod {
            len: coded.len(),
            period: pattern.len(),
        });
    }
    Ok(coded
        .iter()
        .zip(pattern.iter().cycle())
        .filter(|(_, &keep)| keep == 1)
        .map(|(&bit, _)| bit)
        .collect())
}

/// Encodes `bits` from the all-zero state and punctures to `rate`.
pub fn encode(bits: &[u8], rate: CodeRate) -> Result<Vec<u8>, PartialPeriod> {
    puncture(&ConvolutionalEncoder::new().encode(bits), rate)
}
