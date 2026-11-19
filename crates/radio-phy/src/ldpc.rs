//! 802.11 HT LDPC codes: the twelve codes, their encoder and the PPDU encoding process
//! (IEEE 802.11-2020 §19.3.11.7; the matrix prototypes are Annex F, in `ldpc_tables`).
//!
//! A code has n = 24·Z bits (Z = 27, 54, 81) and a parity-check matrix H of (1 − R)·24 block
//! rows and 24 block columns. Each block is the zero matrix or the Z × Z identity with its
//! columns cyclically shifted to the right by the prototype's entry (§19.3.11.7.4): bit c of
//! a block with shift s is checked by row r of the block when c = (r + s) mod Z.
//!
//! A codeword is c = (i_0 … i_{k−1}, p_0 … p_{n−k−1}) with H·cᵀ = 0 (§19.3.11.7.3): the
//! information bits first, systematic. The parity part of every prototype has one column with
//! three blocks (shifts 1, 0, 1: top, somewhere in the middle, bottom) and then a double
//! diagonal of unshifted identities, which is what makes the encoder a back-substitution;
//! `LdpcCode::new` checks that structure instead of assuming it.

use crate::ldpc_tables::PROTOTYPES;

/// Codeword block lengths of the HT LDPC codes (§19.3.11.7.2, Table 19-15).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LdpcLength {
    /// 648 bits, Z = 27.
    N648,
    /// 1296 bits, Z = 54.
    N1296,
    /// 1944 bits, Z = 81.
    N1944,
}

impl LdpcLength {
    /// Codeword length n in bits.
    pub fn bits(self) -> usize {
        match self {
            LdpcLength::N648 => 648,
            LdpcLength::N1296 => 1296,
            LdpcLength::N1944 => 1944,
        }
    }
}

/// Coding rates of the HT LDPC codes (§19.3.11.7.2, Table 19-15).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LdpcRate {
    /// Rate 1/2.
    R1_2,
    /// Rate 2/3.
    R2_3,
    /// Rate 3/4.
    R3_4,
    /// Rate 5/6.
    R5_6,
}

impl LdpcRate {
    /// The rate as numerator and denominator.
    pub fn fraction(self) -> (usize, usize) {
        match self {
            LdpcRate::R1_2 => (1, 2),
            LdpcRate::R2_3 => (2, 3),
            LdpcRate::R3_4 => (3, 4),
            LdpcRate::R5_6 => (5, 6),
        }
    }
}

/// What the LDPC functions refuse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LdpcError {
    /// The information block is not k bits long.
    InfoLength {
        /// Bits offered.
        got: usize,
        /// Bits the code takes (k).
        want: usize,
    },
    /// A value that is not 0 or 1 among the bits.
    NotABit,
    /// The PPDU parameters describe no PPDU (no payload, no bits per symbol, an STBC factor
    /// other than 1 or 2).
    Parameters,
    /// The payload is not the `n_pld` bits the plan was made for.
    PayloadLength {
        /// Bits offered.
        got: usize,
        /// Bits the plan was made for.
        want: usize,
    },
}

impl std::fmt::Display for LdpcError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LdpcError::InfoLength { got, want } => {
                write!(f, "{got} information bits offered, the code takes {want}")
            }
            LdpcError::NotABit => write!(f, "a bit must be 0 or 1"),
            LdpcError::Parameters => write!(f, "the parameters describe no LDPC PPDU"),
            LdpcError::PayloadLength { got, want } => {
                write!(f, "{got} payload bits offered, the plan is for {want}")
            }
        }
    }
}

impl std::error::Error for LdpcError {}

/// One of the twelve HT LDPC codes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LdpcCode {
    length: LdpcLength,
    rate: LdpcRate,
    z: usize,
    rows: &'static [[i8; 24]],
    /// Row of the middle block of the first parity column (the one with shift 0).
    middle: usize,
}

impl LdpcCode {
    /// The code of this length and rate. `None` only if the prototype table does not have the
    /// structure the encoder relies on (it has: `g1_ldpc` checks all twelve).
    pub fn new(length: LdpcLength, rate: LdpcRate) -> Option<Self> {
        let rows = PROTOTYPES[length as usize][rate as usize];
        let z = length.bits() / 24;
        let m = rows.len();
        let kb = 24 - m;
        // Parity part: column kb holds shift 1 at the top and bottom and shift 0 once between;
        // columns kb + 1 + j hold unshifted identities in rows j and j + 1, nothing else.
        let first: Vec<(usize, i8)> = (0..m)
            .filter(|&r| rows[r][kb] >= 0)
            .map(|r| (r, rows[r][kb]))
            .collect();
        let middle = match first[..] {
            [(0, 1), (mid, 0), (last, 1)] if last == m - 1 => mid,
            _ => return None,
        };
        for j in 0..m - 1 {
            for (r, row) in rows.iter().enumerate() {
                let want = if r == j || r == j + 1 { 0 } else { -1 };
                if row[kb + 1 + j] != want {
                    return None;
                }
            }
        }
        Some(Self {
            length,
            rate,
            z,
            rows,
            middle,
        })
    }

    /// Codeword length n.
    pub fn n(&self) -> usize {
        self.length.bits()
    }

    /// Information bits per codeword, k = n·R.
    pub fn k(&self) -> usize {
        (24 - self.rows.len()) * self.z
    }

    /// Subblock size Z.
    pub fn z(&self) -> usize {
        self.z
    }

    /// Block length of the code.
    pub fn length(&self) -> LdpcLength {
        self.length
    }

    /// Rate of the code.
    pub fn rate(&self) -> LdpcRate {
        self.rate
    }

    /// The matrix prototype (Annex F): block rows of 24 entries, each the cyclic shift of a
    /// Z × Z identity or −1 for the zero block.
    pub fn prototype(&self) -> &'static [[i8; 24]] {
        self.rows
    }

    /// The parity-check matrix as its rows: for each of the n − k checks, the positions of
    /// the codeword bits it adds, ascending.
    pub fn checks(&self) -> Vec<Vec<usize>> {
        let z = self.z;
        let mut out = Vec::with_capacity(self.rows.len() * z);
        for row in self.rows {
            for r in 0..z {
                let mut check: Vec<usize> = row
                    .iter()
                    .enumerate()
                    .filter(|(_, &s)| s >= 0)
                    .map(|(col, &s)| col * z + (r + s as usize) % z)
                    .collect();
                check.sort_unstable();
                out.push(check);
            }
        }
        out
    }

    /// The n − k parity bits of `info` (k bits).
    fn parity(&self, info: &[u8]) -> Vec<u8> {
        let (z, m) = (self.z, self.rows.len());
        let kb = 24 - m;
        // λ_r = the information part of block row r: Σ_j P^{s(r,j)}·u_j.
        let mut lambda = vec![0u8; m * z];
        for (r, row) in self.rows.iter().enumerate() {
            for (j, &s) in row[..kb].iter().enumerate() {
                if s < 0 {
                    continue;
                }
                for i in 0..z {
                    lambda[r * z + i] ^= info[j * z + (i + s as usize) % z];
                }
            }
        }
        // Adding all block rows cancels the double diagonal and leaves (P¹ + P⁰ + P¹)·p_0 =
        // p_0: the first parity block is the sum of the λ.
        let mut p = vec![0u8; m * z];
        for r in 0..m {
            for i in 0..z {
                p[i] ^= lambda[r * z + i];
            }
        }
        // Row r reads λ_r + (first column's block)·p_0 + p_r + p_{r+1} = 0, with no p_r term
        // in row 0: solve downwards.
        for r in 0..m - 1 {
            for i in 0..z {
                let first = if r == 0 {
                    p[(i + 1) % z]
                } else if r == self.middle {
                    p[i]
                } else {
                    0
                };
                let prev = if r == 0 { 0 } else { p[r * z + i] };
                p[(r + 1) * z + i] = lambda[r * z + i] ^ first ^ prev;
            }
        }
        p
    }

    /// Encode k information bits into the n-bit codeword (information bits, then parity).
    pub fn encode(&self, info: &[u8]) -> Result<Vec<u8>, LdpcError> {
        if info.len() != self.k() {
            return Err(LdpcError::InfoLength {
                got: info.len(),
                want: self.k(),
            });
        }
        if info.iter().any(|&b| b > 1) {
            return Err(LdpcError::NotABit);
        }
        let mut c = info.to_vec();
        c.extend(self.parity(info));
        Ok(c)
    }

    /// Whether `word` (n bits) satisfies every parity check.
    pub fn is_codeword(&self, word: &[u8]) -> bool {
        if word.len() != self.n() || word.iter().any(|&b| b > 1) {
            return false;
        }
        let z = self.z;
        self.rows.iter().all(|row| {
            (0..z).all(|r| {
                row.iter()
                    .enumerate()
                    .filter(|(_, &s)| s >= 0)
                    .fold(0, |acc, (col, &s)| {
                        acc ^ word[col * z + (r + s as usize) % z]
                    })
                    == 0
            })
        })
    }
}

/// The derived parameters of an LDPC PPDU (§19.3.11.7.5, steps a to e).
///
/// The fields are public to be read. A plan is consistent when it comes from
/// [`LdpcPlan::new`]; one put together by hand is refused by the encoding functions if its
/// numbers do not fit its code (`LdpcError::Parameters`), never trusted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LdpcPlan {
    /// Payload bits: SERVICE and PSDU (N_pld).
    pub n_pld: usize,
    /// Coding rate.
    pub rate: LdpcRate,
    /// Codeword length (L_LDPC).
    pub length: LdpcLength,
    /// Number of codewords (N_CW).
    pub n_cw: usize,
    /// Bits available in the PPDU's OFDM symbols, after the possible increment (N_avbits).
    pub n_avbits: usize,
    /// Shortening bits (N_shrt).
    pub n_shrt: usize,
    /// Punctured bits (N_punc).
    pub n_punc: usize,
    /// Repeated bits (N_rep).
    pub n_rep: usize,
    /// OFDM symbols of the Data field (N_SYM).
    pub n_sym: usize,
}

impl LdpcPlan {
    /// The plan for `n_pld` payload bits (16 + 8·LENGTH) at `n_cbps` coded bits per OFDM
    /// symbol and `m_stbc` = 2 with STBC, 1 without. §19.3.11.7.5 a) to e), in integers.
    pub fn new(
        n_pld: usize,
        n_cbps: usize,
        rate: LdpcRate,
        m_stbc: usize,
    ) -> Result<Self, LdpcError> {
        let (num, den) = rate.fraction();
        // The bounds first: they keep every product below inside 32 bits.
        if n_pld == 0
            || n_pld > 1 << 24
            || n_cbps == 0
            || n_cbps > 1 << 20
            || !(1..=2).contains(&m_stbc)
            || !(n_cbps * num).is_multiple_of(den)
        {
            return Err(LdpcError::Parameters);
        }
        // a) (19-36): N_avbits = N_CBPS·m·⌈N_pld / (N_CBPS·R·m)⌉.
        let per_step = n_cbps * m_stbc;
        let mut n_avbits = per_step * (n_pld * den).div_ceil(per_step * num);
        // b) Table 19-16. "N_avbits ≥ N_pld + c·(1 − R)" is kept in integers by × den.
        let room = |c: usize| n_avbits * den >= n_pld * den + c * (den - num);
        let (n_cw, length) = if n_avbits <= 648 {
            (
                1,
                if room(912) {
                    LdpcLength::N1296
                } else {
                    LdpcLength::N648
                },
            )
        } else if n_avbits <= 1296 {
            (
                1,
                if room(1464) {
                    LdpcLength::N1944
                } else {
                    LdpcLength::N1296
                },
            )
        } else if n_avbits <= 1944 {
            (1, LdpcLength::N1944)
        } else if n_avbits <= 2592 {
            (
                2,
                if room(2916) {
                    LdpcLength::N1944
                } else {
                    LdpcLength::N1296
                },
            )
        } else {
            ((n_pld * den).div_ceil(1944 * num), LdpcLength::N1944)
        };
        let l = length.bits();
        // c) (19-37).
        let n_shrt = (n_cw * l * num / den).saturating_sub(n_pld);
        // d) (19-38) to (19-40).
        let punc = |avbits: usize| (n_cw * l).saturating_sub(avbits + n_shrt);
        let mut n_punc = punc(n_avbits);
        let parity = n_cw * l * (den - num); // N_CW·L·(1 − R), × den
        let many = 10 * n_punc * den > parity && 10 * n_shrt * (den - num) < 12 * n_punc * num;
        let too_many = 10 * n_punc * den > 3 * parity;
        if many || too_many {
            n_avbits += per_step;
            n_punc = punc(n_avbits);
        }
        // e) (19-42).
        let n_rep = n_avbits.saturating_sub(n_cw * l * (den - num) / den + n_pld);
        Ok(Self {
            n_pld,
            rate,
            length,
            n_cw,
            n_avbits,
            n_shrt,
            n_punc,
            n_rep,
            n_sym: n_avbits / n_cbps,
        })
    }

    /// `total` spread over the codewords: the first `total mod N_CW` get one more.
    pub(crate) fn share_of(&self, total: usize, cw: usize) -> usize {
        total / self.n_cw + usize::from(cw < total % self.n_cw)
    }

    /// Whether the numbers fit the code: what `new` guarantees and a hand-made plan may not.
    pub(crate) fn is_consistent(&self) -> bool {
        let (num, den) = self.rate.fraction();
        let l = self.length.bits();
        let k = l * num / den;
        self.n_cw > 0
            && self.n_cw.checked_mul(k) == self.n_pld.checked_add(self.n_shrt)
            && self.n_shrt.div_ceil(self.n_cw) < k
            && self.n_punc.div_ceil(self.n_cw) < l - k
            && self.n_rep <= 1 << 24
    }

    /// The information blocks handed to the encoder: the payload spread over the codewords
    /// with the shortening zeros at the end of each (step c). Concatenated, N_CW·k bits.
    pub fn shortened(&self, payload: &[u8]) -> Result<Vec<u8>, LdpcError> {
        if payload.len() != self.n_pld {
            return Err(LdpcError::PayloadLength {
                got: payload.len(),
                want: self.n_pld,
            });
        }
        if !self.is_consistent() {
            return Err(LdpcError::Parameters);
        }
        let (num, den) = self.rate.fraction();
        let k = self.length.bits() * num / den;
        let mut out = Vec::with_capacity(self.n_cw * k);
        let mut at = 0;
        for cw in 0..self.n_cw {
            let data = k - self.share_of(self.n_shrt, cw);
            out.extend_from_slice(&payload[at..at + data]);
            out.resize((cw + 1) * k, 0);
            at += data;
        }
        Ok(out)
    }

    /// The codewords of the PPDU before shortening bits are removed: N_CW·L bits.
    pub fn codewords(&self, payload: &[u8]) -> Result<Vec<u8>, LdpcError> {
        let code = LdpcCode::new(self.length, self.rate).ok_or(LdpcError::Parameters)?;
        let info = self.shortened(payload)?;
        let mut out = Vec::with_capacity(self.n_cw * code.n());
        for block in info.chunks(code.k()) {
            out.extend(code.encode(block)?);
        }
        Ok(out)
    }

    /// The coded bits of the Data field: per codeword the data bits, the parity bits less
    /// the punctured ones, then the repeated bits (steps c to f). N_avbits bits.
    pub fn encode(&self, payload: &[u8]) -> Result<Vec<u8>, LdpcError> {
        let (num, den) = self.rate.fraction();
        let l = self.length.bits();
        let k = l * num / den;
        let words = self.codewords(payload)?;
        let mut out = Vec::with_capacity(self.n_avbits);
        for (cw, word) in words.chunks(l).enumerate() {
            let data = k - self.share_of(self.n_shrt, cw);
            let parity = l - k - self.share_of(self.n_punc, cw);
            let start = out.len();
            out.extend_from_slice(&word[..data]);
            out.extend_from_slice(&word[k..k + parity]);
            // Repetition copies from the codeword as transmitted so far, from its first bit,
            // going round again if one pass is not enough.
            let sent = out.len() - start;
            for i in 0..self.share_of(self.n_rep, cw) {
                out.push(out[start + i % sent]);
            }
        }
        Ok(out)
    }
}
