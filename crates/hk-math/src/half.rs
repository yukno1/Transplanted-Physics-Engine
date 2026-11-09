//! 16-bit floating point (`hkFloat16` / `hkHalf16`).
//!
//! Port of `hkFloat16::setReal<ROUND>` and `hkFloat16::getFloat32` from
//! `original/Common/Source/Common/Base/Types/hkBaseTypes.cpp` (lines 31–127).
//! The bit manipulation is copied verbatim, including the round-to-nearest branch
//! used by the default assignment (`setReal<true>`).
//!
//! ```
//! use hk_math::Half;
//! let h = Half::from_f32(1.0);
//! assert_eq!(h.to_f32(), 1.0);
//! assert_eq!(h, Half::ONE);
//! ```

/// Half-precision float stored as its raw 16-bit encoding.
///
/// Equality is bitwise (matching `hkHalf16::operator==`), so `+0.0` and `-0.0`
/// compare unequal, and NaN payloads are preserved.
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct Half(u16);

impl Half {
    /// `+0.0`.
    pub const ZERO: Self = Self(0x0000);
    /// `1.0` (`0x3C00`).
    pub const ONE: Self = Self(0x3C00);
    /// Largest finite half (`65504.0`, `0x7BFF`).
    pub const MAX: Self = Self(0x7BFF);
    /// Smallest positive subnormal (`2^-24`).
    pub const MIN_POSITIVE_SUBNORMAL: Self = Self(0x0001);
    /// `+∞`.
    pub const INFINITY: Self = Self(0x7C00);
    /// Canonical NaN.
    pub const NAN: Self = Self(0x7FFF);

    /// Raw 16-bit encoding.
    #[inline]
    pub const fn to_bits(self) -> u16 {
        self.0
    }

    /// Construct from the raw 16-bit encoding (any bits are valid).
    #[inline]
    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    /// `hkFloat16::setReal<true>` — `f32` to half, round to nearest even.
    pub fn from_f32(value: f32) -> Self {
        let bits = value.to_bits();
        let sign = (bits & 0x8000_0000) >> 16;
        let mut int_value = bits & 0x7FFF_FFFF;

        let result: u32 = if int_value > 0x477F_E000 {
            // Too large for a half; NaN or infinity.
            if (int_value & 0x7F80_0000) == 0x7F80_0000 && (int_value & 0x007F_FFFF) != 0 {
                0x7FFF // NaN
            } else {
                0x7C00 // INF
            }
        } else {
            if int_value < 0x3880_0000 {
                // Subnormal output: shift the implicit bit down.
                //
                // The C++ code computes `113 - (intValue >> 23)` and shifts an
                // `hkUint32` by that amount. For zero the count exceeds 31 (UB in
                // C++, masked to 5 bits on x86); `wrapping_shr` reproduces the
                // x86 behavior without panicking in debug builds.
                let shift = 113 - (int_value >> 23);
                int_value = (0x0080_0000 | (int_value & 0x007F_FFFF)).wrapping_shr(shift);
            } else {
                // Rebias exponent from f32 (127) to half (15), with wrapping like
                // the unsigned arithmetic in the SDK.
                int_value = int_value.wrapping_add(0xC800_0000);
            }
            // ROUND = true: add 0x0FFF + the lowest kept bit (round half to even).
            ((int_value + 0x0FFF + ((int_value >> 13) & 1)) >> 13) & 0x7FFF
        };

        Self((result | sign) as u16)
    }

    /// `hkFloat16::setReal<false>` — truncating conversion (no rounding).
    pub fn from_f32_truncate(value: f32) -> Self {
        const ROUND: bool = false;
        let bits = value.to_bits();
        let sign = (bits & 0x8000_0000) >> 16;
        let mut int_value = bits & 0x7FFF_FFFF;

        let result: u32 = if int_value > 0x477F_E000 {
            if (int_value & 0x7F80_0000) == 0x7F80_0000 && (int_value & 0x007F_FFFF) != 0 {
                0x7FFF
            } else {
                0x7C00
            }
        } else {
            if int_value < 0x3880_0000 {
                let shift = 113 - (int_value >> 23);
                int_value = (0x0080_0000 | (int_value & 0x007F_FFFF)).wrapping_shr(shift);
            } else {
                int_value = int_value.wrapping_add(0xC800_0000);
            }
            if ROUND {
                ((int_value + 0x0FFF + ((int_value >> 13) & 1)) >> 13) & 0x7FFF
            } else {
                int_value >> 13
            }
        };

        Self((result | sign) as u16)
    }

    /// `hkFloat16::getFloat32` — half to `f32`.
    pub fn to_f32(self) -> f32 {
        let mut mantissa = u32::from(self.0 & 0x03FF);
        let mut exponent = u32::from(self.0 & 0x7C00);

        if exponent == 0x7C00 {
            // INF / NaN; keep mantissa bits so NaN payloads survive.
            exponent = 0x8F;
        } else if exponent != 0 {
            // Normalized.
            exponent = u32::from((self.0 >> 10) & 0x1F);
        } else if mantissa != 0 {
            // Denormalized: normalize the value.
            exponent = 1;
            loop {
                exponent = exponent.wrapping_sub(1);
                mantissa <<= 1;
                if (mantissa & 0x0400) != 0 {
                    break;
                }
            }
            mantissa &= 0x03FF;
        } else {
            // Zero: exponent arithmetic cancels out below.
            exponent = 0u32.wrapping_sub(112);
        }

        let result = (u32::from(self.0 & 0x8000) << 16)
            | (exponent.wrapping_add(112) << 23)
            | (mantissa << 13);
        f32::from_bits(result)
    }

    /// Whether the value is finite (not INF/NaN).
    #[inline]
    pub const fn is_finite(self) -> bool {
        (self.0 & 0x7C00) != 0x7C00
    }

    /// Whether the value is NaN.
    #[inline]
    pub const fn is_nan(self) -> bool {
        (self.0 & 0x7C00) == 0x7C00 && (self.0 & 0x03FF) != 0
    }

    /// Whether the value is ±∞.
    #[inline]
    pub const fn is_infinite(self) -> bool {
        (self.0 & 0x7FFF) == 0x7C00
    }

    /// Sign bit.
    #[inline]
    pub const fn is_negative(self) -> bool {
        (self.0 & 0x8000) != 0
    }
}

impl From<f32> for Half {
    #[inline]
    fn from(value: f32) -> Self {
        Self::from_f32(value)
    }
}

impl From<Half> for f32 {
    #[inline]
    fn from(value: Half) -> Self {
        value.to_f32()
    }
}

impl core::fmt::Debug for Half {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Half")
            .field("bits", &format_args!("0x{:04X}", self.0))
            .field("value", &self.to_f32())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_values_round_trip() {
        for value in [0.0f32, 1.0, -1.0, 0.5, -0.5, 2.0, 10.0, 100.0, -3.0] {
            let half = Half::from_f32(value);
            assert_eq!(
                half.to_f32(),
                value,
                "value {value} must round-trip exactly"
            );
        }
    }

    #[test]
    fn zero_signs_are_distinct_bits() {
        assert_eq!(Half::from_f32(0.0).to_bits(), 0x0000);
        assert_eq!(Half::from_f32(-0.0).to_bits(), 0x8000);
        assert_ne!(Half::from_f32(0.0), Half::from_f32(-0.0));
        assert_eq!(Half::from_f32(0.0).to_f32(), 0.0);
    }

    #[test]
    fn rounding_and_subnormals() {
        // The smallest subnormal is 2^-24; half of it rounds to zero (ties to even).
        let smallest = Half::MIN_POSITIVE_SUBNORMAL.to_f32();
        assert_eq!(smallest, 2.0f32.powi(-24));
        assert_eq!(Half::from_f32(smallest / 2.0).to_bits(), 0x0000);

        // 1 + 2^-10 is exactly representable in half precision.
        assert_eq!(Half::from_f32(1.0 + 2.0f32.powi(-10)).to_bits(), 0x3C01);
        // 1 + 2^-11 is exactly halfway to the next half value: ties to even (0x3C00).
        assert_eq!(Half::from_f32(1.0 + 2.0f32.powi(-11)).to_bits(), 0x3C00);
    }

    #[test]
    fn saturation_infinity_and_nan() {
        assert_eq!(
            Half::from_f32(1.0e30).to_bits(),
            0x7C00,
            "overflow saturates to +INF"
        );
        assert_eq!(Half::from_f32(-1.0e30).to_bits(), 0xFC00);
        assert_eq!(Half::from_f32(f32::INFINITY).to_bits(), 0x7C00);
        assert!(Half::from_f32(f32::NAN).is_nan());
        assert_eq!(Half::MAX.to_f32(), 65504.0);
    }

    #[test]
    fn denormal_decoding() {
        // 0x0001 = 2^-24 ≈ 5.96e-8
        assert!(Half::MIN_POSITIVE_SUBNORMAL.to_f32() > 0.0);
        assert!(Half::MIN_POSITIVE_SUBNORMAL.to_f32() < Half::from_f32(1e-7).to_f32());
        // Largest denormal: (1023 / 1024) * 2^-14
        let largest_denormal = Half::from_bits(0x03FF).to_f32();
        assert!((largest_denormal - 6.097_556e-5).abs() < 1e-10);
    }

    #[test]
    fn truncate_differs_from_round() {
        // 1 + 2^-11 rounds up but truncates down (mantissa grab discards bits).
        let value = 1.0f32 + 2.0f32.powi(-11) + 2.0f32.powi(-13);
        let rounded = Half::from_f32(value).to_bits();
        let truncated = Half::from_f32_truncate(value).to_bits();
        assert_eq!(rounded, 0x3C01);
        assert_eq!(truncated, 0x3C00);
    }

    #[test]
    fn size_is_two_bytes() {
        assert_eq!(core::mem::size_of::<Half>(), 2);
    }
}
