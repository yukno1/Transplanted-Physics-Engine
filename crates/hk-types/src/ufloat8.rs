//! `hkUFloat8` — an 8-bit quantized non-negative float.
//!
//! Port of `hkUFloat8` (`hkUFloat8.h` / `hkUFloat8.cpp`). The encoding uses the
//! same 256-entry exponential table (`s_encodedFloats`) and the same 7-step
//! binary search, so encoded bytes produced by the C++ SDK decode to bit-identical
//! `f32` values here. The documented range of the original is
//! `[0.01, 1_000_000.0]` with an average relative error of about 7%.
//!
//! ```
//! use hk_types::UFloat8;
//! let q = UFloat8::from(1234.5f32);
//! let back: f32 = q.into();
//! assert!((back - 1234.5).abs() / 1234.5 < 0.07);
//! assert_eq!(UFloat8::from(0.0f32).raw(), 0);
//! assert_eq!(UFloat8::from(1_000_000.0f32).raw(), 255);
//! ```

/// Number of table entries (`hkUFloat8::MAX_VALUE`).
pub const MAX_VALUE: usize = 256;

/// `hkUFloat8_eps`, the smallest non-zero value the encoding can represent
/// (`decode(s_encodedFloats[1])`).
pub const EPS: f32 = 0.01;

/// `hkUFloat8_maxValue`, the largest value the encoding is designed for.
pub const MAX_REPRESENTABLE: f32 = 1_000_000.0;

/// `hkUFloat8::ENCODE_MANTISSA_SHIFT` = `FLOAT_MANTISSA_BITS - ENCODED_MANTISSA_BITS`.
const ENCODE_MANTISSA_SHIFT: u32 = 23 - 11;

/// `hkUFloat8::DECODE_EXPONENT_BIAS` = `ENCODED_EXPONENT_BIAS << (23 - shift)`.
const DECODE_EXPONENT_BIAS: u32 = 119 << 11;

/// The encoding table, byte-for-byte identical to
/// `hkUFloat8::s_encodedFloats` in `hkUFloat8.cpp`.
#[rustfmt::skip]
const ENCODED_FLOATS: [u16; MAX_VALUE] = [
    0x0000, 0x0A3D, 0x123D, 0x175C, 0x1A3D, 0x1CCC, 0x1F5C, 0x20F5,
    0x2193, 0x223B, 0x22EF, 0x23AF, 0x247D, 0x2559, 0x2643, 0x273E,
    0x2825, 0x28B4, 0x294D, 0x29F1, 0x2AA0, 0x2B5A, 0x2C22, 0x2CF7,
    0x2DDB, 0x2ECF, 0x2FD4, 0x3075, 0x3109, 0x31A8, 0x3252, 0x3308,
    0x33CA, 0x3499, 0x3577, 0x3663, 0x3760, 0x3837, 0x38C8, 0x3962,
    0x3A07, 0x3AB7, 0x3B74, 0x3C3D, 0x3D15, 0x3DFB, 0x3EF0, 0x3FF7,
    0x4000, 0x411E, 0x41BE, 0x4269, 0x4321, 0x43E4, 0x44B5, 0x4595,
    0x4684, 0x4783, 0x484A, 0x48DB, 0x4977, 0x4A1E, 0x4AD0, 0x4B8E,
    0x4C59, 0x4D32, 0x4E1A, 0x4F12, 0x500D, 0x509B, 0x5132, 0x51D4,
    0x5281, 0x533A, 0x53FF, 0x54D2, 0x55B3, 0x56A4, 0x57A6, 0x585C,
    0x58EF, 0x598C, 0x5A34, 0x5AE8, 0x5BA8, 0x5C75, 0x5D50, 0x5E3A,
    0x5F34, 0x601F, 0x60AE, 0x6147, 0x61EA, 0x6298, 0x6353, 0x641A,
    0x64EF, 0x65D2, 0x66C5, 0x67C9, 0x686F, 0x6903, 0x69A2, 0x6A4B,
    0x6B00, 0x6BC2, 0x6C91, 0x6D6E, 0x6E5A, 0x6F56, 0x7032, 0x70C2,
    0x715C, 0x7200, 0x72B0, 0x736C, 0x7435, 0x750C, 0x75F1, 0x76E6,
    0x77ED, 0x7882, 0x7918, 0x79B8, 0x7A62, 0x7B19, 0x7BDC, 0x7CAD,
    0x7D8C, 0x7E7A, 0x7F79, 0x8044, 0x80D5, 0x8171, 0x8217, 0x82C8,
    0x8386, 0x8451, 0x8529, 0x8611, 0x8708, 0x8808, 0x8895, 0x892C,
    0x89CD, 0x8A7A, 0x8B32, 0x8BF7, 0x8CC9, 0x8DAA, 0x8E9B, 0x8F9B,
    0x9057, 0x90E9, 0x9186, 0x922E, 0x92E1, 0x93A0, 0x946C, 0x9547,
    0x9630, 0x972A, 0x981A, 0x98A8, 0x9941, 0x99E3, 0x9A91, 0x9B4B,
    0x9C12, 0x9CE6, 0x9DC9, 0x9EBB, 0x9FBF, 0xA069, 0xA0FD, 0xA19C,
    0xA244, 0xA2F9, 0xA3BA, 0xA488, 0xA565, 0xA650, 0xA74C, 0xA82C,
    0xA8BC, 0xA956, 0xA9FA, 0xAAA9, 0xAB65, 0xAC2D, 0xAD03, 0xADE8,
    0xAEDD, 0xAFE2, 0xB07C, 0xB112, 0xB1B1, 0xB25C, 0xB312, 0xB3D4,
    0xB4A4, 0xB583, 0xB670, 0xB76E, 0xB83F, 0xB8D0, 0xB96B, 0xBA10,
    0xBAC1, 0xBB7E, 0xBC48, 0xBD20, 0xBE07, 0xBEFE, 0xC003, 0xC08F,
    0xC126, 0xC1C7, 0xC273, 0xC32B, 0xC3EF, 0xC4C1, 0xC5A1, 0xC691,
    0xC791, 0xC851, 0xC8E3, 0xC980, 0xCA27, 0xCAD9, 0xCB98, 0xCC64,
    0xCD3E, 0xCE27, 0xCF20, 0xD015, 0xD0A3, 0xD13B, 0xD1DD, 0xD28A,
    0xD344, 0xD40A, 0xD4DE, 0xD5C0, 0xD6B2, 0xD7B4, 0xD864, 0xD8F7,
    0xD995, 0xDA3E, 0xDAF2, 0xDBB2, 0xDC80, 0xDD5C, 0xDE47, 0xDF42,
];

/// An unsigned float quantized into one byte.
///
/// Bitwise equality: two values are equal iff their encoded bytes are equal
/// (matching `hkUFloat8::operator==`).
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct UFloat8 {
    value: u8,
}

impl UFloat8 {
    /// Zero (`hkUFloat8::setZero` value).
    pub const ZERO: Self = Self { value: 0 };

    /// The maximum encoded value (`hkUFloat8::setMax` value, byte 255).
    pub const MAX: Self = Self { value: 255 };

    /// The raw encoded byte.
    #[inline]
    pub const fn raw(self) -> u8 {
        self.value
    }

    /// Build from a raw encoded byte (all 256 values are valid).
    #[inline]
    pub const fn from_raw(value: u8) -> Self {
        Self { value }
    }

    /// `hkUFloat8::isZero`.
    #[inline]
    pub const fn is_zero(self) -> bool {
        self.value == 0
    }

    /// `hkUFloat8::setZero`.
    #[inline]
    pub const fn set_zero(&mut self) {
        self.value = 0;
    }

    /// `hkUFloat8::isMax`.
    #[inline]
    pub const fn is_max(self) -> bool {
        self.value == 255
    }

    /// `hkUFloat8::setMax`.
    #[inline]
    pub const fn set_max(&mut self) {
        self.value = 255;
    }

    /// `hkUFloat8::setArithmeticMean`: `this = (a + b) / 2` on the encoded bytes.
    ///
    /// The original computes the mean in the encoded domain (`sqrt(a*b)` in the
    /// decoded domain), not in float space.
    #[inline]
    pub fn set_arithmetic_mean(&mut self, a: Self, b: Self) {
        self.value = ((u16::from(a.value) + u16::from(b.value)) >> 1) as u8;
    }

    /// `hkUFloat8::decodeFloat`: expand a table entry to `f32`.
    ///
    /// The bit trick is the C++ `union` reinterpretation expressed as
    /// `f32::from_bits`; entry `0` maps to exactly `0.0`.
    #[inline]
    pub fn decode_float(encoded: u16) -> f32 {
        if encoded == 0 {
            return 0.0;
        }
        let bits = (u32::from(encoded) + DECODE_EXPONENT_BIAS) << ENCODE_MANTISSA_SHIFT;
        f32::from_bits(bits)
    }

    /// `hkUFloat8::operator=(const float&)`: nearest table entry by the original
    /// 7-step binary search.
    pub fn from_f32(value: f32) -> Self {
        let min_value = Self::decode_float(ENCODED_FLOATS[1]);
        if value < min_value {
            return Self::ZERO;
        }

        let mut minf: usize = 0;
        let mut maxf: usize = MAX_VALUE;
        let mut midf: usize = MAX_VALUE >> 1;

        for _ in 0..7 {
            if value > Self::decode_float(ENCODED_FLOATS[midf]) {
                minf = midf;
            } else {
                maxf = midf;
            }
            midf = (minf + maxf) >> 1;
        }

        if value > Self::decode_float(ENCODED_FLOATS[midf]) && midf < MAX_VALUE - 1 {
            midf += 1;
        }

        Self { value: midf as u8 }
    }
}

impl From<f32> for UFloat8 {
    #[inline]
    fn from(value: f32) -> Self {
        Self::from_f32(value)
    }
}

impl From<f64> for UFloat8 {
    /// `hkUFloat8::operator=(const double&)` — goes through `f32` first.
    #[inline]
    fn from(value: f64) -> Self {
        Self::from_f32(value as f32)
    }
}

impl From<UFloat8> for f32 {
    #[inline]
    fn from(value: UFloat8) -> Self {
        UFloat8::decode_float(ENCODED_FLOATS[usize::from(value.value)])
    }
}

impl core::fmt::Debug for UFloat8 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let decoded: f32 = (*self).into();
        f.debug_struct("UFloat8")
            .field("raw", &self.value)
            .field("value", &decoded)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode(raw: u8) -> f32 {
        UFloat8::from_raw(raw).into()
    }

    #[test]
    fn table_has_the_original_shape() {
        assert_eq!(ENCODED_FLOATS.len(), 256);
        assert_eq!(ENCODED_FLOATS[0], 0);
        assert_eq!(ENCODED_FLOATS[255], 0xDF42);
        assert_eq!(MAX_VALUE, 256);
    }

    #[test]
    fn zero_decodes_to_zero() {
        assert_eq!(decode(0), 0.0);
        assert!(UFloat8::ZERO.is_zero());
        assert_eq!(f32::from(UFloat8::ZERO), 0.0);
    }

    #[test]
    fn smallest_non_zero_entry_is_epsilon() {
        // hkUFloat8_eps is documented as 0.01f; entry 1 must decode near it.
        let smallest = decode(1);
        assert!(smallest > 0.0);
        assert!(
            (smallest - EPS).abs() <= 0.001 * EPS,
            "entry 1 decoded to {smallest}, expected ~{EPS}"
        );
    }

    #[test]
    fn largest_entry_is_about_one_million() {
        let largest = decode(255);
        assert!(
            (largest - MAX_REPRESENTABLE).abs() / MAX_REPRESENTABLE < 0.01,
            "entry 255 decoded to {largest}"
        );
    }

    #[test]
    fn decoded_values_are_monotonic() {
        let mut previous = decode(0);
        for raw in 1..=255u8 {
            let current = decode(raw);
            assert!(
                current > previous,
                "decoded table must increase strictly at raw {raw}"
            );
            previous = current;
        }
    }

    #[test]
    fn encoding_zero_and_tiny_values() {
        assert_eq!(UFloat8::from(0.0f32).raw(), 0);
        assert_eq!(UFloat8::from(0.001f32).raw(), 0);
        assert_eq!(UFloat8::from(-1.0f32).raw(), 0, "negatives clamp to zero");
    }

    #[test]
    fn encoding_saturates_at_max() {
        assert_eq!(UFloat8::from(1_000_000.0f32).raw(), 255);
        assert_eq!(UFloat8::from(10_000_000.0f32).raw(), 255);
        assert!(UFloat8::from(1_000_000.0f32).is_max());
    }

    #[test]
    fn low_end_quantization_is_absolute() {
        // The first eight table entries step by `hkUFloat8_eps` (0.01) instead of
        // exponentially, so in `[0.01, 0.07]` the error is absolute, not relative.
        // 0.0105 falls between entries 1 (~0.01) and 2 (~0.02); like the C++
        // encoder's final `midf++` step, it rounds up to entry 2.
        let q = UFloat8::from(0.0105f32);
        assert_eq!(q.raw(), 2, "must match the C++ binary-search result");
        let back: f32 = q.into();
        assert!((back - 0.0105).abs() <= 0.011);

        let q = UFloat8::from(0.06f32);
        let back: f32 = q.into();
        assert!((back - 0.06).abs() <= 0.011);
    }

    #[test]
    fn round_trip_relative_error() {
        // C++ documentation: average error 7%; the encoder can land one table
        // step away from the nearest entry, so allow up to 15% for spot checks.
        for value in [1.0f32, 3.7, 42.0, 500.0, 1234.5, 12_345.0, 250_000.0] {
            let quantized = UFloat8::from(value);
            let back: f32 = quantized.into();
            let relative_error = ((back - value) / value).abs();
            assert!(
                relative_error <= 0.15,
                "value {value} decoded to {back} (raw {}), error {relative_error}",
                quantized.raw()
            );
        }
    }

    #[test]
    fn arithmetic_mean_is_in_encoded_space() {
        let a = UFloat8::from_raw(10);
        let b = UFloat8::from_raw(20);
        let mut mean = UFloat8::default();
        mean.set_arithmetic_mean(a, b);
        assert_eq!(mean.raw(), 15);

        let mut mean2 = UFloat8::default();
        mean2.set_arithmetic_mean(UFloat8::from_raw(0), UFloat8::from_raw(1));
        assert_eq!(mean2.raw(), 0, "integer truncation, like the SDK");
        assert_eq!(mean2, UFloat8::ZERO);
    }

    #[test]
    fn set_zero_and_set_max() {
        let mut q = UFloat8::from_raw(42);
        q.set_zero();
        assert!(q.is_zero());
        q.set_max();
        assert!(q.is_max());
    }

    #[test]
    fn double_goes_through_f32() {
        let from_double = UFloat8::from(0.1f64);
        let from_float = UFloat8::from(0.1f32);
        assert_eq!(from_double, from_float);
    }
}
