//! Bit manipulation helpers.
//!
//! Port of the bit helpers in
//! `original/Common/Source/Common/Base/Types/hkBaseTypes.h` (lines 44–230).
//! Function names are Rust-style (`set_bits` for `hkSetBits`), signatures keep
//! the original semantics, including the "no sign extension" conversions.

use core::mem;

/// Maps an integer type to its same-width unsigned counterpart.
///
/// Port of `hkTrait::UnsignedFor` (`hkBaseTypes.h` line 109). This is what makes
/// [`set_bits`] and [`clear_bits`] sign-extension free for signed inputs, exactly
/// like the C++ template.
pub trait Bits: Copy + 'static {
    /// Unsigned type with the same width as `Self`.
    type Unsigned: Copy
        + 'static
        + core::ops::Not<Output = Self::Unsigned>
        + core::ops::BitAnd<Self::Unsigned, Output = Self::Unsigned>
        + core::ops::BitOr<Self::Unsigned, Output = Self::Unsigned>;

    /// Reinterpret the value as its unsigned counterpart (two's complement).
    fn to_unsigned(self) -> Self::Unsigned;

    /// Convert back from the unsigned counterpart.
    fn from_unsigned(value: Self::Unsigned) -> Self;
}

macro_rules! impl_bits_unsigned {
    ($($ty:ty),* $(,)?) => {
        $(
            impl Bits for $ty {
                type Unsigned = $ty;

                #[inline]
                fn to_unsigned(self) -> Self::Unsigned {
                    self
                }

                #[inline]
                fn from_unsigned(value: Self::Unsigned) -> Self {
                    value
                }
            }
        )*
    };
}

macro_rules! impl_bits_signed {
    ($($ty:ty => $unsigned:ty),* $(,)?) => {
        $(
            impl Bits for $ty {
                type Unsigned = $unsigned;

                #[inline]
                fn to_unsigned(self) -> Self::Unsigned {
                    self as $unsigned
                }

                #[inline]
                fn from_unsigned(value: Self::Unsigned) -> Self {
                    value as $ty
                }
            }
        )*
    };
}

impl_bits_unsigned!(u8, u16, u32, u64);
impl_bits_signed!(i8 => u8, i16 => u16, i32 => u32, i64 => u64);

/// Widening / narrowing cast between unsigned integer types.
///
/// Exists so [`set_bits`] accepts a mask whose type differs in width from the
/// value, mirroring the C++ `UVTYPE(UMTYPE(mask))` truncating conversion.
pub trait CastUnsigned<Target>: Copy {
    /// Perform the `as`-style cast.
    fn cast_unsigned(self) -> Target;
}

macro_rules! impl_cast_same {
    ($($ty:ty),* $(,)?) => {
        $(
            impl CastUnsigned<$ty> for $ty {
                #[inline]
                fn cast_unsigned(self) -> $ty {
                    self
                }
            }
        )*
    };
}

macro_rules! impl_cast_cross {
    ($from:ty => $($to:ty),+ $(,)?) => {
        $(
            impl CastUnsigned<$to> for $from {
                #[inline]
                fn cast_unsigned(self) -> $to {
                    self as $to
                }
            }
        )+
    };
}

impl_cast_same!(u8, u16, u32, u64);
impl_cast_cross!(u8 => u16, u32, u64);
impl_cast_cross!(u16 => u8, u32, u64);
impl_cast_cross!(u32 => u8, u16, u64);
impl_cast_cross!(u64 => u8, u16, u32);

/// Turn off bits of a value without sign-extension issues.
///
/// Port of `hkClearBits` (`hkBaseTypes.h` line 152). The mask is first truncated
/// to the *unsigned* counterpart of `VTYPE`, then inverted, so a narrow mask does
/// not accidentally leave high bits set in a wider value.
///
/// ```
/// use hk_types::bitops::clear_bits;
/// assert_eq!(clear_bits(0xFFFF_FFFFu32, 0x0000_0003u32), 0xFFFF_FFFC);
/// assert_eq!(clear_bits(-1i32, 3u32), -4);
/// // A `u16` mask on a `u32` value is widened exactly like the C++ template.
/// assert_eq!(clear_bits(0x1234_5678u32, 0x00FFu16), 0x1234_5600);
/// ```
#[inline]
pub fn clear_bits<V: Bits, M: Bits>(value: V, mask: M) -> V
where
    M::Unsigned: CastUnsigned<V::Unsigned>,
{
    let mask: V::Unsigned = mask.to_unsigned().cast_unsigned();
    V::from_unsigned(value.to_unsigned() & !mask)
}

/// Set bits of a value without sign-extension issues.
///
/// Port of `hkSetBits` (`hkBaseTypes.h` line 161).
///
/// ```
/// use hk_types::bitops::set_bits;
/// assert_eq!(set_bits(0u32, 0xF0u32), 0xF0);
/// assert_eq!(set_bits(0i8, 0x01u8), 1);
/// ```
#[inline]
pub fn set_bits<V: Bits, M: Bits>(value: V, mask: M) -> V
where
    M::Unsigned: CastUnsigned<V::Unsigned>,
{
    let mask: V::Unsigned = mask.to_unsigned().cast_unsigned();
    V::from_unsigned(value.to_unsigned() | mask)
}

/// `(1 << width) - 1`, defined for `width == 32`.
///
/// Port of `hkBitmask` (`hkBaseTypes.h` line 205). Shifting by 32 or more is
/// undefined in C++, so the original computes both cases without an oversized
/// shift; this port keeps the same formulation.
#[inline]
pub const fn bitmask(width: u32) -> u32 {
    ((1u32 << (width & 31)).wrapping_sub(1)).wrapping_sub((width & 32) >> 5)
}

/// `(1 << width) - 1`, defined for `width == 64`.
///
/// Port of `hkBitmask64` (`hkBaseTypes.h` line 219).
#[inline]
pub const fn bitmask64(width: u32) -> u64 {
    ((1u64 << (width & 63)).wrapping_sub(1)).wrapping_sub(((width & 64) >> 6) as u64)
}

/// Power-of-two rounding in the style of `hkNextPowerOf2` / `hkPreviousPowerOf2`.
///
/// Implemented as a trait because Rust cannot add inherent methods to primitive
/// integer types. The methods are named `power_of_two_ceil`/`power_of_two_floor`
/// rather than `next_power_of_two`/`previous_power_of_two` so that the standard
/// library's inherent `u32::next_power_of_two` (which returns `1` for `0`) does
/// not silently win method resolution. Prefer the free functions
/// [`next_power_of_two`] / [`previous_power_of_two`].
///
/// The methods match the C++ bit-smearing algorithm, including the edge case
/// `0 → 0` (C++ relies on wrapping arithmetic for that input).
pub trait PowerOfTwo: Copy {
    /// Smallest power of two greater than or equal to `self`; `0` maps to `0`.
    fn power_of_two_ceil(self) -> Self;

    /// Largest power of two less than or equal to `self`; `0` maps to `0`.
    fn power_of_two_floor(self) -> Self;
}

macro_rules! impl_power_of_two {
    ($ty:ty, $($shift:expr),+ $(,)?) => {
        impl PowerOfTwo for $ty {
            #[inline]
            fn power_of_two_ceil(self) -> Self {
                // C++: in -= 1; smear; in + 1. For input 0 the subtraction wraps
                // and the result wraps back to 0, hence the explicit early return.
                if self == 0 {
                    return 0;
                }
                let mut v = self - 1;
                $(v |= v >> $shift;)+
                v.wrapping_add(1)
            }

            #[inline]
            fn power_of_two_floor(self) -> Self {
                let mut v = self;
                $(v |= v >> $shift;)+
                v - (v >> 1)
            }
        }
    };
}

impl_power_of_two!(u32, 16, 8, 4, 2, 1);
impl_power_of_two!(u64, 32, 16, 8, 4, 2, 1);

/// Free-function form of [`PowerOfTwo::power_of_two_ceil`].
///
/// Port of `hkNextPowerOf2` (`hkBaseTypes.h` line 169).
///
/// ```
/// use hk_types::bitops::next_power_of_two;
/// assert_eq!(next_power_of_two(3u32), 4);
/// assert_eq!(next_power_of_two(0u32), 0);
/// ```
#[inline]
pub fn next_power_of_two<T: PowerOfTwo>(value: T) -> T {
    value.power_of_two_ceil()
}

/// Free-function form of [`PowerOfTwo::power_of_two_floor`].
///
/// Port of `hkPreviousPowerOf2` (`hkBaseTypes.h` line 187).
#[inline]
pub fn previous_power_of_two<T: PowerOfTwo>(value: T) -> T {
    value.power_of_two_floor()
}

/// Integer arithmetic on `(unalignedVal + alignment - 1) & ~(alignment - 1)`.
///
/// Port of `hkAlign` (`hkBaseTypes.h` line 44). `alignment` must be a power of two.
///
/// ```
/// use hk_types::bitops::align;
/// assert_eq!(align(5, 4), 8);
/// assert_eq!(align(8, 4), 8);
/// ```
#[inline]
pub const fn align(unaligned: i32, alignment: i32) -> i32 {
    (unaligned + alignment - 1) & !(alignment - 1)
}

/// Types whose every bit pattern is a valid value of the type.
///
/// This is the (safe-function) contract of [`bit_cast`]: reinterpreting bits of
/// one such type as another of the same size can never create an invalid value
/// (no `bool`, `char` or enum). It is sealed to numeric primitives in practice.
///
/// # Safety
///
/// Implementors must guarantee that *every* possible bit pattern of the type is
/// a valid, fully initialized value (no padding bytes with invalid values, no
/// invalid discriminants). The provided implementations cover the primitive
/// numeric types only; do not implement this for `bool`, `char`, references or
/// enums.
pub unsafe trait BitCastable: Copy + 'static {}

macro_rules! impl_bit_castable {
    ($($ty:ty),* $(,)?) => {
        $(
            // SAFETY: every bit pattern of these primitive numeric types is valid.
            unsafe impl BitCastable for $ty {}
        )*
    };
}

impl_bit_castable!(u8, u16, u32, u64, usize, i8, i16, i32, i64, isize, f32, f64);

/// Reinterpret between same-sized values.
///
/// Port of `hkBitCast` (`hkBaseTypes.h` line 90). Both types must be
/// [`BitCastable`]; the size equality is checked (and optimized away when
/// statically known).
///
/// ```
/// use hk_types::bitops::bit_cast;
/// let bits: u32 = bit_cast(&1.0f32);
/// assert_eq!(bits, 0x3F80_0000);
/// let back: f32 = bit_cast(&bits);
/// assert_eq!(back, 1.0);
/// ```
#[inline]
pub fn bit_cast<Dst: BitCastable, Src: BitCastable>(src: &Src) -> Dst {
    assert!(
        mem::size_of::<Dst>() == mem::size_of::<Src>(),
        "bit_cast requires equal source and destination sizes"
    );
    // SAFETY: sizes are equal and both types accept every bit pattern, so the
    // copied bytes are a valid `Dst`.
    unsafe { mem::transmute_copy(src) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clear_bits_matches_cpp_semantics() {
        assert_eq!(clear_bits(0xFFFF_FFFFu32, 0x0000_0003u32), 0xFFFF_FFFC);
        assert_eq!(clear_bits(0xFFFF_FFFFu32, 0xFFu8), 0xFFFF_FF00);
        assert_eq!(clear_bits(0xF0u8, 0x0Fu8), 0xF0);
        assert_eq!(clear_bits(-1i32, 3i32), -4);
        assert_eq!(clear_bits(-1i64, 0xFFFFu16), !0xFFFFi64);
        // Clearing bits that are not set keeps the value unchanged.
        assert_eq!(clear_bits(0x1234u16, 0x0F0Fu16), 0x1030);
    }

    #[test]
    fn set_bits_matches_cpp_semantics() {
        assert_eq!(set_bits(0u32, 0xF0u32), 0xF0);
        assert_eq!(set_bits(0xFFFF_FF00u32, 0xFFu8), 0xFFFF_FFFF);
        assert_eq!(set_bits(0i8, 0x81u8), -127);
        assert_eq!(set_bits(0i64, 0x8000_0000_0000_0000u64), i64::MIN);
    }

    #[test]
    fn bitmask_handles_full_width() {
        assert_eq!(bitmask(0), 0);
        assert_eq!(bitmask(1), 1);
        assert_eq!(bitmask(8), 0xFF);
        assert_eq!(bitmask(31), 0x7FFF_FFFF);
        assert_eq!(bitmask(32), u32::MAX);
        assert_eq!(bitmask64(0), 0);
        assert_eq!(bitmask64(1), 1);
        assert_eq!(bitmask64(63), i64::MAX as u64);
        assert_eq!(bitmask64(64), u64::MAX);
    }

    #[test]
    fn power_of_two_rounding() {
        assert_eq!(next_power_of_two(0u32), 0);
        assert_eq!(next_power_of_two(1u32), 1);
        assert_eq!(next_power_of_two(2u32), 2);
        assert_eq!(next_power_of_two(3u32), 4);
        assert_eq!(next_power_of_two(1023u32), 1024);
        assert_eq!(next_power_of_two(1024u32), 1024);
        assert_eq!(next_power_of_two(1025u32), 2048);
        assert_eq!(next_power_of_two(0x4000_0000u32), 0x4000_0000);
        assert_eq!(next_power_of_two(0x4000_0001u32), 0x8000_0000);

        assert_eq!(previous_power_of_two(0u32), 0);
        assert_eq!(previous_power_of_two(1u32), 1);
        assert_eq!(previous_power_of_two(3u32), 2);
        assert_eq!(previous_power_of_two(5u32), 4);
        assert_eq!(previous_power_of_two(1024u32), 1024);
        assert_eq!(previous_power_of_two(1025u32), 1024);

        assert_eq!(next_power_of_two(0u64), 0);
        assert_eq!(next_power_of_two(1u64 << 40), 1u64 << 40);
        assert_eq!(next_power_of_two((1u64 << 40) + 1), 1u64 << 41);
        assert_eq!(previous_power_of_two(1u64 << 63), 1u64 << 63);
    }

    #[test]
    fn align_rounds_up() {
        assert_eq!(align(0, 16), 0);
        assert_eq!(align(1, 16), 16);
        assert_eq!(align(16, 16), 16);
        assert_eq!(align(17, 16), 32);
        assert_eq!(align(5, 4), 8);
        assert_eq!(align(1234, 256), 1280);
    }

    #[test]
    fn bit_cast_round_trips() {
        let one: f32 = 1.0;
        let bits: u32 = bit_cast(&one);
        assert_eq!(bits, 0x3F80_0000);
        let back: f32 = bit_cast(&bits);
        assert_eq!(back, 1.0);

        let negative_zero: u32 = bit_cast(&-0.0f32);
        assert_eq!(negative_zero, 0x8000_0000, "sign bit is preserved");
    }
}
