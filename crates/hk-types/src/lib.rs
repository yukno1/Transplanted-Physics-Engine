//! `hk-types` — Havok base types.
//!
//! Rust port of the scalar / handle / reference-counting / signal layer of
//! `original/Common/Source/Common/Base/Types/` (migration Phase 1, see
//! `README.md` §4).
//!
//! # Mapping to the original SDK
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`Real`], [`Int32`]… | `hkReal`, `hkInt32`… in `hkBaseDefs.h` | `hkReal` is `f32` (never `f64`) |
//! | [`Handle`] | `hkHandle<TYPE, INVALID_VALUE, DISCRIMINANT>` / `HK_DECLARE_HANDLE` | tag type plays the role of the C++ discriminant |
//! | [`RefCount`], [`RefCounted`], [`RefPtr`] | `hkReferencedObject`, `hkRefPtr` | intrusive atomic refcount; see the module docs |
//! | [`Pair`] | `hkPair` | plain two-field struct |
//! | [`UFloat8`] | `hkUFloat8` | same 256-entry exponential table and binary-search encoder |
//! | [`Signal`] | `hkSignal0` … `hkSignal6` | observer / slot-list replacement |
//! | [`bitops`] | `hkSetBits`, `hkClearBits`, `hkNextPowerOf2`, `hkBitmask`, `hkBitCast`, `hkAlign` | Rust naming |
//! | `hkBool` | — | maps to `bool` |
//! | `hkOptional<T>` | — | maps to `Option<T>` |
//! | `HK_ASSERT` / `HK_ASSERT_NO_MSG` | — | maps to [`hk_assert!`] (`debug_assert!` semantics) |
//!
//! Deliberately **not** ported in this phase (see README §4 Phase 1):
//! reflection macros (`HK_DECLARE_CLASS`, `HK_REFLECT_AS_*`), the `hkHandleMap`
//! helpers, `hkVarArgs`, `hkPtrAndInt`, and the full `hkOptional` container —
//! Rust's `Option` covers the behaviour we need.
//!
//! # Determinism
//!
//! Havok relies on bit-exact `f32` results. Everything here keeps operations in
//! plain IEEE-754 `f32`/`i32` order; no fused multiply-add, reassociation or
//! fast-math style transformations are used. [`bitops::bit_cast`] is a
//! bit-pattern reinterpretation, not a numeric conversion.
//!
//! # Module layout
//!
//! * [`scalar`] — fixed-width scalar typedefs.
//! * [`bitops`] — bit twiddling helpers.
//! * [`pair`] — `hkPair`.
//! * [`handle`] — typed opaque handles and the [`declare_handle!`] macro.
//! * [`refptr`] — intrusive reference counting.
//! * [`ufloat8`] — 8-bit quantized float.
//! * [`signal`] — observer pattern.
//! * [`result`] — `hkResult`-style error codes.

pub mod bitops;
pub mod handle;
pub mod pair;
pub mod refptr;
pub mod result;
pub mod scalar;
pub mod signal;
pub mod ufloat8;

pub use bitops::{
    BitCastable, Bits, CastUnsigned, PowerOfTwo, align, bit_cast, bitmask, bitmask64, clear_bits,
    next_power_of_two, previous_power_of_two, set_bits,
};
pub use handle::Handle;
pub use pair::Pair;
pub use refptr::{RefCount, RefCounted, RefPtr};
pub use result::HkResult;
pub use scalar::{
    Bool32, Bool64, Float32, Float64, Int8, Int16, Int32, Int64, Real, Time, Uint8, Uint16, Uint32,
    Uint64, Ulong,
};
pub use signal::{ConnectionId, Signal};
pub use ufloat8::UFloat8;

/// `debug_assert!` with Havok-style naming.
///
/// Port of `HK_ASSERT` / `HK_ASSERT_NO_MSG` (README §4 Phase 1: "HK_ASSERT →
/// `debug_assert!`"). Compiled out in release builds like `debug_assert!`.
///
/// # Examples
///
/// ```
/// # use hk_types::hk_assert;
/// hk_assert!(1 + 1 == 2);
/// hk_assert!(1 + 1 == 2, "arithmetic is broken");
/// ```
#[macro_export]
macro_rules! hk_assert {
    ($cond:expr $(,)?) => {
        debug_assert!($cond)
    };
    ($cond:expr, $($arg:tt)+) => {
        debug_assert!($cond, $($arg)+)
    };
}
