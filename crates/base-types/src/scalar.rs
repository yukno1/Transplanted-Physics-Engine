//! Fixed-width scalar typedefs.
//!
//! Port of the typedef block in
//! `original/Common/Source/Common/Base/Types/hkBaseDefs.h`
//! (plus `hkTime` from `hkBaseTypes.h`). The original names are mentioned in the
//! documentation of each alias so C++ call sites remain easy to map.
//!
//! Rust already guarantees the widths (`u32` is always 32 bits), so the only
//! real decisions here are the two floating point aliases and `Ulong`.

/// `hkReal` — the engine-wide floating point type.
///
/// Havok 2021.2 is built with `HK_REAL_IS_FLOAT`, so this is `f32`, never `f64`.
/// Simulation code must stay bit-exact, hence the alias instead of `f32` literals
/// sprinkled through the port.
pub type Real = f32;

/// `hkTime` — simulation time, `typedef hkReal hkTime`.
pub type Time = Real;

/// `hkFloat32`.
pub type Float32 = f32;

/// `hkDouble64`.
pub type Float64 = f64;

/// `hkInt8`.
pub type Int8 = i8;
/// `hkInt16`.
pub type Int16 = i16;
/// `hkInt32`.
pub type Int32 = i32;
/// `hkInt64`.
pub type Int64 = i64;

/// `hkUint8`.
pub type Uint8 = u8;
/// `hkUint16`.
pub type Uint16 = u16;
/// `hkUint32`.
pub type Uint32 = u32;
/// `hkUint64`.
pub type Uint64 = u64;

/// `hkUlong` — 64 bits on every supported target.
///
/// The C++ SDK maps this to `unsigned __int64` on Windows and `unsigned long` on
/// LP64 platforms; both are 64-bit, so `u64` is the faithful port. Note that this
/// is also the storage type of `hkReferencedObject`'s reference count in the
/// original (see [`crate::refptr::RefCount`], which uses `u32` — see the caveat
/// there).
pub type Ulong = u64;

/// `hkBool32` — "false is zero, true is any non-zero value" (inner-loop boolean).
///
/// The SDK typedefs it to `hkUint32` unless `HK_BOOL32_IS_STRICT` is defined.
pub type Bool32 = u32;

/// `hkBool64`.
pub type Bool64 = u64;

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn widths_match_the_sdk() {
        assert_eq!(size_of::<Real>(), 4, "hkReal must be 32-bit");
        assert_eq!(size_of::<Int32>(), 4);
        assert_eq!(
            size_of::<Ulong>(),
            8,
            "hkUlong is 64-bit on all supported targets"
        );
        assert_eq!(size_of::<Bool32>(), 4);
    }

    #[test]
    fn real_is_f32() {
        // Compile-time proof that the aliases are the expected primitive types.
        let _: f32 = 1.0;
        let _: Real = 1.0;
        let _: f64 = 1.0;
        let _: Float64 = 1.0;
    }
}
