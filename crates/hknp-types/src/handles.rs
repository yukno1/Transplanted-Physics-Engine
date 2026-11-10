//! Opaque handles used by the physics runtime.
//!
//! Ports of the `HK_DECLARE_HANDLE` declarations in
//! `original/Physics/Source/Physics/Physics/hknpTypes.h` and the ID headers under
//! `Dynamics/{Material,Motion,Body,Constraint}/`.
//!
//! Every handle stores an unsigned value and has an invalid sentinel, exactly like
//! the C++ typedefs: `hknpMaterialId` is `u16` with `0xffff` invalid,
//! `hknpShapeKey` is `u32` with `0xffffffff` invalid, etc. Handles are explicit:
//! `Handle::new(raw)` is the only constructor, and `value()` asserts validity in
//! debug builds (`hknp*Id::valueUnchecked()` is available as `raw()`).

macro_rules! define_handle {
    (
        $(#[$meta:meta])*
        $name:ident, $raw:ty, $invalid:expr
    ) => {
        $(#[$meta])*
        #[repr(transparent)]
        #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name($raw);

        impl $name {
            /// The invalid sentinel (`INVALID` in the C++ handle).
            pub const INVALID: Self = Self($invalid);

            /// Construct from a raw value (explicit, like the C++ constructor).
            #[inline]
            pub const fn new(value: $raw) -> Self {
                Self(value)
            }

            /// The invalid handle.
            #[inline]
            pub const fn invalid() -> Self {
                Self::INVALID
            }

            /// The raw value, asserting validity in debug builds.
            #[inline]
            pub const fn value(self) -> $raw {
                debug_assert!(self.is_valid(), "handle is invalid");
                self.0
            }

            /// The raw value without any validity check (`valueUnchecked`).
            #[inline]
            pub const fn raw(self) -> $raw {
                self.0
            }

            /// `isValid`: the raw value is not the invalid sentinel.
            #[inline]
            pub const fn is_valid(self) -> bool {
                self.0 != $invalid
            }

            /// `setInvalid`.
            #[inline]
            pub const fn set_invalid(&mut self) {
                self.0 = $invalid;
            }
        }

        impl Default for $name {
            /// The invalid handle (the C++ default constructor leaves storage
            /// uninitialized; choosing the sentinel is the safe Rust equivalent).
            #[inline]
            fn default() -> Self {
                Self::INVALID
            }
        }

        impl core::fmt::Debug for $name {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                if self.is_valid() {
                    write!(f, concat!(stringify!($name), "({})"), self.0)
                } else {
                    write!(f, concat!(stringify!($name), "(invalid)"))
                }
            }
        }
    };
}

define_handle! {
    /// `hknpMotionId` — motion identifier (`hkUint32`, invalid `HK_INT32_MAX`).
    MotionId, u32, 0x7FFF_FFFF
}

impl MotionId {
    /// `hknpMotionId::PRESET_STATIC` — the motion shared by static bodies.
    pub const PRESET_STATIC: Self = Self::new(0);
}

define_handle! {
    /// `hknpMaterialId` — material identifier (`hkUint16`, invalid `0xffff`).
    MaterialId, u16, 0xFFFF
}

impl MaterialId {
    /// `hknpMaterialId::PRESET_DEFAULT` (friction 0.5/0.5, restitution 0).
    pub const PRESET_DEFAULT: Self = Self::new(0);
    /// `hknpMaterialId::PRESET_BROAD_PHASE_TRIGGER`.
    pub const PRESET_BROAD_PHASE_TRIGGER: Self = Self::new(1);
    /// `hknpMaterialId::PRESET_NARROW_PHASE_TRIGGER`.
    pub const PRESET_NARROW_PHASE_TRIGGER: Self = Self::new(2);
    /// `hknpMaterialId::PRESET_CONTACT_SOLVER_TRIGGER`.
    pub const PRESET_CONTACT_SOLVER_TRIGGER: Self = Self::new(3);
}

define_handle! {
    /// `hknpMaterialPaletteEntryId` — palette entry (`hkUint16`, invalid `0xffff`).
    MaterialPaletteEntryId, u16, 0xFFFF
}

define_handle! {
    /// `hknpMotionPropertiesId` — motion property template identifier
    /// (`hkUint16`, invalid `0xffff`).
    MotionPropertiesId, u16, 0xFFFF
}

define_handle! {
    /// `hknpBodyQualityId` — body quality preset (`hkUint8`, invalid `0xff`).
    BodyQualityId, u8, 0xFF
}

impl BodyQualityId {
    /// `hknpBodyQualityId::PRESET_STATIC` (default behaviour for static bodies).
    pub const PRESET_STATIC: Self = Self::new(0);
    /// `hknpBodyQualityId::PRESET_DEBRIS`.
    pub const PRESET_DEBRIS: Self = Self::new(1);
    /// `hknpBodyQualityId::PRESET_DEBRIS_TRIANGLE_WELDED`.
    pub const PRESET_DEBRIS_TRIANGLE_WELDED: Self = Self::new(2);
}

define_handle! {
    /// `hknpShapeKey` — identifier of a leaf of a composite shape
    /// (`hkUint32`, invalid `0xffffffff`).
    ShapeKey, u32, 0xFFFF_FFFF
}

define_handle! {
    /// `hknpSolverId` — solver identifier (`hkUint32`, invalid `HK_INT32_MAX`).
    SolverId, u32, 0x7FFF_FFFF
}

define_handle! {
    /// `hknpImmediateConstraintId` — immediate constraint identifier
    /// (`hkUint16`, invalid `0xffff`).
    ImmediateConstraintId, u16, 0xFFFF
}

define_handle! {
    /// `hknpConstraintSolverId` — constraint solver slot (`hkUint8`, invalid `0xf`).
    ConstraintSolverId, u8, 0x0F
}

define_handle! {
    /// `hknpConnectivityGraph::EdgeId`.
    ConnectivityEdgeId, u32, 0xFFFF_FFFF
}

define_handle! {
    /// `hknpConnectivityGraph::NodeId`.
    ConnectivityNodeId, u16, 0xFFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_sentinels_match_the_sdk() {
        assert_eq!(MotionId::INVALID.raw(), 0x7FFF_FFFF);
        assert_eq!(MaterialId::INVALID.raw(), 0xFFFF);
        assert_eq!(ShapeKey::INVALID.raw(), 0xFFFF_FFFF);
        assert_eq!(SolverId::INVALID.raw(), 0x7FFF_FFFF);
        assert_eq!(ImmediateConstraintId::INVALID.raw(), 0xFFFF);
        assert_eq!(BodyQualityId::INVALID.raw(), 0xFF);
        assert_eq!(ConstraintSolverId::INVALID.raw(), 0x0F);
        assert_eq!(ConnectivityNodeId::INVALID.raw(), 0xFFFF);
    }

    #[test]
    fn validity_and_defaults() {
        assert!(!MaterialId::invalid().is_valid());
        assert!(!ShapeKey::default().is_valid());
        assert!(MaterialId::new(0).is_valid());
        assert!(MaterialId::PRESET_DEFAULT.is_valid());

        let mut key = ShapeKey::new(7);
        assert_eq!(key.raw(), 7);
        assert_eq!(key.value(), 7);
        key.set_invalid();
        assert!(!key.is_valid());
    }

    #[test]
    fn presets() {
        assert_eq!(MotionId::PRESET_STATIC.raw(), 0);
        assert_eq!(MaterialId::PRESET_DEFAULT.raw(), 0);
        assert_eq!(MaterialId::PRESET_CONTACT_SOLVER_TRIGGER.raw(), 3);
        assert_eq!(BodyQualityId::PRESET_DEBRIS_TRIANGLE_WELDED.raw(), 2);
    }

    #[test]
    fn handles_are_hashable_and_ordered() {
        use std::collections::HashSet;
        let mut set = HashSet::new();
        assert!(set.insert(ShapeKey::new(1)));
        assert!(!set.insert(ShapeKey::new(1)));
        assert!(ShapeKey::new(1) < ShapeKey::new(2));
        assert_eq!(core::mem::size_of::<MaterialId>(), 2);
        assert_eq!(core::mem::size_of::<ShapeKey>(), 4);
    }
}
