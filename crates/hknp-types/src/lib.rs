//! `hknp-types` — hknp core types.
//!
//! Rust port of the identifier and enum layer under
//! `original/Physics/Source/Physics/Physics/` (migration Phase 5, README §4):
//! `hknpTypes.h` plus the `Dynamics/{Body,Motion,Material,Constraint}` ID
//! headers.
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`BodyId`] | `hknpBodyId` | 64-bit ID; MSB selects internal index vs user ID |
//! | [`BodyIndex`] | `hknpBodyIndex` | 24-bit index + 8-bit serial |
//! | [`MotionId`], [`MaterialId`], [`ShapeKey`], [`SolverId`], … | `HK_DECLARE_HANDLE` ids | explicit constructors, invalid sentinels match the SDK |
//! | [`MotionType`] | `hknpMotionType` | STATIC / KEYFRAMED / DYNAMIC |
//! | [`ShapeType`] | `hknpShapeType` | full enum incl. composites and user types |
//! | [`CollisionDispatchType`] | `hknpCollisionDispatchType` | narrow-phase dispatch selection |
//! | [`CollisionFlags`], [`ShapeFlags`], [`WorldAddedState`] | `hknpCollisionFlags`, `hknpShape::FlagsEnum`, `hknpBody::WorldAddedState` | bit sets |
//!
//! # Not ported in this phase
//!
//! `hknpWorldReader/Writer`, `hknpStepInput`, `hknpDeactivationState`,
//! `hknpCollisionFilter`, `hknpQueryFilterData`, `hknpCommandDispatchType`,
//! `hknpHalf` and the body/material structs themselves. Those arrive with
//! `hknp-body` / `hknp-world` (Phase 6) and the query/filter work (Phase 7).
//!
//! # Dependency note
//!
//! The crate deliberately has no `hk-types` dependency: every handle here has a
//! different raw width (u8/u16/u32/u64), while `hk_types::Handle` is a u32 newtype.
//! The ID semantics follow the C++ typedefs exactly instead.

pub mod body;
pub mod enums;
pub mod flags;
pub mod handles;

pub use body::{BodyId, BodyIndex};
pub use enums::{CollisionDispatchType, MotionType, ShapeType};
pub use flags::{CollisionFlags, ShapeFlags, WorldAddedState};
pub use handles::{
    BodyQualityId, ConnectivityEdgeId, ConnectivityNodeId, ConstraintId, ConstraintIndex,
    ConstraintSolverId, ImmediateConstraintId, MaterialId, MaterialPaletteEntryId, MotionId,
    MotionPropertiesId, ShapeKey, SolverId,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handles_and_enums_compose() {
        let body = BodyId::from_body_index(BodyIndex::new(3, 1));
        assert!(body.is_valid());
        let motion = MotionId::new(1);
        assert_eq!(motion.value(), 1);
        assert_eq!(MotionType::Keyframed.as_raw(), 1);
        assert!(CollisionFlags::IS_KEYFRAMED.bits() > 0);
    }
}
