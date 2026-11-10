//! Collision, shape and world-state flags.
//!
//! Ports of `hknpCollisionFlags` (`Collide/hknpCollisionFlags.h`), the internal
//! `hknpShape::FlagsEnum` and the `hknpBody::WorldAddedState` bits.

macro_rules! define_flags {
    ($name:ident, $raw:ty) => {
        impl $name {
            /// No flags set.
            pub const NONE: Self = Self(0);

            /// Construct from raw bits.
            #[inline]
            pub const fn from_bits(bits: $raw) -> Self {
                Self(bits)
            }

            /// Raw bit representation.
            #[inline]
            pub const fn bits(self) -> $raw {
                self.0
            }

            /// Whether no flags are set.
            #[inline]
            pub const fn is_empty(self) -> bool {
                self.0 == 0
            }

            /// Whether `other` is fully contained in this set.
            #[inline]
            pub const fn contains(self, other: Self) -> bool {
                (self.0 & other.0) == other.0
            }

            /// Add flags.
            #[inline]
            pub fn insert(&mut self, other: Self) {
                self.0 |= other.0;
            }

            /// Remove flags.
            #[inline]
            pub fn remove(&mut self, other: Self) {
                self.0 &= !other.0;
            }

            /// Set or clear flags depending on `enabled`.
            #[inline]
            pub fn set(&mut self, other: Self, enabled: bool) {
                if enabled {
                    self.insert(other);
                } else {
                    self.remove(other);
                }
            }
        }

        impl core::ops::BitOr for $name {
            type Output = Self;

            #[inline]
            fn bitor(self, rhs: Self) -> Self {
                Self(self.0 | rhs.0)
            }
        }

        impl core::ops::BitAnd for $name {
            type Output = Self;

            #[inline]
            fn bitand(self, rhs: Self) -> Self {
                Self(self.0 & rhs.0)
            }
        }

        impl core::ops::BitXor for $name {
            type Output = Self;

            #[inline]
            fn bitxor(self, rhs: Self) -> Self {
                Self(self.0 ^ rhs.0)
            }
        }

        impl core::ops::Not for $name {
            type Output = Self;

            #[inline]
            fn not(self) -> Self {
                Self(!self.0)
            }
        }

        impl core::ops::BitOrAssign for $name {
            #[inline]
            fn bitor_assign(&mut self, rhs: Self) {
                self.0 |= rhs.0;
            }
        }

        impl core::ops::BitAndAssign for $name {
            #[inline]
            fn bitand_assign(&mut self, rhs: Self) {
                self.0 &= rhs.0;
            }
        }
    };
}

/// `hknpCollisionFlags` — simulation behaviour flags stored by bodies/materials.
///
/// Only the body-related flags are ported so far; the long list of modifier
/// flags (restitution, trigger events, welding, …) is introduced together with
/// the modifiers in later phases.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct CollisionFlags(u32);

impl CollisionFlags {
    /// `IS_STATIC`.
    pub const IS_STATIC: Self = Self(1 << 0);
    /// `IS_DYNAMIC`.
    pub const IS_DYNAMIC: Self = Self(1 << 1);
    /// `IS_KEYFRAMED`.
    pub const IS_KEYFRAMED: Self = Self(1 << 2);
    /// `IS_ACTIVE`.
    pub const IS_ACTIVE: Self = Self(1 << 3);
    /// `IS_PARTICLES_COLLIDER`.
    pub const IS_PARTICLES_COLLIDER: Self = Self(1 << 4);
    /// `DONT_COLLIDE`.
    pub const DONT_COLLIDE: Self = Self(1 << 5);
    /// `DONT_BUILD_CONTACT_JACOBIANS`.
    pub const DONT_BUILD_CONTACT_JACOBIANS: Self = Self(1 << 6);
    /// `DISABLE_PENETRATION_RECOVERY`.
    pub const DISABLE_PENETRATION_RECOVERY: Self = Self(1 << 7);
}

define_flags!(CollisionFlags, u32);

/// `hknpShape::FlagsEnum` — internal shape capabilities.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct ShapeFlags(u16);

impl ShapeFlags {
    /// `IS_CONVEX_SHAPE` — can be cast to `hknpConvexShape`.
    pub const IS_CONVEX_SHAPE: Self = Self(1 << 0);
    /// `IS_CONVEX_POLYTOPE_SHAPE` — convex shape with face information.
    pub const IS_CONVEX_POLYTOPE_SHAPE: Self = Self(1 << 1);
    /// `IS_COMPOSITE_SHAPE` — can be cast to `hknpCompositeShape`.
    pub const IS_COMPOSITE_SHAPE: Self = Self(1 << 2);
    /// `IS_HEIGHT_FIELD_SHAPE`.
    pub const IS_HEIGHT_FIELD_SHAPE: Self = Self(1 << 3);
    /// `USE_SINGLE_POINT_MANIFOLD`.
    pub const USE_SINGLE_POINT_MANIFOLD: Self = Self(1 << 4);
    /// `IS_INTERIOR_TRIANGLE`.
    pub const IS_INTERIOR_TRIANGLE: Self = Self(1 << 5);
    /// `SUPPORTS_COLLISIONS_WITH_INTERIOR_TRIANGLES`.
    pub const SUPPORTS_COLLISIONS_WITH_INTERIOR_TRIANGLES: Self = Self(1 << 6);
    /// `USE_NORMAL_TO_FIND_SUPPORT_PLANE`.
    pub const USE_NORMAL_TO_FIND_SUPPORT_PLANE: Self = Self(1 << 7);
    /// `IS_TRIANGLE_OR_QUAD_SHAPE`.
    pub const IS_TRIANGLE_OR_QUAD_SHAPE: Self = Self(1 << 9);
    /// `IS_QUAD_SHAPE`.
    pub const IS_QUAD_SHAPE: Self = Self(1 << 10);
    /// `IS_SDF_EDGE_COLLISION_ENABLED`.
    pub const IS_SDF_EDGE_COLLISION_ENABLED: Self = Self(1 << 11);
    /// `USER_FLAG_0`.
    pub const USER_FLAG_0: Self = Self(1 << 12);
    /// `USER_FLAG_1`.
    pub const USER_FLAG_1: Self = Self(1 << 13);
    /// `USER_FLAG_2`.
    pub const USER_FLAG_2: Self = Self(1 << 14);
    /// `HAS_SURFACE_VELOCITY`.
    pub const HAS_SURFACE_VELOCITY: Self = Self(1 << 15);
}

define_flags!(ShapeFlags, u16);

/// `hknpBody::WorldAddedState` — body membership bits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct WorldAddedState(u8);

impl WorldAddedState {
    /// `ADDED_TO_WORLD`.
    pub const ADDED_TO_WORLD: Self = Self(1 << 0);
    /// `IN_DEFERRED_BROADPHASE_ADD_QUEUE`.
    pub const IN_DEFERRED_BROADPHASE_ADD_QUEUE: Self = Self(1 << 1);
    /// `ADDED_TO_BROADPHASE`.
    pub const ADDED_TO_BROADPHASE: Self = Self(1 << 2);
}

define_flags!(WorldAddedState, u8);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collision_flags_bit_values() {
        assert_eq!(CollisionFlags::IS_STATIC.bits(), 1);
        assert_eq!(CollisionFlags::IS_DYNAMIC.bits(), 2);
        assert_eq!(CollisionFlags::IS_KEYFRAMED.bits(), 4);
        assert_eq!(CollisionFlags::IS_ACTIVE.bits(), 8);
        assert_eq!(CollisionFlags::DONT_COLLIDE.bits(), 1 << 5);
        assert_eq!(CollisionFlags::DISABLE_PENETRATION_RECOVERY.bits(), 1 << 7);
    }

    #[test]
    fn flag_set_operations() {
        let flags = CollisionFlags::IS_STATIC | CollisionFlags::IS_ACTIVE;
        assert!(flags.contains(CollisionFlags::IS_STATIC));
        assert!(!flags.contains(CollisionFlags::IS_DYNAMIC));
        assert!(!flags.is_empty());

        let mut mutable = CollisionFlags::NONE;
        mutable.insert(CollisionFlags::IS_DYNAMIC);
        mutable.insert(CollisionFlags::DONT_COLLIDE);
        assert!(mutable.contains(CollisionFlags::DONT_COLLIDE));
        mutable.remove(CollisionFlags::DONT_COLLIDE);
        assert!(!mutable.contains(CollisionFlags::DONT_COLLIDE));
        mutable.set(CollisionFlags::IS_ACTIVE, true);
        assert!(mutable.contains(CollisionFlags::IS_ACTIVE));
        mutable.set(CollisionFlags::IS_ACTIVE, false);
        assert!(!mutable.contains(CollisionFlags::IS_ACTIVE));

        assert_eq!(
            CollisionFlags::from_bits(0b1).bits(),
            1,
            "raw bit round trip"
        );
    }

    #[test]
    fn shape_flags() {
        let convex = ShapeFlags::IS_CONVEX_SHAPE | ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE;
        assert!(convex.contains(ShapeFlags::IS_CONVEX_SHAPE));
        assert!(!convex.contains(ShapeFlags::IS_COMPOSITE_SHAPE));
        assert_eq!(ShapeFlags::USER_FLAG_2.bits(), 1 << 14);
        assert_eq!(ShapeFlags::HAS_SURFACE_VELOCITY.bits(), 1 << 15);
    }

    #[test]
    fn world_added_state() {
        let mut state = WorldAddedState::ADDED_TO_WORLD;
        assert!(state.contains(WorldAddedState::ADDED_TO_WORLD));
        state.insert(WorldAddedState::ADDED_TO_BROADPHASE);
        assert!(state.contains(WorldAddedState::ADDED_TO_BROADPHASE));
        state.remove(WorldAddedState::ADDED_TO_WORLD);
        assert!(!state.contains(WorldAddedState::ADDED_TO_WORLD));
    }
}
