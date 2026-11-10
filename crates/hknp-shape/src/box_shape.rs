//! Box (`hknpBoxShape`).
//!
//! Port of `Collide/Shape/Convex/Box/hknpBoxShape.{h,inl}`. The C++ shape keeps
//! an OBB (`m_obb`) plus `m_convexRadius`; the OBB transform belongs to the body
//! instance, so this port represents the box by its half extents in the shape's
//! local frame and applies the instance transform through [`Shape::calc_aabb`].
//!
//! The physical surface is the Minkowski sum of the box and a sphere of
//! `convex_radius`: AABB/support include the radius, the reported volume is the
//! nominal box volume (radius is rounding, not solid), matching the SDK's
//! geometry/rounding split.

use core::any::Any;

use hk_math::{Aabb, Vector4};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::{DEFAULT_CONVEX_RADIUS, Shape, unit_direction};

/// Oriented box in the local frame (`hknpBoxShape`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxShape {
    /// Half extents of the OBB (excluding convex radius).
    pub half_extents: Vector4,
    /// Convex radius (`hknpBoxShape::m_convexRadius`, default
    /// [`DEFAULT_CONVEX_RADIUS`]).
    pub convex_radius: f32,
}

impl BoxShape {
    /// Construct a box from half extents with the SDK default convex radius.
    #[inline]
    pub fn new(half_extents: Vector4) -> Self {
        Self {
            half_extents,
            convex_radius: DEFAULT_CONVEX_RADIUS,
        }
    }

    /// Construct a box with an explicit convex radius.
    ///
    /// # Panics
    ///
    /// Panics in debug builds for negative extents/radius.
    #[inline]
    pub fn with_convex_radius(half_extents: Vector4, convex_radius: f32) -> Self {
        debug_assert!(
            half_extents.x >= 0.0
                && half_extents.y >= 0.0
                && half_extents.z >= 0.0
                && convex_radius >= 0.0,
            "invalid box extents/radius"
        );
        Self {
            half_extents,
            convex_radius,
        }
    }

    /// Full extent along each axis including the convex radius.
    #[inline]
    pub fn outer_half_extents(&self) -> Vector4 {
        self.half_extents + Vector4::splat(self.convex_radius)
    }
}

impl Shape for BoxShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Box
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Convex
    }

    fn flags(&self) -> ShapeFlags {
        ShapeFlags::IS_CONVEX_SHAPE | ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE
    }

    fn aabb(&self) -> Aabb {
        let extent = self.outer_half_extents();
        Aabb::new(
            Vector4::from_xyz(-extent.x, -extent.y, -extent.z),
            Vector4::from_xyz(extent.x, extent.y, extent.z),
        )
    }

    fn bounding_radius(&self) -> f32 {
        self.outer_half_extents().length3()
    }

    fn volume(&self) -> Option<f32> {
        let h = self.half_extents;
        Some(8.0 * h.x * h.y * h.z)
    }

    fn support(&self, direction: Vector4) -> Option<Vector4> {
        let sign = |value: f32| if value < 0.0 { -1.0 } else { 1.0 };
        let mut point = Vector4::from_xyz(
            sign(direction.x) * self.half_extents.x,
            sign(direction.y) * self.half_extents.y,
            sign(direction.z) * self.half_extents.z,
        );
        if let Some(unit) = unit_direction(direction) {
            point += unit * self.convex_radius;
        }
        Some(point)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_and_volume() {
        let box_ = BoxShape::with_convex_radius(Vector4::from_xyz(1.0, 2.0, 3.0), 0.0);
        assert_eq!(box_.aabb().min, Vector4::from_xyz(-1.0, -2.0, -3.0));
        assert_eq!(box_.aabb().max, Vector4::from_xyz(1.0, 2.0, 3.0));
        assert_eq!(box_.volume().unwrap(), 48.0);
        assert!((box_.bounding_radius() - (14.0f32).sqrt()).abs() < 1e-6);
    }

    #[test]
    fn convex_radius_expands_aabb_and_support() {
        let box_ = BoxShape::with_convex_radius(Vector4::from_xyz(1.0, 1.0, 1.0), 0.5);
        assert_eq!(box_.aabb().max, Vector4::from_xyz(1.5, 1.5, 1.5));
        // Support along +X: box corner (1,1,1) is not the +X support; the support
        // corner is (1,1,1) rounded by radius in direction X → (1.5,1,1).
        let support = box_.support(Vector4::UNIT_X).unwrap();
        assert_eq!(support, Vector4::from_xyz(1.5, 1.0, 1.0));
        // Default convex radius is applied by `new`.
        assert_eq!(
            BoxShape::new(Vector4::splat(1.0)).convex_radius,
            DEFAULT_CONVEX_RADIUS
        );
    }

    #[test]
    fn support_follows_direction_signs() {
        let box_ = BoxShape::with_convex_radius(Vector4::from_xyz(1.0, 2.0, 3.0), 0.0);
        let support = box_.support(Vector4::from_xyz(-1.0, 1.0, -1.0)).unwrap();
        assert_eq!(support, Vector4::from_xyz(-1.0, 2.0, -3.0));
        let support = box_.support(Vector4::from_xyz(1.0, -1.0, 1.0)).unwrap();
        assert_eq!(support, Vector4::from_xyz(1.0, -2.0, 3.0));
    }

    #[test]
    fn zero_components_use_positive_extent() {
        let box_ = BoxShape::with_convex_radius(Vector4::from_xyz(1.0, 2.0, 3.0), 0.0);
        let support = box_.support(Vector4::UNIT_X).unwrap();
        assert_eq!(support, Vector4::from_xyz(1.0, 2.0, 3.0));
    }
}
