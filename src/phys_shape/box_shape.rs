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

use glam::{Mat3, Quat, Vec4};

use crate::base_math::{Aabb, Vec4Ext};
use crate::phys_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::phys_shape::{DEFAULT_CONVEX_RADIUS, MassConfig, MassError, MassProperties, Shape, unit_direction};

/// Oriented box in the local frame (`hknpBoxShape`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxShape {
    /// Half extents of the OBB (excluding convex radius).
    pub half_extents: Vec4,
    /// Convex radius (`hknpBoxShape::m_convexRadius`, default
    /// [`DEFAULT_CONVEX_RADIUS`]).
    pub convex_radius: f32,
}

impl BoxShape {
    /// Construct a box from half extents with the SDK default convex radius.
    #[inline]
    pub fn new(half_extents: Vec4) -> Self {
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
    pub fn with_convex_radius(half_extents: Vec4, convex_radius: f32) -> Self {
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
    pub fn outer_half_extents(&self) -> Vec4 {
        self.half_extents + Vec4::splat(self.convex_radius)
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
        Aabb::from_corners(
            glam::Vec3A::new(-extent.x, -extent.y, -extent.z),
            glam::Vec3A::new(extent.x, extent.y, extent.z),
        )
    }

    fn bounding_radius(&self) -> f32 {
        // Legacy `length3`: xyz-only length with the fixed scalar order.
        let extent = self.outer_half_extents();
        extent.dot3(extent).sqrt()
    }

    fn volume(&self) -> Option<f32> {
        let h = self.half_extents;
        Some(8.0 * h.x * h.y * h.z)
    }

    fn mass_properties(&self, config: MassConfig) -> Result<MassProperties, MassError> {
        let h = self.half_extents;
        let volume = 8.0 * h.x * h.y * h.z;
        let inertia = Mat3::from_diagonal(glam::Vec3::new(
            volume / 3.0 * (h.y * h.y + h.z * h.z),
            volume / 3.0 * (h.x * h.x + h.z * h.z),
            volume / 3.0 * (h.x * h.x + h.y * h.y),
        ));
        MassProperties::from_geometry(volume, Vec4::ZERO, inertia, Quat::IDENTITY, config)
    }

    fn support(&self, direction: Vec4) -> Option<Vec4> {
        let sign = |value: f32| if value < 0.0 { -1.0 } else { 1.0 };
        let h = self.half_extents;
        let mut point = Vec4::new(
            sign(direction.x) * h.x,
            sign(direction.y) * h.y,
            sign(direction.z) * h.z,
            0.0,
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
    use glam::Vec3A;

    #[test]
    fn aabb_and_volume() {
        let box_ = BoxShape::with_convex_radius(Vec4::from_xyz(1.0, 2.0, 3.0), 0.0);
        assert_eq!(box_.aabb().min, Vec3A::new(-1.0, -2.0, -3.0));
        assert_eq!(box_.aabb().max, Vec3A::new(1.0, 2.0, 3.0));
        assert_eq!(box_.volume().unwrap(), 48.0);
        assert!((box_.bounding_radius() - (14.0f32).sqrt()).abs() < 1e-6);
    }

    #[test]
    fn convex_radius_expands_aabb_and_support() {
        let box_ = BoxShape::with_convex_radius(Vec4::from_xyz(1.0, 1.0, 1.0), 0.5);
        assert_eq!(box_.aabb().max, Vec3A::new(1.5, 1.5, 1.5));
        // Support along +X: box corner (1,1,1) is not the +X support; the support
        // corner is (1,1,1) rounded by radius in direction X → (1.5,1,1).
        let support = box_.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap();
        assert_eq!(support, Vec4::from_xyz(1.5, 1.0, 1.0));
        // Default convex radius is applied by `new`.
        assert_eq!(
            BoxShape::new(Vec4::splat(1.0)).convex_radius,
            DEFAULT_CONVEX_RADIUS
        );
    }

    #[test]
    fn support_follows_direction_signs() {
        let box_ = BoxShape::with_convex_radius(Vec4::from_xyz(1.0, 2.0, 3.0), 0.0);
        let support = box_.support(Vec4::from_xyz(-1.0, 1.0, -1.0)).unwrap();
        assert_eq!(support, Vec4::from_xyz(-1.0, 2.0, -3.0));
        let support = box_.support(Vec4::from_xyz(1.0, -1.0, 1.0)).unwrap();
        assert_eq!(support, Vec4::from_xyz(1.0, -2.0, 3.0));
    }

    #[test]
    fn zero_components_use_positive_extent() {
        let box_ = BoxShape::with_convex_radius(Vec4::from_xyz(1.0, 2.0, 3.0), 0.0);
        let support = box_.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap();
        assert_eq!(support, Vec4::from_xyz(1.0, 2.0, 3.0));
    }
}
