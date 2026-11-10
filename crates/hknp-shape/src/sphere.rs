//! Sphere (`hknpSphereShape`).
//!
//! Port of `Collide/Shape/Convex/Sphere/hknpSphereShape.{h,inl}`. The C++ shape
//! stores a `hkFloat3 m_center` and `hkReal m_radius`; here the center is a
//! [`Vector4`] whose `w` is ignored (points carry `w = 0` in the port).

use core::any::Any;

use hk_math::{Aabb, Vector4};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::{Shape, aabb_bounding_radius, unit_direction};

/// Sphere (`hknpSphereShape`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphereShape {
    /// Sphere center (local space).
    pub center: Vector4,
    /// Sphere radius.
    pub radius: f32,
}

impl SphereShape {
    /// Construct a sphere at `center` with `radius`.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if `radius` is negative or not finite.
    #[inline]
    pub fn new(center: Vector4, radius: f32) -> Self {
        debug_assert!(radius >= 0.0 && radius.is_finite(), "invalid sphere radius");
        Self { center, radius }
    }
}

impl Shape for SphereShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Sphere
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Convex
    }

    fn flags(&self) -> ShapeFlags {
        ShapeFlags::IS_CONVEX_SHAPE
    }

    fn aabb(&self) -> Aabb {
        Aabb::new(
            Vector4::from_xyz(
                self.center.x - self.radius,
                self.center.y - self.radius,
                self.center.z - self.radius,
            ),
            Vector4::from_xyz(
                self.center.x + self.radius,
                self.center.y + self.radius,
                self.center.z + self.radius,
            ),
        )
    }

    fn bounding_radius(&self) -> f32 {
        aabb_bounding_radius(&self.aabb())
    }

    fn volume(&self) -> Option<f32> {
        Some(4.0 / 3.0 * core::f32::consts::PI * self.radius * self.radius * self.radius)
    }

    fn support(&self, direction: Vector4) -> Option<Vector4> {
        match unit_direction(direction) {
            Some(unit) => Some(self.center + unit * self.radius),
            None => Some(self.center),
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_sphere_at_origin() {
        let sphere = SphereShape::new(Vector4::ZERO, 1.0);
        let aabb = sphere.aabb();
        assert_eq!(aabb.min, Vector4::from_xyz(-1.0, -1.0, -1.0));
        assert_eq!(aabb.max, Vector4::from_xyz(1.0, 1.0, 1.0));
        assert!((sphere.bounding_radius() - (3.0f32).sqrt()).abs() < 1e-6);
        assert!((sphere.volume().unwrap() - 4.0 / 3.0 * std::f32::consts::PI).abs() < 1e-5);
    }

    #[test]
    fn support_is_center_plus_radius() {
        let sphere = SphereShape::new(Vector4::from_xyz(1.0, 2.0, 3.0), 2.5);
        let support = sphere.support(Vector4::UNIT_X).unwrap();
        assert_eq!(support, Vector4::from_xyz(3.5, 2.0, 3.0));
        let support = sphere.support(Vector4::new(1.0, 1.0, 0.0, 0.0)).unwrap();
        let expected = 2.5 / (2.0f32).sqrt();
        assert!((support.x - (1.0 + expected)).abs() < 1e-5);
        assert!((support.y - (2.0 + expected)).abs() < 1e-5);
    }

    #[test]
    fn zero_direction_returns_center() {
        let sphere = SphereShape::new(Vector4::from_xyz(4.0, 0.0, 0.0), 1.0);
        assert_eq!(sphere.support(Vector4::ZERO).unwrap(), sphere.center);
    }

    #[test]
    fn offset_sphere_aabb() {
        let sphere = SphereShape::new(Vector4::from_xyz(10.0, -20.0, 30.0), 3.0);
        let aabb = sphere.aabb();
        assert!(
            aabb.min
                .all_equal3(Vector4::from_xyz(7.0, -23.0, 27.0), 0.0)
        );
        assert!(
            aabb.max
                .all_equal3(Vector4::from_xyz(13.0, -17.0, 33.0), 0.0)
        );
    }
}
