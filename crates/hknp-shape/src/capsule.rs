//! Capsule (`hknpCapsuleShape`).
//!
//! Port of `Collide/Shape/Convex/Capsule/hknpCapsuleShape.{h,inl}`. A capsule is
//! the Minkowski sum of a line segment (`m_pointA` → `m_pointB`) and a sphere of
//! `radius` (`m_radius`); the SDK stores the two points as the hull vertices and
//! the radius as the convex radius.

use core::any::Any;

use hk_math::{Aabb, Vector4};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::{Shape, aabb_bounding_radius, unit_direction};

/// Capsule (`hknpCapsuleShape`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapsuleShape {
    /// Segment endpoint A.
    pub point_a: Vector4,
    /// Segment endpoint B.
    pub point_b: Vector4,
    /// Radius of the sphere swept along the segment.
    pub radius: f32,
}

impl CapsuleShape {
    /// Construct a capsule from two endpoints and a radius.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if `radius` is negative.
    #[inline]
    pub fn new(point_a: Vector4, point_b: Vector4, radius: f32) -> Self {
        debug_assert!(radius >= 0.0, "invalid capsule radius");
        Self {
            point_a,
            point_b,
            radius,
        }
    }

    /// Unit axis from A to B, or `None` for a degenerate segment.
    #[inline]
    pub fn axis(&self) -> Option<Vector4> {
        unit_direction(self.point_b - self.point_a)
    }

    /// Segment length.
    #[inline]
    pub fn segment_length(&self) -> f32 {
        (self.point_b - self.point_a).length3()
    }

    /// Support point on the segment: the capsule hull has two vertices, so the
    /// extreme vertex along `direction` wins (dot == 0 keeps endpoint A).
    fn support_on_segment(&self, direction: Vector4) -> Vector4 {
        if direction.dot3(self.point_b - self.point_a) > 0.0 {
            self.point_b
        } else {
            self.point_a
        }
    }
}

impl Shape for CapsuleShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Capsule
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Convex
    }

    fn flags(&self) -> ShapeFlags {
        ShapeFlags::IS_CONVEX_SHAPE
    }

    fn aabb(&self) -> Aabb {
        let r = self.radius;
        Aabb::new(
            Vector4::from_xyz(
                self.point_a.x.min(self.point_b.x) - r,
                self.point_a.y.min(self.point_b.y) - r,
                self.point_a.z.min(self.point_b.z) - r,
            ),
            Vector4::from_xyz(
                self.point_a.x.max(self.point_b.x) + r,
                self.point_a.y.max(self.point_b.y) + r,
                self.point_a.z.max(self.point_b.z) + r,
            ),
        )
    }

    fn bounding_radius(&self) -> f32 {
        aabb_bounding_radius(&self.aabb())
    }

    fn volume(&self) -> Option<f32> {
        let pi = core::f32::consts::PI;
        let cylinder = pi * self.radius * self.radius * self.segment_length();
        let caps = 4.0 / 3.0 * pi * self.radius * self.radius * self.radius;
        Some(cylinder + caps)
    }

    fn support(&self, direction: Vector4) -> Option<Vector4> {
        let segment_point = self.support_on_segment(direction);
        match unit_direction(direction) {
            Some(unit) => Some(segment_point + unit * self.radius),
            None => Some(segment_point),
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
    fn aabb_and_volume_of_y_capsule() {
        let capsule = CapsuleShape::new(
            Vector4::from_xyz(0.0, -1.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            0.5,
        );
        let aabb = capsule.aabb();
        assert!(
            aabb.min
                .all_equal3(Vector4::from_xyz(-0.5, -1.5, -0.5), 0.0)
        );
        assert!(aabb.max.all_equal3(Vector4::from_xyz(0.5, 1.5, 0.5), 0.0));
        let expected = std::f32::consts::PI * 0.25 * 2.0 + 4.0 / 3.0 * std::f32::consts::PI * 0.125;
        assert!((capsule.volume().unwrap() - expected).abs() < 1e-5);
        assert_eq!(capsule.segment_length(), 2.0);
    }

    #[test]
    fn support_uses_closest_segment_point() {
        let capsule = CapsuleShape::new(
            Vector4::from_xyz(0.0, -1.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            1.0,
        );
        let support = capsule.support(Vector4::UNIT_X).unwrap();
        // Direction X projects onto the segment at t = 0 (dot == 0), so the
        // support is endpoint A plus one radius in X.
        assert_eq!(support.x, 1.0);
        assert!((-1.0..=1.0).contains(&support.y));
        assert_eq!(support.z, 0.0);

        let support = capsule.support(Vector4::UNIT_Y).unwrap();
        assert_eq!(support, Vector4::from_xyz(0.0, 2.0, 0.0));
    }

    #[test]
    fn degenerate_capsule_behaves_like_sphere() {
        let capsule = CapsuleShape::new(
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Vector4::from_xyz(1.0, 0.0, 0.0),
            2.0,
        );
        let support = capsule.support(Vector4::UNIT_X).unwrap();
        assert_eq!(support, Vector4::from_xyz(3.0, 0.0, 0.0));
        assert_eq!(capsule.axis(), None);
        assert_eq!(capsule.segment_length(), 0.0);
    }

    #[test]
    fn diagonal_capsule_support() {
        let capsule = CapsuleShape::new(Vector4::ZERO, Vector4::from_xyz(2.0, 2.0, 0.0), 1.0);
        let support = capsule.support(Vector4::UNIT_X).unwrap();
        // Direction X has a positive projection on the segment, so endpoint B
        // is the hull support point; the radius is added along X.
        let expected = Vector4::from_xyz(3.0, 2.0, 0.0);
        assert!(support.all_equal3(expected, 1e-5), "got {support:?}");
    }
}
