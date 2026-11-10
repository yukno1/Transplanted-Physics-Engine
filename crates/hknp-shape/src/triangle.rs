//! Triangle (`hknpTriangleShape`).
//!
//! Port of `Collide/Shape/Convex/Triangle/hknpTriangleShape.{h,inl}`. The C++
//! shape stores up to four points (`m_points[4]`) to support quads; this port
//! only implements the triangle form (`m_numPoints = 3`). The convex radius
//! inflates the AABB and support point.

use core::any::Any;

use hk_math::{Aabb, Vector4};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::{Shape, aabb_bounding_radius, unit_direction};

/// Triangle (`hknpTriangleShape`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriangleShape {
    /// Triangle vertices.
    pub points: [Vector4; 3],
    /// Convex radius.
    pub convex_radius: f32,
}

impl TriangleShape {
    /// Construct a triangle with the SDK default convex radius
    /// ([`crate::DEFAULT_CONVEX_RADIUS`]).
    #[inline]
    pub fn new(a: Vector4, b: Vector4, c: Vector4) -> Self {
        Self::with_convex_radius(a, b, c, crate::DEFAULT_CONVEX_RADIUS)
    }

    /// Construct a triangle with an explicit convex radius.
    #[inline]
    pub fn with_convex_radius(a: Vector4, b: Vector4, c: Vector4, convex_radius: f32) -> Self {
        debug_assert!(convex_radius >= 0.0, "invalid triangle convex radius");
        Self {
            points: [a, b, c],
            convex_radius,
        }
    }

    /// Triangle normal (not normalized; zero for a degenerate triangle).
    #[inline]
    pub fn normal(&self) -> Vector4 {
        (self.points[1] - self.points[0]).cross(self.points[2] - self.points[0])
    }

    /// Area of the triangle.
    #[inline]
    pub fn area(&self) -> f32 {
        0.5 * self.normal().length3()
    }
}

impl Shape for TriangleShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Triangle
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Convex
    }

    fn flags(&self) -> ShapeFlags {
        ShapeFlags::IS_CONVEX_SHAPE | ShapeFlags::IS_TRIANGLE_OR_QUAD_SHAPE
    }

    fn aabb(&self) -> Aabb {
        let mut min = self.points[0];
        let mut max = self.points[0];
        for point in &self.points[1..] {
            min = min.min(*point);
            max = max.max(*point);
        }
        let r = self.convex_radius;
        Aabb::new(
            Vector4::from_xyz(min.x - r, min.y - r, min.z - r),
            Vector4::from_xyz(max.x + r, max.y + r, max.z + r),
        )
    }

    fn bounding_radius(&self) -> f32 {
        aabb_bounding_radius(&self.aabb())
    }

    fn volume(&self) -> Option<f32> {
        // A triangle has zero solid volume (the convex radius is surface rounding).
        Some(0.0)
    }

    fn support(&self, direction: Vector4) -> Option<Vector4> {
        let mut best = self.points[0];
        let mut best_dot = best.dot3(direction);
        for point in &self.points[1..] {
            let dot = point.dot3(direction);
            if dot > best_dot {
                best = *point;
                best_dot = dot;
            }
        }
        if let Some(unit) = unit_direction(direction) {
            best += unit * self.convex_radius;
        }
        Some(best)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_covers_vertices_and_radius() {
        let triangle = TriangleShape::with_convex_radius(
            Vector4::ZERO,
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            0.1,
        );
        let aabb = triangle.aabb();
        assert!(
            aabb.min
                .all_equal3(Vector4::from_xyz(-0.1, -0.1, -0.1), 0.0)
        );
        assert!(aabb.max.all_equal3(Vector4::from_xyz(1.1, 1.1, 0.1), 0.0));
        assert!((triangle.area() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn support_picks_extreme_vertex() {
        let triangle = TriangleShape::with_convex_radius(
            Vector4::ZERO,
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            0.0,
        );
        assert_eq!(
            triangle.support(Vector4::UNIT_X).unwrap(),
            Vector4::from_xyz(1.0, 0.0, 0.0)
        );
        assert_eq!(
            triangle.support(Vector4::UNIT_Y).unwrap(),
            Vector4::from_xyz(0.0, 1.0, 0.0)
        );
        // Diagonal direction: the two positive vertices tie; the first one wins.
        let support = triangle.support(Vector4::new(1.0, 1.0, 0.0, 0.0)).unwrap();
        assert!((support.x + support.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn volume_is_zero_and_normal_is_oriented() {
        let triangle = TriangleShape::with_convex_radius(
            Vector4::ZERO,
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Vector4::from_xyz(0.0, 0.0, 1.0),
            0.0,
        );
        assert_eq!(triangle.volume().unwrap(), 0.0);
        assert_eq!(triangle.normal(), Vector4::from_xyz(0.0, -1.0, 0.0));
    }
}
