//! Triangle (`hknpTriangleShape`).
//!
//! Port of `Collide/Shape/Convex/Triangle/hknpTriangleShape.{h,inl}`. The C++
//! shape stores up to four points (`m_points[4]`) to support quads; this port
//! only implements the triangle form (`m_numPoints = 3`). The convex radius
//! inflates the AABB and support point.

use core::any::Any;

use glam::{Mat3, Quat, Vec4};

use crate::base_math::{Aabb, QuatExt, Vec4Ext};
use crate::phys_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::phys_shape::{
    MassConfig, MassError, MassProperties, Shape, aabb_bounding_radius, outer_product,
    unit_direction,
};

/// Triangle (`hknpTriangleShape`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriangleShape {
    /// Triangle vertices.
    pub points: [Vec4; 3],
    /// Convex radius.
    pub convex_radius: f32,
}

impl TriangleShape {
    /// Construct a triangle with the SDK default convex radius
    /// ([`crate::phys_shape::DEFAULT_CONVEX_RADIUS`]).
    #[inline]
    pub fn new(a: Vec4, b: Vec4, c: Vec4) -> Self {
        Self::with_convex_radius(a, b, c, crate::phys_shape::DEFAULT_CONVEX_RADIUS)
    }

    /// Construct a triangle with an explicit convex radius.
    #[inline]
    pub fn with_convex_radius(a: Vec4, b: Vec4, c: Vec4, convex_radius: f32) -> Self {
        debug_assert!(convex_radius >= 0.0, "invalid triangle convex radius");
        Self {
            points: [a, b, c],
            convex_radius,
        }
    }

    /// Triangle normal (not normalized; zero for a degenerate triangle).
    #[inline]
    pub fn normal(&self) -> Vec4 {
        (self.points[1] - self.points[0]).cross3(self.points[2] - self.points[0])
    }

    /// Area of the triangle.
    #[inline]
    pub fn area(&self) -> f32 {
        // Legacy `length3`: xyz-only length with the fixed scalar order.
        let normal = self.normal();
        0.5 * normal.dot3(normal).sqrt()
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
        Aabb::from_corners(
            glam::Vec3A::new(min.x - r, min.y - r, min.z - r),
            glam::Vec3A::new(max.x + r, max.y + r, max.z + r),
        )
    }

    fn bounding_radius(&self) -> f32 {
        aabb_bounding_radius(&self.aabb())
    }

    fn volume(&self) -> Option<f32> {
        // A triangle has zero solid volume (the convex radius is surface rounding).
        Some(0.0)
    }

    fn mass_properties(&self, config: MassConfig) -> Result<MassProperties, MassError> {
        let area = self.area();
        if area <= 0.0 {
            return MassProperties::from_geometry(
                0.0,
                self.points[0],
                Mat3::ZERO,
                Quat::IDENTITY,
                config,
            );
        }

        let p0 = self.points[0];
        let p1 = self.points[1];
        let p2 = self.points[2];
        let center = (p0 + p1 + p2) / 3.0;
        let sum = p0 + p1 + p2;
        let squares = outer_product(p0, p0) + outer_product(p1, p1) + outer_product(p2, p2);
        let second_moment = (outer_product(sum, sum) + squares) * (area / 12.0);
        let shifted = second_moment - outer_product(center, center) * area;
        let trace = shifted.x_axis.x + shifted.y_axis.y + shifted.z_axis.z;
        let inertia = Mat3::from_diagonal(glam::Vec3::splat(trace)) - shifted;

        let normal =
            unit_direction(self.normal()).unwrap_or_else(|| Vec4::new(0.0, 0.0, 1.0, 0.0));
        let reference = if normal.x.abs() < 0.9 {
            Vec4::new(1.0, 0.0, 0.0, 0.0)
        } else {
            Vec4::new(0.0, 1.0, 0.0, 0.0)
        };
        let tangent_a = normal.cross3(reference).normalize_or_zero3();
        let tangent_b = normal.cross3(tangent_a).normalize_or_zero3();
        let major_axis_space = Quat::from_matrix_sdk(Mat3::from_cols(
            glam::Vec3::new(tangent_a.x, tangent_a.y, tangent_a.z),
            glam::Vec3::new(tangent_b.x, tangent_b.y, tangent_b.z),
            glam::Vec3::new(normal.x, normal.y, normal.z),
        ));

        MassProperties::from_geometry(area, center, inertia, major_axis_space, config)
    }

    fn support(&self, direction: Vec4) -> Option<Vec4> {
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
    use glam::Vec3A;

    #[test]
    fn aabb_covers_vertices_and_radius() {
        let triangle = TriangleShape::with_convex_radius(
            Vec4::ZERO,
            Vec4::from_xyz(1.0, 0.0, 0.0),
            Vec4::from_xyz(0.0, 1.0, 0.0),
            0.1,
        );
        let aabb = triangle.aabb();
        assert!(aabb.min.abs_diff_eq(Vec3A::new(-0.1, -0.1, -0.1), 0.0));
        assert!(aabb.max.abs_diff_eq(Vec3A::new(1.1, 1.1, 0.1), 0.0));
        assert!((triangle.area() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn support_picks_extreme_vertex() {
        let triangle = TriangleShape::with_convex_radius(
            Vec4::ZERO,
            Vec4::from_xyz(1.0, 0.0, 0.0),
            Vec4::from_xyz(0.0, 1.0, 0.0),
            0.0,
        );
        assert_eq!(
            triangle.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap(),
            Vec4::from_xyz(1.0, 0.0, 0.0)
        );
        assert_eq!(
            triangle.support(Vec4::new(0.0, 1.0, 0.0, 0.0)).unwrap(),
            Vec4::from_xyz(0.0, 1.0, 0.0)
        );
        // Diagonal direction: the two positive vertices tie; the first one wins.
        let support = triangle.support(Vec4::new(1.0, 1.0, 0.0, 0.0)).unwrap();
        assert!((support.x + support.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn volume_is_zero_and_normal_is_oriented() {
        let triangle = TriangleShape::with_convex_radius(
            Vec4::ZERO,
            Vec4::from_xyz(1.0, 0.0, 0.0),
            Vec4::from_xyz(0.0, 0.0, 1.0),
            0.0,
        );
        assert_eq!(triangle.volume().unwrap(), 0.0);
        assert_eq!(triangle.normal(), Vec4::from_xyz(0.0, -1.0, 0.0));
    }
}
