//! Cylinder (`hknpCylinderShape`).
//!
//! Port of `Collide/Shape/Convex/Cylinder/hknpCylinderShape.{h,inl}`. The C++
//! shape stores its endpoints in `m_a`/`m_b` (with the radius/num-sides packed in
//! the `w` components) plus a convex radius; here the fields are explicit.
//!
//! The support function is the exact finite-cylinder support: axial component
//! along the segment plus the radial direction projected onto the disc.

use core::any::Any;

use glam::{Mat3, Quat, Vec4};

use crate::base_math::{Aabb, QuatExt, Vec4Ext};
use crate::phys_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::phys_shape::{
    DEFAULT_CONVEX_RADIUS, MassConfig, MassError, MassProperties, Shape, aabb_bounding_radius,
    outer_product, unit_direction,
};

/// Finite cylinder (`hknpCylinderShape`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylinderShape {
    /// Axis endpoint A.
    pub point_a: Vec4,
    /// Axis endpoint B.
    pub point_b: Vec4,
    /// Cylinder radius.
    pub radius: f32,
    /// Number of sides used to tessellate the cylinder (`numSides`, default 16).
    pub num_sides: u32,
    /// Convex radius (`hknpCylinderShape::m_convexRadius`).
    pub convex_radius: f32,
}

impl CylinderShape {
    /// Construct with the SDK defaults (`numSides = 16`, default convex radius).
    #[inline]
    pub fn new(point_a: Vec4, point_b: Vec4, radius: f32) -> Self {
        Self::with_params(point_a, point_b, radius, 16, DEFAULT_CONVEX_RADIUS)
    }

    /// Construct with explicit tessellation and convex radius.
    ///
    /// # Panics
    ///
    /// Panics in debug builds for a negative radius/convex radius or fewer than
    /// three sides.
    #[inline]
    pub fn with_params(
        point_a: Vec4,
        point_b: Vec4,
        radius: f32,
        num_sides: u32,
        convex_radius: f32,
    ) -> Self {
        debug_assert!(
            radius >= 0.0 && convex_radius >= 0.0 && num_sides >= 3,
            "invalid cylinder parameters"
        );
        Self {
            point_a,
            point_b,
            radius,
            num_sides,
            convex_radius,
        }
    }

    /// Segment length (cylinder height).
    #[inline]
    pub fn height(&self) -> f32 {
        // Legacy `length3`: xyz-only length with the fixed scalar order.
        let segment = self.point_b - self.point_a;
        segment.dot3(segment).sqrt()
    }

    /// Cross-sectional area of the tessellated polygon.
    fn polygon_area(&self) -> f32 {
        let sides = self.num_sides as f32;
        let angle = 2.0 * core::f32::consts::PI / sides;
        0.5 * sides * self.radius * self.radius * angle.sin()
    }
}

impl Shape for CylinderShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Cylinder
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Convex
    }

    fn flags(&self) -> ShapeFlags {
        ShapeFlags::IS_CONVEX_SHAPE
    }

    fn aabb(&self) -> Aabb {
        let radius = self.radius + self.convex_radius;
        let segment = self.point_b - self.point_a;
        let length = segment.dot3(segment).sqrt();
        if length == 0.0 {
            return Aabb::from_corners(
                glam::Vec3A::new(
                    self.point_a.x - radius,
                    self.point_a.y - radius,
                    self.point_a.z - radius,
                ),
                glam::Vec3A::new(
                    self.point_a.x + radius,
                    self.point_a.y + radius,
                    self.point_a.z + radius,
                ),
            );
        }

        let axis = segment / length;
        let center = (self.point_a + self.point_b) * 0.5;
        let half_height = length * 0.5;
        // Exact support extent of the finite cylinder along each world axis:
        // h*|axis_i| + r*sqrt(1 - axis_i^2) (plus the convex radius).
        let radial = |axis_component: f32| (1.0 - axis_component * axis_component).max(0.0).sqrt();
        let extent = Vec4::from_xyz(
            half_height * axis.x.abs() + radius * radial(axis.x),
            half_height * axis.y.abs() + radius * radial(axis.y),
            half_height * axis.z.abs() + radius * radial(axis.z),
        );
        Aabb::from_corners(
            glam::Vec3A::new(
                center.x - extent.x,
                center.y - extent.y,
                center.z - extent.z,
            ),
            glam::Vec3A::new(
                center.x + extent.x,
                center.y + extent.y,
                center.z + extent.z,
            ),
        )
    }

    fn bounding_radius(&self) -> f32 {
        aabb_bounding_radius(&self.aabb())
    }

    fn volume(&self) -> Option<f32> {
        Some(self.polygon_area() * self.height())
    }

    fn mass_properties(&self, config: MassConfig) -> Result<MassProperties, MassError> {
        let height = self.height();
        let radius_squared = self.radius * self.radius;
        let volume = self.polygon_area() * height;
        let axial = 0.5 * volume * radius_squared;
        let perpendicular = volume * (3.0 * radius_squared + height * height) / 12.0;

        let axis = unit_direction(self.point_b - self.point_a)
            .unwrap_or_else(|| Vec4::new(1.0, 0.0, 0.0, 0.0));
        let reference = if axis.x.abs() < 0.9 {
            Vec4::new(1.0, 0.0, 0.0, 0.0)
        } else {
            Vec4::new(0.0, 1.0, 0.0, 0.0)
        };
        let perpendicular_a = axis.cross3(reference).normalize_or_zero3();
        let perpendicular_b = axis.cross3(perpendicular_a).normalize_or_zero3();
        let major_axis_space = Quat::from_matrix_sdk(Mat3::from_cols(
            glam::Vec3::new(perpendicular_a.x, perpendicular_a.y, perpendicular_a.z),
            glam::Vec3::new(perpendicular_b.x, perpendicular_b.y, perpendicular_b.z),
            glam::Vec3::new(axis.x, axis.y, axis.z),
        ));

        let axial_outer = outer_product(axis, axis);
        let unit_inertia = Mat3::IDENTITY * perpendicular + axial_outer * (axial - perpendicular);
        MassProperties::from_geometry(
            volume,
            (self.point_a + self.point_b) * 0.5,
            unit_inertia,
            major_axis_space,
            config,
        )
    }

    fn support(&self, direction: Vec4) -> Option<Vec4> {
        let a = self.point_a;
        let b = self.point_b;
        let segment = b - a;
        let length = segment.dot3(segment).sqrt();
        if length == 0.0 {
            // Degenerate cylinder: behave like a sphere at point A.
            return Some(match unit_direction(direction) {
                Some(unit) => a + unit * self.radius,
                None => a,
            });
        }

        let axis = segment / length;
        let center = (a + b) * 0.5;
        let half_height = length * 0.5;
        let axial = axis
            * (half_height
                * if direction.dot3(axis) < 0.0 {
                    -1.0
                } else {
                    1.0
                });

        let radial_projection = direction - axis * direction.dot3(axis);
        let radial_length = radial_projection.dot3(radial_projection).sqrt();
        let radial = if radial_length > 1e-6 {
            radial_projection / radial_length * self.radius
        } else {
            // Direction is parallel to the axis: any rim point works. Pick a
            // stable perpendicular.
            let reference = if axis.x.abs() < 0.9 {
                Vec4::new(1.0, 0.0, 0.0, 0.0)
            } else {
                Vec4::new(0.0, 1.0, 0.0, 0.0)
            };
            let perpendicular = axis.cross3(reference).normalize_or_zero3();
            perpendicular * self.radius
        };

        let mut point = center + axial + radial;
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

    fn y_cylinder(radius: f32, half_height: f32) -> CylinderShape {
        CylinderShape::with_params(
            Vec4::from_xyz(0.0, -half_height, 0.0),
            Vec4::from_xyz(0.0, half_height, 0.0),
            radius,
            16,
            0.0,
        )
    }

    #[test]
    fn aabb_and_volume() {
        let cylinder = y_cylinder(1.0, 2.0);
        let aabb = cylinder.aabb();
        assert!(aabb.min.abs_diff_eq(Vec3A::new(-1.0, -2.0, -1.0), 0.0));
        assert!(aabb.max.abs_diff_eq(Vec3A::new(1.0, 2.0, 1.0), 0.0));
        assert_eq!(cylinder.height(), 4.0);
        let side = 16.0 * 1.0 * 1.0 * (2.0 * std::f32::consts::PI / 16.0).sin() * 0.5;
        assert!((cylinder.volume().unwrap() - side * 4.0).abs() < 1e-5);
    }

    #[test]
    fn radial_support() {
        let cylinder = y_cylinder(1.0, 2.0);
        let support = cylinder.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap();
        assert_eq!(support, Vec4::from_xyz(1.0, 2.0, 0.0));

        let support = cylinder.support(Vec4::new(0.0, 0.0, 1.0, 0.0)).unwrap();
        assert_eq!(support, Vec4::from_xyz(0.0, 2.0, 1.0));

        // Downward radial direction selects the lower cap.
        let support = cylinder.support(Vec4::from_xyz(0.0, 0.0, -1.0)).unwrap();
        assert_eq!(support.z, -1.0);
        // The axial component is unconstrained for a purely radial direction;
        // the port keeps the +axis convention (dot == 0).
        assert_eq!(support.y, 2.0);
    }

    #[test]
    fn axial_support_uses_finite_cap() {
        let cylinder = y_cylinder(1.0, 2.0);
        let support = cylinder.support(Vec4::new(0.0, 1.0, 0.0, 0.0)).unwrap();
        assert!((support.x * support.x + support.z * support.z - 1.0).abs() < 1e-5);
        assert_eq!(support.y, 2.0);
    }

    #[test]
    fn convex_radius_adds_to_support() {
        let cylinder = CylinderShape::with_params(
            Vec4::from_xyz(0.0, -1.0, 0.0),
            Vec4::from_xyz(0.0, 1.0, 0.0),
            1.0,
            16,
            0.5,
        );
        let support = cylinder.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap();
        assert_eq!(support, Vec4::from_xyz(1.5, 1.0, 0.0));
    }

    #[test]
    fn degenerate_cylinder_is_a_sphere() {
        let cylinder = CylinderShape::with_params(Vec4::ZERO, Vec4::ZERO, 2.0, 16, 0.0);
        let support = cylinder.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap();
        assert_eq!(support, Vec4::from_xyz(2.0, 0.0, 0.0));
        assert_eq!(cylinder.height(), 0.0);
        assert_eq!(cylinder.volume().unwrap(), 0.0);
    }
}
