//! Convex hull / convex polytope (`hknpConvexHull`, `hknpConvexPolytopeShape`).
//!
//! Simplified port of `Collide/Shape/Convex/hknpConvexHull.{h,inl}`. The SDK
//! stores the hull vertices, planes, face/edge connectivity and a convex radius;
//! this port keeps the vertex list and optional triangle faces (used for volume
//! and, later, for face queries), plus the convex radius.
//!
//! A hull with no faces still supports AABB/support queries and reports an
//! unknown volume. [`ConvexHullShape::try_from_points`] builds the vertex and
//! face set from a point cloud through `base-geometry`; lower-dimensional
//! inputs keep the no-face fallback. The C++ SDK performs this step at load
//! time through its internal geometry-processing utilities.

use core::any::Any;

use glam::{Mat3, Quat, Vec4};

use crate::base_geometry::convex_hull::{build_convex_hull, build_convex_hull_with_tolerance};
use crate::base_math::{Aabb, Vec4Ext};
use crate::phys_types::{CollisionDispatchType, ShapeFlags, ShapeType};

pub use crate::base_geometry::convex_hull::HullError as ConvexHullBuildError;

use crate::phys_shape::mass::mass_from_aabb;
use crate::phys_shape::{
    DEFAULT_CONVEX_RADIUS, MassConfig, MassError, MassProperties, Shape, aabb_bounding_radius,
    outer_product, unit_direction,
};

/// Convex hull (`hknpConvexHull` / `hknpConvexPolytopeShape`).
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexHullShape {
    vertices: Vec<Vec4>,
    faces: Vec<[u32; 3]>,
    convex_radius: f32,
}

impl ConvexHullShape {
    /// Construct a hull from vertices and (optionally) outward-oriented triangle
    /// faces.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if the vertex list is empty, a face index is out of
    /// range, or the convex radius is negative.
    pub fn new(vertices: Vec<Vec4>, faces: Vec<[u32; 3]>, convex_radius: f32) -> Self {
        debug_assert!(!vertices.is_empty(), "convex hull needs vertices");
        debug_assert!(convex_radius >= 0.0, "invalid convex radius");
        for face in &faces {
            for &index in face {
                debug_assert!((index as usize) < vertices.len(), "face index out of range");
            }
        }
        Self {
            vertices,
            faces,
            convex_radius,
        }
    }

    /// Hull vertices.
    #[inline]
    pub fn vertices(&self) -> &[Vec4] {
        &self.vertices
    }

    /// Triangle faces (indices into [`ConvexHullShape::vertices`]).
    #[inline]
    pub fn faces(&self) -> &[[u32; 3]] {
        &self.faces
    }

    /// Convex radius.
    #[inline]
    pub fn convex_radius(&self) -> f32 {
        self.convex_radius
    }

    /// Build a hull from vertices only (no faces, volume unknown).
    #[inline]
    pub fn from_vertices(vertices: Vec<Vec4>) -> Self {
        Self::new(vertices, Vec::new(), DEFAULT_CONVEX_RADIUS)
    }

    /// Build a hull directly from a point cloud with the default weld tolerance.
    ///
    /// Returns an error for empty or non-finite input. Lower-dimensional
    /// clouds produce a valid shape with no faces, so [`Shape::volume`] stays
    /// `None` while AABB/support queries remain available.
    pub fn try_from_points(
        points: Vec<Vec4>,
        convex_radius: f32,
    ) -> Result<Self, ConvexHullBuildError> {
        let hull = build_convex_hull(&points)?;
        Ok(Self::new(
            hull.vertices().to_vec(),
            hull.faces().to_vec(),
            convex_radius,
        ))
    }

    /// Build a hull from a point cloud using an explicit weld tolerance.
    pub fn try_from_points_with_tolerance(
        points: Vec<Vec4>,
        convex_radius: f32,
        tolerance: f32,
    ) -> Result<Self, ConvexHullBuildError> {
        let hull = build_convex_hull_with_tolerance(&points, tolerance)?;
        Ok(Self::new(
            hull.vertices().to_vec(),
            hull.faces().to_vec(),
            convex_radius,
        ))
    }

    /// Convenience wrapper around [`ConvexHullShape::try_from_points`].
    ///
    /// # Panics
    ///
    /// Panics when the point cloud is empty or contains non-finite coordinates.
    #[inline]
    pub fn from_points(points: Vec<Vec4>, convex_radius: f32) -> Self {
        Self::try_from_points(points, convex_radius)
            .expect("failed to build convex hull from points")
    }
}

impl Shape for ConvexHullShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Convex
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Convex
    }

    fn flags(&self) -> ShapeFlags {
        let mut flags = ShapeFlags::IS_CONVEX_SHAPE;
        if !self.faces.is_empty() {
            flags.insert(ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE);
        }
        flags
    }

    fn aabb(&self) -> Aabb {
        let mut min = self.vertices[0];
        let mut max = self.vertices[0];
        for vertex in &self.vertices[1..] {
            min = min.min(*vertex);
            max = max.max(*vertex);
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
        if self.faces.is_empty() {
            return None;
        }
        let vertices = &self.vertices;
        let mut six_volume = 0.0f32;
        for &[a, b, c] in &self.faces {
            let pa = vertices[a as usize];
            let pb = vertices[b as usize];
            let pc = vertices[c as usize];
            six_volume += pa.dot3(pb.cross3(pc));
        }
        Some(six_volume.abs() / 6.0)
    }

    fn mass_properties(&self, config: MassConfig) -> Result<MassProperties, MassError> {
        if self.faces.is_empty() {
            return mass_from_aabb(&self.aabb(), config);
        }

        let vertices = &self.vertices;
        let mut signed_volume = 0.0f32;
        let mut center_integral = Vec4::ZERO;
        let mut second_moment = Mat3::ZERO;
        for &[a, b, c] in &self.faces {
            let pa = vertices[a as usize];
            let pb = vertices[b as usize];
            let pc = vertices[c as usize];
            let tetra_volume = pa.dot3(pb.cross3(pc)) / 6.0;
            if tetra_volume == 0.0 {
                continue;
            }
            let sum = pa + pb + pc;
            let squares =
                outer_product(pa, pa) + outer_product(pb, pb) + outer_product(pc, pc);
            signed_volume += tetra_volume;
            center_integral += sum * (tetra_volume / 4.0);
            second_moment += (outer_product(sum, sum) + squares) * (tetra_volume / 20.0);
        }

        if signed_volume.abs() <= f32::EPSILON {
            return mass_from_aabb(&self.aabb(), config);
        }
        let sign = if signed_volume < 0.0 { -1.0 } else { 1.0 };
        let center_of_mass = center_integral / signed_volume;
        let second_moment = second_moment * sign;
        let trace = second_moment.x_axis.x + second_moment.y_axis.y + second_moment.z_axis.z;
        let inertia_origin = Mat3::from_diagonal(glam::Vec3::splat(trace)) - second_moment;
        let distance_squared = center_of_mass.dot3(center_of_mass);
        let shift = Mat3::from_diagonal(glam::Vec3::splat(distance_squared))
            - outer_product(center_of_mass, center_of_mass);
        let inertia = inertia_origin - shift * signed_volume.abs();

        MassProperties::from_geometry(
            signed_volume.abs(),
            center_of_mass,
            inertia,
            Quat::IDENTITY,
            config,
        )
    }

    fn support(&self, direction: Vec4) -> Option<Vec4> {
        let mut best = self.vertices[0];
        let mut best_dot = best.dot3(direction);
        for vertex in &self.vertices[1..] {
            let dot = vertex.dot3(direction);
            if dot > best_dot {
                best = *vertex;
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

    fn cube_vertices(half: f32) -> Vec<Vec4> {
        vec![
            Vec4::new(-half, -half, -half, 0.0),
            Vec4::new(half, -half, -half, 0.0),
            Vec4::new(half, half, -half, 0.0),
            Vec4::new(-half, half, -half, 0.0),
            Vec4::new(-half, -half, half, 0.0),
            Vec4::new(half, -half, half, 0.0),
            Vec4::new(half, half, half, 0.0),
            Vec4::new(-half, half, half, 0.0),
        ]
    }

    fn cube_faces() -> Vec<[u32; 3]> {
        vec![
            [0, 2, 1],
            [0, 3, 2], // bottom
            [4, 5, 6],
            [4, 6, 7], // top
            [0, 1, 5],
            [0, 5, 4], // side
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ]
    }

    #[test]
    fn cube_volume_support_and_aabb() {
        let hull = ConvexHullShape::new(cube_vertices(1.0), cube_faces(), 0.0);
        assert!((hull.volume().unwrap() - 8.0).abs() < 1e-4);
        // Equal-dot vertices are resolved by list order; assert the extreme
        // coordinate rather than one particular tie-winner.
        let support = hull.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap();
        assert_eq!(support.x, 1.0);
        assert!(support.y.abs() <= 1.0 && support.z.abs() <= 1.0);
        let support = hull.support(Vec4::new(0.0, 1.0, 0.0, 0.0)).unwrap();
        assert_eq!(support.y, 1.0);
        let aabb = hull.aabb();
        assert!(aabb.min.abs_diff_eq(Vec3A::splat(-1.0), 0.0));
        assert!(aabb.max.abs_diff_eq(Vec3A::splat(1.0), 0.0));
        assert!(hull.flags().contains(ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE));
    }

    #[test]
    fn convex_radius_inflates_aabb_and_support() {
        let hull = ConvexHullShape::new(cube_vertices(1.0), cube_faces(), 0.5);
        assert_eq!(hull.aabb().max.x, 1.5);
        let support = hull.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap();
        assert_eq!(support.x, 1.5);
        // Volume excludes the rounding.
        assert!((hull.volume().unwrap() - 8.0).abs() < 1e-4);
    }

    #[test]
    fn vertices_without_faces_have_unknown_volume() {
        let hull = ConvexHullShape::from_vertices(cube_vertices(2.0));
        assert_eq!(hull.volume(), None);
        assert!(!hull.flags().contains(ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE));
        assert_eq!(hull.aabb().max.x, 2.0 + DEFAULT_CONVEX_RADIUS);
    }

    #[test]
    fn tetrahedron_volume() {
        let vertices = vec![
            Vec4::ZERO,
            Vec4::from_xyz(1.0, 0.0, 0.0),
            Vec4::from_xyz(0.0, 1.0, 0.0),
            Vec4::from_xyz(0.0, 0.0, 1.0),
        ];
        let faces = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
        let hull = ConvexHullShape::new(vertices, faces, 0.0);
        assert!((hull.volume().unwrap() - 1.0 / 6.0).abs() < 1e-6);
    }

    #[test]
    fn builds_cube_from_point_cloud() {
        let mut points = cube_vertices(1.0);
        points.extend(cube_vertices(1.0));
        points.push(Vec4::ZERO);
        let hull = ConvexHullShape::from_points(points, 0.0);
        assert_eq!(hull.vertices().len(), 8);
        assert_eq!(hull.faces().len(), 12);
        assert!((hull.volume().unwrap() - 8.0).abs() < 1e-3);
        assert!(hull.flags().contains(ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE));
        let mass = hull.mass_properties(MassConfig::DEFAULT).unwrap();
        assert!((mass.volume - 8.0).abs() < 1e-3);
    }

    #[test]
    fn point_cloud_lower_dimension_has_unknown_volume() {
        let hull = ConvexHullShape::from_points(vec![Vec4::ZERO, Vec4::new(1.0, 0.0, 0.0, 0.0)], 0.0);
        assert_eq!(hull.volume(), None);
        assert!(!hull.flags().contains(ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE));
    }

    #[test]
    fn point_cloud_errors_are_reported() {
        assert!(ConvexHullShape::try_from_points(Vec::new(), 0.0).is_err());
        assert!(
            ConvexHullShape::try_from_points(vec![Vec4::new(f32::NAN, 0.0, 0.0, 0.0)], 0.0)
                .is_err()
        );
    }
}
