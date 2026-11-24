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

use crate::base_geometry::convex_hull::{build_convex_hull, build_convex_hull_with_tolerance};
use crate::base_math::{Aabb, Matrix3, Quaternion, Vector4};
use crate::phys_types::{CollisionDispatchType, ShapeFlags, ShapeType};

pub use crate::base_geometry::convex_hull::HullError as ConvexHullBuildError;

use crate::phys_shape::mass::{
    mass_from_aabb, matrix_add, matrix_diagonal, matrix_outer, matrix_scale, matrix_sub,
    matrix_trace,
};
use crate::phys_shape::{
    DEFAULT_CONVEX_RADIUS, MassConfig, MassError, MassProperties, Shape, aabb_bounding_radius,
    unit_direction,
};

/// Convex hull (`hknpConvexHull` / `hknpConvexPolytopeShape`).
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexHullShape {
    vertices: Vec<Vector4>,
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
    pub fn new(vertices: Vec<Vector4>, faces: Vec<[u32; 3]>, convex_radius: f32) -> Self {
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
    pub fn vertices(&self) -> &[Vector4] {
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
    pub fn from_vertices(vertices: Vec<Vector4>) -> Self {
        Self::new(vertices, Vec::new(), DEFAULT_CONVEX_RADIUS)
    }

    /// Build a hull directly from a point cloud with the default weld tolerance.
    ///
    /// Returns an error for empty or non-finite input. Lower-dimensional
    /// clouds produce a valid shape with no faces, so [`Shape::volume`] stays
    /// `None` while AABB/support queries remain available.
    pub fn try_from_points(
        points: Vec<Vector4>,
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
        points: Vec<Vector4>,
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
    pub fn from_points(points: Vec<Vector4>, convex_radius: f32) -> Self {
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
        Aabb::new(
            Vector4::from_xyz(min.x - r, min.y - r, min.z - r),
            Vector4::from_xyz(max.x + r, max.y + r, max.z + r),
        )
    }

    fn bounding_radius(&self) -> f32 {
        aabb_bounding_radius(&self.aabb())
    }

    fn volume(&self) -> Option<f32> {
        if self.faces.is_empty() {
            return None;
        }
        let mut six_volume = 0.0f32;
        for &[a, b, c] in &self.faces {
            let pa = self.vertices[a as usize];
            let pb = self.vertices[b as usize];
            let pc = self.vertices[c as usize];
            six_volume += pa.dot3(pb.cross(pc));
        }
        Some(six_volume.abs() / 6.0)
    }

    fn mass_properties(&self, config: MassConfig) -> Result<MassProperties, MassError> {
        if self.faces.is_empty() {
            return mass_from_aabb(&self.aabb(), config);
        }

        let mut signed_volume = 0.0f32;
        let mut center_integral = Vector4::ZERO;
        let mut second_moment = Matrix3::ZERO;
        for &[a, b, c] in &self.faces {
            let pa = self.vertices[a as usize];
            let pb = self.vertices[b as usize];
            let pc = self.vertices[c as usize];
            let tetra_volume = pa.dot3(pb.cross(pc)) / 6.0;
            if tetra_volume == 0.0 {
                continue;
            }
            let sum = pa + pb + pc;
            let squares = matrix_add(
                &matrix_add(&matrix_outer(pa, pa), &matrix_outer(pb, pb)),
                &matrix_outer(pc, pc),
            );
            signed_volume += tetra_volume;
            center_integral += sum * (tetra_volume / 4.0);
            second_moment = matrix_add(
                &second_moment,
                &matrix_scale(
                    matrix_add(&matrix_outer(sum, sum), &squares),
                    tetra_volume / 20.0,
                ),
            );
        }

        if signed_volume.abs() <= f32::EPSILON {
            return mass_from_aabb(&self.aabb(), config);
        }
        let sign = if signed_volume < 0.0 { -1.0 } else { 1.0 };
        let center_of_mass = center_integral / signed_volume;
        let second_moment = matrix_scale(second_moment, sign);
        let inertia_origin = matrix_sub(
            &matrix_diagonal(Vector4::splat(matrix_trace(second_moment))),
            &second_moment,
        );
        let distance_squared = center_of_mass.dot3(center_of_mass);
        let shift = matrix_sub(
            &matrix_diagonal(Vector4::splat(distance_squared)),
            &matrix_outer(center_of_mass, center_of_mass),
        );
        let inertia = matrix_sub(&inertia_origin, &matrix_scale(shift, signed_volume.abs()));

        MassProperties::from_geometry(
            signed_volume.abs(),
            center_of_mass,
            inertia,
            Quaternion::IDENTITY,
            config,
        )
    }

    fn support(&self, direction: Vector4) -> Option<Vector4> {
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

    fn cube_vertices(half: f32) -> Vec<Vector4> {
        vec![
            Vector4::new(-half, -half, -half, 0.0),
            Vector4::new(half, -half, -half, 0.0),
            Vector4::new(half, half, -half, 0.0),
            Vector4::new(-half, half, -half, 0.0),
            Vector4::new(-half, -half, half, 0.0),
            Vector4::new(half, -half, half, 0.0),
            Vector4::new(half, half, half, 0.0),
            Vector4::new(-half, half, half, 0.0),
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
        let support = hull.support(Vector4::UNIT_X).unwrap();
        assert_eq!(support.x, 1.0);
        assert!(support.y.abs() <= 1.0 && support.z.abs() <= 1.0);
        let support = hull.support(Vector4::UNIT_Y).unwrap();
        assert_eq!(support.y, 1.0);
        let aabb = hull.aabb();
        assert!(aabb.min.all_equal3(Vector4::splat(-1.0), 0.0));
        assert!(aabb.max.all_equal3(Vector4::splat(1.0), 0.0));
        assert!(hull.flags().contains(ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE));
    }

    #[test]
    fn convex_radius_inflates_aabb_and_support() {
        let hull = ConvexHullShape::new(cube_vertices(1.0), cube_faces(), 0.5);
        assert_eq!(hull.aabb().max.x, 1.5);
        let support = hull.support(Vector4::UNIT_X).unwrap();
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
            Vector4::ZERO,
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            Vector4::from_xyz(0.0, 0.0, 1.0),
        ];
        let faces = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
        let hull = ConvexHullShape::new(vertices, faces, 0.0);
        assert!((hull.volume().unwrap() - 1.0 / 6.0).abs() < 1e-6);
    }

    #[test]
    fn builds_cube_from_point_cloud() {
        let mut points = cube_vertices(1.0);
        points.extend(cube_vertices(1.0));
        points.push(Vector4::ZERO);
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
        let hull = ConvexHullShape::from_points(vec![Vector4::ZERO, Vector4::UNIT_X], 0.0);
        assert_eq!(hull.volume(), None);
        assert!(!hull.flags().contains(ShapeFlags::IS_CONVEX_POLYTOPE_SHAPE));
    }

    #[test]
    fn point_cloud_errors_are_reported() {
        assert!(ConvexHullShape::try_from_points(Vec::new(), 0.0).is_err());
        assert!(
            ConvexHullShape::try_from_points(vec![Vector4::new(f32::NAN, 0.0, 0.0, 0.0)], 0.0)
                .is_err()
        );
    }
}
