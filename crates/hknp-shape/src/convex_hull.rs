//! Convex hull / convex polytope (`hknpConvexHull`, `hknpConvexPolytopeShape`).
//!
//! Simplified port of `Collide/Shape/Convex/hknpConvexHull.{h,inl}`. The SDK
//! stores the hull vertices, planes, face/edge connectivity and a convex radius;
//! this port keeps the vertex list and optional triangle faces (used for volume
//! and, later, for face queries), plus the convex radius.
//!
//! A hull with no faces still supports AABB/support queries; its volume is
//! `None` because the port does not compute a hull from the vertex cloud (the C++
//! shape is built by `hknpConvexHullUtil` at load time).

use core::any::Any;

use hk_math::{Aabb, Vector4};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::{DEFAULT_CONVEX_RADIUS, Shape, aabb_bounding_radius, unit_direction};

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
}
