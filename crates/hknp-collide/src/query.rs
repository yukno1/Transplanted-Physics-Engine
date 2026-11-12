//! Query interface: ray casts and point-to-shape distance.
//!
//! Subset of `original/Physics/Source/Physics/Physics/Collide/Query/`
//! (migration follow-up to Phase 7, README §4).
//!
//! Ray casts against convex shapes use **conservative advancement**: the ray
//! point is advanced towards the shape by the GJK distance divided by the ray
//! alignment with the surface normal, until contact or a miss. This works for
//! every shape with a support function (sphere, box, capsule, cylinder,
//! triangle, convex hull) through one code path; the query shape itself is
//! exposed as a degenerate point via GJK.
//!
//! # Conventions
//!
//! * `direction` is normalized defensively; `max_length` is measured in world
//!   units along the normalized direction.
//! * `RayHit::hit_fraction` is the distance from `origin` to the hit, in
//!   `[0, max_length]`; `position` is on the shape surface and `normal` is the
//!   outward surface normal.
//! * A ray starting inside the shape hits at fraction `0` with `normal`
//!   approximated by `-direction`.
//! * Shapes without a support function (meshes/compounds/height fields) return
//!   `None`; traversal of composite children is future work.

use core::any::Any;

use hk_math::{Aabb, Transform, Vector4};
use hknp_shape::Shape;
use hknp_types::ShapeType;

use crate::gjk::{GjkResult, closest_points};

/// Maximum conservative-advancement steps before giving up.
const MAX_ADVANCE_STEPS: usize = 32;
/// Contact threshold for the ray point.
const HIT_EPSILON: f32 = 1e-5;

/// A ray/shape intersection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Distance from the ray origin to the hit along the (normalized) ray.
    pub hit_fraction: f32,
    /// Hit point on the shape surface.
    pub position: Vector4,
    /// Outward surface normal at the hit.
    pub normal: Vector4,
}

/// Cast a ray against a convex shape.
///
/// Returns `None` when the shape has no support function, the direction is
/// degenerate, or the ray misses within `max_length`.
pub fn raycast(
    shape: &dyn Shape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    if !shape.is_convex() {
        return None;
    }
    let direction_length = direction.length3();
    if direction_length <= 1e-10 || max_length <= 0.0 {
        return None;
    }
    let direction = direction / direction_length;

    let mut fraction = 0.0;
    for _ in 0..MAX_ADVANCE_STEPS {
        let point = origin + direction * fraction;
        let result = closest_points(&PointShape(point), &Transform::IDENTITY, shape, transform)?;

        if result.intersects {
            // The ray point is inside the shape; this can only be the origin.
            return Some(RayHit {
                hit_fraction: fraction,
                position: point,
                normal: -direction,
            });
        }
        if result.distance <= HIT_EPSILON {
            return Some(RayHit {
                hit_fraction: fraction,
                position: result.point_b,
                normal: result.normal,
            });
        }

        // Advance by the distance divided by the closing speed; a non-negative
        // alignment means the ray points away from the shape.
        let alignment = direction.dot3(result.normal);
        if alignment >= -1e-8 {
            return None;
        }
        let advance = result.distance / -alignment;
        if advance <= HIT_EPSILON {
            return None;
        }
        fraction += advance;
        if fraction > max_length {
            return None;
        }
    }
    None
}

/// GJK distance between a world-space point and a shape.
///
/// Used by world-level closest-point queries; returns `None` for shapes
/// without a support function. The result's `normal` points from the shape
/// surface towards `point`, `point_b` is the witness on the shape, and
/// `intersects` is set when `point` is inside the shape.
pub fn point_distance(
    shape: &dyn Shape,
    transform: &Transform,
    point: Vector4,
) -> Option<GjkResult> {
    closest_points(&PointShape(point), &Transform::IDENTITY, shape, transform)
}

/// Ray/AABB slab test over the world axes.
///
/// Returns the entry/exit fractions `(tmin, tmax)` of the infinite ray through
/// the box; a negative `tmin` means the origin lies inside the box. Returns
/// `None` when the ray misses. Axis order is fixed (x, y, z) for determinism;
/// callers clip the interval against their own range.
pub fn ray_aabb(origin: Vector4, direction: Vector4, aabb: &Aabb) -> Option<(f32, f32)> {
    let mut tmin = f32::NEG_INFINITY;
    let mut tmax = f32::INFINITY;
    for axis in 0..3 {
        let origin_component = origin.component(axis);
        let direction_component = direction.component(axis);
        let (min, max) = (aabb.min.component(axis), aabb.max.component(axis));
        if direction_component.abs() <= 1e-12 {
            if origin_component < min || origin_component > max {
                return None;
            }
        } else {
            let inverse = 1.0 / direction_component;
            let (mut entry, mut exit) = (
                (min - origin_component) * inverse,
                (max - origin_component) * inverse,
            );
            if entry > exit {
                core::mem::swap(&mut entry, &mut exit);
            }
            tmin = tmin.max(entry);
            tmax = tmax.min(exit);
            if tmin > tmax {
                return None;
            }
        }
    }
    Some((tmin, tmax))
}

/// Degenerate convex shape that is a single world-space point, used to run
/// point-vs-shape queries through the GJK pipeline.
#[derive(Clone, Copy, Debug)]
struct PointShape(Vector4);

impl Shape for PointShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Convex
    }

    fn aabb(&self) -> Aabb {
        Aabb::from_point(self.0)
    }

    fn bounding_radius(&self) -> f32 {
        0.0
    }

    fn volume(&self) -> Option<f32> {
        Some(0.0)
    }

    fn support(&self, _direction: Vector4) -> Option<Vector4> {
        Some(self.0)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;
    use hknp_shape::{BoxShape, CapsuleShape, ConvexHullShape, SphereShape};
    use std::sync::Arc;

    fn sphere(radius: f32, center: Vector4) -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(center, radius)) as Arc<dyn Shape>
    }

    fn box_shape(half: f32) -> Arc<dyn Shape> {
        Arc::new(BoxShape::with_convex_radius(Vector4::splat(half), 0.0)) as Arc<dyn Shape>
    }

    #[test]
    fn ray_hits_sphere_from_outside() {
        let shape = sphere(1.0, Vector4::from_xyz(0.0, 5.0, 0.0));
        let hit = raycast(
            shape.as_ref(),
            &Transform::IDENTITY,
            Vector4::ZERO,
            Vector4::UNIT_Y,
            10.0,
        )
        .expect("ray must hit the sphere");
        assert!(
            (hit.hit_fraction - 4.0).abs() < 1e-3,
            "fraction {}",
            hit.hit_fraction
        );
        assert!(
            hit.position
                .all_equal3(Vector4::from_xyz(0.0, 4.0, 0.0), 1e-3),
            "position {:?}",
            hit.position
        );
        assert!(
            hit.normal.all_equal3(-Vector4::UNIT_Y, 1e-3),
            "normal {:?}",
            hit.normal
        );
    }

    #[test]
    fn ray_misses_sphere_within_range() {
        let shape = sphere(1.0, Vector4::from_xyz(3.0, 5.0, 0.0));
        assert!(
            raycast(
                shape.as_ref(),
                &Transform::IDENTITY,
                Vector4::ZERO,
                Vector4::UNIT_Y,
                10.0,
            )
            .is_none()
        );
    }

    #[test]
    fn ray_hits_sphere_from_inside() {
        let shape = sphere(1.0, Vector4::ZERO);
        let hit = raycast(
            shape.as_ref(),
            &Transform::IDENTITY,
            Vector4::ZERO,
            Vector4::UNIT_Y,
            5.0,
        )
        .expect("inside ray must hit immediately");
        assert_eq!(hit.hit_fraction, 0.0);
        assert!(hit.normal.dot3(Vector4::UNIT_Y) < 0.0);
    }

    #[test]
    fn ray_hits_rotated_box() {
        let shape = box_shape(1.0);
        let rotation = Quaternion::from_axis_angle(Vector4::UNIT_Z, core::f32::consts::FRAC_PI_4);
        let transform = Transform::from_quaternion(rotation, Vector4::ZERO);
        // A ray along +Y towards the rotated box still starts inside the box's
        // AABB, so cast from above the corner.
        let hit = raycast(
            shape.as_ref(),
            &transform,
            Vector4::from_xyz(0.0, 5.0, 0.0),
            -Vector4::UNIT_Y,
            10.0,
        )
        .expect("ray must hit the box");
        assert!(hit.hit_fraction < 5.0 - 1.0 + 1e-3);
        assert!(
            hit.hit_fraction > 5.0 - 1.5 - 1e-3,
            "fraction {}",
            hit.hit_fraction
        );
        assert!(
            hit.normal.dot3(Vector4::UNIT_Y) > 0.9,
            "normal {:?}",
            hit.normal
        );
    }

    #[test]
    fn ray_hits_capsule() {
        let shape: Arc<dyn Shape> = Arc::new(CapsuleShape::new(
            Vector4::from_xyz(0.0, 0.0, 0.0),
            Vector4::from_xyz(0.0, 2.0, 0.0),
            0.5,
        ));
        let hit = raycast(
            shape.as_ref(),
            &Transform::IDENTITY,
            Vector4::from_xyz(0.0, 10.0, 0.0),
            -Vector4::UNIT_Y,
            20.0,
        )
        .expect("ray must hit the capsule cap");
        assert!(
            (hit.hit_fraction - (10.0 - 2.5)).abs() < 1e-2,
            "fraction {}",
            hit.hit_fraction
        );
    }

    #[test]
    fn ray_hits_convex_hull() {
        // Octahedron around the origin.
        let vertices = vec![
            Vector4::from_xyz(1.0, 0.0, 0.0),
            Vector4::from_xyz(-1.0, 0.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            Vector4::from_xyz(0.0, -1.0, 0.0),
            Vector4::from_xyz(0.0, 0.0, 1.0),
            Vector4::from_xyz(0.0, 0.0, -1.0),
        ];
        let shape: Arc<dyn Shape> = Arc::new(ConvexHullShape::new(vertices, Vec::new(), 0.0));
        let hit = raycast(
            shape.as_ref(),
            &Transform::IDENTITY,
            Vector4::from_xyz(0.0, 5.0, 0.0),
            -Vector4::UNIT_Y,
            10.0,
        )
        .expect("ray must hit the hull");
        assert!(
            (hit.hit_fraction - 4.0).abs() < 1e-2,
            "fraction {}",
            hit.hit_fraction
        );
    }

    #[test]
    fn ray_respects_max_length() {
        let shape = sphere(1.0, Vector4::from_xyz(0.0, 5.0, 0.0));
        assert!(
            raycast(
                shape.as_ref(),
                &Transform::IDENTITY,
                Vector4::ZERO,
                Vector4::UNIT_Y,
                3.5,
            )
            .is_none()
        );
        assert!(
            raycast(
                shape.as_ref(),
                &Transform::IDENTITY,
                Vector4::ZERO,
                Vector4::UNIT_Y,
                4.5,
            )
            .is_some()
        );
    }

    #[test]
    fn ray_aabb_slab_test() {
        let aabb =
            Aabb::from_center_half_extents(Vector4::from_xyz(0.0, 5.0, 0.0), Vector4::splat(1.0));
        let (entry, exit) = ray_aabb(Vector4::ZERO, Vector4::UNIT_Y, &aabb).unwrap();
        assert!((entry - 4.0).abs() < 1e-5);
        assert!((exit - 6.0).abs() < 1e-5);

        // Miss off to the side.
        assert!(ray_aabb(Vector4::from_xyz(5.0, 0.0, 0.0), Vector4::UNIT_Y, &aabb).is_none());

        // Parallel and inside along the ray axis.
        let (entry, exit) =
            ray_aabb(Vector4::from_xyz(0.0, 5.5, 0.0), Vector4::UNIT_X, &aabb).unwrap();
        assert!((entry - -1.0).abs() < 1e-5);
        assert!((exit - 1.0).abs() < 1e-5);
    }

    #[test]
    fn point_distance_reports_surface_and_inside() {
        let shape = sphere(1.0, Vector4::ZERO);

        let result = point_distance(
            shape.as_ref(),
            &Transform::IDENTITY,
            Vector4::from_xyz(3.0, 0.0, 0.0),
        )
        .unwrap();
        assert!((result.distance - 2.0).abs() < 1e-3);
        assert!(result.point_b.all_equal3(Vector4::UNIT_X, 1e-3));
        assert!(!result.intersects);

        let result = point_distance(shape.as_ref(), &Transform::IDENTITY, Vector4::ZERO).unwrap();
        assert!(result.intersects);
    }
}
