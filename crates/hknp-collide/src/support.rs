//! Support-function plumbing shared by GJK and EPA.

use hk_math::{Transform, Vector4};
use hknp_shape::Shape;

/// World-space support point of a convex shape in world direction `direction`.
///
/// Returns `None` for shapes without a support function (composites, meshes).
pub(crate) fn world_support(
    shape: &dyn Shape,
    transform: &Transform,
    direction: Vector4,
) -> Option<Vector4> {
    let local_direction = transform.rotation.rotate_inverse(direction);
    shape
        .support(local_direction)
        .map(|point| transform.transform_point(point))
}

/// Both sides of a Minkowski-difference support point.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SupportVertex {
    /// `support_a(direction) - support_b(-direction)`.
    pub minkowski: Vector4,
    /// Witness point on shape A.
    pub point_a: Vector4,
    /// Witness point on shape B.
    pub point_b: Vector4,
}

/// Compute a Minkowski-difference support vertex in `direction`.
pub(crate) fn support_vertex(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
    direction: Vector4,
) -> Option<SupportVertex> {
    let point_a = world_support(shape_a, transform_a, direction)?;
    let point_b = world_support(shape_b, transform_b, -direction)?;
    Some(SupportVertex {
        minkowski: point_a - point_b,
        point_a,
        point_b,
    })
}
