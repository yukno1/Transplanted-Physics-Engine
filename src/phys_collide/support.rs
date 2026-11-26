//! Support-function plumbing shared by GJK and EPA.

use crate::base_math::{Transform, Vector4};
use crate::phys_shape::{Shape, to_vec4, to_vector4};

/// World-space support point of a convex shape in world direction `direction`.
///
/// Returns `None` for shapes without a support function (composites, meshes).
///
/// 过渡期边界：`Shape::support` 已迁移 glam（Phase 3 步骤 2），本模块随
/// phys_collide 迁移（步骤 4）时删除转换。
pub(crate) fn world_support(
    shape: &dyn Shape,
    transform: &Transform,
    direction: Vector4,
) -> Option<Vector4> {
    let local_direction = transform.rotation.rotate_inverse(direction);
    shape
        .support(to_vec4(local_direction))
        .map(|point| transform.transform_point(to_vector4(point)))
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
