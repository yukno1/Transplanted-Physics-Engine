//! Support-function plumbing shared by GJK and EPA.
//!
//! `Transform` stays legacy per the migration boundary convention
//! (`docs/refactor-glam-plan.md` 步骤 2 惯例①)；本模块在边界把 rotation/translation
//! 逐位转成 glam 后全程 glam 运算，公式与 legacy `Matrix3::mul_vec` /
//! `Matrix3::rotate_inverse` / `Transform::transform_point` 逐位一致。

use glam::{Mat3, Vec3, Vec4};

use crate::base_math::{Mat3Ext, Transform, Vec4Ext};
use crate::phys_shape::{Shape, to_mat3, to_vec4};

/// legacy `Matrix3::rotate_inverse`（`matrix3.rs:133-135`）的 glam 版本：
/// `xyz` 分量为各列与 `v` 的 `dot3`（固定加法顺序），`w` 恒为零。
#[inline]
pub(crate) fn inverse_rotate(rotation: Mat3, v: Vec4) -> Vec4 {
    Vec4::from_xyz(
        rotation.x_axis.dot(v.truncate()),
        rotation.y_axis.dot(v.truncate()),
        rotation.z_axis.dot(v.truncate()),
    )
}

/// legacy `Transform::transform_point`（`transform.rs:88-90`，
/// `R * p + t` 列展开顺序）的 glam 版本，逐位一致。
#[inline]
pub(crate) fn transform_point(transform: &Transform, rotation: Mat3, point: Vec4) -> Vec4 {
    let rotated = rotation.mul_vec_sdk(Vec3::new(point.x, point.y, point.z));
    Vec4::new(rotated.x, rotated.y, rotated.z, 0.0) + to_vec4(transform.translation)
}

/// World-space support point of a convex shape in world direction `direction`.
///
/// Returns `None` for shapes without a support function (composites, meshes).
pub(crate) fn world_support(
    shape: &dyn Shape,
    transform: &Transform,
    direction: Vec4,
) -> Option<Vec4> {
    let rotation = to_mat3(&transform.rotation);
    let local_direction = inverse_rotate(rotation, direction);
    shape
        .support(local_direction)
        .map(|point| transform_point(transform, rotation, point))
}

/// Both sides of a Minkowski-difference support point.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SupportVertex {
    /// `support_a(direction) - support_b(-direction)`.
    pub minkowski: Vec4,
    /// Witness point on shape A.
    pub point_a: Vec4,
    /// Witness point on shape B.
    pub point_b: Vec4,
}

/// Compute a Minkowski-difference support vertex in `direction`.
pub(crate) fn support_vertex(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
    direction: Vec4,
) -> Option<SupportVertex> {
    let point_a = world_support(shape_a, transform_a, direction)?;
    let point_b = world_support(shape_b, transform_b, -direction)?;
    Some(SupportVertex {
        minkowski: point_a - point_b,
        point_a,
        point_b,
    })
}
