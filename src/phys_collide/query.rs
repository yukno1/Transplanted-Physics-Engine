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
//! Composite shapes are traversed with their acceleration structures:
//! [`MeshShape`] uses its BVH over triangle ranges, [`HeightFieldShape`] walks
//! the grid cells inside the ray interval and [`CompoundShape`] recurses into
//! its children with composed transforms. Leaf triangles use the analytic
//! two-sided Möller–Trumbore test.
//!
//! # Conventions
//!
//! * `direction` is normalized defensively; `max_length` is measured in world
//!   units along the normalized direction.
//! * `RayHit::hit_fraction` is the distance from `origin` to the hit, in
//!   `[0, max_length]`; `position` is on the shape surface and `normal` is the
//!   outward surface normal (flipped towards the ray for triangle hits).
//! * A ray starting inside a convex shape hits at fraction `0` with `normal`
//!   approximated by `-direction`.
//! * Mesh/height-field/compound targets return the closest leaf hit; ties are
//!   broken by leaf order so results are deterministic.

use core::any::Any;

use crate::base_math::{Aabb, Transform, Vector4};
use crate::phys_shape::{
    BoxShape, CapsuleShape, CompoundShape, ConvexHullShape, CylinderShape, HeightFieldShape,
    MeshShape, Shape, SphereShape,
};
use crate::phys_types::{ShapeKey, ShapeType};

use crate::phys_collide::gjk::{GjkResult, closest_points};

/// Deterministic query-side leaf key kinds.
///
/// These are portable identifiers for diagnostic/trigger granularity; they are
/// **not** claimed to be bit-identical to SDK `hknpShapeKey` values.
pub mod leaf_key {
    use crate::phys_types::ShapeKey;

    /// Direct convex hit (no composite leaf).
    pub const CONVEX: u32 = 0;
    /// Mesh triangle leaf.
    pub const MESH: u32 = 1;
    /// Height-field cell/sub-triangle leaf.
    pub const HEIGHT_FIELD: u32 = 2;
    /// Compound child (combined with its nested leaf key).
    pub const COMPOUND: u32 = 3;
    /// A face of a zero-radius convex hull (index is the face index).
    pub const CONVEX_HULL_FACE: u32 = 4;

    /// Encode a leaf kind and index into a stable [`ShapeKey`].
    pub fn new(kind: u32, index: u32) -> ShapeKey {
        ShapeKey::new(((kind & 0xff) << 24) | (index & 0x00ff_ffff))
    }

    /// Encode a height-field cell/sub-triangle leaf.
    pub fn height_field(cell_z: usize, cell_x: usize, sub_triangle: usize) -> ShapeKey {
        new(
            HEIGHT_FIELD,
            ((cell_z as u32) << 18) | ((cell_x as u32) << 1) | (sub_triangle as u32 & 1),
        )
    }

    /// Combine a compound child index with its nested leaf key.
    pub fn compound_child(child_index: usize, inner: ShapeKey) -> ShapeKey {
        let index = ((child_index as u32 & 0xff) << 16) | (inner.raw() & 0xffff);
        new(COMPOUND, index)
    }
}

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
    /// Leaf shape key (face index for a zero-radius convex hull, otherwise
    /// invalid for direct convex hits).
    pub shape_key: ShapeKey,
}

/// Cast a ray against a convex or composite shape.
///
/// Returns `None` when the direction is degenerate or the ray misses within
/// `max_length`. Composite targets recurse into their children/leaves.
pub fn raycast(
    shape: &dyn Shape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    let direction_length = direction.length3();
    if direction_length <= 1e-10 || max_length <= 0.0 {
        return None;
    }
    let direction = direction / direction_length;
    match shape.shape_type() {
        ShapeType::Mesh => raycast_mesh(
            shape
                .as_any()
                .downcast_ref::<MeshShape>()
                .expect("mesh shape type without MeshShape"),
            transform,
            origin,
            direction,
            max_length,
        ),
        ShapeType::HeightField => raycast_height_field(
            shape
                .as_any()
                .downcast_ref::<HeightFieldShape>()
                .expect("height field shape type without HeightFieldShape"),
            transform,
            origin,
            direction,
            max_length,
        ),
        ShapeType::Compound => raycast_compound(
            shape
                .as_any()
                .downcast_ref::<CompoundShape>()
                .expect("compound shape type without CompoundShape"),
            transform,
            origin,
            direction,
            max_length,
        ),
        ShapeType::Convex => {
            if let Some(sphere) = shape.as_any().downcast_ref::<SphereShape>() {
                if let Some(hit) = raycast_sphere(sphere, transform, origin, direction, max_length)
                {
                    return Some(hit);
                }
            }
            if let Some(box_shape) = shape.as_any().downcast_ref::<BoxShape>() {
                if let Some(hit) = raycast_box(box_shape, transform, origin, direction, max_length)
                {
                    return Some(hit);
                }
            }
            if let Some(hull) = shape.as_any().downcast_ref::<ConvexHullShape>() {
                if let Some(hit) =
                    raycast_convex_hull_faces(hull, transform, origin, direction, max_length)
                {
                    return Some(hit);
                }
            }
            raycast_convex(shape, transform, origin, direction, max_length)
        }
        _ => raycast_convex(shape, transform, origin, direction, max_length),
    }
}

/// Analytic sphere ray cast (radial surface normal).
fn raycast_sphere(
    sphere: &SphereShape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    let local_origin = transform.inverse_transform_point(origin);
    let local_direction = transform.rotation.rotate_inverse(direction);
    // 过渡期边界：`SphereShape` 字段已迁移 glam（Phase 3 步骤 2）。
    let center = crate::phys_shape::to_vector4(sphere.center);
    let offset = local_origin - center;
    let b = offset.dot3(local_direction);
    let c = offset.dot3(offset) - sphere.radius * sphere.radius;
    let discriminant = b * b - c;
    if discriminant < 0.0 {
        return None;
    }
    let root = discriminant.sqrt();
    let near = -b - root;
    // An origin inside the sphere is reported by the generic GJK path.
    if near < 0.0 || near > max_length {
        return None;
    }
    let local_point = local_origin + local_direction * near;
    let local_normal = (local_point - center).normalized3();
    Some(RayHit {
        hit_fraction: near,
        position: transform.transform_point(local_point),
        normal: transform.rotation.rotate(local_normal),
        shape_key: ShapeKey::INVALID,
    })
}

/// Analytic box ray cast (slab test with face normals).
///
/// The convex radius inflates the nominal half extents; rounded edges are
/// approximated by the closest face normal.
fn raycast_box(
    box_shape: &BoxShape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    let local_origin = transform.inverse_transform_point(origin);
    let local_direction = transform.rotation.rotate_inverse(direction);
    let radius = box_shape.convex_radius.max(0.0);
    let half =
        crate::phys_shape::to_vector4(box_shape.half_extents) + Vector4::splat(radius);
    let mut entry = f32::NEG_INFINITY;
    let mut exit = f32::INFINITY;
    let mut axis = 0;
    let mut sign = 1.0f32;
    for index in 0..3 {
        let origin_component = local_origin.component(index);
        let direction_component = local_direction.component(index);
        let extent = half.component(index);
        if direction_component.abs() <= 1e-12 {
            if origin_component.abs() > extent {
                return None;
            }
            continue;
        }
        let (mut first, mut second) = (
            (-extent - origin_component) / direction_component,
            (extent - origin_component) / direction_component,
        );
        let face_sign = if first > second { -1.0 } else { 1.0 };
        if first > second {
            core::mem::swap(&mut first, &mut second);
        }
        if first > entry {
            entry = first;
            axis = index;
            sign = face_sign;
        }
        exit = exit.min(second);
    }
    if entry < 0.0 || entry > exit || entry > max_length {
        return None;
    }
    let local_point = local_origin + local_direction * entry;
    let local_normal = match axis {
        0 => Vector4::UNIT_X * sign,
        1 => Vector4::UNIT_Y * sign,
        _ => Vector4::UNIT_Z * sign,
    };
    Some(RayHit {
        hit_fraction: entry,
        position: transform.transform_point(local_point),
        normal: transform.rotation.rotate(local_normal),
        shape_key: ShapeKey::INVALID,
    })
}

/// Exact face-level ray cast for a zero-radius convex hull.
///
/// Falls back to `None` when the hull has no face information or carries a
/// convex radius (the triangle faces describe the nominal polyhedron, not the
/// rounded support surface). Callers then use the generic GJK path.
fn raycast_convex_hull_faces(
    hull: &ConvexHullShape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    if hull.faces().is_empty() || hull.convex_radius().abs() > 1e-6 {
        return None;
    }
    let local_origin = transform.inverse_transform_point(origin);
    let local_direction = transform.rotation.rotate_inverse(direction);
    let vertices = hull.vertices();
    let mut best: Option<(f32, usize, Vector4)> = None;
    for (index, face) in hull.faces().iter().enumerate() {
        let (Some(&a), Some(&b), Some(&c)) = (
            vertices.get(face[0] as usize),
            vertices.get(face[1] as usize),
            vertices.get(face[2] as usize),
        ) else {
            continue;
        };
        // 过渡期边界：`ConvexHullShape` 顶点已迁移 glam（Phase 3 步骤 2）。
        let Some((fraction, normal)) = ray_triangle(
            local_origin,
            local_direction,
            crate::phys_shape::to_vector4(a),
            crate::phys_shape::to_vector4(b),
            crate::phys_shape::to_vector4(c),
        ) else {
            continue;
        };
        if fraction > max_length {
            continue;
        }
        let better = best.is_none_or(|(current, _, _)| fraction < current);
        if better {
            best = Some((fraction, index, normal));
        }
    }
    best.map(|(fraction, index, local_normal)| RayHit {
        hit_fraction: fraction,
        position: transform.transform_point(local_origin + local_direction * fraction),
        normal: transform.rotation.rotate(local_normal),
        shape_key: leaf_key::new(leaf_key::CONVEX_HULL_FACE, index as u32),
    })
}

/// Cast a ray against a convex shape.
fn raycast_convex(
    shape: &dyn Shape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    if !shape.is_convex() {
        return None;
    }

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
                shape_key: ShapeKey::INVALID,
            });
        }
        if result.distance <= HIT_EPSILON {
            // Keep the GJK hit distance/position; only replace the witness
            // normal when an analytic surface normal can be derived.
            let local_point = transform.inverse_transform_point(result.point_b);
            let normal = refine_convex_normal(shape, local_point)
                .map(|local| transform.rotation.rotate(local))
                .filter(|candidate| candidate.length_squared3() > 0.5)
                .map(|mut candidate| {
                    candidate.normalize3();
                    if candidate.dot3(direction) > 0.0 {
                        -candidate
                    } else {
                        candidate
                    }
                })
                .unwrap_or(result.normal);
            return Some(RayHit {
                hit_fraction: fraction,
                position: result.point_b,
                normal,
                shape_key: ShapeKey::INVALID,
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

/// Refine a GJK witness normal using the shape's analytic surface.
fn refine_convex_normal(shape: &dyn Shape, local_point: Vector4) -> Option<Vector4> {
    if let Some(capsule) = shape.as_any().downcast_ref::<CapsuleShape>() {
        return capsule_normal(capsule, local_point);
    }
    if let Some(cylinder) = shape.as_any().downcast_ref::<CylinderShape>() {
        return cylinder_normal(cylinder, local_point);
    }
    if let Some(hull) = shape.as_any().downcast_ref::<ConvexHullShape>() {
        return hull_rounded_normal(hull, local_point);
    }
    None
}

fn capsule_normal(capsule: &CapsuleShape, point: Vector4) -> Option<Vector4> {
    // 过渡期边界：`CapsuleShape` 字段已迁移 glam（Phase 3 步骤 2）。
    let point_a = crate::phys_shape::to_vector4(capsule.point_a);
    let point_b = crate::phys_shape::to_vector4(capsule.point_b);
    let segment = point_b - point_a;
    let length_sq = segment.length_squared3();
    let t = if length_sq <= f32::EPSILON {
        0.0
    } else {
        ((point - point_a).dot3(segment) / length_sq).clamp(0.0, 1.0)
    };
    let closest = point_a + segment * t;
    let normal = point - closest;
    if normal.length_squared3() <= 1e-12 {
        None
    } else {
        Some(normal.normalized3())
    }
}

fn cylinder_normal(cylinder: &CylinderShape, point: Vector4) -> Option<Vector4> {
    // 过渡期边界：`CylinderShape` 字段已迁移 glam（Phase 3 步骤 2）。
    let point_a = crate::phys_shape::to_vector4(cylinder.point_a);
    let point_b = crate::phys_shape::to_vector4(cylinder.point_b);
    let segment = point_b - point_a;
    let length_sq = segment.length_squared3();
    let center = (point_a + point_b) * 0.5;
    if length_sq <= f32::EPSILON {
        let normal = point - point_a;
        return if normal.length_squared3() <= 1e-12 {
            None
        } else {
            Some(normal.normalized3())
        };
    }

    let axis = segment.normalized3();
    let half_height = length_sq.sqrt() * 0.5;
    let relative = point - center;
    let axial = relative.dot3(axis);
    let radial = relative - axis * axial;
    let radial_length = radial.length3();
    let radial_unit = if radial_length > 1e-6 {
        radial / radial_length
    } else {
        stable_perpendicular(axis)
    };
    let nominal =
        center + axis * axial.clamp(-half_height, half_height) + radial_unit * cylinder.radius;
    let normal = point - nominal;
    if normal.length_squared3() <= 1e-12 {
        None
    } else {
        Some(normal.normalized3())
    }
}

fn hull_rounded_normal(hull: &ConvexHullShape, point: Vector4) -> Option<Vector4> {
    if hull.faces().is_empty() {
        return None;
    }
    let vertices = hull.vertices();
    let mut best: Option<(f32, Vector4)> = None;
    for &face in hull.faces() {
        let (Some(&a), Some(&b), Some(&c)) = (
            vertices.get(face[0] as usize),
            vertices.get(face[1] as usize),
            vertices.get(face[2] as usize),
        ) else {
            continue;
        };
        // 过渡期边界：`ConvexHullShape` 顶点已迁移 glam（Phase 3 步骤 2）。
        let closest = closest_point_on_triangle(
            point,
            crate::phys_shape::to_vector4(a),
            crate::phys_shape::to_vector4(b),
            crate::phys_shape::to_vector4(c),
        );
        let distance_sq = (point - closest).length_squared3();
        if best.is_none_or(|(current, _)| distance_sq < current) {
            best = Some((distance_sq, closest));
        }
    }
    let normal = point - best?.1;
    if normal.length_squared3() <= 1e-12 {
        None
    } else {
        Some(normal.normalized3())
    }
}

fn stable_perpendicular(axis: Vector4) -> Vector4 {
    let reference = if axis.x.abs() < 0.9 {
        Vector4::UNIT_X
    } else {
        Vector4::UNIT_Y
    };
    axis.cross(reference).normalized3()
}

/// Closest point on triangle `ABC` to `P` (Ericson's barycentric algorithm).
fn closest_point_on_triangle(p: Vector4, a: Vector4, b: Vector4, c: Vector4) -> Vector4 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot3(ap);
    let d2 = ac.dot3(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = ab.dot3(bp);
    let d4 = ac.dot3(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denominator = d1 - d3;
        if denominator.abs() > f32::EPSILON {
            return a + ab * (d1 / denominator);
        }
    }
    let cp = p - c;
    let d5 = ab.dot3(cp);
    let d6 = ac.dot3(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denominator = d2 - d6;
        if denominator.abs() > f32::EPSILON {
            return a + ac * (d2 / denominator);
        }
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 >= d3 && d5 >= d6 {
        let denominator = (d4 - d3) + (d5 - d6);
        if denominator.abs() > f32::EPSILON {
            return b + (c - b) * ((d4 - d3) / denominator);
        }
    }
    let denominator = va + vb + vc;
    if denominator <= f32::EPSILON {
        return a;
    }
    let v = vb / denominator;
    let w = vc / denominator;
    a + ab * v + ac * w
}

/// Resolve a deterministic leaf [`ShapeKey`] from a local-space surface point.
///
/// Used by both query-side diagnostics and world-level trigger events so the
/// two streams share one encoding. Returns `ShapeKey::INVALID` for shapes that
/// have no ported leaf subdivision or when no leaf can be resolved.
pub fn leaf_key_at_local_point(shape: &dyn Shape, local_point: Vector4) -> ShapeKey {
    match shape.shape_type() {
        ShapeType::Convex => {
            if let Some(hull) = shape.as_any().downcast_ref::<ConvexHullShape>() {
                return hull_face_key_at_local_point(hull, local_point);
            }
        }
        ShapeType::Mesh => {
            if let Some(mesh) = shape.as_any().downcast_ref::<MeshShape>() {
                return mesh_leaf_key_at_local_point(mesh, local_point);
            }
        }
        ShapeType::HeightField => {
            if let Some(height_field) = shape.as_any().downcast_ref::<HeightFieldShape>() {
                return height_field_leaf_key_at_local_point(height_field, local_point);
            }
        }
        ShapeType::Compound => {
            if let Some(compound) = shape.as_any().downcast_ref::<CompoundShape>() {
                return compound_leaf_key_at_local_point(compound, local_point);
            }
        }
        _ => {}
    }
    ShapeKey::INVALID
}

fn hull_face_key_at_local_point(hull: &ConvexHullShape, local_point: Vector4) -> ShapeKey {
    if hull.faces().is_empty() {
        return ShapeKey::INVALID;
    }
    let vertices = hull.vertices();
    let mut best: Option<(f32, u32)> = None;
    for (index, &face) in hull.faces().iter().enumerate() {
        let (Some(&a), Some(&b), Some(&c)) = (
            vertices.get(face[0] as usize),
            vertices.get(face[1] as usize),
            vertices.get(face[2] as usize),
        ) else {
            continue;
        };
        // 过渡期边界：`ConvexHullShape` 顶点已迁移 glam（Phase 3 步骤 2）。
        let closest = closest_point_on_triangle(
            local_point,
            crate::phys_shape::to_vector4(a),
            crate::phys_shape::to_vector4(b),
            crate::phys_shape::to_vector4(c),
        );
        let distance_sq = (local_point - closest).length_squared3();
        if best.is_none_or(|(current, _)| distance_sq < current) {
            best = Some((distance_sq, index as u32));
        }
    }
    best.map_or(ShapeKey::INVALID, |(_, index)| {
        leaf_key::new(leaf_key::CONVEX_HULL_FACE, index)
    })
}

fn mesh_leaf_key_at_local_point(mesh: &MeshShape, local_point: Vector4) -> ShapeKey {
    if mesh.bvh_nodes().is_empty() {
        return ShapeKey::INVALID;
    }
    let mut best: Option<(f32, u32)> = None;
    let mut stack = vec![0usize];
    while let Some(node_index) = stack.pop() {
        let node = mesh.bvh_nodes()[node_index];
        let aabb_distance = node.aabb.signed_distance_to_point(local_point).1.max(0.0);
        if best.is_some_and(|(current, _)| aabb_distance * aabb_distance > current) {
            continue;
        }
        match node.children {
            Some((left, right)) => {
                stack.push(right as usize);
                stack.push(left as usize);
            }
            None => {
                for &triangle_index in mesh.node_triangles(node_index) {
                    let triangle = mesh.triangles()[triangle_index as usize];
                    let a = mesh.vertices()[triangle[0] as usize];
                    let b = mesh.vertices()[triangle[1] as usize];
                    let c = mesh.vertices()[triangle[2] as usize];
                    let closest = closest_point_on_triangle(local_point, a, b, c);
                    let distance_sq = (local_point - closest).length_squared3();
                    if best.is_none_or(|(current, _)| distance_sq < current) {
                        best = Some((distance_sq, triangle_index));
                    }
                }
            }
        }
    }
    best.map_or(ShapeKey::INVALID, |(_, index)| {
        leaf_key::new(leaf_key::MESH, index)
    })
}

fn height_field_leaf_key_at_local_point(
    height_field: &HeightFieldShape,
    local_point: Vector4,
) -> ShapeKey {
    let size_x = height_field.int_size_x();
    let size_z = height_field.int_size_z();
    if size_x < 2 || size_z < 2 {
        return ShapeKey::INVALID;
    }
    let scale = height_field.scale();
    let cell_x = (local_point.x / scale.x).floor().max(0.0) as usize;
    let cell_z = (local_point.z / scale.z).floor().max(0.0) as usize;
    let cell_x = cell_x.min(size_x - 2);
    let cell_z = cell_z.min(size_z - 2);

    let p00 = height_field.vertex(cell_x, cell_z);
    let p10 = height_field.vertex(cell_x + 1, cell_z);
    let p01 = height_field.vertex(cell_x, cell_z + 1);
    let p11 = height_field.vertex(cell_x + 1, cell_z + 1);
    let mut best: Option<(f32, usize)> = None;
    for (points, sub_triangle) in [([p00, p10, p11], 0usize), ([p00, p11, p01], 1usize)] {
        let closest = closest_point_on_triangle(local_point, points[0], points[1], points[2]);
        let distance_sq = (local_point - closest).length_squared3();
        if best.is_none_or(|(current, _)| distance_sq < current) {
            best = Some((distance_sq, sub_triangle));
        }
    }
    best.map_or(ShapeKey::INVALID, |(_, sub_triangle)| {
        leaf_key::height_field(cell_z, cell_x, sub_triangle)
    })
}

fn compound_leaf_key_at_local_point(compound: &CompoundShape, local_point: Vector4) -> ShapeKey {
    let mut best: Option<(f32, usize, ShapeKey)> = None;
    for (child_index, child) in compound.children().iter().enumerate() {
        let child_point = child.transform.inverse_transform_point(local_point);
        let distance = closest_points(
            &PointShape(child_point),
            &Transform::IDENTITY,
            child.shape.as_ref(),
            &Transform::IDENTITY,
        )
        .map(|result| result.distance.max(0.0))
        .unwrap_or_else(|| {
            child
                .shape
                .aabb()
                .signed_distance_to_point(child_point)
                .1
                .max(0.0)
        });
        if best.is_none_or(|(current, _, _)| distance < current) {
            let inner = leaf_key_at_local_point(child.shape.as_ref(), child_point);
            best = Some((distance, child_index, inner));
        }
    }
    best.map_or(ShapeKey::INVALID, |(_, child_index, inner)| {
        leaf_key::compound_child(child_index, inner)
    })
}

/// Traverse a mesh's BVH and test its leaf triangles.
fn raycast_mesh(
    mesh: &MeshShape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    if mesh.bvh_nodes().is_empty() {
        return None;
    }
    let local_origin = transform.inverse_transform_point(origin);
    let local_direction = transform.rotation.rotate_inverse(direction);
    let mut best: Option<(f32, ShapeKey, Vector4, Vector4)> = None;

    let mut stack = vec![0usize];
    while let Some(node_index) = stack.pop() {
        let node = mesh.bvh_nodes()[node_index];
        let Some((entry, exit)) = ray_aabb(local_origin, local_direction, &node.aabb) else {
            continue;
        };
        if entry > max_length || exit < 0.0 {
            continue;
        }
        match node.children {
            Some((left, right)) => {
                stack.push(right as usize);
                stack.push(left as usize);
            }
            None => {
                for &triangle_index in mesh.node_triangles(node_index) {
                    let triangle = mesh.triangles()[triangle_index as usize];
                    let a = mesh.vertices()[triangle[0] as usize];
                    let b = mesh.vertices()[triangle[1] as usize];
                    let c = mesh.vertices()[triangle[2] as usize];
                    let Some((t, normal)) = ray_triangle(local_origin, local_direction, a, b, c)
                    else {
                        continue;
                    };
                    if t > max_length {
                        continue;
                    }
                    let key = leaf_key::new(leaf_key::MESH, triangle_index);
                    let better = match &best {
                        Some((current, current_key, _, _)) => (t, key) < (*current, *current_key),
                        None => true,
                    };
                    if better {
                        best = Some((t, key, local_origin + local_direction * t, normal));
                    }
                }
            }
        }
    }

    best.map(|(fraction, key, position, normal)| RayHit {
        hit_fraction: fraction,
        position: transform.transform_point(position),
        normal: transform.rotation.rotate(normal),
        shape_key: key,
    })
}

/// Best height-field leaf hit: `(distance, leaf key, local position, local
/// normal)`.
type HeightFieldHit = (f32, ShapeKey, Vector4, Vector4);

/// Walk the height-field cells inside the ray interval.
fn raycast_height_field(
    heightfield: &HeightFieldShape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    let size_x = heightfield.int_size_x();
    let size_z = heightfield.int_size_z();
    if size_x < 2 || size_z < 2 {
        return None;
    }
    let local_origin = transform.inverse_transform_point(origin);
    let local_direction = transform.rotation.rotate_inverse(direction);
    let (entry, exit) = ray_aabb(local_origin, local_direction, &heightfield.aabb())?;
    let start = entry.max(0.0);
    let end = exit.min(max_length);
    if start > end {
        return None;
    }
    let point_a = local_origin + local_direction * start;
    let point_b = local_origin + local_direction * end;
    let scale = heightfield.scale();
    let min_x = point_a.x.min(point_b.x) / scale.x;
    let max_x = point_a.x.max(point_b.x) / scale.x;
    let min_z = point_a.z.min(point_b.z) / scale.z;
    let max_z = point_a.z.max(point_b.z) / scale.z;

    let cell_min_x = min_x.floor().max(0.0) as usize;
    let cell_max_x = (max_x.ceil() as usize).min(size_x - 2);
    let cell_min_z = min_z.floor().max(0.0) as usize;
    let cell_max_z = (max_z.ceil() as usize).min(size_z - 2);
    if cell_min_x > cell_max_x || cell_min_z > cell_max_z {
        return None;
    }

    let mut best: Option<HeightFieldHit> = None;
    for z in cell_min_z..=cell_max_z {
        for x in cell_min_x..=cell_max_x {
            let p00 = heightfield.vertex(x, z);
            let p10 = heightfield.vertex(x + 1, z);
            let p01 = heightfield.vertex(x, z + 1);
            let p11 = heightfield.vertex(x + 1, z + 1);
            for (points, sub) in [([p00, p10, p11], 0usize), ([p00, p11, p01], 1usize)] {
                let Some((t, normal)) = ray_triangle(
                    local_origin,
                    local_direction,
                    points[0],
                    points[1],
                    points[2],
                ) else {
                    continue;
                };
                if t > max_length {
                    continue;
                }
                let key = leaf_key::height_field(z, x, sub);
                let better = match &best {
                    Some((current, current_key, _, _)) => (t, key) < (*current, *current_key),
                    None => true,
                };
                if better {
                    best = Some((t, key, local_origin + local_direction * t, normal));
                }
            }
        }
    }

    best.map(|(fraction, key, position, normal)| RayHit {
        hit_fraction: fraction,
        position: transform.transform_point(position),
        normal: transform.rotation.rotate(normal),
        shape_key: key,
    })
}

/// Recurse into compound children with composed transforms.
fn raycast_compound(
    compound: &CompoundShape,
    transform: &Transform,
    origin: Vector4,
    direction: Vector4,
    max_length: f32,
) -> Option<RayHit> {
    let mut best: Option<(f32, usize, RayHit)> = None;
    for (child_index, child) in compound.children().iter().enumerate() {
        let child_transform = compose_transform(transform, &child.transform);
        let Some(hit) = raycast(
            child.shape.as_ref(),
            &child_transform,
            origin,
            direction,
            max_length,
        ) else {
            continue;
        };
        let hit = RayHit {
            shape_key: leaf_key::compound_child(child_index, hit.shape_key),
            ..hit
        };
        let better = match &best {
            Some((fraction, index, _)) => (hit.hit_fraction, child_index) < (*fraction, *index),
            None => true,
        };
        if better {
            best = Some((hit.hit_fraction, child_index, hit));
        }
    }
    best.map(|(_, _, hit)| hit)
}

/// Analytic two-sided ray/triangle intersection (Möller–Trumbore).
///
/// Returns the ray parameter `t >= 0` and a unit normal oriented against the
/// ray direction. Degenerate or parallel triangles return `None`.
fn ray_triangle(
    origin: Vector4,
    direction: Vector4,
    a: Vector4,
    b: Vector4,
    c: Vector4,
) -> Option<(f32, Vector4)> {
    let edge_ab = b - a;
    let edge_ac = c - a;
    let pvec = direction.cross(edge_ac);
    let determinant = edge_ab.dot3(pvec);
    if determinant.abs() <= 1e-10 {
        return None;
    }
    let inverse = 1.0 / determinant;
    let tvec = origin - a;
    let u = tvec.dot3(pvec) * inverse;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qvec = tvec.cross(edge_ab);
    let v = direction.dot3(qvec) * inverse;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = edge_ac.dot3(qvec) * inverse;
    if t < 0.0 {
        return None;
    }
    let mut normal = edge_ab.cross(edge_ac);
    let length = normal.length3();
    if length <= 1e-10 {
        return None;
    }
    normal *= 1.0 / length;
    if normal.dot3(direction) > 0.0 {
        normal = -normal;
    }
    Some((t, normal))
}

/// Compose a parent world transform with a child local transform.
fn compose_transform(parent: &Transform, child: &Transform) -> Transform {
    Transform::new(
        parent.rotation.mul_mat(&child.rotation),
        parent.transform_point(child.translation),
    )
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
        let (min, max) = (aabb.min[axis], aabb.max[axis]);
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

    fn support(&self, _direction: glam::Vec4) -> Option<glam::Vec4> {
        Some(crate::phys_shape::to_vec4(self.0))
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_math::Quaternion;
    use crate::base_math::Vec4Ext;
    use glam::Vec4;
    use crate::phys_shape::{BoxShape, CapsuleShape, ConvexHullShape, CylinderShape, SphereShape};
    use std::sync::Arc;

    fn sphere(radius: f32, center: Vector4) -> Arc<dyn Shape> {
        // 过渡期边界：构造已迁移 glam（Phase 3 步骤 2），测试输入保持 legacy。
        Arc::new(SphereShape::new(crate::phys_shape::to_vec4(center), radius))
            as Arc<dyn Shape>
    }

    fn box_shape(half: f32) -> Arc<dyn Shape> {
        Arc::new(BoxShape::with_convex_radius(Vec4::splat(half), 0.0)) as Arc<dyn Shape>
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
            Vec4::from_xyz(0.0, 0.0, 0.0),
            Vec4::from_xyz(0.0, 2.0, 0.0),
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
    fn capsule_side_normal_is_radial() {
        let shape: Arc<dyn Shape> = Arc::new(CapsuleShape::new(
            Vec4::from_xyz(0.0, 0.0, 0.0),
            Vec4::from_xyz(0.0, 2.0, 0.0),
            0.5,
        ));
        let hit = raycast(
            shape.as_ref(),
            &Transform::IDENTITY,
            Vector4::from_xyz(5.0, 1.0, 0.0),
            -Vector4::UNIT_X,
            10.0,
        )
        .expect("ray must hit the capsule side");
        assert!(
            hit.normal.all_equal3(Vector4::UNIT_X, 1e-2),
            "normal {:?}",
            hit.normal
        );
    }

    #[test]
    fn cylinder_side_normal_is_radial() {
        let shape: Arc<dyn Shape> = Arc::new(CylinderShape::with_params(
            Vec4::from_xyz(0.0, -1.0, 0.0),
            Vec4::from_xyz(0.0, 1.0, 0.0),
            1.0,
            16,
            0.0,
        ));
        let hit = raycast(
            shape.as_ref(),
            &Transform::IDENTITY,
            Vector4::from_xyz(5.0, 0.0, 0.0),
            -Vector4::UNIT_X,
            10.0,
        )
        .expect("ray must hit the cylinder side");
        assert!(
            hit.normal.all_equal3(Vector4::UNIT_X, 1e-2),
            "normal {:?}",
            hit.normal
        );
    }

    #[test]
    fn rounded_hull_face_normal_is_analytic() {
        let points = vec![
            Vec4::from_xyz(-1.0, -1.0, -1.0),
            Vec4::from_xyz(1.0, -1.0, -1.0),
            Vec4::from_xyz(1.0, 1.0, -1.0),
            Vec4::from_xyz(-1.0, 1.0, -1.0),
            Vec4::from_xyz(-1.0, -1.0, 1.0),
            Vec4::from_xyz(1.0, -1.0, 1.0),
            Vec4::from_xyz(1.0, 1.0, 1.0),
            Vec4::from_xyz(-1.0, 1.0, 1.0),
        ];
        let shape: Arc<dyn Shape> = Arc::new(ConvexHullShape::from_points(points, 0.2));
        let hit = raycast(
            shape.as_ref(),
            &Transform::IDENTITY,
            Vector4::from_xyz(5.0, 0.0, 0.0),
            -Vector4::UNIT_X,
            10.0,
        )
        .expect("ray must hit the rounded hull");
        assert!(
            hit.normal.all_equal3(Vector4::UNIT_X, 1e-2),
            "normal {:?}",
            hit.normal
        );
    }

    #[test]
    fn ray_hits_convex_hull() {
        // Octahedron around the origin.
        let vertices = vec![
            Vec4::from_xyz(1.0, 0.0, 0.0),
            Vec4::from_xyz(-1.0, 0.0, 0.0),
            Vec4::from_xyz(0.0, 1.0, 0.0),
            Vec4::from_xyz(0.0, -1.0, 0.0),
            Vec4::from_xyz(0.0, 0.0, 1.0),
            Vec4::from_xyz(0.0, 0.0, -1.0),
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
    fn hull_face_raycast_returns_exact_normal_and_face_key() {
        let vertices = vec![
            Vec4::from_xyz(-1.0, 1.0, -1.0),
            Vec4::from_xyz(1.0, 1.0, -1.0),
            Vec4::from_xyz(1.0, 1.0, 1.0),
            Vec4::from_xyz(-1.0, 1.0, 1.0),
            Vec4::from_xyz(-1.0, -1.0, -1.0),
            Vec4::from_xyz(1.0, -1.0, -1.0),
            Vec4::from_xyz(1.0, -1.0, 1.0),
            Vec4::from_xyz(-1.0, -1.0, 1.0),
        ];
        let faces = vec![[0, 1, 2], [0, 2, 3]];
        let shape = ConvexHullShape::new(vertices, faces, 0.0);
        let hit = raycast(
            &shape,
            &Transform::IDENTITY,
            Vector4::from_xyz(0.0, 5.0, 0.0),
            -Vector4::UNIT_Y,
            10.0,
        )
        .expect("ray must hit the hull face");
        assert!((hit.hit_fraction - 4.0).abs() < 1e-5);
        assert!(hit.normal.dot3(Vector4::UNIT_Y) > 0.999);
        assert_eq!(hit.shape_key, leaf_key::new(leaf_key::CONVEX_HULL_FACE, 0));
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

    fn quad_mesh(size: f32) -> Arc<dyn Shape> {
        Arc::new(MeshShape::new(
            vec![
                Vector4::from_xyz(-size, 0.0, -size),
                Vector4::from_xyz(size, 0.0, -size),
                Vector4::from_xyz(size, 0.0, size),
                Vector4::from_xyz(-size, 0.0, size),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )) as Arc<dyn Shape>
    }

    #[test]
    fn ray_hits_mesh_triangle_through_bvh() {
        let mesh = quad_mesh(2.0);
        let hit = raycast(
            mesh.as_ref(),
            &Transform::IDENTITY,
            Vector4::from_xyz(0.5, 5.0, 0.5),
            -Vector4::UNIT_Y,
            10.0,
        )
        .expect("ray must hit the mesh quad");
        assert!(
            (hit.hit_fraction - 5.0).abs() < 1e-3,
            "fraction {}",
            hit.hit_fraction
        );
        assert!(
            hit.normal.dot3(Vector4::UNIT_Y) > 0.9,
            "normal {:?}",
            hit.normal
        );
        assert!(
            hit.position
                .all_equal3(Vector4::from_xyz(0.5, 0.0, 0.5), 1e-3),
            "position {:?}",
            hit.position
        );
    }

    #[test]
    fn ray_misses_mesh_outside_its_silhouette() {
        let mesh = quad_mesh(1.0);
        assert!(
            raycast(
                mesh.as_ref(),
                &Transform::IDENTITY,
                Vector4::from_xyz(5.0, 5.0, 0.0),
                -Vector4::UNIT_Y,
                10.0,
            )
            .is_none()
        );
    }

    #[test]
    fn ray_hits_rotated_mesh() {
        // Rotate the quad 90° about X: its plane normal becomes -Z.
        let mesh = quad_mesh(1.0);
        let rotation = Quaternion::from_axis_angle(Vector4::UNIT_X, core::f32::consts::FRAC_PI_2);
        let transform = Transform::from_quaternion(rotation, Vector4::ZERO);
        let hit = raycast(
            mesh.as_ref(),
            &transform,
            Vector4::from_xyz(0.0, 0.0, 5.0),
            -Vector4::UNIT_Z,
            10.0,
        )
        .expect("ray must hit the rotated mesh");
        assert!(
            (hit.hit_fraction - 5.0).abs() < 1e-3,
            "fraction {}",
            hit.hit_fraction
        );
        assert!(
            hit.normal.dot3(Vector4::UNIT_Z) > 0.9,
            "normal {:?}",
            hit.normal
        );
    }

    #[test]
    fn ray_hits_height_field() {
        let heightfield: Arc<dyn Shape> = Arc::new(
            HeightFieldShape::new(4, 4, Vector4::ONE, vec![0.0; 16]).expect("valid height field"),
        );
        let hit = raycast(
            heightfield.as_ref(),
            &Transform::IDENTITY,
            Vector4::from_xyz(1.5, 5.0, 1.5),
            -Vector4::UNIT_Y,
            10.0,
        )
        .expect("ray must hit the flat height field");
        assert!(
            (hit.hit_fraction - 5.0).abs() < 1e-3,
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
    fn ray_hits_compound_child_with_world_transform() {
        use crate::phys_shape::{CompoundShape, ShapeInstance};
        let compound: Arc<dyn Shape> = Arc::new(CompoundShape::new(vec![
            ShapeInstance::new(
                Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 2.0, 0.0)),
                sphere(1.0, Vector4::ZERO),
            ),
            ShapeInstance::new(
                Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(5.0, 2.0, 0.0)),
                sphere(1.0, Vector4::ZERO),
            ),
        ]));
        let transform = Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, core::f32::consts::FRAC_PI_2),
            Vector4::from_xyz(10.0, 0.0, 0.0),
        );
        // Child sphere at local (0,2,0) maps to world (8,0,0); a ray along -Z
        // from z=5 reaches its surface at z=1 (fraction 4).
        let hit = raycast(
            compound.as_ref(),
            &transform,
            Vector4::from_xyz(8.0, 0.0, 5.0),
            -Vector4::UNIT_Z,
            10.0,
        )
        .expect("ray must hit the rotated compound child");
        assert!(
            (hit.hit_fraction - 4.0).abs() < 1e-2,
            "fraction {}",
            hit.hit_fraction
        );
    }

    #[test]
    fn mesh_ray_respects_max_length() {
        let mesh = quad_mesh(1.0);
        // Start below the plane: the hit is 5 units away, beyond a 3 unit ray.
        assert!(
            raycast(
                mesh.as_ref(),
                &Transform::IDENTITY,
                Vector4::from_xyz(0.0, -5.0, 0.0),
                Vector4::UNIT_Y,
                3.0,
            )
            .is_none()
        );
    }

    #[test]
    fn leaf_key_resolver_covers_hull_mesh_height_field_and_compound() {
        let hull = ConvexHullShape::new(
            vec![
                Vec4::ZERO,
                Vec4::new(1.0, 0.0, 0.0, 0.0),
                Vec4::new(0.0, 1.0, 0.0, 0.0),
            ],
            vec![[0, 1, 2]],
            0.0,
        );
        let face_center =
            (Vector4::ZERO + Vector4::UNIT_X + Vector4::UNIT_Y) * (1.0 / 3.0);
        assert_eq!(
            leaf_key_at_local_point(&hull, face_center),
            leaf_key::new(leaf_key::CONVEX_HULL_FACE, 0)
        );

        let mesh = quad_mesh(2.0);
        assert_eq!(
            leaf_key_at_local_point(mesh.as_ref(), Vector4::from_xyz(1.5, 0.0, -1.5)),
            leaf_key::new(leaf_key::MESH, 0)
        );

        let heightfield = HeightFieldShape::new(4, 4, Vector4::ONE, vec![0.0; 16]).unwrap();
        assert_eq!(
            leaf_key_at_local_point(&heightfield, Vector4::from_xyz(1.5, 0.0, 1.5)),
            leaf_key::height_field(1, 1, 0)
        );

        use crate::phys_shape::ShapeInstance;
        let compound = CompoundShape::new(vec![
            ShapeInstance::new(Transform::IDENTITY, sphere(1.0, Vector4::ZERO)),
            ShapeInstance::new(
                Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(5.0, 0.0, 0.0)),
                sphere(1.0, Vector4::ZERO),
            ),
        ]);
        assert_eq!(
            leaf_key_at_local_point(&compound, Vector4::from_xyz(1.0, 0.0, 0.0)),
            leaf_key::compound_child(0, ShapeKey::INVALID)
        );
    }
}
