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

use base_math::{Aabb, Transform, Vector4};
use phys_shape::{CompoundShape, HeightFieldShape, MeshShape, Shape};
use phys_types::ShapeType;

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
        _ => raycast_convex(shape, transform, origin, direction, max_length),
    }
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
    let mut best: Option<(f32, u32, Vector4, Vector4)> = None;

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
                    let better = match &best {
                        Some((current, index, _, _)) => (t, triangle_index) < (*current, *index),
                        None => true,
                    };
                    if better {
                        best = Some((
                            t,
                            triangle_index,
                            local_origin + local_direction * t,
                            normal,
                        ));
                    }
                }
            }
        }
    }

    best.map(|(fraction, _, position, normal)| RayHit {
        hit_fraction: fraction,
        position: transform.transform_point(position),
        normal: transform.rotation.rotate(normal),
    })
}

/// Best height-field leaf hit: `(distance, (cell z, cell x, sub-triangle),
/// local position, local normal)`.
type HeightFieldHit = (f32, (usize, usize, usize), Vector4, Vector4);

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
                let key = (z, x, sub);
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

    best.map(|(fraction, _, position, normal)| RayHit {
        hit_fraction: fraction,
        position: transform.transform_point(position),
        normal: transform.rotation.rotate(normal),
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
    use base_math::Quaternion;
    use phys_shape::{BoxShape, CapsuleShape, ConvexHullShape, SphereShape};
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
        use phys_shape::{CompoundShape, ShapeInstance};
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
}
