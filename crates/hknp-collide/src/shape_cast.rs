//! Shape casts (conservative advancement) against convex and composite
//! shapes.
//!
//! Subset of `original/Physics/Source/Physics/Physics/Collide/Query/`
//! (`hknpShapeCastFuncs` / `hknpFullCast`, GSK shape-cast branch). The port uses
//! **conservative advancement** built on the existing GJK closest-point query:
//!
//! 1. Compute the GJK distance `d` and separating normal `n` (target → query)
//!    between the query shape at its current swept pose and the target.
//! 2. The distance closes at the rate `-direction · n`; advance the query pose
//!    by `d / (-direction · n)` (the advance cannot overshoot the first
//!    contact for convex pairs).
//! 3. Repeat until `d <= tolerance` (hit), the cast distance is exhausted
//!    (miss), or the query starts penetrating.
//!
//! Composite targets (mesh, height field, compound) are traversed with their
//! own acceleration structures (mesh BVH, height-field cell grid, compound
//! child list) and each leaf is tested with the same convex routine.
//!
//! # Conventions
//!
//! * `direction` is normalized defensively; `max_distance` is measured in
//!   world units along it.
//! * `ShapeCastHit::fraction` is the travel distance at contact, in
//!   `[0, max_distance]`.
//! * `normal` points from the hit shape towards the query shape, matching
//!   `hknpCollisionResult::m_normal` for shape casts.
//! * `position` is the contact point on the hit shape and `point_on_query` the
//!   corresponding witness on the query shape at the hit pose.
//! * A query shape that is already overlapping returns `fraction == 0` with
//!   `start_penetrating == true`.
//! * Rotating casts (the SDK's `WingedCast`/rotation branch) are not ported;
//!   the query shape keeps its start orientation.
//! * The query shape must be convex (sphere/box/capsule/cylinder/triangle/
//!   hull). Composite query shapes are future work.

use hk_math::{Aabb, Transform, Vector4};
use hknp_shape::{CompoundShape, HeightFieldShape, MeshShape, Shape, TriangleShape};
use hknp_types::ShapeType;

use crate::gjk::closest_points;

/// `HKNP_DEFAULT_SHAPE_CAST_ACCURACY`.
pub const DEFAULT_SHAPE_CAST_TOLERANCE: f32 = 0.001;

/// Maximum conservative-advancement iterations.
const MAX_ITERATIONS: usize = 64;

/// Result of a shape cast.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShapeCastHit {
    /// Distance travelled along the normalized cast direction at contact.
    pub fraction: f32,
    /// Contact point on the hit shape (world space).
    pub position: Vector4,
    /// Contact normal pointing from the hit shape to the query shape.
    pub normal: Vector4,
    /// Witness point on the query shape at the hit pose.
    pub point_on_query: Vector4,
    /// Whether the query shape was already overlapping at the start.
    pub start_penetrating: bool,
}

/// Cast a convex query shape along `direction` against another shape.
///
/// Returns `None` for a degenerate direction/distance, a non-convex query
/// shape, or when no contact is found within `max_distance`.
pub fn shape_cast(
    query_shape: &dyn Shape,
    query_transform: &Transform,
    target_shape: &dyn Shape,
    target_transform: &Transform,
    direction: Vector4,
    max_distance: f32,
    tolerance: f32,
) -> Option<ShapeCastHit> {
    if !query_shape.is_convex() {
        return None;
    }
    let direction_length = direction.length3();
    if direction_length <= 1e-10 || max_distance <= 0.0 {
        return None;
    }
    let direction = direction * (1.0 / direction_length);
    let tolerance = tolerance.max(0.0);

    match target_shape.shape_type() {
        ShapeType::Mesh => cast_mesh(
            query_shape,
            query_transform,
            target_shape
                .as_any()
                .downcast_ref::<MeshShape>()
                .expect("mesh shape type without MeshShape"),
            target_transform,
            direction,
            max_distance,
            tolerance,
        ),
        ShapeType::HeightField => cast_height_field(
            query_shape,
            query_transform,
            target_shape
                .as_any()
                .downcast_ref::<HeightFieldShape>()
                .expect("height field shape type without HeightFieldShape"),
            target_transform,
            direction,
            max_distance,
            tolerance,
        ),
        ShapeType::Compound => cast_compound(
            query_shape,
            query_transform,
            target_shape
                .as_any()
                .downcast_ref::<CompoundShape>()
                .expect("compound shape type without CompoundShape"),
            target_transform,
            direction,
            max_distance,
            tolerance,
        ),
        _ => cast_convex(
            query_shape,
            query_transform,
            target_shape,
            target_transform,
            direction,
            max_distance,
            tolerance,
        ),
    }
}

/// Conservative advancement against one convex target.
#[allow(clippy::too_many_arguments)]
fn cast_convex(
    query_shape: &dyn Shape,
    query_transform: &Transform,
    target_shape: &dyn Shape,
    target_transform: &Transform,
    direction: Vector4,
    max_distance: f32,
    tolerance: f32,
) -> Option<ShapeCastHit> {
    let mut pose = *query_transform;
    let first = closest_points(query_shape, &pose, target_shape, target_transform)?;
    if first.intersects {
        return Some(start_penetrating_hit(&first, direction));
    }
    let mut distance = first.distance;
    let mut normal = unit_or(first.normal, -direction);
    if distance <= tolerance {
        return Some(contact_hit(0.0, &first, normal, true));
    }

    for _ in 0..MAX_ITERATIONS {
        let closing = -direction.dot3(normal);
        if closing <= 1e-6 {
            return None;
        }
        let advance = distance / closing;
        if !advance.is_finite() || advance <= 0.0 {
            return None;
        }
        let traveled = pose.translation + direction * advance;
        if (traveled - query_transform.translation).length3() > max_distance {
            return None;
        }
        pose.translation = traveled;
        let result = closest_points(query_shape, &pose, target_shape, target_transform)?;
        let traveled_distance = (pose.translation - query_transform.translation).length3();
        if result.intersects {
            let normal = unit_or(result.normal, -direction);
            return Some(contact_hit(traveled_distance, &result, normal, false));
        }
        if result.distance <= tolerance {
            return Some(contact_hit(
                traveled_distance,
                &result,
                unit_or(result.normal, normal),
                false,
            ));
        }
        if result.distance >= distance - 1e-7 {
            // No progress: numerical stagnation rather than a contact.
            return None;
        }
        distance = result.distance;
        normal = unit_or(result.normal, normal);
    }
    None
}

fn start_penetrating_hit(result: &crate::gjk::GjkResult, direction: Vector4) -> ShapeCastHit {
    ShapeCastHit {
        fraction: 0.0,
        position: result.point_b,
        normal: unit_or(result.normal, -direction),
        point_on_query: result.point_a,
        start_penetrating: true,
    }
}

fn contact_hit(
    fraction: f32,
    result: &crate::gjk::GjkResult,
    normal: Vector4,
    start_penetrating: bool,
) -> ShapeCastHit {
    ShapeCastHit {
        fraction,
        position: result.point_b,
        normal,
        point_on_query: result.point_a,
        start_penetrating,
    }
}

fn unit_or(direction: Vector4, fallback: Vector4) -> Vector4 {
    let length = direction.length3();
    if length > 1e-10 {
        direction * (1.0 / length)
    } else {
        fallback
    }
}

/// Swept world-space AABB of the query shape for broad-phase culling.
fn swept_aabb(
    query_shape: &dyn Shape,
    query_transform: &Transform,
    direction: Vector4,
    max_distance: f32,
) -> Aabb {
    let mut swept = query_shape.calc_aabb(query_transform);
    let end = Transform::new(
        query_transform.rotation,
        query_transform.translation + direction * max_distance,
    );
    swept.include_aabb(&query_shape.calc_aabb(&end));
    swept
}

/// Local-space AABB of the swept query shape in the target's space.
fn local_swept_aabb(
    query_shape: &dyn Shape,
    query_transform: &Transform,
    target_transform: &Transform,
    direction: Vector4,
    max_distance: f32,
) -> Aabb {
    let swept = swept_aabb(query_shape, query_transform, direction, max_distance);
    hknp_shape::transform_aabb(&target_transform.inverse(), &swept)
}

fn cast_mesh(
    query_shape: &dyn Shape,
    query_transform: &Transform,
    mesh: &MeshShape,
    mesh_transform: &Transform,
    direction: Vector4,
    max_distance: f32,
    tolerance: f32,
) -> Option<ShapeCastHit> {
    let local_swept = local_swept_aabb(
        query_shape,
        query_transform,
        mesh_transform,
        direction,
        max_distance,
    );
    let mut best: Option<(f32, u32, ShapeCastHit)> = None;
    if mesh.bvh_nodes().is_empty() {
        return None;
    }
    let mut stack = vec![0usize];
    while let Some(node_index) = stack.pop() {
        let node = mesh.bvh_nodes()[node_index];
        if !node.aabb.overlaps(&local_swept) {
            continue;
        }
        match node.children {
            Some((left, right)) => {
                // Push right first so the left child is visited first
                // (deterministic traversal order).
                stack.push(right as usize);
                stack.push(left as usize);
            }
            None => {
                for &triangle_index in mesh.node_triangles(node_index) {
                    let triangle = mesh.triangles()[triangle_index as usize];
                    let points = [
                        mesh.vertices()[triangle[0] as usize],
                        mesh.vertices()[triangle[1] as usize],
                        mesh.vertices()[triangle[2] as usize],
                    ];
                    let leaf =
                        TriangleShape::with_convex_radius(points[0], points[1], points[2], 0.0);
                    let Some(hit) = cast_convex(
                        query_shape,
                        query_transform,
                        &leaf,
                        mesh_transform,
                        direction,
                        max_distance,
                        tolerance,
                    ) else {
                        continue;
                    };
                    let better = match &best {
                        Some((fraction, index, _)) => {
                            (hit.fraction, triangle_index) < (*fraction, *index)
                        }
                        None => true,
                    };
                    if better {
                        best = Some((hit.fraction, triangle_index, hit));
                    }
                }
            }
        }
    }
    best.map(|(_, _, hit)| hit)
}

fn cast_height_field(
    query_shape: &dyn Shape,
    query_transform: &Transform,
    heightfield: &HeightFieldShape,
    heightfield_transform: &Transform,
    direction: Vector4,
    max_distance: f32,
    tolerance: f32,
) -> Option<ShapeCastHit> {
    let local_swept = local_swept_aabb(
        query_shape,
        query_transform,
        heightfield_transform,
        direction,
        max_distance,
    );
    let size_x = heightfield.int_size_x();
    let size_z = heightfield.int_size_z();
    if size_x < 2 || size_z < 2 {
        return None;
    }
    let scale = heightfield.scale();
    let cell_min_x = (local_swept.min.x / scale.x).floor().max(0.0) as usize;
    let cell_max_x = (local_swept.max.x / scale.x).ceil().max(0.0) as usize;
    let cell_min_z = (local_swept.min.z / scale.z).floor().max(0.0) as usize;
    let cell_max_z = (local_swept.max.z / scale.z).ceil().max(0.0) as usize;
    let cell_min_x = cell_min_x.min(size_x - 2);
    let cell_min_z = cell_min_z.min(size_z - 2);
    let cell_max_x = cell_max_x.min(size_x - 2);
    let cell_max_z = cell_max_z.min(size_z - 2);
    if cell_min_x > cell_max_x || cell_min_z > cell_max_z {
        return None;
    }
    let mut best: Option<(f32, (usize, usize, usize), ShapeCastHit)> = None;
    for z in cell_min_z..=cell_max_z {
        for x in cell_min_x..=cell_max_x {
            let p00 = heightfield.vertex(x, z);
            let p10 = heightfield.vertex(x + 1, z);
            let p01 = heightfield.vertex(x, z + 1);
            let p11 = heightfield.vertex(x + 1, z + 1);
            for (points, sub) in [([p00, p10, p11], 0usize), ([p00, p11, p01], 1usize)] {
                let leaf = TriangleShape::with_convex_radius(points[0], points[1], points[2], 0.0);
                let Some(hit) = cast_convex(
                    query_shape,
                    query_transform,
                    &leaf,
                    heightfield_transform,
                    direction,
                    max_distance,
                    tolerance,
                ) else {
                    continue;
                };
                let key = (z, x, sub);
                let better = match &best {
                    Some((fraction, current, _)) => (hit.fraction, key) < (*fraction, *current),
                    None => true,
                };
                if better {
                    best = Some((hit.fraction, key, hit));
                }
            }
        }
    }
    best.map(|(_, _, hit)| hit)
}

fn cast_compound(
    query_shape: &dyn Shape,
    query_transform: &Transform,
    compound: &CompoundShape,
    compound_transform: &Transform,
    direction: Vector4,
    max_distance: f32,
    tolerance: f32,
) -> Option<ShapeCastHit> {
    let local_swept = local_swept_aabb(
        query_shape,
        query_transform,
        compound_transform,
        direction,
        max_distance,
    );
    let mut best: Option<(f32, usize, ShapeCastHit)> = None;
    for (child_index, child) in compound.children().iter().enumerate() {
        if !child.calc_aabb().overlaps(&local_swept) {
            continue;
        }
        let child_transform = compose_transform(compound_transform, &child.transform);
        let Some(hit) = shape_cast(
            query_shape,
            query_transform,
            child.shape.as_ref(),
            &child_transform,
            direction,
            max_distance,
            tolerance,
        ) else {
            continue;
        };
        let better = match &best {
            Some((fraction, index, _)) => (hit.fraction, child_index) < (*fraction, *index),
            None => true,
        };
        if better {
            best = Some((hit.fraction, child_index, hit));
        }
    }
    best.map(|(_, _, hit)| hit)
}

/// Compose a parent world transform with a child local transform.
fn compose_transform(parent: &Transform, child: &Transform) -> Transform {
    Transform::new(
        parent.rotation.mul_mat(&child.rotation),
        parent.transform_point(child.translation),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;
    use hknp_shape::{BoxShape, SphereShape};
    use std::sync::Arc;

    fn sphere(radius: f32) -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
    }

    fn box_shape(half_extents: Vector4) -> Arc<dyn Shape> {
        Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
    }

    #[test]
    fn sphere_cast_into_ground_reports_contact_distance() {
        let query = sphere(0.5);
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 5.0, 0.0));
        let ground = box_shape(Vector4::from_xyz(5.0, 0.5, 5.0));
        let ground_transform =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, -0.5, 0.0));
        let hit = shape_cast(
            query.as_ref(),
            &start,
            ground.as_ref(),
            &ground_transform,
            -Vector4::UNIT_Y,
            10.0,
            DEFAULT_SHAPE_CAST_TOLERANCE,
        )
        .expect("sphere must hit the ground");
        assert!(
            (hit.fraction - 4.5).abs() < 5e-3,
            "fraction {}",
            hit.fraction
        );
        assert!(
            hit.position.all_equal3(Vector4::ZERO, 1e-2),
            "position {:?}",
            hit.position
        );
        assert!(
            hit.normal.all_equal3(Vector4::UNIT_Y, 1e-2),
            "normal {:?}",
            hit.normal
        );
        assert!(!hit.start_penetrating);
    }

    #[test]
    fn cast_miss_beyond_range_is_none() {
        let query = sphere(0.5);
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 5.0, 0.0));
        let ground = box_shape(Vector4::from_xyz(5.0, 0.5, 5.0));
        let ground_transform =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, -0.5, 0.0));
        assert!(
            shape_cast(
                query.as_ref(),
                &start,
                ground.as_ref(),
                &ground_transform,
                -Vector4::UNIT_Y,
                3.0,
                DEFAULT_SHAPE_CAST_TOLERANCE,
            )
            .is_none()
        );
    }

    #[test]
    fn cast_starting_inside_reports_penetration() {
        let query = sphere(1.0);
        let start = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::ZERO);
        let target = sphere(1.0);
        let hit = shape_cast(
            query.as_ref(),
            &start,
            target.as_ref(),
            &Transform::IDENTITY,
            Vector4::UNIT_Y,
            10.0,
            DEFAULT_SHAPE_CAST_TOLERANCE,
        )
        .expect("overlapping cast must report at fraction 0");
        assert_eq!(hit.fraction, 0.0);
        assert!(hit.start_penetrating);
    }

    #[test]
    fn cast_parallel_to_surface_misses() {
        let query = sphere(0.5);
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 5.0, 0.0));
        let wall = box_shape(Vector4::from_xyz(0.5, 5.0, 5.0));
        let wall_transform =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(3.0, 0.0, 0.0));
        assert!(
            shape_cast(
                query.as_ref(),
                &start,
                wall.as_ref(),
                &wall_transform,
                Vector4::UNIT_Z,
                10.0,
                DEFAULT_SHAPE_CAST_TOLERANCE,
            )
            .is_none()
        );
    }

    #[test]
    fn capsule_cast_hits_mesh_triangle() {
        // A downward capsule above a single flat triangle at y = 0.
        let query = Arc::new(hknp_shape::CapsuleShape::new(
            Vector4::from_xyz(0.0, 0.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            0.5,
        )) as Arc<dyn Shape>;
        let mesh = Arc::new(MeshShape::new(
            vec![
                Vector4::from_xyz(-2.0, 0.0, -2.0),
                Vector4::from_xyz(2.0, 0.0, -2.0),
                Vector4::from_xyz(0.0, 0.0, 2.0),
            ],
            vec![[0, 1, 2]],
        )) as Arc<dyn Shape>;
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 5.0, 0.0));
        let hit = shape_cast(
            query.as_ref(),
            &start,
            mesh.as_ref(),
            &Transform::IDENTITY,
            -Vector4::UNIT_Y,
            10.0,
            DEFAULT_SHAPE_CAST_TOLERANCE,
        )
        .expect("capsule must hit the mesh triangle");
        // The capsule's lowest point is 0.5 below its shape center.
        assert!(
            (hit.fraction - 4.5).abs() < 1e-2,
            "fraction {}",
            hit.fraction
        );
        assert!(
            hit.normal.dot3(Vector4::UNIT_Y) > 0.9,
            "normal {:?}",
            hit.normal
        );
    }

    #[test]
    fn cast_against_mesh_uses_bvh_and_respects_range() {
        // Two separated quads; a short cast over the first must miss the
        // second and a long one must reach it.
        let mesh = Arc::new(MeshShape::new(
            vec![
                Vector4::from_xyz(-1.0, 0.0, -1.0),
                Vector4::from_xyz(1.0, 0.0, -1.0),
                Vector4::from_xyz(1.0, 0.0, 1.0),
                Vector4::from_xyz(-1.0, 0.0, 1.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )) as Arc<dyn Shape>;
        let query = sphere(0.25);
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 3.0, 0.0));
        assert!(
            shape_cast(
                query.as_ref(),
                &start,
                mesh.as_ref(),
                &Transform::IDENTITY,
                -Vector4::UNIT_Y,
                1.0,
                DEFAULT_SHAPE_CAST_TOLERANCE,
            )
            .is_none()
        );
        let hit = shape_cast(
            query.as_ref(),
            &start,
            mesh.as_ref(),
            &Transform::IDENTITY,
            -Vector4::UNIT_Y,
            10.0,
            DEFAULT_SHAPE_CAST_TOLERANCE,
        )
        .expect("long cast must hit the mesh");
        assert!(
            (hit.fraction - 2.75).abs() < 5e-3,
            "fraction {}",
            hit.fraction
        );
    }

    #[test]
    fn cast_against_height_field_finds_the_surface() {
        let heightfield = Arc::new(
            HeightFieldShape::new(3, 3, Vector4::ONE, vec![0.0; 9]).expect("valid height field"),
        ) as Arc<dyn Shape>;
        let query = sphere(1.0);
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 5.0, 1.0));
        let hit = shape_cast(
            query.as_ref(),
            &start,
            heightfield.as_ref(),
            &Transform::IDENTITY,
            -Vector4::UNIT_Y,
            10.0,
            DEFAULT_SHAPE_CAST_TOLERANCE,
        )
        .expect("sphere must hit the flat height field");
        assert!(
            (hit.fraction - 4.0).abs() < 1e-2,
            "fraction {}",
            hit.fraction
        );
    }

    #[test]
    fn cast_against_compound_selects_the_nearest_child() {
        use hknp_shape::{CompoundShape, ShapeInstance};
        let compound = Arc::new(CompoundShape::new(vec![
            ShapeInstance::new(
                Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(-3.0, 0.0, 0.0)),
                sphere(1.0),
            ),
            ShapeInstance::new(
                Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(3.0, 0.0, 0.0)),
                sphere(1.0),
            ),
        ])) as Arc<dyn Shape>;
        let query = sphere(0.5);
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(-3.0, 5.0, 0.0));
        let hit = shape_cast(
            query.as_ref(),
            &start,
            compound.as_ref(),
            &Transform::IDENTITY,
            -Vector4::UNIT_Y,
            10.0,
            DEFAULT_SHAPE_CAST_TOLERANCE,
        )
        .expect("cast must hit the first child");
        // sphere top at y = 1, query bottom offset 0.5: travel 3.5.
        assert!(
            (hit.fraction - 3.5).abs() < 1e-2,
            "fraction {}",
            hit.fraction
        );
    }

    #[test]
    fn non_convex_query_shape_is_rejected() {
        let mesh = Arc::new(MeshShape::new(
            vec![
                Vector4::ZERO,
                Vector4::from_xyz(1.0, 0.0, 0.0),
                Vector4::from_xyz(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        )) as Arc<dyn Shape>;
        let target = sphere(1.0);
        assert!(
            shape_cast(
                mesh.as_ref(),
                &Transform::IDENTITY,
                target.as_ref(),
                &Transform::IDENTITY,
                Vector4::UNIT_Y,
                10.0,
                DEFAULT_SHAPE_CAST_TOLERANCE,
            )
            .is_none()
        );
    }

    #[test]
    fn casts_are_deterministic() {
        let query = sphere(0.5);
        let ground = box_shape(Vector4::from_xyz(5.0, 0.5, 5.0));
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 5.0, 0.0));
        let ground_transform =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, -0.5, 0.0));
        let first = shape_cast(
            query.as_ref(),
            &start,
            ground.as_ref(),
            &ground_transform,
            -Vector4::UNIT_Y,
            10.0,
            1e-3,
        );
        let second = shape_cast(
            query.as_ref(),
            &start,
            ground.as_ref(),
            &ground_transform,
            -Vector4::UNIT_Y,
            10.0,
            1e-3,
        );
        assert_eq!(first, second);
    }
}
