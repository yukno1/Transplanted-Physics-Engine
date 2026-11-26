//! World-level queries (`Collide/Query/` subset): ray casts, shape casts,
//! closest points and AABB overlap queries against all live bodies.
//!
//! Bodies are visited in body-manager slot order; ties are broken by the
//! lower `BodyId`, so results are deterministic for a given scene. The
//! [`QueryFilter`] controls which bodies take part (ignoring a cast body,
//! restricting motion types). Competing results are compared by
//! `(distance, body)` so traversal order cannot leak into the result.
//!
//! Composite bodies (mesh, height field, compound) are supported by all ray
//! and shape casts through [`crate::phys_collide`]'s leaf traversal. Closest-point
//! queries remain convex-only (GJK needs a support function).

use crate::base_math::{Aabb, Transform, Vector4};
use crate::phys_body::Body;
use crate::phys_collide::query::ray_aabb;
use crate::phys_collide::{
    DEFAULT_SHAPE_CAST_TOLERANCE, closest_points as gjk_closest_points, point_distance, shape_cast,
};
use crate::phys_shape::Shape;
use crate::phys_types::{BodyId, ShapeKey};

use crate::phys_world::world::World;

/// A world ray-cast hit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayCastHit {
    /// The hit body.
    pub body: BodyId,
    /// Distance from the ray origin to the hit along the normalized direction.
    pub hit_fraction: f32,
    /// Hit point on the body's shape surface (world space).
    pub position: Vector4,
    /// Outward surface normal at the hit.
    pub normal: Vector4,
    /// Leaf shape key (convex-hull face or composite leaf; invalid for other
    /// direct convex hits).
    pub shape_key: ShapeKey,
}

/// A world shape-cast hit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShapeCastHitInfo {
    /// The hit body.
    pub body: BodyId,
    /// Distance travelled by the query shape before contact.
    pub distance: f32,
    /// Contact point on the hit body (world space).
    pub position: Vector4,
    /// Contact normal pointing from the hit body to the query shape.
    pub normal: Vector4,
    /// Witness point on the query shape at the hit pose.
    pub point_on_query: Vector4,
    /// Whether the query shape started overlapping the body.
    pub start_penetrating: bool,
    /// Leaf shape key (invalid for a direct convex hit).
    pub shape_key: ShapeKey,
}

/// Closest point on a body's shape to a query point or query shape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosestPointHit {
    /// The hit body.
    pub body: BodyId,
    /// Closest point on the body's shape surface (world space).
    pub point: Vector4,
    /// Distance from the query; `0` when the query is inside the shape.
    pub distance: f32,
}

/// Closest points between a convex query shape and a body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosestPointsHit {
    /// The hit body.
    pub body: BodyId,
    /// Closest point on the body's shape (world space).
    pub point_on_body: Vector4,
    /// Closest point on the query shape (world space).
    pub point_on_query: Vector4,
    /// Unit separating normal, from the body towards the query shape.
    pub normal: Vector4,
    /// Signed distance (negative when the shapes overlap).
    pub distance: f32,
}

/// Filter for world queries (`hknpCollisionQuery` filter subset).
///
/// By default every body participates. Builder methods narrow the set; the
/// query results never include ignored bodies and ties are still resolved by
/// the lower body id.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QueryFilter {
    /// Body to exclude (for shape casts this is usually the casting body).
    pub ignore_body: Option<BodyId>,
    /// Include static bodies (default `true`).
    pub include_static: bool,
    /// Include dynamic bodies (default `true`).
    pub include_dynamic: bool,
    /// Include keyframed bodies (default `true`).
    pub include_keyframed: bool,
    /// Query-side collision filter info (`hknpQueryFilterData::
    /// m_collisionFilterInfo`, default `0`). When the world has a
    /// [`proton::collide::CollisionFilter`], each candidate body must pass
    /// `is_query_enabled(filter_info, body.collision_filter_info)`.
    pub filter_info: u32,
}

impl Default for QueryFilter {
    fn default() -> Self {
        Self {
            ignore_body: None,
            include_static: true,
            include_dynamic: true,
            include_keyframed: true,
            filter_info: 0,
        }
    }
}

impl QueryFilter {
    /// Exclude one body from the query.
    #[must_use]
    pub fn with_ignore_body(mut self, body: BodyId) -> Self {
        self.ignore_body = Some(body);
        self
    }

    /// Set the query-side collision filter info (`hknpQueryFilterData`).
    #[must_use]
    pub fn with_filter_info(mut self, filter_info: u32) -> Self {
        self.filter_info = filter_info;
        self
    }

    /// Include or exclude static bodies.
    #[must_use]
    pub fn include_static(mut self, include: bool) -> Self {
        self.include_static = include;
        self
    }

    /// Include or exclude dynamic bodies.
    #[must_use]
    pub fn include_dynamic(mut self, include: bool) -> Self {
        self.include_dynamic = include;
        self
    }

    /// Include or exclude keyframed bodies.
    #[must_use]
    pub fn include_keyframed(mut self, include: bool) -> Self {
        self.include_keyframed = include;
        self
    }

    fn matches(&self, body: &Body) -> bool {
        if self.ignore_body == Some(body.id) {
            return false;
        }
        if body.is_static() && !self.include_static {
            return false;
        }
        if body.is_dynamic() && !self.include_dynamic {
            return false;
        }
        if body.is_keyframed() && !self.include_keyframed {
            return false;
        }
        true
    }
}

impl World {
    /// Motion-type check plus the world's collision filter decision
    /// (`hknpCollisionFilter::isCollisionEnabled(queryFilterData, body)`).
    fn query_allows(&self, filter: &QueryFilter, body: &Body) -> bool {
        if !filter.matches(body) {
            return false;
        }
        if let Some(collision_filter) = self.collision_filter() {
            if !collision_filter.is_query_enabled(filter.filter_info, body.collision_filter_info) {
                return false;
            }
        }
        true
    }

    /// Cast a ray through the world and return the closest hit
    /// (`castRay` closest-hit subset) with the default filter.
    ///
    /// `direction` is normalized defensively; `max_length` is measured in
    /// world units. Bodies are tested against their cached AABB first and then
    /// against their shape(s), including composite leaves.
    pub fn raycast(
        &self,
        origin: Vector4,
        direction: Vector4,
        max_length: f32,
    ) -> Option<RayCastHit> {
        self.raycast_filtered(origin, direction, max_length, &QueryFilter::default())
    }

    /// Ray cast restricted by `filter`.
    pub fn raycast_filtered(
        &self,
        origin: Vector4,
        direction: Vector4,
        max_length: f32,
        filter: &QueryFilter,
    ) -> Option<RayCastHit> {
        let direction_length = direction.length3();
        if direction_length <= 1e-10 || max_length <= 0.0 {
            return None;
        }
        let direction = direction * (1.0 / direction_length);

        let mut best: Option<RayCastHit> = None;
        for body in self.bodies() {
            if !self.query_allows(filter, body) {
                continue;
            }
            // Coarse AABB clip against the ray interval.
            let Some((entry, exit)) = ray_aabb(origin, direction, &body.aabb) else {
                continue;
            };
            if entry > max_length || exit < 0.0 {
                continue;
            }
            let Some(hit) = crate::phys_collide::raycast(
                body.shape.as_ref(),
                &body.transform,
                origin,
                direction,
                max_length,
            ) else {
                continue;
            };
            let candidate = RayCastHit {
                body: body.id,
                hit_fraction: hit.hit_fraction,
                position: hit.position,
                normal: hit.normal,
                shape_key: hit.shape_key,
            };
            best = Some(match best {
                Some(current)
                    if current.hit_fraction < candidate.hit_fraction
                        || (current.hit_fraction == candidate.hit_fraction
                            && current.body < candidate.body) =>
                {
                    current
                }
                _ => candidate,
            });
        }
        best
    }

    /// Sweep a convex query shape through the world and return the first
    /// contact (`castShape` closest-hit subset).
    ///
    /// The query shape keeps its start orientation. `direction` is normalized
    /// defensively; `max_distance` is measured in world units. Composite bodies
    /// are supported leaf-by-leaf.
    pub fn cast_shape(
        &self,
        shape: &dyn Shape,
        transform: &Transform,
        direction: Vector4,
        max_distance: f32,
        filter: &QueryFilter,
    ) -> Option<ShapeCastHitInfo> {
        if !shape.is_convex() {
            return None;
        }
        let direction_length = direction.length3();
        if direction_length <= 1e-10 || max_distance <= 0.0 {
            return None;
        }
        let direction = direction * (1.0 / direction_length);
        let swept = swept_aabb(shape, transform, direction, max_distance);

        let mut best: Option<ShapeCastHitInfo> = None;
        for body in self.bodies() {
            if !self.query_allows(filter, body) {
                continue;
            }
            if !body.aabb.overlaps(&swept) {
                continue;
            }
            let Some(hit) = shape_cast(
                shape,
                transform,
                body.shape.as_ref(),
                &body.transform,
                direction,
                max_distance,
                DEFAULT_SHAPE_CAST_TOLERANCE,
            ) else {
                continue;
            };
            let candidate = ShapeCastHitInfo {
                body: body.id,
                distance: hit.fraction,
                position: hit.position,
                normal: hit.normal,
                point_on_query: hit.point_on_query,
                start_penetrating: hit.start_penetrating,
                shape_key: hit.shape_key,
            };
            best = Some(match best {
                Some(current)
                    if current.distance < candidate.distance
                        || (current.distance == candidate.distance
                            && current.body < candidate.body) =>
                {
                    current
                }
                _ => candidate,
            });
        }
        best
    }

    /// Find the closest point on any body shape to `position`
    /// (`getClosestPoint` closest-hit subset).
    ///
    /// `max_distance` bounds the search; convex shapes only. When the point
    /// lies inside a shape, the hit reports that body with distance `0`.
    pub fn closest_point(&self, position: Vector4, max_distance: f32) -> Option<ClosestPointHit> {
        self.closest_point_filtered(position, max_distance, &QueryFilter::default())
    }

    /// Point query restricted by `filter`.
    pub fn closest_point_filtered(
        &self,
        position: Vector4,
        max_distance: f32,
        filter: &QueryFilter,
    ) -> Option<ClosestPointHit> {
        let mut best: Option<ClosestPointHit> = None;
        for body in self.bodies() {
            if !self.query_allows(filter, body) || !body.shape.is_convex() {
                continue;
            }
            // Coarse rejection: distance to the cached AABB (clamped point).
            let clamped = Vector4::from_xyz(
                position.x.clamp(body.aabb.min.x, body.aabb.max.x),
                position.y.clamp(body.aabb.min.y, body.aabb.max.y),
                position.z.clamp(body.aabb.min.z, body.aabb.max.z),
            );
            if (position - clamped).length3() > max_distance {
                continue;
            }
            let Some(result) = point_distance(body.shape.as_ref(), &body.transform, position)
            else {
                continue;
            };
            let distance = if result.intersects {
                0.0
            } else {
                result.distance
            };
            if distance > max_distance {
                continue;
            }
            let candidate = ClosestPointHit {
                body: body.id,
                point: result.point_b,
                distance,
            };
            best = Some(match best {
                Some(current)
                    if current.distance < candidate.distance
                        || (current.distance == candidate.distance
                            && current.body < candidate.body) =>
                {
                    current
                }
                _ => candidate,
            });
        }
        best
    }

    /// Closest points between a convex query shape and the nearest body
    /// (`getClosestPoints` closest-hit subset).
    ///
    /// Reports a negative distance when the query shape overlaps the body.
    pub fn closest_points_to_shape(
        &self,
        shape: &dyn Shape,
        transform: &Transform,
        max_distance: f32,
        filter: &QueryFilter,
    ) -> Option<ClosestPointsHit> {
        if !shape.is_convex() || max_distance <= 0.0 {
            return None;
        }
        let query_aabb = shape.calc_aabb(transform);
        let mut expanded = query_aabb;
        expanded.expand_by(max_distance);

        let mut best: Option<ClosestPointsHit> = None;
        for body in self.bodies() {
            if !self.query_allows(filter, body) || !body.shape.is_convex() {
                continue;
            }
            if !body.aabb.overlaps(&expanded) {
                continue;
            }
            let Some(result) =
                gjk_closest_points(shape, transform, body.shape.as_ref(), &body.transform)
            else {
                continue;
            };
            if result.intersects {
                // Overlapping convex pairs get their penetration depth (and
                // surface points) from the EPA-backed collision query.
                let candidate = match crate::phys_collide::collide(
                    shape,
                    transform,
                    body.shape.as_ref(),
                    &body.transform,
                ) {
                    Some(manifold) if !manifold.points.is_empty() => {
                        let point = manifold.points[0];
                        let distance = point.distance;
                        if distance > max_distance {
                            continue;
                        }
                        ClosestPointsHit {
                            body: body.id,
                            point_on_body: point.position,
                            point_on_query: point.position + manifold.normal * distance,
                            normal: manifold.normal,
                            distance,
                        }
                    }
                    _ => {
                        if 0.0 > max_distance {
                            continue;
                        }
                        ClosestPointsHit {
                            body: body.id,
                            point_on_body: result.point_b,
                            point_on_query: result.point_a,
                            normal: result.normal,
                            distance: 0.0,
                        }
                    }
                };
                best = Some(match best {
                    Some(current)
                        if current.distance < candidate.distance
                            || (current.distance == candidate.distance
                                && current.body < candidate.body) =>
                    {
                        current
                    }
                    _ => candidate,
                });
                continue;
            }
            let distance = result.distance;
            if distance > max_distance {
                continue;
            }
            let candidate = ClosestPointsHit {
                body: body.id,
                point_on_body: result.point_b,
                point_on_query: result.point_a,
                normal: result.normal,
                distance,
            };
            best = Some(match best {
                Some(current)
                    if current.distance < candidate.distance
                        || (current.distance == candidate.distance
                            && current.body < candidate.body) =>
                {
                    current
                }
                _ => candidate,
            });
        }
        best
    }

    /// All bodies whose cached world AABB overlaps `aabb`
    /// (`hknpAabbQuery` subset), in body slot order.
    pub fn query_aabb(&self, aabb: &Aabb, filter: &QueryFilter) -> Vec<BodyId> {
        self.bodies()
            .filter(|body| self.query_allows(filter, body) && body.aabb.overlaps(aabb))
            .map(|body| body.id)
            .collect()
    }

    /// The cached transform of a body (world space).
    pub fn body_transform(&self, body_id: BodyId) -> Option<Transform> {
        self.body(body_id).map(|body| body.transform)
    }
}

/// Swept world AABB of a shape for broad-phase culling.
fn swept_aabb(
    shape: &dyn Shape,
    transform: &Transform,
    direction: Vector4,
    max_distance: f32,
) -> Aabb {
    let mut swept = shape.calc_aabb(transform);
    let end = Transform::new(
        transform.rotation,
        transform.translation + direction * max_distance,
    );
    swept.include_aabb(&shape.calc_aabb(&end));
    swept
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_math::Quaternion;
    use crate::phys_body::BodyCinfo;
    use crate::phys_shape::{BoxShape, MeshShape, Shape, SphereShape};
    use crate::phys_types::MotionType;
    use crate::phys_world::WorldCinfo;
    use crate::base_math::Vec4Ext;
    use glam::Vec4;
    use std::sync::Arc;

    fn sphere_shape(radius: f32) -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(Vec4::ZERO, radius)) as Arc<dyn Shape>
    }

    fn box_shape(half_extents: Vec4) -> Arc<dyn Shape> {
        Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
    }

    fn mesh_ground(size: f32) -> Arc<dyn Shape> {
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
    fn raycast_finds_closest_body() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 10.0, 0.0)),
        );
        let near = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 4.0, 0.0)),
        );

        let hit = world
            .raycast(Vector4::ZERO, Vector4::UNIT_Y, 50.0)
            .expect("ray must hit a sphere");
        assert_eq!(hit.body, near);
        assert!(
            (hit.hit_fraction - 3.0).abs() < 1e-2,
            "fraction {}",
            hit.hit_fraction
        );
        assert!(
            hit.normal.all_equal3(-Vector4::UNIT_Y, 1e-2),
            "normal {:?}",
            hit.normal
        );

        assert!(world.raycast(Vector4::ZERO, Vector4::UNIT_Y, 2.0).is_none());
    }

    #[test]
    fn raycast_breaks_ties_by_lower_body_id() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let first = world.create_body(
            &BodyCinfo::new(box_shape(Vec4::splat(1.0)))
                .with_position(Vector4::from_xyz(0.0, 10.0, 0.0)),
        );
        let second = world.create_body(
            &BodyCinfo::new(box_shape(Vec4::splat(1.0)))
                .with_position(Vector4::from_xyz(0.0, 10.0, 0.0)),
        );
        // Two identical boxes share a pose: both produce the same hit fraction
        // and the lower body id wins.
        let hit = world
            .raycast(Vector4::ZERO, Vector4::UNIT_Y, 100.0)
            .expect("ray must hit a box");
        assert_eq!(hit.body, first);
        assert!(first < second);
    }

    #[test]
    fn raycast_hits_rotated_body() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        world.create_body(
            &BodyCinfo::new(box_shape(Vec4::splat(1.0)))
                .with_orientation(Quaternion::from_axis_angle(
                    Vector4::UNIT_Z,
                    core::f32::consts::FRAC_PI_4,
                ))
                .with_position(Vector4::from_xyz(0.0, 5.0, 0.0)),
        );
        let hit = world
            .raycast(Vector4::from_xyz(0.0, 10.0, 0.0), -Vector4::UNIT_Y, 20.0)
            .expect("ray must hit the rotated box");
        assert!(hit.hit_fraction < 5.0, "fraction {}", hit.hit_fraction);
    }

    #[test]
    fn raycast_hits_mesh_body() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let ground = world.create_body(
            &BodyCinfo::new(mesh_ground(5.0)).with_position(Vector4::from_xyz(0.0, 0.0, 0.0)),
        );
        let hit = world
            .raycast(Vector4::from_xyz(1.0, 4.0, 1.0), -Vector4::UNIT_Y, 10.0)
            .expect("ray must hit the mesh ground");
        assert_eq!(hit.body, ground);
        assert!(
            (hit.hit_fraction - 4.0).abs() < 1e-3,
            "fraction {}",
            hit.hit_fraction
        );
        assert!(hit.normal.dot3(Vector4::UNIT_Y) > 0.9);
    }

    #[test]
    fn raycast_filter_excludes_bodies() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let ignored = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(0.0, 4.0, 0.0)),
        );
        let far = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 10.0, 0.0)),
        );
        let filter = QueryFilter::default().with_ignore_body(ignored);
        let hit = world
            .raycast_filtered(Vector4::ZERO, Vector4::UNIT_Y, 50.0, &filter)
            .expect("ray must skip the ignored body");
        assert_eq!(hit.body, far);

        let filter = QueryFilter::default()
            .include_static(false)
            .include_keyframed(false)
            .include_dynamic(true);
        let hit = world
            .raycast_filtered(Vector4::ZERO, Vector4::UNIT_Y, 50.0, &filter)
            .expect("dynamic-only filter must hit the ignored body");
        assert_eq!(hit.body, ignored);
    }

    #[test]
    fn closest_point_finds_nearest_body() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let near = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 3.0, 0.0)),
        );
        world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 20.0, 0.0)),
        );

        let hit = world
            .closest_point(Vector4::from_xyz(0.0, 0.0, 0.0), 50.0)
            .expect("a sphere is within range");
        assert_eq!(hit.body, near);
        assert!(
            (hit.distance - 2.0).abs() < 1e-2,
            "distance {}",
            hit.distance
        );
        assert!(
            hit.point.all_equal3(Vector4::from_xyz(0.0, 2.0, 0.0), 1e-2),
            "point {:?}",
            hit.point
        );

        assert!(world.closest_point(Vector4::ZERO, 1.0).is_none());
    }

    #[test]
    fn closest_point_reports_inside_shapes() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let body = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 5.0, 0.0)),
        );
        let hit = world
            .closest_point(Vector4::from_xyz(0.0, 5.0, 0.0), 10.0)
            .expect("point is inside a sphere");
        assert_eq!(hit.body, body);
        assert_eq!(hit.distance, 0.0);
    }

    #[test]
    fn closest_points_to_shape_finds_nearest_surface() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let body = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 4.0, 0.0)),
        );
        let query = sphere_shape(0.5);
        let hit = world
            .closest_points_to_shape(
                query.as_ref(),
                &Transform::from_quaternion(Quaternion::IDENTITY, Vector4::ZERO),
                10.0,
                &QueryFilter::default(),
            )
            .expect("a separated sphere is within range");
        assert_eq!(hit.body, body);
        assert!(
            (hit.distance - 2.5).abs() < 1e-2,
            "distance {}",
            hit.distance
        );
        assert!(
            hit.point_on_body
                .all_equal3(Vector4::from_xyz(0.0, 3.0, 0.0), 1e-2),
            "point_on_body {:?}",
            hit.point_on_body
        );
        assert!(
            hit.point_on_query
                .all_equal3(Vector4::from_xyz(0.0, 0.5, 0.0), 1e-2),
            "point_on_query {:?}",
            hit.point_on_query
        );
        assert!(hit.normal.dot3(Vector4::UNIT_Y) < -0.9);
    }

    #[test]
    fn closest_points_report_overlap_as_negative_distance() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let body = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 1.0, 0.0)),
        );
        let query = sphere_shape(0.5);
        let filter = QueryFilter::default();
        let hit = world
            .closest_points_to_shape(query.as_ref(), &Transform::IDENTITY, 10.0, &filter)
            .expect("overlapping query must report a hit");
        assert_eq!(hit.body, body);
        assert!(
            hit.distance < 0.0,
            "overlap distance {} must be negative",
            hit.distance
        );
    }

    #[test]
    fn cast_shape_finds_mesh_ground() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let ground = world.create_body(&BodyCinfo::new(mesh_ground(5.0)));
        let query = sphere_shape(0.5);
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 5.0, 0.0));
        let hit = world
            .cast_shape(
                query.as_ref(),
                &start,
                -Vector4::UNIT_Y,
                10.0,
                &QueryFilter::default(),
            )
            .expect("sphere cast must hit the mesh ground");
        assert_eq!(hit.body, ground);
        assert!(
            (hit.distance - 4.5).abs() < 5e-3,
            "distance {}",
            hit.distance
        );
        assert!(
            hit.normal.dot3(Vector4::UNIT_Y) > 0.9,
            "normal {:?}",
            hit.normal
        );
        assert!(!hit.start_penetrating);
    }

    #[test]
    fn cast_shape_ignores_the_casting_body() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let ground = world.create_body(
            &BodyCinfo::new(box_shape(Vec4::from_xyz(10.0, 0.5, 10.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        let caster = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5)).with_position(Vector4::from_xyz(0.0, 5.0, 0.0)),
        );
        let query = sphere_shape(0.5);
        let start =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 5.0, 0.0));
        let filter = QueryFilter::default().with_ignore_body(caster);
        let hit = world
            .cast_shape(query.as_ref(), &start, -Vector4::UNIT_Y, 10.0, &filter)
            .expect("cast must hit the ground, not the caster");
        assert_eq!(hit.body, ground);
    }

    #[test]
    fn cast_shape_reports_start_penetration() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let body = world.create_body(
            &BodyCinfo::new(box_shape(Vec4::splat(1.0)))
                .with_position(Vector4::from_xyz(0.0, 0.0, 0.0)),
        );
        let query = sphere_shape(0.5);
        let start = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::ZERO);
        let hit = world
            .cast_shape(
                query.as_ref(),
                &start,
                Vector4::UNIT_X,
                5.0,
                &QueryFilter::default(),
            )
            .expect("overlapping cast must report");
        assert_eq!(hit.body, body);
        assert!(hit.start_penetrating);
        assert_eq!(hit.distance, 0.0);
    }

    #[test]
    fn aabb_query_reports_overlapping_bodies_in_slot_order() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let first = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(0.0, 0.0, 0.0)),
        );
        let second = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(1.5, 0.0, 0.0)),
        );
        world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(20.0, 0.0, 0.0)),
        );
        let aabb = Aabb::from_center_half_extents(Vector4::ZERO, Vector4::splat(1.0));
        let bodies = world.query_aabb(&aabb, &QueryFilter::default());
        assert_eq!(bodies, vec![first, second]);

        let filter = QueryFilter::default().with_ignore_body(first);
        assert_eq!(
            world.query_aabb(&aabb, &filter),
            vec![second],
            "filters apply to AABB queries too"
        );
    }

    #[test]
    fn raycast_respects_dynamic_motion() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let body = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(0.0, 10.0, 0.0))
                .with_linear_velocity(-Vector4::UNIT_Y),
        );
        world.step(0.5);
        let hit = world
            .raycast(Vector4::ZERO, Vector4::UNIT_Y, 50.0)
            .expect("sphere moved into the ray");
        assert_eq!(hit.body, body);
        // Sphere center is now at y = 10 - 0.5*1 = 9.5; surface at 8.5.
        assert!(
            (hit.hit_fraction - 8.5).abs() < 1e-1,
            "fraction {}",
            hit.hit_fraction
        );
    }
}
