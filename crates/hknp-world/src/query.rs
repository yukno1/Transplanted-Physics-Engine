//! World-level queries (`Collide/Query/` subset): ray casts and
//! closest-point queries against all live bodies.
//!
//! Bodies are visited in body-manager slot order; ties are broken by the
//! lower `BodyId`, so results are deterministic for a given scene. Collision
//! flags and filters do not affect queries (matching the SDK, where filtering
//! is a separate concern). Non-convex shapes (mesh/compound/height field) are
//! skipped: leaf traversal is future work.

use hk_math::{Transform, Vector4};
use hknp_collide::point_distance;
use hknp_collide::query::ray_aabb;
use hknp_types::BodyId;

use crate::world::World;

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
}

/// Closest point on a body's shape to a query point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosestPointHit {
    /// The hit body.
    pub body: BodyId,
    /// Closest point on the shape surface (world space).
    pub point: Vector4,
    /// Distance from the query point; `0` when the point is inside the shape.
    pub distance: f32,
}

impl World {
    /// Cast a ray through the world and return the closest hit
    /// (`castRay` closest-hit subset).
    ///
    /// `direction` is normalized defensively; `max_length` is measured in
    /// world units. Bodies are tested against their cached AABB first and
    /// then via a GJK-based conservative-advancement shape cast.
    pub fn raycast(
        &self,
        origin: Vector4,
        direction: Vector4,
        max_length: f32,
    ) -> Option<RayCastHit> {
        let direction_length = direction.length3();
        if direction_length <= 1e-10 || max_length <= 0.0 {
            return None;
        }
        let direction = direction / direction_length;

        let mut best: Option<RayCastHit> = None;
        for body in self.bodies() {
            // Coarse AABB clip against the ray interval.
            let Some((entry, exit)) = ray_aabb(origin, direction, &body.aabb) else {
                continue;
            };
            if entry > max_length || exit < 0.0 {
                continue;
            }
            let Some(hit) = hknp_collide::raycast(
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

    /// Find the closest point on any body shape to `position`
    /// (`getClosestPoint` closest-hit subset).
    ///
    /// `max_distance` bounds the search; convex shapes only. When the point
    /// lies inside a shape, the hit reports that body with distance `0`.
    pub fn closest_point(&self, position: Vector4, max_distance: f32) -> Option<ClosestPointHit> {
        let mut best: Option<ClosestPointHit> = None;
        for body in self.bodies() {
            if !body.shape.is_convex() {
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

    /// The cached transform of a body (world space).
    pub fn body_transform(&self, body_id: BodyId) -> Option<Transform> {
        self.body(body_id).map(|body| body.transform)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WorldCinfo;
    use hk_math::Quaternion;
    use hknp_body::BodyCinfo;
    use hknp_shape::{BoxShape, Shape, SphereShape};
    use hknp_types::MotionType;
    use std::sync::Arc;

    fn sphere_shape(radius: f32) -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
    }

    fn box_shape(half_extents: Vector4) -> Arc<dyn Shape> {
        Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
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
            &BodyCinfo::new(box_shape(Vector4::splat(1.0)))
                .with_position(Vector4::from_xyz(0.0, 10.0, 0.0)),
        );
        let second = world.create_body(
            &BodyCinfo::new(box_shape(Vector4::splat(1.0)))
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
            &BodyCinfo::new(box_shape(Vector4::splat(1.0)))
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
