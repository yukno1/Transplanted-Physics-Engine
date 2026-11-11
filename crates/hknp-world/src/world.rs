//! `hknpWorld` — world assembly and single-threaded step pipeline.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use hk_math::{Transform, Vector4};
use hknp_body::{Body, BodyCinfo, BodyManager, Motion, MotionManager};
use hknp_collide::{BodyAabb, BroadPhase, BroadPhaseConfig, collide_with_tolerance};
use hknp_shape::Shape;
use hknp_solver::{ContactConstraint, Solver, SolverBody, SolverSettings};
use hknp_types::{BodyId, CollisionFlags, MotionId, MotionType};

use crate::{BodySnapshot, ContactManifold, WorldCinfo, WorldSnapshot, WorldStats};

/// Immutable narrow-phase candidate (body pair plus shape/transform copies).
#[derive(Clone)]
struct ContactCandidate {
    body_a: BodyId,
    body_b: BodyId,
    shape_a: Arc<dyn Shape>,
    transform_a: Transform,
    shape_b: Arc<dyn Shape>,
    transform_b: Transform,
}

/// Physics world (`hknpWorld`).
///
/// Owns body/motion managers, the broad phase and the contact solver. Phase 8
/// [`World::step`] runs collide → solve → integrate over convex shapes using a
/// deterministic single-threaded pipeline.
#[derive(Debug)]
pub struct World {
    cinfo: WorldCinfo,
    gravity: Vector4,
    bodies: BodyManager,
    motions: MotionManager,
    broad_phase: BroadPhase,
    solver: Solver,
    step_count: u64,
    last_delta_time: f32,
    last_manifold_count: usize,
    last_contact_count: usize,
}

impl World {
    /// Create a world from construction info.
    pub fn new(cinfo: WorldCinfo) -> Self {
        let gravity = cinfo.gravity;
        let solver_settings = SolverSettings {
            iterations: cinfo.solver_iterations,
            ..SolverSettings::default()
        };
        Self {
            bodies: BodyManager::with_capacity(cinfo.body_buffer_capacity),
            motions: MotionManager::with_capacity(cinfo.motion_buffer_capacity),
            broad_phase: BroadPhase::new(BroadPhaseConfig::default()),
            solver: Solver::new(solver_settings),
            cinfo,
            gravity,
            step_count: 0,
            last_delta_time: 0.0,
            last_manifold_count: 0,
            last_contact_count: 0,
        }
    }

    /// The construction info used to create this world.
    pub fn cinfo(&self) -> &WorldCinfo {
        &self.cinfo
    }

    /// Current gravity (`getGravity`).
    pub fn gravity(&self) -> Vector4 {
        self.gravity
    }

    /// Set gravity (`setGravity`).
    pub fn set_gravity(&mut self, gravity: Vector4) {
        self.gravity = gravity;
    }

    /// Number of live bodies.
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Number of solver steps taken (`getStepCount`).
    pub fn step_count(&self) -> u64 {
        self.step_count
    }

    /// Delta time of the most recent step.
    pub fn last_delta_time(&self) -> f32 {
        self.last_delta_time
    }

    /// Resolve a body (`getBody`).
    pub fn body(&self, body_id: BodyId) -> Option<&Body> {
        self.bodies.get(body_id)
    }

    /// Resolve a body mutably.
    pub fn body_mut(&mut self, body_id: BodyId) -> Option<&mut Body> {
        self.bodies.get_mut(body_id)
    }

    /// Iterate over live bodies in slot order.
    pub fn bodies(&self) -> impl Iterator<Item = &Body> + '_ {
        self.bodies.iter()
    }

    /// Resolve a motion (`getMotion`).
    pub fn motion(&self, motion_id: MotionId) -> Option<&Motion> {
        self.motions.get(motion_id)
    }

    /// Resolve a motion mutably.
    pub fn motion_mut(&mut self, motion_id: MotionId) -> Option<&mut Motion> {
        self.motions.get_mut(motion_id)
    }

    /// Create and add a body (`allocateBody` + `addBody` collapsed).
    ///
    /// Static bodies share `MotionId::PRESET_STATIC`; dynamic/keyframed bodies
    /// get a fresh motion slot. The shape in the cinfo is shared via `Arc`.
    pub fn create_body(&mut self, cinfo: &BodyCinfo) -> BodyId {
        let (motion_id, motion_flags) = match cinfo.motion_type {
            MotionType::Static => (MotionId::PRESET_STATIC, CollisionFlags::IS_STATIC),
            MotionType::Keyframed => {
                let motion = Motion::new(
                    MotionType::Keyframed,
                    cinfo.position,
                    cinfo.orientation,
                    cinfo.linear_velocity,
                    cinfo.angular_velocity,
                    cinfo.mass,
                );
                (self.motions.allocate(motion), CollisionFlags::IS_KEYFRAMED)
            }
            MotionType::Dynamic => {
                let motion = Motion::new(
                    MotionType::Dynamic,
                    cinfo.position,
                    cinfo.orientation,
                    cinfo.linear_velocity,
                    cinfo.angular_velocity,
                    cinfo.mass,
                );
                (
                    self.motions.allocate(motion),
                    CollisionFlags::IS_DYNAMIC | CollisionFlags::IS_ACTIVE,
                )
            }
        };

        let mut body = Body::new(cinfo.shape.clone(), cinfo.transform(), motion_id);
        body.flags = cinfo.flags | motion_flags;
        body.material_id = cinfo.material_id;
        body.quality_id = cinfo.quality_id;
        body.collision_filter_info = cinfo.collision_filter_info;
        body.user_data = cinfo.user_data;
        body.max_depenetration_speed = cinfo.max_depenetration_speed;
        body.collision_look_ahead_distance = cinfo.collision_look_ahead_distance;
        body.name = cinfo.name.clone();
        body.refresh_aabb();
        self.bodies.allocate(body)
    }

    /// Destroy a body and its motion (`destroyBodies`).
    ///
    /// Motion sharing (`allocateAttachedBody`) is not ported yet, so a non-static
    /// body owns its motion and the motion slot is released here.
    pub fn destroy_body(&mut self, body_id: BodyId) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        let motion_id = body.motion_id;
        if motion_id != MotionId::PRESET_STATIC {
            self.motions.destroy(motion_id);
        }
        self.bodies.destroy(body_id)
    }

    /// Set a body transform (and its motion transform for non-static bodies).
    pub fn set_body_transform(&mut self, body_id: BodyId, transform: Transform) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        let motion_id = body.motion_id;
        if body.is_static() {
            return self
                .bodies
                .get_mut(body_id)
                .map(|body| body.set_transform(transform))
                .is_some();
        }
        if let Some(motion) = self.motions.get_mut(motion_id) {
            motion.set_transform(transform);
        }
        self.bodies
            .get_mut(body_id)
            .map(|body| body.set_transform(transform))
            .is_some()
    }

    /// Set a body's world position, preserving its orientation.
    pub fn set_body_position(&mut self, body_id: BodyId, position: Vector4) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        let rotation = body.transform.rotation_quaternion();
        self.set_body_transform(body_id, Transform::from_quaternion(rotation, position))
    }

    /// Set a body's linear velocity (dynamic/keyframed motions only).
    pub fn set_body_linear_velocity(&mut self, body_id: BodyId, velocity: Vector4) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        if body.is_static() {
            return false;
        }
        self.motions
            .get_mut(body.motion_id)
            .map(|motion| motion.linear_velocity = velocity)
            .is_some()
    }

    /// Set a body's angular velocity (dynamic/keyframed motions only, world space).
    pub fn set_body_angular_velocity(&mut self, body_id: BodyId, velocity: Vector4) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        if body.is_static() {
            return false;
        }
        self.motions
            .get_mut(body.motion_id)
            .map(|motion| motion.angular_velocity = velocity)
            .is_some()
    }

    /// A body's linear velocity.
    pub fn body_linear_velocity(&self, body_id: BodyId) -> Option<Vector4> {
        let body = self.bodies.get(body_id)?;
        self.motions
            .get(body.motion_id)
            .map(|motion| motion.linear_velocity)
    }

    /// A body's angular velocity.
    pub fn body_angular_velocity(&self, body_id: BodyId) -> Option<Vector4> {
        let body = self.bodies.get(body_id)?;
        self.motions
            .get(body.motion_id)
            .map(|motion| motion.angular_velocity)
    }

    /// A body's world position.
    pub fn body_position(&self, body_id: BodyId) -> Option<Vector4> {
        self.bodies
            .get(body_id)
            .map(|body| body.transform.translation)
    }

    /// A body's cached world AABB.
    pub fn body_aabb(&self, body_id: BodyId) -> Option<hk_math::Aabb> {
        self.bodies.get(body_id).map(|body| body.aabb)
    }

    /// Manifolds generated during the most recent step.
    pub fn last_manifold_count(&self) -> usize {
        self.last_manifold_count
    }

    /// Contact points generated during the most recent step.
    pub fn last_contact_count(&self) -> usize {
        self.last_contact_count
    }

    /// Advance the world by `delta_time` using one thread.
    ///
    /// Shortcut for [`World::step_with_threads(delta_time, 1)`].
    pub fn step(&mut self, delta_time: f32) {
        self.step_with_threads(delta_time, 1);
    }

    /// Advance the world by `delta_time`, parallelizing narrow-phase contact
    /// generation over `threads` workers.
    ///
    /// The pipeline remains gravity → broad phase → narrow phase → solve →
    /// integrate. Broad phase and the sequential-impulse solver are unchanged;
    /// each broad-phase candidate is evaluated independently and the per-chunk
    /// results are re-sorted by candidate index, so the contact/manifold order
    /// (and therefore the solver impulse order) is identical to the
    /// single-threaded path.
    pub fn step_with_threads(&mut self, delta_time: f32, threads: usize) {
        debug_assert!(
            delta_time >= 0.0 && delta_time.is_finite(),
            "invalid step delta time"
        );

        let contacts = if threads > 1 {
            self.collect_contacts_parallel(threads)
        } else {
            self.collect_contacts()
        };
        self.last_manifold_count = contacts.len();
        self.last_contact_count = contacts
            .iter()
            .map(|contact| contact.manifold.num_points())
            .sum();

        let gravity = self.gravity;
        if delta_time > 0.0 {
            let dynamic_motions: Vec<MotionId> = self
                .bodies
                .iter()
                .filter(|body| body.is_dynamic())
                .map(|body| body.motion_id)
                .collect();
            for motion_id in dynamic_motions {
                if let Some(motion) = self.motions.get_mut(motion_id) {
                    motion.linear_velocity += gravity * delta_time;
                }
            }
        }

        // Solver bodies are ordered like the body manager; contacts reference
        // these indices.
        let body_ids: Vec<BodyId> = self.bodies.iter().map(|body| body.id).collect();
        let index_map: HashMap<BodyId, usize> = body_ids
            .iter()
            .enumerate()
            .map(|(index, body_id)| (*body_id, index))
            .collect();
        let mut solver_bodies: Vec<SolverBody> = Vec::with_capacity(body_ids.len());
        for body in self.bodies.iter() {
            let motion = self.motions.get(body.motion_id);
            let (inverse_mass, inverse_inertia) = if body.is_dynamic() {
                (
                    motion.map(Motion::inverse_mass).unwrap_or(0.0),
                    motion
                        .map(|motion| motion.inverse_inertia)
                        .unwrap_or(Vector4::ZERO),
                )
            } else {
                (0.0, Vector4::ZERO)
            };
            let linear_velocity = motion
                .map(|motion| motion.linear_velocity)
                .unwrap_or(Vector4::ZERO);
            let angular_velocity = motion
                .map(|motion| motion.angular_velocity)
                .unwrap_or(Vector4::ZERO);
            solver_bodies.push(SolverBody::new(
                inverse_mass,
                inverse_inertia,
                body.transform.translation,
                linear_velocity,
                angular_velocity,
            ));
        }

        let mut constraints = Vec::with_capacity(self.last_contact_count);
        for contact in &contacts {
            let (Some(&body_a), Some(&body_b)) = (
                index_map.get(&contact.body_a),
                index_map.get(&contact.body_b),
            ) else {
                continue;
            };
            for point in &contact.manifold.points {
                // Contact point midway between the two surfaces.
                let constraint_point =
                    point.position + contact.manifold.normal * (point.distance * 0.5);
                let penetration = (-point.distance).max(0.0);
                constraints.push(ContactConstraint::new(
                    body_a,
                    body_b,
                    contact.manifold.normal,
                    constraint_point,
                    penetration,
                    0.5,
                ));
            }
        }
        self.solver
            .solve(&mut solver_bodies, &constraints, delta_time);

        // Write solved velocities back to dynamic motions.
        for (index, body_id) in body_ids.iter().enumerate() {
            let Some(body) = self.bodies.get(*body_id) else {
                continue;
            };
            let solved = solver_bodies[index];
            if solved.inverse_mass > 0.0 {
                if let Some(motion) = self.motions.get_mut(body.motion_id) {
                    motion.linear_velocity = solved.linear_velocity;
                    motion.angular_velocity = solved.angular_velocity;
                }
            }
        }

        // Integrate and refresh body transforms/AABBs for the next step.
        let dynamic_bodies: Vec<(BodyId, MotionId)> = self
            .bodies
            .iter()
            .filter(|body| body.is_dynamic())
            .map(|body| (body.id, body.motion_id))
            .collect();
        for (body_id, motion_id) in dynamic_bodies {
            let Some(motion) = self.motions.get_mut(motion_id) else {
                continue;
            };
            motion.integrate(delta_time);
            let transform = motion.transform();
            if let Some(body) = self.bodies.get_mut(body_id) {
                body.set_transform(transform);
            }
        }

        self.step_count += 1;
        self.last_delta_time = delta_time;
    }

    /// Run the collide stage and return contact manifolds for the current
    /// body positions.
    ///
    /// Static-static and keyframed/keyframed pairs are skipped, as are bodies
    /// carrying `DONT_COLLIDE`. Narrow-phase generation is convex-only; shape
    /// pairs without a support function yield no manifold.
    pub fn collect_contacts(&self) -> Vec<ContactManifold> {
        contacts_from_candidates(&self.contact_candidates(), self.cinfo.collision_tolerance)
    }

    /// Parallel narrow-phase variant of [`World::collect_contacts`].
    ///
    /// Work is partitioned into deterministic contiguous chunks with
    /// `hk_thread::parallel_for_chunks`; chunk results are sorted back into
    /// broad-phase pair order before returning.
    pub fn collect_contacts_parallel(&self, threads: usize) -> Vec<ContactManifold> {
        let candidates = self.contact_candidates();
        // Spawning scoped workers per step costs more than a small narrow phase;
        // below this threshold the sequential path is used.
        const MIN_PARALLEL_CANDIDATES: usize = 32;
        if candidates.len() < MIN_PARALLEL_CANDIDATES || threads <= 1 {
            return contacts_from_candidates(&candidates, self.cinfo.collision_tolerance);
        }

        let results: Mutex<Vec<(usize, ContactManifold)>> = Mutex::new(Vec::new());
        let tolerance = self.cinfo.collision_tolerance;
        hk_thread::parallel::parallel_for_chunks(candidates.len(), threads, |start, end| {
            let mut local = Vec::new();
            for (index, candidate) in candidates.iter().enumerate().take(end).skip(start) {
                if let Some(manifold) = collide_candidate(candidate, tolerance) {
                    local.push((index, manifold));
                }
            }
            if !local.is_empty() {
                results
                    .lock()
                    .expect("parallel contact worker panicked")
                    .extend(local);
            }
        });

        let mut results = results
            .into_inner()
            .expect("parallel contact worker panicked");
        results.sort_unstable_by_key(|(index, _)| *index);
        results.into_iter().map(|(_, manifold)| manifold).collect()
    }

    fn contact_candidates(&self) -> Vec<ContactCandidate> {
        let entries: Vec<BodyAabb> = self
            .bodies
            .iter()
            .map(|body| BodyAabb::new(body.id, body.aabb))
            .collect();
        let pairs = self.broad_phase.find_pairs(&entries);
        let mut candidates = Vec::new();
        for pair in pairs {
            let (Some(body_a), Some(body_b)) = (self.bodies.get(pair.a), self.bodies.get(pair.b))
            else {
                continue;
            };
            if !body_a.is_dynamic() && !body_b.is_dynamic() {
                continue;
            }
            if body_a.flags.contains(CollisionFlags::DONT_COLLIDE)
                || body_b.flags.contains(CollisionFlags::DONT_COLLIDE)
            {
                continue;
            }
            candidates.push(ContactCandidate {
                body_a: pair.a,
                body_b: pair.b,
                shape_a: Arc::clone(&body_a.shape),
                transform_a: body_a.transform,
                shape_b: Arc::clone(&body_b.shape),
                transform_b: body_b.transform,
            });
        }
        candidates
    }

    /// Snapshot of world counters.
    pub fn stats(&self) -> WorldStats {
        WorldStats {
            body_count: self.bodies.len(),
            motion_count: self.motions.len(),
            step_count: self.step_count,
            last_manifold_count: self.last_manifold_count,
            last_contact_count: self.last_contact_count,
        }
    }

    /// Capture a deterministic snapshot of all bodies in slot order.
    ///
    /// This is the Rust-side input for `hvk-trace v1` golden traces and for
    /// bitwise determinism checks.
    pub fn snapshot(&self) -> WorldSnapshot {
        let bodies = self
            .bodies
            .iter()
            .map(|body| {
                let motion = self.motions.get(body.motion_id);
                BodySnapshot {
                    body: body.id,
                    position: body.transform.translation,
                    orientation: motion
                        .map(|motion| motion.orientation)
                        .unwrap_or_else(|| body.transform.rotation_quaternion()),
                    linear_velocity: motion
                        .map(|motion| motion.linear_velocity)
                        .unwrap_or(Vector4::ZERO),
                    angular_velocity: motion
                        .map(|motion| motion.angular_velocity)
                        .unwrap_or(Vector4::ZERO),
                }
            })
            .collect();
        WorldSnapshot {
            step_count: self.step_count,
            bodies,
        }
    }
}

fn contacts_from_candidates(
    candidates: &[ContactCandidate],
    tolerance: f32,
) -> Vec<ContactManifold> {
    candidates
        .iter()
        .filter_map(|candidate| collide_candidate(candidate, tolerance))
        .collect()
}

fn collide_candidate(candidate: &ContactCandidate, tolerance: f32) -> Option<ContactManifold> {
    collide_with_tolerance(
        candidate.shape_a.as_ref(),
        &candidate.transform_a,
        candidate.shape_b.as_ref(),
        &candidate.transform_b,
        tolerance,
    )
    .map(|manifold| ContactManifold {
        body_a: candidate.body_a,
        body_b: candidate.body_b,
        manifold,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;
    use hknp_shape::{BoxShape, Shape, SphereShape};
    use hknp_types::{BodyQualityId, MaterialId, ShapeType};
    use std::f32::consts::FRAC_PI_2;
    use std::sync::Arc;

    fn sphere_shape(radius: f32) -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
    }

    fn box_shape(half_extents: Vector4) -> Arc<dyn Shape> {
        Arc::new(BoxShape::with_convex_radius(half_extents, 0.0)) as Arc<dyn Shape>
    }

    fn parallel_scene() -> World {
        let mut world = World::new(WorldCinfo::new());
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(3.0, 0.5, 3.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)));
        world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(1.0, 0.5, 0.0)));
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::splat(0.5)))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(-1.0, 0.5, 0.0)),
        );
        world
    }

    fn dynamic_cinfo(radius: f32) -> BodyCinfo {
        BodyCinfo::new(sphere_shape(radius))
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
    }

    #[test]
    fn create_static_and_dynamic_bodies() {
        let mut world = World::new(WorldCinfo::new());
        let static_id = world.create_body(&BodyCinfo::new(sphere_shape(1.0)));
        let dynamic_id = world.create_body(&dynamic_cinfo(1.0));

        assert_eq!(world.body_count(), 2);
        let static_body = world.body(static_id).unwrap();
        assert!(static_body.is_static());
        assert_eq!(static_body.motion_id, MotionId::PRESET_STATIC);
        assert_eq!(static_body.shape.shape_type(), ShapeType::Sphere);

        let dynamic_body = world.body(dynamic_id).unwrap();
        assert!(dynamic_body.is_dynamic());
        assert_ne!(dynamic_body.motion_id, MotionId::PRESET_STATIC);
        assert_eq!(world.motion(dynamic_body.motion_id).unwrap().mass, 1.0);
    }

    #[test]
    fn body_cinfo_fields_are_copied() {
        let cinfo = BodyCinfo::new(sphere_shape(1.0))
            .with_motion_type(MotionType::Dynamic)
            .with_material(MaterialId::new(3))
            .with_quality(BodyQualityId::new(2))
            .with_collision_filter_info(0x42)
            .with_user_data(99)
            .with_name("test body")
            .with_flags(CollisionFlags::DONT_COLLIDE);
        let mut world = World::new(WorldCinfo::new());
        let id = world.create_body(&cinfo);
        let body = world.body(id).unwrap();
        assert_eq!(body.material_id, MaterialId::new(3));
        assert_eq!(body.quality_id, BodyQualityId::new(2));
        assert_eq!(body.collision_filter_info, 0x42);
        assert_eq!(body.user_data, 99);
        assert_eq!(body.name.as_deref(), Some("test body"));
        assert!(body.flags.contains(CollisionFlags::DONT_COLLIDE));
    }

    #[test]
    fn step_integrates_dynamic_bodies_only() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let static_id = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0)).with_position(Vector4::from_xyz(100.0, 0.0, 0.0)),
        );
        let keyframed_id = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0))
                .with_motion_type(MotionType::Keyframed)
                .with_position(Vector4::from_xyz(0.0, 100.0, 0.0))
                .with_linear_velocity(Vector4::UNIT_X),
        );
        let dynamic_id =
            world.create_body(&dynamic_cinfo(1.0).with_linear_velocity(Vector4::UNIT_X));

        world.step(0.5);
        assert_eq!(world.step_count(), 1);
        assert!((world.last_delta_time() - 0.5).abs() < 1e-6);

        let static_body = world.body(static_id).unwrap();
        assert!(
            static_body
                .transform
                .translation
                .all_equal3(Vector4::from_xyz(100.0, 0.0, 0.0), 1e-6)
        );

        let keyframed_body = world.body(keyframed_id).unwrap();
        assert!(
            keyframed_body
                .transform
                .translation
                .all_equal3(Vector4::from_xyz(0.0, 100.0, 0.0), 1e-6)
        );

        let dynamic_body = world.body(dynamic_id).unwrap();
        assert!(
            dynamic_body
                .transform
                .translation
                .all_equal3(Vector4::from_xyz(0.5, 0.0, 0.0), 1e-6)
        );
        assert!(
            world
                .body_aabb(dynamic_id)
                .unwrap()
                .min
                .all_equal3(Vector4::from_xyz(-0.5, -1.0, -1.0), 1e-6)
        );
    }

    #[test]
    fn step_integrates_angular_velocity() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let id = world.create_body(
            &dynamic_cinfo(1.0).with_angular_velocity(Vector4::from_xyz(0.0, 0.0, FRAC_PI_2)),
        );
        world.step(1.0);
        let body = world.body(id).unwrap();
        let mapped = body.transform.transform_point(Vector4::UNIT_X);
        assert!(mapped.all_equal3(Vector4::UNIT_Y, 1e-5));
        assert!((body.transform.rotation_quaternion().length() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn gravity_accelerates_dynamic_body() {
        let mut world = World::new(WorldCinfo::new());
        let id = world
            .create_body(&dynamic_cinfo(1.0).with_position(Vector4::from_xyz(0.0, 100.0, 0.0)));
        world.step(0.5);
        let velocity = world.body_linear_velocity(id).unwrap();
        assert!((velocity.y + 9.81 * 0.5).abs() < 1e-4, "vy {}", velocity.y);
        let position = world.body_position(id).unwrap();
        assert!(position.y < 100.0);
    }

    #[test]
    fn falling_sphere_settles_on_static_ground() {
        let mut world = World::new(WorldCinfo::new());
        let ground = world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(3.0, 0.5, 3.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        let sphere =
            world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)));

        for _ in 0..240 {
            world.step(1.0 / 60.0);
        }

        assert!(world.last_manifold_count() > 0);
        assert!(world.last_contact_count() > 0);
        let sphere_position = world.body_position(sphere).unwrap();
        assert!(
            sphere_position.y > 0.35 && sphere_position.y < 0.75,
            "sphere y {}",
            sphere_position.y
        );
        let velocity = world.body_linear_velocity(sphere).unwrap();
        assert!(velocity.y.abs() < 0.2, "sphere vy {}", velocity.y);
        let ground_position = world.body_position(ground).unwrap();
        assert!(ground_position.all_equal3(Vector4::from_xyz(0.0, -0.5, 0.0), 1e-6));
    }

    #[test]
    fn collect_contacts_finds_overlapping_pair() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let a = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        );
        let b = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(1.5, 0.0, 0.0)),
        );
        let contacts = world.collect_contacts();
        assert_eq!(contacts.len(), 1);
        assert_eq!(contacts[0].body_a, a.min(b));
        assert_eq!(contacts[0].body_b, a.max(b));
        assert!(!contacts[0].manifold.points.is_empty());
    }

    #[test]
    fn velocity_and_transform_accessors() {
        let mut world = World::new(WorldCinfo::new());
        let static_id = world.create_body(&BodyCinfo::new(sphere_shape(1.0)));
        let dynamic_id = world.create_body(&dynamic_cinfo(1.0));

        assert!(world.set_body_linear_velocity(dynamic_id, Vector4::UNIT_Y));
        assert_eq!(
            world.body_linear_velocity(dynamic_id),
            Some(Vector4::UNIT_Y)
        );
        assert!(!world.set_body_linear_velocity(static_id, Vector4::UNIT_Y));
        assert_eq!(world.body_linear_velocity(static_id), Some(Vector4::ZERO));

        assert!(world.set_body_transform(
            dynamic_id,
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(3.0, 0.0, 0.0))
        ));
        assert!(
            world
                .body_position(dynamic_id)
                .unwrap()
                .all_equal3(Vector4::from_xyz(3.0, 0.0, 0.0), 1e-6)
        );
    }

    #[test]
    fn destroy_body_releases_motion_slot() {
        let mut world = World::new(WorldCinfo::new());
        let dynamic_id = world.create_body(&dynamic_cinfo(1.0));
        let motion_id = world.body(dynamic_id).unwrap().motion_id;
        assert!(world.destroy_body(dynamic_id));
        assert!(world.body(dynamic_id).is_none());
        assert!(world.motion(motion_id).is_none());
        assert_eq!(world.body_count(), 0);

        assert!(!world.destroy_body(dynamic_id), "double destroy fails");
        assert!(!world.destroy_body(BodyId::invalid()));
    }

    #[test]
    fn gravity_can_be_changed() {
        let mut world = World::new(WorldCinfo::new());
        assert_eq!(world.gravity(), Vector4::from_xyz(0.0, -9.81, 0.0));
        world.set_gravity(Vector4::from_xyz(0.0, 1.0, 0.0));
        assert_eq!(world.gravity(), Vector4::from_xyz(0.0, 1.0, 0.0));
    }

    #[test]
    fn stats_track_bodies_and_motions() {
        let world = World::new(WorldCinfo::new());
        assert_eq!(
            world.stats(),
            WorldStats {
                body_count: 0,
                motion_count: 0,
                step_count: 0,
                last_manifold_count: 0,
                last_contact_count: 0,
            }
        );
    }

    #[test]
    fn parallel_contact_collection_matches_single_threaded() {
        let world = parallel_scene();
        let single = world.collect_contacts();
        assert!(!single.is_empty(), "scene must generate contacts");
        for threads in [1, 2, 4, 8] {
            assert_eq!(
                world.collect_contacts_parallel(threads),
                single,
                "parallel narrow phase differs at {threads} threads"
            );
        }
    }

    #[test]
    fn step_with_threads_matches_single_threaded() {
        let mut single = parallel_scene();
        let mut parallel = parallel_scene();
        for step in 0..120 {
            single.step(1.0 / 60.0);
            parallel.step_with_threads(1.0 / 60.0, 4);
            assert_eq!(
                single.last_manifold_count(),
                parallel.last_manifold_count(),
                "manifold count differs at step {step}"
            );
            assert_eq!(
                single.last_contact_count(),
                parallel.last_contact_count(),
                "contact count differs at step {step}"
            );
        }
        assert_eq!(single.snapshot(), parallel.snapshot());
    }
}
