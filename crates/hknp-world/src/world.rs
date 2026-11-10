//! `hknpWorld` — world assembly and step skeleton.

use hk_math::{Transform, Vector4};
use hknp_body::{Body, BodyCinfo, BodyManager, Motion, MotionManager};
use hknp_types::{BodyId, CollisionFlags, MotionId, MotionType};

use crate::{WorldCinfo, WorldStats};

/// Physics world (`hknpWorld`).
///
/// Owns body/motion managers and the world-level simulation settings. Phase 6
/// provides the assembly and kinematic step skeleton: bodies can be created,
/// destroyed, transformed and queried, and [`World::step`] integrates dynamic
/// motions with their velocities. Collision detection (Phase 7) and contact
/// solving (Phase 8) are not wired in yet, so `step` does not resolve contacts.
#[derive(Debug)]
pub struct World {
    cinfo: WorldCinfo,
    gravity: Vector4,
    bodies: BodyManager,
    motions: MotionManager,
    step_count: u64,
    last_delta_time: f32,
}

impl World {
    /// Create a world from construction info.
    pub fn new(cinfo: WorldCinfo) -> Self {
        let gravity = cinfo.gravity;
        let bodies = BodyManager::with_capacity(cinfo.body_buffer_capacity);
        let motions = MotionManager::with_capacity(cinfo.motion_buffer_capacity);
        Self {
            cinfo,
            gravity,
            bodies,
            motions,
            step_count: 0,
            last_delta_time: 0.0,
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

    /// Advance the world by `delta_time`.
    ///
    /// This is the Phase 6 kinematic skeleton: dynamic motions integrate their
    /// linear/angular velocity, and body transforms/AABBs are refreshed. There is
    /// no collision detection or contact solving yet (Phases 7–8); gravity is
    /// stored but does not act on bodies until the solver/integrator phase.
    pub fn step(&mut self, delta_time: f32) {
        debug_assert!(
            delta_time >= 0.0 && delta_time.is_finite(),
            "invalid step delta time"
        );

        // Collect first to avoid holding a body borrow while mutating motions.
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

    /// Snapshot of world counters.
    pub fn stats(&self) -> WorldStats {
        WorldStats {
            body_count: self.bodies.len(),
            motion_count: self.motions.len(),
            step_count: self.step_count,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;
    use hknp_shape::{Shape, SphereShape};
    use hknp_types::{BodyQualityId, MaterialId, ShapeType};
    use std::f32::consts::FRAC_PI_2;
    use std::sync::Arc;

    fn sphere_shape(radius: f32) -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
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
        let mut world = World::new(WorldCinfo::new());
        let static_id = world.create_body(&BodyCinfo::new(sphere_shape(1.0)));
        let keyframed_id = world.create_body(
            &BodyCinfo::new(sphere_shape(1.0))
                .with_motion_type(MotionType::Keyframed)
                .with_linear_velocity(Vector4::UNIT_X),
        );
        let dynamic_id =
            world.create_body(&dynamic_cinfo(1.0).with_linear_velocity(Vector4::UNIT_X));

        world.step(0.5);
        assert_eq!(world.step_count(), 1);
        assert!((world.last_delta_time() - 0.5).abs() < 1e-6);

        let static_body = world.body(static_id).unwrap();
        assert_eq!(static_body.transform.translation, Vector4::ZERO);

        let keyframed_body = world.body(keyframed_id).unwrap();
        assert_eq!(keyframed_body.transform.translation, Vector4::ZERO);

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
        let mut world = World::new(WorldCinfo::new());
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
            }
        );
    }
}
