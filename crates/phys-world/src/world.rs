//! `hknpWorld` — world assembly and single-threaded step pipeline.

use std::collections::{HashMap, HashSet};
use std::fmt;
use std::sync::{Arc, Mutex};

use base_math::{Quaternion, Transform, Vector4};
use phys_body::{Body, BodyCinfo, BodyManager, Motion, MotionManager};
use phys_collide::{
    BodyAabb, BroadPhase, BroadPhaseConfig, CollisionFilter, collide_with_tolerance,
};
use phys_shape::Shape;
use phys_solver::{
    BallAndSocketJoint, ContactConstraint, ContactImpulse, FixedJoint, HingeJoint, HingeLimits,
    HingeMotor, JointConstraint, JointImpulse, PointToPlaneJoint, PrismaticJoint, PrismaticMotor,
    RagdollJoint, Solver, SolverBody, SolverSettings, SpringJoint, WheelDrive, WheelJoint,
    WheelSuspension,
};
use phys_types::{BodyId, CollisionFlags, ConstraintId, MotionId, MotionType};

use crate::cache::{ContactImpulseCache, ContactKey};
use crate::constraint::{
    BallAndSocketCinfo, FixedCinfo, HingeCinfo, HingeLimitsCinfo, HingeMotorCinfo, JointSlot,
    PointToPlaneCinfo, PrismaticCinfo, PrismaticMotorCinfo, RagdollCinfo, SpringCinfo, WheelCinfo,
    WheelDriveCinfo, WheelSuspensionCinfo, WorldJoint,
};
use crate::island::{IslandInfo, build_islands};
use crate::{BodySnapshot, ContactManifold, WorldCinfo, WorldSnapshot, WorldStats};

/// Per-body deactivation state (the world's deactivation-manager subset).
#[derive(Clone, Copy, Debug, Default)]
struct SleepState {
    /// Seconds the body has continuously stayed below the sleeping thresholds.
    timer: f32,
    /// Whether the body is currently asleep (velocity zero, no narrow phase).
    sleeping: bool,
}

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
/// Owns body/motion managers, the broad phase, joint constraints and the
/// contact solver. [`World::step`] runs collide → solve (contacts + joints) →
/// integrate over convex shapes using a deterministic single-threaded
/// pipeline.
pub struct World {
    cinfo: WorldCinfo,
    gravity: Vector4,
    bodies: BodyManager,
    motions: MotionManager,
    broad_phase: BroadPhase,
    joints: Vec<Option<JointSlot>>,
    joint_count: usize,
    solver: Solver,
    contact_cache: ContactImpulseCache,
    sleep_states: HashMap<BodyId, SleepState>,
    last_islands: Vec<IslandInfo>,
    /// `hknpWorld::m_collisionFilter` — optional collision filter consulted
    /// during contact generation and queries.
    collision_filter: Option<Arc<dyn CollisionFilter>>,
    step_count: u64,
    last_delta_time: f32,
    last_manifold_count: usize,
    last_contact_count: usize,
}

impl fmt::Debug for World {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("World")
            .field("cinfo", &self.cinfo)
            .field("gravity", &self.gravity)
            .field("bodies", &self.bodies)
            .field("motions", &self.motions)
            .field("broad_phase", &self.broad_phase)
            .field("joints", &self.joints)
            .field("joint_count", &self.joint_count)
            .field("solver", &self.solver)
            .field("contact_cache", &self.contact_cache)
            .field("sleep_states", &self.sleep_states)
            .field("last_islands", &self.last_islands)
            .field("collision_filter", &self.collision_filter.is_some())
            .field("step_count", &self.step_count)
            .field("last_delta_time", &self.last_delta_time)
            .field("last_manifold_count", &self.last_manifold_count)
            .field("last_contact_count", &self.last_contact_count)
            .finish()
    }
}

impl World {
    /// Create a world from construction info.
    pub fn new(cinfo: WorldCinfo) -> Self {
        let gravity = cinfo.gravity;
        let solver_settings = SolverSettings {
            iterations: cinfo.solver_iterations,
            warm_starting: cinfo.enable_warm_starting,
            ..SolverSettings::default()
        };
        Self {
            bodies: BodyManager::with_capacity(cinfo.body_buffer_capacity),
            motions: MotionManager::with_capacity(cinfo.motion_buffer_capacity),
            broad_phase: BroadPhase::new(BroadPhaseConfig::default()),
            joints: Vec::new(),
            joint_count: 0,
            solver: Solver::new(solver_settings),
            contact_cache: ContactImpulseCache::default(),
            sleep_states: HashMap::new(),
            last_islands: Vec::new(),
            collision_filter: None,
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

    /// Set the collision filter (`setCollisionFilter`); `None` removes it.
    ///
    /// The filter is consulted for every broad-phase candidate pair during
    /// contact generation and for world queries (through
    /// [`crate::QueryFilter::filter_info`]).
    pub fn set_collision_filter(&mut self, filter: Option<Arc<dyn CollisionFilter>>) {
        self.collision_filter = filter;
    }

    /// The current collision filter, if any (`getCollisionFilter`).
    pub fn collision_filter(&self) -> Option<&Arc<dyn CollisionFilter>> {
        self.collision_filter.as_ref()
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
    /// body owns its motion and the motion slot is released here. Joint
    /// constraints referencing the body are destroyed with it.
    pub fn destroy_body(&mut self, body_id: BodyId) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        let motion_id = body.motion_id;
        if motion_id != MotionId::PRESET_STATIC {
            self.motions.destroy(motion_id);
        }
        let destroyed = self.bodies.destroy(body_id);
        if destroyed {
            self.remove_joints_of(body_id);
            self.sleep_states.remove(&body_id);
        }
        destroyed
    }

    /// Whether a dynamic body is currently asleep (deactivation subset).
    ///
    /// Static and keyframed bodies are never asleep. Bodies are awake unless
    /// [`WorldCinfo::enable_sleeping`] put their island to sleep.
    pub fn body_is_sleeping(&self, body_id: BodyId) -> Option<bool> {
        let body = self.bodies.get(body_id)?;
        Some(body.is_dynamic() && self.is_sleeping(body_id))
    }

    /// Wake a body and every body transitively connected to it through live
    /// joint constraints (contacts propagate the wake one step later).
    ///
    /// Returns `false` when the body does not exist or is not dynamic.
    pub fn wake_body(&mut self, body_id: BodyId) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        if !body.is_dynamic() {
            return false;
        }

        // Breadth-first traversal of the joint graph, in slot order.
        let mut queue = vec![body_id];
        let mut visited = HashSet::new();
        while let Some(current) = queue.pop() {
            if !visited.insert(current) {
                continue;
            }
            if let Some(state) = self.sleep_states.get_mut(&current) {
                state.sleeping = false;
                state.timer = 0.0;
            }
            for (_, joint) in self.constraints_of_body(current) {
                let (body_a, body_b) = joint.bodies();
                for neighbour in [body_a, body_b] {
                    if neighbour != current
                        && !visited.contains(&neighbour)
                        && self
                            .bodies
                            .get(neighbour)
                            .map(|body| body.is_dynamic())
                            .unwrap_or(false)
                    {
                        queue.push(neighbour);
                    }
                }
            }
        }
        true
    }

    /// Internal sleeping check (bodies absent from the map are awake).
    fn is_sleeping(&self, body_id: BodyId) -> bool {
        self.cinfo.enable_sleeping
            && self
                .sleep_states
                .get(&body_id)
                .map(|state| state.sleeping)
                .unwrap_or(false)
    }

    /// Clear a dynamic body's sleep state after user-driven state changes.
    fn wake_if_dynamic(&mut self, body_id: BodyId) {
        if self
            .bodies
            .get(body_id)
            .map(|body| body.is_dynamic())
            .unwrap_or(false)
        {
            if let Some(state) = self.sleep_states.get_mut(&body_id) {
                state.sleeping = false;
                state.timer = 0.0;
            }
        }
    }

    /// Add a ball-and-socket joint (`addConstraint`).
    ///
    /// Returns [`ConstraintId::INVALID`] when either body does not exist.
    pub fn create_ball_socket_constraint(&mut self, info: &BallAndSocketCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::BallAndSocket(*info))
    }

    /// Add a hinge joint (`addConstraint`).
    ///
    /// Returns [`ConstraintId::INVALID`] when either body does not exist.
    pub fn create_hinge_constraint(&mut self, info: &HingeCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::Hinge(*info))
    }

    /// Add a fixed joint (`addConstraint`).
    ///
    /// Returns [`ConstraintId::INVALID`] when either body does not exist.
    pub fn create_fixed_constraint(&mut self, info: &FixedCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::Fixed(*info))
    }

    /// Add a ragdoll joint (`addConstraint`).
    ///
    /// Returns [`ConstraintId::INVALID`] when either body does not exist.
    pub fn create_ragdoll_constraint(&mut self, info: &RagdollCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::Ragdoll(*info))
    }

    /// Add a prismatic joint (`addConstraint`).
    ///
    /// Returns [`ConstraintId::INVALID`] when either body does not exist.
    pub fn create_prismatic_constraint(&mut self, info: &PrismaticCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::Prismatic(*info))
    }

    /// Add a stiff-spring joint (`addConstraint`).
    ///
    /// Returns [`ConstraintId::INVALID`] when either body does not exist.
    pub fn create_spring_constraint(&mut self, info: &SpringCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::Spring(*info))
    }

    /// Add a point-to-plane joint (`addConstraint`).
    ///
    /// Returns [`ConstraintId::INVALID`] when either body does not exist.
    pub fn create_point_to_plane_constraint(&mut self, info: &PointToPlaneCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::PointToPlane(*info))
    }

    /// Add a wheel joint (`addConstraint`).
    ///
    /// Returns [`ConstraintId::INVALID`] when either body does not exist.
    pub fn create_wheel_constraint(&mut self, info: &WheelCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::Wheel(*info))
    }

    /// Set the steering angle of a wheel joint (radians; rotates the
    /// chassis-side axle about the steering axis).
    ///
    /// Fails for IDs that do not resolve to a live wheel joint.
    pub fn set_steering_angle(&mut self, id: ConstraintId, angle: f32) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Wheel(info) => {
                    info.steering_angle = angle;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    /// Set or clear the suspension spring-damper of a wheel joint.
    ///
    /// Fails for IDs that do not resolve to a live wheel joint.
    pub fn set_wheel_suspension(
        &mut self,
        id: ConstraintId,
        suspension: Option<WheelSuspensionCinfo>,
    ) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Wheel(info) => {
                    info.suspension = suspension;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    /// Set or clear the drive motor of a wheel joint (`None` is frictionless).
    ///
    /// Fails for IDs that do not resolve to a live wheel joint.
    pub fn set_wheel_drive(&mut self, id: ConstraintId, drive: Option<WheelDriveCinfo>) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Wheel(info) => {
                    info.drive = drive;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    /// Resolve the joint slot of a live constraint.
    fn live_joint_slot(&self, id: ConstraintId) -> Option<&JointSlot> {
        if !id.is_valid() || !id.is_constraint_index() {
            return None;
        }
        let slot = id.as_constraint_index().value() as usize;
        self.joints.get(slot)?.as_ref()
    }

    /// Destroy a joint constraint (`removeConstraint`).
    ///
    /// Slots are never reused, so the ID cannot alias a later constraint.
    pub fn destroy_constraint(&mut self, id: ConstraintId) -> bool {
        if !id.is_valid() || !id.is_constraint_index() {
            return false;
        }
        let slot = id.as_constraint_index().value() as usize;
        if slot >= self.joints.len() || self.joints[slot].is_none() {
            return false;
        }
        self.joints[slot] = None;
        self.joint_count -= 1;
        true
    }

    /// Resolve a joint constraint (`getConstraint`).
    pub fn constraint(&self, id: ConstraintId) -> Option<&WorldJoint> {
        if !id.is_valid() || !id.is_constraint_index() {
            return None;
        }
        let slot = id.as_constraint_index().value() as usize;
        self.joints.get(slot)?.as_ref().map(|slot| &slot.joint)
    }

    /// Number of live joint constraints.
    pub fn constraint_count(&self) -> usize {
        self.joint_count
    }

    /// Enable or disable a joint constraint; disabled joints are skipped by
    /// the solver but keep their ID. Disabling also clears the joint's cached
    /// warm-start impulses.
    pub fn set_constraint_enabled(&mut self, id: ConstraintId, enabled: bool) -> bool {
        if !id.is_valid() || !id.is_constraint_index() {
            return false;
        }
        let slot = id.as_constraint_index().value() as usize;
        match self.joints.get_mut(slot) {
            Some(Some(entry)) => {
                entry.enabled = enabled;
                if !enabled {
                    entry.impulses = JointImpulse::default();
                }
                true
            }
            _ => false,
        }
    }

    /// Enable or disable contact generation between the joint's two bodies
    /// (`hknpConstraint::DISABLE_COLLISIONS`).
    ///
    /// When disabled, the broad-phase pair is skipped for this constraint's
    /// body pair during [`World::collect_contacts`] and stepping. Returns
    /// `false` for IDs that do not resolve to a live joint.
    pub fn set_constraint_collisions_enabled(&mut self, id: ConstraintId, enabled: bool) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => {
                slot.collisions_enabled = enabled;
                true
            }
            None => false,
        }
    }

    /// Whether a live constraint currently allows collisions between its two
    /// bodies (default `true`).
    pub fn constraint_collisions_enabled(&self, id: ConstraintId) -> Option<bool> {
        self.live_joint_slot(id).map(|slot| slot.collisions_enabled)
    }

    /// Iterate the live constraints attached to `body_id`, in slot order
    /// (`hknpBodyToConstraintsMap` subset).
    pub fn constraints_of_body(
        &self,
        body_id: BodyId,
    ) -> impl Iterator<Item = (ConstraintId, &WorldJoint)> + '_ {
        self.joints
            .iter()
            .enumerate()
            .filter_map(move |(slot, entry)| {
                let entry = entry.as_ref()?;
                let (body_a, body_b) = entry.joint.bodies();
                if body_a == body_id || body_b == body_id {
                    Some((crate::constraint::constraint_id(slot), &entry.joint))
                } else {
                    None
                }
            })
    }

    /// Set or clear the cone/twist limits of a ragdoll joint.
    ///
    /// Fails for IDs that do not resolve to a live ragdoll joint.
    pub fn set_ragdoll_limits(
        &mut self,
        id: ConstraintId,
        cone_angle: f32,
        twist_min: f32,
        twist_max: f32,
    ) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Ragdoll(info) => {
                    info.cone_angle = cone_angle.max(0.0);
                    info.twist_min = twist_min;
                    info.twist_max = twist_max.max(twist_min);
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    /// Set the travel limits of a prismatic joint.
    ///
    /// Fails for IDs that do not resolve to a live prismatic joint.
    pub fn set_prismatic_limits(
        &mut self,
        id: ConstraintId,
        min_translation: f32,
        max_translation: f32,
    ) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Prismatic(info) => {
                    info.min_translation = min_translation.min(max_translation);
                    info.max_translation = max_translation;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    /// Set or clear the angle limits of a hinge joint (`None` clears them).
    ///
    /// Fails for IDs that do not resolve to a live hinge joint.
    pub fn set_hinge_limits(&mut self, id: ConstraintId, limits: Option<HingeLimitsCinfo>) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Hinge(info) => {
                    info.limits = limits;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    /// Set or clear the velocity motor of a hinge joint (`None` clears it).
    ///
    /// Fails for IDs that do not resolve to a live hinge joint.
    pub fn set_hinge_motor(&mut self, id: ConstraintId, motor: Option<HingeMotorCinfo>) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Hinge(info) => {
                    info.motor = motor;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    /// Set or clear the motor of a prismatic joint (`None` clears it).
    ///
    /// Fails for IDs that do not resolve to a live prismatic joint.
    pub fn set_prismatic_motor(
        &mut self,
        id: ConstraintId,
        motor: Option<PrismaticMotorCinfo>,
    ) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Prismatic(info) => {
                    info.motor = motor;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    fn joint_slot_mut(&mut self, id: ConstraintId) -> Option<&mut JointSlot> {
        if !id.is_valid() || !id.is_constraint_index() {
            return None;
        }
        let slot = id.as_constraint_index().value() as usize;
        self.joints.get_mut(slot)?.as_mut()
    }

    fn push_joint(&mut self, joint: WorldJoint) -> ConstraintId {
        let (body_a, body_b) = joint.bodies();
        if self.bodies.get(body_a).is_none() || self.bodies.get(body_b).is_none() {
            return ConstraintId::INVALID;
        }
        let slot = self.joints.len();
        self.joints.push(Some(JointSlot {
            joint,
            enabled: true,
            collisions_enabled: true,
            impulses: JointImpulse::default(),
        }));
        self.joint_count += 1;
        crate::constraint::constraint_id(slot)
    }

    fn remove_joints_of(&mut self, body_id: BodyId) {
        for slot in self.joints.iter_mut() {
            let Some(joint_slot) = slot.as_ref() else {
                continue;
            };
            let (body_a, body_b) = joint_slot.joint.bodies();
            if body_a == body_id || body_b == body_id {
                // Slots are never reused; drop the joint with its body.
                *slot = None;
            }
        }
        self.joint_count = self.joints.iter().filter(|slot| slot.is_some()).count();
    }

    /// Transform the stored local-space joints into solver joints for the
    /// current body poses, in slot order.
    ///
    /// Returns `(slot index, solver joint)` pairs so the step can route the
    /// solver's per-joint impulse rows back into the joint slots for warm
    /// starting.
    fn build_solver_joints(
        &self,
        index_map: &HashMap<BodyId, usize>,
    ) -> Vec<(usize, JointConstraint)> {
        let mut result = Vec::new();
        for (slot_index, slot) in self.joints.iter().enumerate() {
            let Some(slot) = slot else {
                continue;
            };
            if !slot.enabled {
                continue;
            }
            let (body_a, body_b) = slot.joint.bodies();
            let (Some(&index_a), Some(&index_b)) = (index_map.get(&body_a), index_map.get(&body_b))
            else {
                continue;
            };
            let (Some(body_a), Some(body_b)) = (self.bodies.get(body_a), self.bodies.get(body_b))
            else {
                continue;
            };
            let transform_a = body_a.transform;
            let transform_b = body_b.transform;
            match slot.joint {
                WorldJoint::BallAndSocket(info) => {
                    result.push((
                        slot_index,
                        JointConstraint::BallAndSocket(BallAndSocketJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a: transform_a.transform_point(info.pivot_a),
                            pivot_b: transform_b.transform_point(info.pivot_b),
                        }),
                    ));
                }
                WorldJoint::Hinge(info) => {
                    let axis_a = transform_a.rotation.rotate(info.axis_a);
                    let axis_b = transform_b.rotation.rotate(info.axis_b);
                    // The hinge angle is measured against the reference axes
                    // whether or not limits are attached (motors need it too);
                    // degenerate references fall back to a deterministic
                    // perpendicular of the LOCAL hinge axis (computed in local
                    // space so the fallback rotates with the body).
                    let reference_a = transform_a
                        .rotation
                        .rotate(perpendicular_reference(info.axis_a, info.reference_axis_a));
                    let reference_b = transform_b
                        .rotation
                        .rotate(perpendicular_reference(info.axis_b, info.reference_axis_b));
                    let angle = hinge_angle(axis_a, axis_b, reference_a, reference_b);
                    let limits = info.limits.map(|limits| HingeLimits {
                        min_angle: limits.min_angle,
                        max_angle: limits.max_angle,
                    });
                    let motor = info.motor.map(|motor| match motor {
                        crate::constraint::HingeMotorCinfo::Velocity(motor) => {
                            HingeMotor::Velocity {
                                target_velocity: motor.target_velocity,
                                max_impulse: motor.max_impulse,
                            }
                        }
                        crate::constraint::HingeMotorCinfo::Position(motor) => {
                            HingeMotor::Position {
                                target_angle: motor.target_angle,
                                tau: motor.tau,
                                max_impulse: motor.max_impulse,
                            }
                        }
                        crate::constraint::HingeMotorCinfo::Spring(motor) => HingeMotor::Spring {
                            target_angle: motor.target_angle,
                            stiffness: motor.stiffness,
                            damping: motor.damping,
                            max_impulse: motor.max_impulse,
                        },
                    });
                    result.push((
                        slot_index,
                        JointConstraint::Hinge(HingeJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a: transform_a.transform_point(info.pivot_a),
                            pivot_b: transform_b.transform_point(info.pivot_b),
                            axis_a,
                            axis_b,
                            angle,
                            limits,
                            motor,
                        }),
                    ));
                }
                WorldJoint::Fixed(info) => {
                    let q_a = transform_a.rotation_quaternion();
                    let q_b = transform_b.rotation_quaternion();
                    // Rotation of B in A's frame, and the error rotation that
                    // applied to A would restore the bound orientation.
                    let relative = q_a.inverse_mul(q_b);
                    let error = relative.mul_inverse(info.relative_orientation);
                    result.push((
                        slot_index,
                        JointConstraint::Fixed(FixedJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a: transform_a.transform_point(info.pivot_a),
                            pivot_b: transform_b.transform_point(info.pivot_b),
                            orientation_error: orientation_error_vector(error),
                        }),
                    ));
                }
                WorldJoint::Ragdoll(info) => {
                    let twist_a = unit_vector(transform_a.rotation.rotate(info.twist_axis_a));
                    let twist_b = unit_vector(transform_b.rotation.rotate(info.twist_axis_b));
                    let swing_angle = twist_a.dot3(twist_b).clamp(-1.0, 1.0).acos();
                    // Reference axes perpendicular to the twist axis define the
                    // twist zero; degenerate reference axes fall back to a
                    // deterministic perpendicular basis.
                    let plane_a = perpendicular_reference(
                        twist_a,
                        transform_a.rotation.rotate(info.plane_axis_a),
                    );
                    let plane_b = perpendicular_reference(
                        twist_a,
                        transform_b.rotation.rotate(info.plane_axis_b),
                    );
                    let twist_angle = twist_a
                        .dot3(plane_a.cross(plane_b))
                        .atan2(plane_a.dot3(plane_b));
                    result.push((
                        slot_index,
                        JointConstraint::Ragdoll(RagdollJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a: transform_a.transform_point(info.pivot_a),
                            pivot_b: transform_b.transform_point(info.pivot_b),
                            twist_axis_a: twist_a,
                            twist_axis_b: twist_b,
                            swing_angle,
                            cone_angle: info.cone_angle,
                            twist_angle,
                            twist_min: info.twist_min,
                            twist_max: info.twist_max,
                        }),
                    ));
                }
                WorldJoint::Prismatic(info) => {
                    let q_a = transform_a.rotation_quaternion();
                    let q_b = transform_b.rotation_quaternion();
                    let relative = q_a.inverse_mul(q_b);
                    let error = relative.mul_inverse(info.relative_orientation);
                    let axis_a = unit_vector(transform_a.rotation.rotate(info.axis_a));
                    let pivot_a = transform_a.transform_point(info.pivot_a);
                    let pivot_b = transform_b.transform_point(info.pivot_b);
                    let motor = info.motor.map(|motor| match motor {
                        PrismaticMotorCinfo::Position(motor) => PrismaticMotor::Position {
                            target_translation: motor.target_translation,
                            tau: motor.tau,
                            max_impulse: motor.max_impulse,
                        },
                        PrismaticMotorCinfo::Spring(motor) => PrismaticMotor::Spring {
                            target_translation: motor.target_translation,
                            stiffness: motor.stiffness,
                            damping: motor.damping,
                            max_impulse: motor.max_impulse,
                        },
                    });
                    result.push((
                        slot_index,
                        JointConstraint::Prismatic(PrismaticJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a,
                            pivot_b,
                            axis_a,
                            axis_b: unit_vector(transform_b.rotation.rotate(info.axis_b)),
                            orientation_error: orientation_error_vector(error),
                            translation: (pivot_a - pivot_b).dot3(axis_a),
                            min_translation: info.min_translation,
                            max_translation: info.max_translation,
                            motor,
                        }),
                    ));
                }
                WorldJoint::Spring(info) => {
                    result.push((
                        slot_index,
                        JointConstraint::Spring(SpringJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a: transform_a.transform_point(info.pivot_a),
                            pivot_b: transform_b.transform_point(info.pivot_b),
                            rest_length: info.rest_length,
                        }),
                    ));
                }
                WorldJoint::PointToPlane(info) => {
                    result.push((
                        slot_index,
                        JointConstraint::PointToPlane(PointToPlaneJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a: transform_a.transform_point(info.pivot_a),
                            pivot_b: transform_b.transform_point(info.pivot_b),
                            plane_normal: unit_vector(
                                transform_b.rotation.rotate(info.plane_normal_b),
                            ),
                            min_distance: info.min_distance,
                            max_distance: info.max_distance,
                        }),
                    ));
                }
                WorldJoint::Wheel(info) => {
                    // Steering rotates the chassis-side axle about the
                    // steering axis (in chassis-local space), matching
                    // `hknpWheelConstraintData::setSteeringAngle`.
                    let steering = Quaternion::from_axis_angle(
                        unit_vector(info.steering_axis_b),
                        info.steering_angle,
                    );
                    let axle_b = unit_vector(
                        transform_b
                            .rotation
                            .rotate(steering.rotate(unit_vector(info.axle_b))),
                    );
                    let axle_a = unit_vector(transform_a.rotation.rotate(info.axle_a));
                    let suspension_axis =
                        unit_vector(transform_b.rotation.rotate(info.suspension_axis_b));
                    let pivot_a = transform_a.transform_point(info.pivot_a);
                    let pivot_b = transform_b.transform_point(info.pivot_b);
                    let suspension = info.suspension.map(|suspension| WheelSuspension {
                        stiffness: suspension.stiffness,
                        damping: suspension.damping,
                        max_impulse: suspension.max_impulse,
                    });
                    let drive = info.drive.map(|drive| WheelDrive {
                        target_spin_velocity: drive.target_spin_velocity,
                        max_impulse: drive.max_impulse,
                    });
                    result.push((
                        slot_index,
                        JointConstraint::Wheel(WheelJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a,
                            pivot_b,
                            suspension_axis,
                            suspension_length: (pivot_a - pivot_b).dot3(suspension_axis),
                            rest_length: info
                                .suspension
                                .map(|suspension| suspension.rest_length)
                                .unwrap_or((pivot_a - pivot_b).dot3(suspension_axis)),
                            min_length: info.min_length,
                            max_length: info.max_length,
                            suspension,
                            axle_a,
                            axle_b,
                            drive,
                        }),
                    ));
                }
            }
        }
        result
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
        self.wake_if_dynamic(body_id);
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
        let motion_id = body.motion_id;
        self.wake_if_dynamic(body_id);
        self.motions
            .get_mut(motion_id)
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
        let motion_id = body.motion_id;
        self.wake_if_dynamic(body_id);
        self.motions
            .get_mut(motion_id)
            .map(|motion| motion.angular_velocity = velocity)
            .is_some()
    }

    /// Switch a non-static body between `Dynamic` and `Keyframed` motion.
    ///
    /// Mirrors the `setBodyMotionType` calls made by
    /// `hknpRagdollStateManager::makeDynamic` / `makeKeyframed`. No motion slot
    /// is reallocated: static transitions and unknown ids return `false`.
    pub fn set_body_motion_type(&mut self, body_id: BodyId, motion_type: MotionType) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        if body.is_static() || motion_type.is_static() {
            return false;
        }
        let motion_id = body.motion_id;
        if motion_id == MotionId::PRESET_STATIC || self.motions.get(motion_id).is_none() {
            return false;
        }
        if let Some(motion) = self.motions.get_mut(motion_id) {
            if motion.motion_type == motion_type {
                return true;
            }
            motion.motion_type = motion_type;
        }
        if let Some(body) = self.bodies.get_mut(body_id) {
            body.flags
                .set(CollisionFlags::IS_DYNAMIC, motion_type.is_dynamic());
            body.flags
                .set(CollisionFlags::IS_KEYFRAMED, !motion_type.is_dynamic());
            body.flags
                .set(CollisionFlags::IS_ACTIVE, motion_type.is_dynamic());
        }
        // A keyframed body has no sleep state; dropping it prevents a stale
        // sleeping flag when the body is later made dynamic again.
        self.sleep_states.remove(&body_id);
        if motion_type.is_dynamic() {
            self.wake_if_dynamic(body_id);
        }
        true
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
    pub fn body_aabb(&self, body_id: BodyId) -> Option<base_math::Aabb> {
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

    /// Constraint islands solved during the most recent step.
    ///
    /// Islands are ordered by their lowest dynamic body and list bodies in
    /// ascending slot order, so the result is deterministic.
    pub fn islands(&self) -> &[IslandInfo] {
        &self.last_islands
    }

    /// Number of constraint islands in the most recent step.
    pub fn last_island_count(&self) -> usize {
        self.last_islands.len()
    }

    /// Drop the cached contact impulses used for warm starting.
    ///
    /// Call this after teleporting bodies or changing friction/mass drastically
    /// when stale impulses would be undesirable; normal use does not need it
    /// (the cache prunes unreachable points every step).
    pub fn clear_contact_cache(&mut self) {
        self.contact_cache.clear();
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
            // Sleeping bodies do not accumulate gravity; they keep zero velocity.
            let dynamic_motions: Vec<MotionId> = self
                .bodies
                .iter()
                .filter(|body| body.is_dynamic() && !self.is_sleeping(body.id))
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

        // Contacts plus their warm-start identity (point on B in B's local
        // space, so the same feature can be found again next step).
        let mut constraints = Vec::with_capacity(self.last_contact_count);
        let mut contact_keys = Vec::with_capacity(self.last_contact_count);
        let mut contact_impulses = Vec::with_capacity(self.last_contact_count);
        for contact in &contacts {
            let (Some(&body_a), Some(&body_b)) = (
                index_map.get(&contact.body_a),
                index_map.get(&contact.body_b),
            ) else {
                continue;
            };
            let Some(transform_b) = self.bodies.get(contact.body_b).map(|body| body.transform)
            else {
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
                let key = ContactKey {
                    body_a: contact.body_a,
                    body_b: contact.body_b,
                    local_point_b: transform_b.inverse_transform_point(point.position),
                    normal: contact.manifold.normal,
                };
                contact_impulses.push(if self.cinfo.enable_warm_starting {
                    self.contact_cache.match_impulse(key)
                } else {
                    ContactImpulse::default()
                });
                contact_keys.push(key);
            }
        }

        let solver_joints = self.build_solver_joints(&index_map);
        // Joint warm start state lives in the constraint slots; the pair list
        // keeps the slot index of every solver joint.
        let (joint_slots, solver_joints): (Vec<usize>, Vec<JointConstraint>) = {
            let mut slots = Vec::with_capacity(solver_joints.len());
            let mut joints = Vec::with_capacity(solver_joints.len());
            for (slot, joint) in solver_joints {
                slots.push(slot);
                joints.push(joint);
            }
            (slots, joints)
        };
        let mut joint_impulses: Vec<JointImpulse> = joint_slots
            .iter()
            .map(|&slot| {
                self.joints[slot]
                    .as_ref()
                    .map(|entry| entry.impulses)
                    .unwrap_or_default()
            })
            .collect();

        // Partition dynamic bodies into constraint islands and solve each one
        // (contacts then joints) with its previous impulses as warm start.
        let is_dynamic: Vec<bool> = solver_bodies.iter().map(SolverBody::is_dynamic).collect();
        let contact_pairs: Vec<(usize, usize)> = constraints
            .iter()
            .map(|constraint| (constraint.body_a, constraint.body_b))
            .collect();
        let joint_pairs: Vec<(usize, usize)> =
            solver_joints.iter().map(JointConstraint::bodies).collect();
        let islands = build_islands(&body_ids, &is_dynamic, &contact_pairs, &joint_pairs);

        self.last_islands = islands
            .iter()
            .map(|island| IslandInfo {
                bodies: island
                    .bodies
                    .iter()
                    .filter_map(|&index| body_ids.get(index).copied())
                    .collect(),
                dynamic_bodies: island.dynamic_bodies,
                contacts: island.contacts.len(),
                joints: island.joints.len(),
            })
            .collect();

        // Wake propagation: an island holding an awake dynamic body or touching
        // a keyframed body wakes all of its sleeping members (a sleeping body
        // only ends up in an island via an active partner or keyframed body).
        if self.cinfo.enable_sleeping {
            let keyframed_members: Vec<bool> = islands
                .iter()
                .map(|island| {
                    island.bodies.iter().any(|&index| {
                        !is_dynamic[index]
                            && self
                                .bodies
                                .get(body_ids[index])
                                .map(|body| body.is_keyframed())
                                .unwrap_or(false)
                    })
                })
                .collect();
            for (island_index, island) in islands.iter().enumerate() {
                let has_awake = island
                    .bodies
                    .iter()
                    .any(|&index| is_dynamic[index] && !self.is_sleeping(body_ids[index]));
                if !has_awake && !keyframed_members[island_index] {
                    continue;
                }
                for &index in &island.bodies {
                    if !is_dynamic[index] {
                        continue;
                    }
                    let body_id = body_ids[index];
                    if let Some(state) = self.sleep_states.get_mut(&body_id) {
                        if state.sleeping {
                            state.sleeping = false;
                            state.timer = 0.0;
                        }
                    }
                }
            }
        }

        // Islands are disjoint in solver-body space (static/keyframed members
        // never change), so solving island groups in parallel and merging the
        // per-island results yields exactly the sequential impulses.
        if threads > 1 && islands.len() >= MIN_PARALLEL_ISLANDS {
            let results = solve_islands_parallel(
                &self.solver,
                &IslandSolveBuffers {
                    solver_bodies: &solver_bodies,
                    constraints: &constraints,
                    contact_impulses: &contact_impulses,
                    solver_joints: &solver_joints,
                    joint_impulses: &joint_impulses,
                },
                &islands,
                delta_time,
                threads,
            );
            for result in results {
                for (index, body) in result.bodies {
                    solver_bodies[index] = body;
                }
                for (index, impulse) in result.contacts {
                    contact_impulses[index] = impulse;
                }
                for (index, impulse) in result.joints {
                    joint_impulses[index] = impulse;
                }
            }
        } else {
            for island in &islands {
                if !island.contacts.is_empty() {
                    self.solver.solve_contacts_subset(
                        &mut solver_bodies,
                        &constraints,
                        &island.contacts,
                        &mut contact_impulses,
                        delta_time,
                    );
                }
                if !island.joints.is_empty() {
                    self.solver.solve_joints_subset_with_impulses(
                        &mut solver_bodies,
                        &solver_joints,
                        &island.joints,
                        &mut joint_impulses,
                        delta_time,
                    );
                }
            }
        }

        // Store the solved joint rows back into the slots for the next step's
        // warm start (a cold start when warm starting is disabled).
        if self.cinfo.enable_warm_starting {
            for (slot, impulse) in joint_slots.iter().zip(&joint_impulses) {
                if let Some(Some(entry)) = self.joints.get_mut(*slot) {
                    entry.impulses = *impulse;
                }
            }
        } else {
            for entry in self.joints.iter_mut().flatten() {
                entry.impulses = JointImpulse::default();
            }
        }

        if self.cinfo.enable_warm_starting {
            self.contact_cache
                .replace(contact_keys.iter().copied().zip(contact_impulses));
        } else {
            self.contact_cache.clear();
        }

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

        // Update deactivation timers and put fully low-energy islands to sleep.
        if self.cinfo.enable_sleeping && delta_time > 0.0 {
            for (index, &body_id) in body_ids.iter().enumerate() {
                if !is_dynamic[index] {
                    continue;
                }
                let state = self.sleep_states.entry(body_id).or_default();
                if state.sleeping {
                    continue;
                }
                let solved = solver_bodies[index];
                let low_energy = solved.linear_velocity.dot3(solved.linear_velocity)
                    < self.cinfo.sleeping_linear_threshold * self.cinfo.sleeping_linear_threshold
                    && solved.angular_velocity.dot3(solved.angular_velocity)
                        < self.cinfo.sleeping_angular_threshold
                            * self.cinfo.sleeping_angular_threshold;
                if low_energy {
                    state.timer += delta_time;
                } else {
                    state.timer = 0.0;
                }
            }

            // An island sleeps only when every dynamic member is low-energy and
            // no keyframed body keeps it active.
            let mut newly_sleeping: Vec<BodyId> = Vec::new();
            let mut sleep_joint_slots: Vec<usize> = Vec::new();
            for island in &islands {
                let ready = island.bodies.iter().all(|&index| {
                    if !is_dynamic[index] {
                        // Keyframed members count as active; statics do not.
                        return self
                            .bodies
                            .get(body_ids[index])
                            .map(|body| !body.is_keyframed())
                            .unwrap_or(true);
                    }
                    self.sleep_states
                        .get(&body_ids[index])
                        .map(|state| state.timer >= self.cinfo.time_to_sleep)
                        .unwrap_or(false)
                });
                if !ready {
                    continue;
                }
                for &index in &island.bodies {
                    if !is_dynamic[index] {
                        continue;
                    }
                    let body_id = body_ids[index];
                    if let Some(state) = self.sleep_states.get_mut(&body_id) {
                        if !state.sleeping {
                            state.sleeping = true;
                            newly_sleeping.push(body_id);
                        }
                    }
                }
                for &joint_index in &island.joints {
                    if let Some(&slot) = joint_slots.get(joint_index) {
                        sleep_joint_slots.push(slot);
                    }
                }
            }
            for body_id in newly_sleeping {
                if let Some(motion_id) = self.bodies.get(body_id).map(|body| body.motion_id) {
                    if let Some(motion) = self.motions.get_mut(motion_id) {
                        motion.linear_velocity = Vector4::ZERO;
                        motion.angular_velocity = Vector4::ZERO;
                    }
                }
            }
            // Sleeping islands lose their contact rows anyway (the narrow phase
            // skips them); stale joint warm-start impulses are cleared too.
            for slot in sleep_joint_slots {
                if let Some(Some(entry)) = self.joints.get_mut(slot) {
                    entry.impulses = JointImpulse::default();
                }
            }
        }

        // Integrate and refresh body transforms/AABBs for the next step.
        // Sleeping bodies keep their frozen pose.
        let dynamic_bodies: Vec<(BodyId, MotionId)> = self
            .bodies
            .iter()
            .filter(|body| body.is_dynamic() && !self.is_sleeping(body.id))
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
    /// `base_thread::parallel_for_chunks`; chunk results are sorted back into
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
        base_thread::parallel::parallel_for_chunks(candidates.len(), threads, |start, end| {
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
        // Constraints with `DISABLE_COLLISIONS` suppress contact generation
        // between their two bodies (the SDK's constraint-pair filter).
        let blocked: std::collections::HashSet<(BodyId, BodyId)> = self
            .joints
            .iter()
            .flatten()
            .filter(|slot| slot.enabled && !slot.collisions_enabled)
            .map(|slot| {
                let (body_a, body_b) = slot.joint.bodies();
                if body_a < body_b {
                    (body_a, body_b)
                } else {
                    (body_b, body_a)
                }
            })
            .collect();

        let entries: Vec<BodyAabb> = self
            .bodies
            .iter()
            .map(|body| BodyAabb::new(body.id, body.aabb))
            .collect();
        let pairs = self.broad_phase.find_pairs(&entries);
        let mut candidates = Vec::new();
        for pair in pairs {
            let key = if pair.a < pair.b {
                (pair.a, pair.b)
            } else {
                (pair.b, pair.a)
            };
            if blocked.contains(&key) {
                continue;
            }
            let (Some(body_a), Some(body_b)) = (self.bodies.get(pair.a), self.bodies.get(pair.b))
            else {
                continue;
            };
            if !body_a.is_dynamic() && !body_b.is_dynamic() {
                continue;
            }
            // Sleeping bodies skip narrow phase against each other and against
            // static geometry; an active or keyframed partner still wakes them.
            let sleep_a = self.is_sleeping(pair.a);
            let sleep_b = self.is_sleeping(pair.b);
            if (sleep_a && sleep_b)
                || (sleep_a && body_b.is_static())
                || (sleep_b && body_a.is_static())
            {
                continue;
            }
            if body_a.flags.contains(CollisionFlags::DONT_COLLIDE)
                || body_b.flags.contains(CollisionFlags::DONT_COLLIDE)
            {
                continue;
            }
            // `hknpCollisionFilter::isCollisionEnabled` body-pair overload.
            if let Some(filter) = &self.collision_filter {
                if !filter.is_body_pair_enabled(
                    pair.a,
                    body_a.collision_filter_info,
                    pair.b,
                    body_b.collision_filter_info,
                ) {
                    continue;
                }
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
            last_island_count: self.last_islands.len(),
            sleeping_body_count: self
                .sleep_states
                .values()
                .filter(|state| state.sleeping)
                .count(),
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

/// Minimum island count for the parallel per-island solver to kick in; below
/// it the sequential path avoids per-step thread spawn overhead.
const MIN_PARALLEL_ISLANDS: usize = 2;

/// Solved state of one island, extracted from a worker's private buffers.
struct IslandSolveResult {
    bodies: Vec<(usize, SolverBody)>,
    contacts: Vec<(usize, ContactImpulse)>,
    joints: Vec<(usize, JointImpulse)>,
}

/// The step's solver buffers handed to [`solve_islands_parallel`].
struct IslandSolveBuffers<'a> {
    solver_bodies: &'a [SolverBody],
    constraints: &'a [ContactConstraint],
    contact_impulses: &'a [ContactImpulse],
    solver_joints: &'a [JointConstraint],
    joint_impulses: &'a [JointImpulse],
}

/// Solve the islands in parallel over deterministic contiguous chunks.
///
/// Each worker clones the solver buffers, solves its islands sequentially (in
/// ascending island order) and returns the per-island state. Islands are
/// disjoint in dynamic solver bodies, so merging is order-independent; static
/// members are copied back unchanged.
fn solve_islands_parallel(
    solver: &Solver,
    buffers: &IslandSolveBuffers<'_>,
    islands: &[crate::island::Island],
    delta_time: f32,
    threads: usize,
) -> Vec<IslandSolveResult> {
    let results: Mutex<Vec<IslandSolveResult>> = Mutex::new(Vec::new());
    base_thread::parallel::parallel_for_chunks(islands.len(), threads, |start, end| {
        let mut bodies = buffers.solver_bodies.to_vec();
        let mut contacts = buffers.contact_impulses.to_vec();
        let mut joints = buffers.joint_impulses.to_vec();
        for island in &islands[start..end] {
            if !island.contacts.is_empty() {
                solver.solve_contacts_subset(
                    &mut bodies,
                    buffers.constraints,
                    &island.contacts,
                    &mut contacts,
                    delta_time,
                );
            }
            if !island.joints.is_empty() {
                solver.solve_joints_subset_with_impulses(
                    &mut bodies,
                    buffers.solver_joints,
                    &island.joints,
                    &mut joints,
                    delta_time,
                );
            }
        }
        let mut local = Vec::with_capacity(end - start);
        for island in &islands[start..end] {
            local.push(IslandSolveResult {
                bodies: island.bodies.iter().map(|&i| (i, bodies[i])).collect(),
                contacts: island.contacts.iter().map(|&i| (i, contacts[i])).collect(),
                joints: island.joints.iter().map(|&i| (i, joints[i])).collect(),
            });
        }
        if !local.is_empty() {
            results
                .lock()
                .expect("parallel island worker panicked")
                .extend(local);
        }
    });
    results
        .into_inner()
        .expect("parallel island worker panicked")
}

/// Signed angle between the reference (twist) axes about the hinge axis.
///
/// Both twist axes are projected onto the plane perpendicular to their hinge
/// axis; the angle is measured from A to B in the sense of `axis_a`, wrapped
/// to `(-pi, pi]`.
fn hinge_angle(axis_a: Vector4, axis_b: Vector4, twist_a: Vector4, twist_b: Vector4) -> f32 {
    let tangential = |twist: Vector4, axis: Vector4| {
        let component = twist - axis * twist.dot3(axis);
        let length = component.length3();
        if length > 1e-10 {
            component * (1.0 / length)
        } else {
            Vector4::ZERO
        }
    };
    let projected_a = tangential(twist_a, axis_a);
    let projected_b = tangential(twist_b, axis_b);
    let sin = projected_a.cross(projected_b).dot3(axis_a);
    let cos = projected_a.dot3(projected_b);
    sin.atan2(cos)
}

/// World-space axis-angle rotation-error vector of a quaternion: rotating a
/// body by this vector restores the bound orientation. Zero when aligned.
fn orientation_error_vector(error: Quaternion) -> Vector4 {
    let imag = error.imag();
    let sin_half = imag.length3();
    if sin_half <= 1e-8 {
        return Vector4::ZERO;
    }
    let axis = Vector4::from_xyz(imag.x / sin_half, imag.y / sin_half, imag.z / sin_half);
    axis * error.angle()
}

/// Normalize a direction, falling back to `+X` for a degenerate input.
fn unit_vector(direction: Vector4) -> Vector4 {
    let length = direction.length3();
    if length > 1e-10 {
        direction * (1.0 / length)
    } else {
        Vector4::UNIT_X
    }
}

/// Project `reference` onto the plane perpendicular to `axis` and normalize;
/// fall back to a deterministic perpendicular basis when degenerate.
fn perpendicular_reference(axis: Vector4, reference: Vector4) -> Vector4 {
    let component = reference - axis * reference.dot3(axis);
    if component.length3() > 1e-10 {
        return unit_vector(component);
    }
    let helper = if axis.x.abs() < 0.9 {
        Vector4::UNIT_X
    } else {
        Vector4::UNIT_Y
    };
    unit_vector(axis.cross(helper))
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
    use crate::constraint::{
        HingePositionMotorCinfo, HingeSpringMotorCinfo, HingeVelocityMotorCinfo,
        PrismaticPositionMotorCinfo, PrismaticSpringMotorCinfo,
    };
    use base_math::Quaternion;
    use phys_shape::{BoxShape, Shape, SphereShape};
    use phys_types::{BodyQualityId, MaterialId, ShapeType};
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
    fn set_body_motion_type_switches_dynamic_and_keyframed() {
        let mut world = World::new(WorldCinfo::new());
        let body = world.create_body(&dynamic_cinfo(0.5));
        let motion_id = world.body(body).unwrap().motion_id;
        assert!(world.set_body_motion_type(body, MotionType::Keyframed));
        assert!(world.body(body).unwrap().is_keyframed());
        assert!(!world.body(body).unwrap().is_dynamic());
        assert_eq!(
            world.motion(motion_id).unwrap().motion_type,
            MotionType::Keyframed
        );
        assert!(world.set_body_motion_type(body, MotionType::Dynamic));
        assert!(world.body(body).unwrap().is_dynamic());
        assert_eq!(world.body(body).unwrap().motion_id, motion_id);

        let static_body = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        assert!(!world.set_body_motion_type(static_body, MotionType::Dynamic));
        assert!(!world.set_body_motion_type(body, MotionType::Static));
        assert!(!world.set_body_motion_type(BodyId::invalid(), MotionType::Keyframed));
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
                last_island_count: 0,
                sleeping_body_count: 0,
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

    // Joint constraint tests.

    #[test]
    fn ball_socket_pendulum_hangs_under_gravity() {
        let mut world = World::new(WorldCinfo::new().with_solver_settings(0.2, 0.0, 16));
        let anchor = Vector4::from_xyz(0.0, 10.0, 0.0);
        let anchor_body =
            world.create_body(&BodyCinfo::new(sphere_shape(0.1)).with_position(anchor));
        let bob = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(anchor + Vector4::from_xyz(2.0, 0.0, 0.0)),
        );
        // Spheres carry no inverse inertia by default (Phase 8 decision); give
        // the bob rotational response so the pendulum can swing.
        let bob_motion = world.body(bob).unwrap().motion_id;
        world.motion_mut(bob_motion).unwrap().inverse_inertia = Vector4::splat(0.5);
        let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            anchor_body,
            bob,
            Vector4::UNIT_X,
            -Vector4::UNIT_X,
        ));
        assert!(id.is_valid());
        assert_eq!(world.constraint_count(), 1);

        // The joint is frictionless, so the pendulum keeps swinging; the test
        // verifies the constraint holds the world pivots coincident and that
        // the bob swings below the anchor.
        let pivot_a_world = anchor + Vector4::UNIT_X;
        let mut max_pivot_error = 0.0f32;
        let mut min_height_below_anchor = 0.0f32;
        for _ in 0..240 {
            world.step(1.0 / 60.0);
            let pivot_b_world = world
                .body_transform(bob)
                .unwrap()
                .transform_point(-Vector4::UNIT_X);
            max_pivot_error = max_pivot_error.max((pivot_b_world - pivot_a_world).length3());
            let delta = world.body_position(bob).unwrap() - anchor;
            min_height_below_anchor = min_height_below_anchor.min(delta.y);
        }
        assert!(
            max_pivot_error < 0.4,
            "pivots drifted apart by {}",
            max_pivot_error
        );
        assert!(
            min_height_below_anchor < -0.8,
            "bob never swung below the anchor (min dy {})",
            min_height_below_anchor
        );
    }

    #[test]
    fn invalid_bodies_reject_constraints() {
        let mut world = World::new(WorldCinfo::new());
        let body = world.create_body(&dynamic_cinfo(0.5));
        let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            BodyId::invalid(),
            body,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        assert_eq!(id, ConstraintId::INVALID);
        assert_eq!(world.constraint_count(), 0);
    }

    #[test]
    fn destroy_constraint_and_body_cleanup() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let a = world.create_body(&dynamic_cinfo(0.5));
        let b = world.create_body(&dynamic_cinfo(0.5));
        let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            a,
            b,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        assert!(id.is_valid());
        assert!(world.set_constraint_enabled(id, false));
        assert!(world.set_constraint_enabled(id, true));
        assert!(world.constraint(id).is_some());

        assert!(world.destroy_constraint(id));
        assert!(!world.destroy_constraint(id), "double destroy fails");
        assert!(world.constraint(id).is_none());
        assert_eq!(world.constraint_count(), 0);

        let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            a,
            b,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        assert!(id.is_valid());
        assert!(world.destroy_body(b));
        assert!(world.constraint(id).is_none(), "joint dies with its body");
        assert_eq!(world.constraint_count(), 0);
    }

    #[test]
    fn fixed_joint_rigid_pair_moves_together() {
        let mut world = World::new(WorldCinfo::new());
        let a = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(0.0, 5.0, 0.0)),
        );
        let b = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(2.0, 5.0, 0.0)),
        );
        world.create_fixed_constraint(&FixedCinfo::between_bodies(
            a,
            b,
            Vector4::from_xyz(1.0, 5.0, 0.0),
            &world.body_transform(a).unwrap(),
            &world.body_transform(b).unwrap(),
        ));

        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }

        let position_a = world.body_position(a).unwrap();
        let position_b = world.body_position(b).unwrap();
        let separation = (position_b - position_a).length3();
        assert!(
            (separation - 2.0).abs() < 0.3,
            "fixed pair separation {}",
            separation
        );
        // Both fall together under gravity.
        assert!(position_a.y < 5.0 - 1.0);
    }

    #[test]
    fn hinge_joint_spins_about_the_axis() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let hub =
            world.create_body(&BodyCinfo::new(sphere_shape(0.5)).with_position(Vector4::ZERO));
        let arm = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(2.0, 0.0, 0.0)),
        );
        let arm_motion = world.body(arm).unwrap().motion_id;
        world.motion_mut(arm_motion).unwrap().inverse_inertia = Vector4::splat(0.5);
        // Hinge about Z at the hub center: the arm can only rotate in-plane.
        world.create_hinge_constraint(&HingeCinfo::new(
            hub,
            arm,
            Vector4::UNIT_X,
            -Vector4::UNIT_X,
            Vector4::UNIT_Z,
            Vector4::UNIT_Z,
        ));
        world.set_body_angular_velocity(arm, Vector4::from_xyz(0.0, 5.0, 0.0));

        // The hinge removes rotation about Y, so after several steps the arm
        // stays near the hub instead of flying away as a free body would.
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let arm_position = world.body_position(arm).unwrap();
        let distance = (arm_position - Vector4::ZERO).length3();
        assert!(distance < 4.0, "arm distance {}", distance);
    }

    fn hinged_arm(world: &mut World) -> (BodyId, BodyId, ConstraintId) {
        let hub =
            world.create_body(&BodyCinfo::new(sphere_shape(0.5)).with_position(Vector4::ZERO));
        let arm = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(2.0, 0.0, 0.0)),
        );
        let arm_motion = world.body(arm).unwrap().motion_id;
        world.motion_mut(arm_motion).unwrap().inverse_inertia = Vector4::splat(0.5);
        let id = world.create_hinge_constraint(&HingeCinfo::new(
            hub,
            arm,
            Vector4::ZERO,
            Vector4::from_xyz(-2.0, 0.0, 0.0),
            Vector4::UNIT_Z,
            Vector4::UNIT_Z,
        ));
        assert!(id.is_valid());
        (hub, arm, id)
    }

    /// World-space hinge angle between the identity-frame twist axis `+X`
    /// mapped through both body rotations, about `+Z`.
    fn current_hinge_angle(world: &World, hub: BodyId, arm: BodyId) -> f32 {
        let transform_a = world.body_transform(hub).unwrap();
        let transform_b = world.body_transform(arm).unwrap();
        hinge_angle(
            Vector4::UNIT_Z,
            Vector4::UNIT_Z,
            transform_a.rotation.rotate(Vector4::UNIT_X),
            transform_b.rotation.rotate(Vector4::UNIT_X),
        )
    }

    #[test]
    fn hinge_limits_drive_the_angle_back_into_range() {
        let mut world = World::new(
            WorldCinfo::new()
                .with_gravity(Vector4::ZERO)
                .with_solver_settings(0.2, 0.0, 16),
        );
        let (_, arm, id) = hinged_arm(&mut world);
        // Start the arm 0.4 rad past the upper bound of ±0.2.
        world.set_body_transform(
            arm,
            Transform::from_quaternion(
                Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.6),
                Vector4::from_xyz(2.0, 0.0, 0.0),
            ),
        );
        let limits = HingeLimitsCinfo::new(-0.2, 0.2);
        assert!(world.set_hinge_limits(id, Some(limits)));

        let hub = hinged_arm_hub(&world);
        let initial = current_hinge_angle(&world, hub, arm);
        let mut min_angle = initial;
        for _ in 0..60 {
            world.step(1.0 / 60.0);
            let angle = current_hinge_angle(&world, hub, arm);
            min_angle = min_angle.min(angle);
        }
        let final_angle = current_hinge_angle(&world, hub, arm);
        assert!(
            final_angle < initial - 0.1,
            "angle did not return toward the bound: {} -> {}",
            initial,
            final_angle
        );
        assert!(
            min_angle >= limits.min_angle - 0.05,
            "limit pushed far through the lower bound: {}",
            min_angle
        );
    }

    fn hinged_arm_hub(world: &World) -> BodyId {
        world.bodies().next().unwrap().id
    }

    #[test]
    fn hinge_motor_drives_constant_spin() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let (_, arm, id) = hinged_arm(&mut world);
        assert!(world.set_hinge_motor(
            id,
            Some(HingeMotorCinfo::Velocity(HingeVelocityMotorCinfo::new(
                2.0, 10.0
            )))
        ));

        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        let angular = world.body_angular_velocity(arm).unwrap();
        assert!(
            (angular.z - 2.0).abs() < 0.2,
            "wz {} must track the motor target",
            angular.z
        );
        // The arm pivots about the hinge point, so it must stay near the hub.
        let distance = (world.body_position(arm).unwrap() - Vector4::ZERO).length3();
        assert!(distance < 4.0, "arm distance {}", distance);
    }

    #[test]
    fn set_hinge_limits_and_motor_validate_ids() {
        let mut world = World::new(WorldCinfo::new());
        let (hub, arm, id) = hinged_arm(&mut world);
        let ball_socket = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            hub,
            arm,
            Vector4::ZERO,
            Vector4::ZERO,
        ));

        let limits = HingeLimitsCinfo::new(-0.2, 0.2);
        assert!(world.set_hinge_limits(id, Some(limits)));
        assert!(world.set_hinge_motor(
            id,
            Some(HingeMotorCinfo::Velocity(HingeVelocityMotorCinfo::new(
                1.0, 5.0
            )))
        ));
        assert!(world.set_hinge_limits(id, None));
        assert!(world.set_hinge_motor(id, None));

        // Non-hinge and invalid IDs are rejected.
        assert!(!world.set_hinge_limits(ball_socket, Some(limits)));
        assert!(!world.set_hinge_motor(ball_socket, None));
        assert!(!world.set_hinge_limits(ConstraintId::INVALID, None));
    }

    /// World-space twist axis of a body for ragdoll tests (`+X` local).
    fn world_twist_axis(world: &World, body: BodyId) -> Vector4 {
        world
            .body_transform(body)
            .expect("live body")
            .rotation
            .rotate(Vector4::UNIT_X)
    }

    #[test]
    fn ragdoll_cone_limit_holds_the_swing_angle() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let parent = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        let child = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        );
        // Give the child real angular inertia so the cone row can act.
        let motion_id = world.body(child).unwrap().motion_id;
        world.motion_mut(motion_id).unwrap().inverse_inertia = Vector4::splat(0.5);

        let id = world.create_ragdoll_constraint(
            &RagdollCinfo::new(
                child,
                parent,
                Vector4::ZERO,
                Vector4::ZERO,
                Vector4::UNIT_X,
                Vector4::UNIT_X,
                Vector4::UNIT_Z,
                Vector4::UNIT_Z,
            )
            .with_limits(0.2, -0.1, 0.1),
        );
        assert!(id.is_valid());

        // Start the child outside the cone: 0.5 rad vs a 0.2 rad limit.
        world.set_body_transform(
            child,
            Transform::from_quaternion(
                Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.5),
                Vector4::ZERO,
            ),
        );

        let initial = world_twist_axis(&world, child).dot3(world_twist_axis(&world, parent));
        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }
        let final_angle = world_twist_axis(&world, child)
            .dot3(world_twist_axis(&world, parent))
            .clamp(-1.0, 1.0)
            .acos();
        assert!(
            final_angle < 0.35,
            "swing must come back into the cone: initial cos {initial}, final {final_angle}"
        );
    }

    #[test]
    fn ragdoll_chain_is_deterministic() {
        let build = || {
            let mut world = World::new(WorldCinfo::new());
            let root = world.create_body(
                &BodyCinfo::new(sphere_shape(0.5)).with_position(Vector4::from_xyz(0.0, 10.0, 0.0)),
            );
            let mut previous = root;
            for index in 1..4 {
                let body = world.create_body(
                    &BodyCinfo::new(sphere_shape(0.5))
                        .with_motion_type(MotionType::Dynamic)
                        .with_mass(1.0)
                        .with_position(Vector4::from_xyz(0.0, 10.0, index as f32)),
                );
                let motion_id = world.body(body).unwrap().motion_id;
                world.motion_mut(motion_id).unwrap().inverse_inertia = Vector4::splat(0.5);
                world.create_ragdoll_constraint(
                    &RagdollCinfo::new(
                        body,
                        previous,
                        Vector4::ZERO,
                        Vector4::ZERO,
                        Vector4::UNIT_X,
                        Vector4::UNIT_X,
                        Vector4::UNIT_Z,
                        Vector4::UNIT_Z,
                    )
                    .with_limits(0.6, -0.3, 0.3),
                );
                previous = body;
            }
            world
        };
        let mut first = build();
        let mut second = build();
        for _ in 0..180 {
            first.step(1.0 / 60.0);
            second.step(1.0 / 60.0);
        }
        assert_eq!(first.snapshot(), second.snapshot());
    }

    #[test]
    fn prismatic_joint_allows_only_slider_travel() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let parent = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        let slider = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        );
        let transform_a = world.body_transform(slider).unwrap();
        let transform_b = world.body_transform(parent).unwrap();
        let id = world.create_prismatic_constraint(
            &PrismaticCinfo::between_bodies(
                slider,
                parent,
                Vector4::ZERO,
                &transform_a,
                &transform_b,
                Vector4::UNIT_X,
            )
            .with_limits(-5.0, 5.0),
        );
        assert!(id.is_valid());
        // Keep the overlapping bodies out of the contact pipeline so the test
        // isolates the slider rows.
        assert!(world.set_constraint_collisions_enabled(id, false));
        world.set_body_linear_velocity(slider, Vector4::from_xyz(2.0, 1.0, 0.0));

        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let position = world.body_position(slider).unwrap();
        assert!(position.x > 1.0, "slider must travel: {position:?}");
        assert!(
            position.y.abs() < 0.15,
            "slider must not drift in y: {position:?}"
        );
        assert!(
            position.z.abs() < 0.15,
            "slider must not drift in z: {position:?}"
        );
    }

    #[test]
    fn prismatic_limits_stop_the_travel() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let parent = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        let slider = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        );
        let transform_a = world.body_transform(slider).unwrap();
        let transform_b = world.body_transform(parent).unwrap();
        let id = world.create_prismatic_constraint(
            &PrismaticCinfo::between_bodies(
                slider,
                parent,
                Vector4::ZERO,
                &transform_a,
                &transform_b,
                Vector4::UNIT_X,
            )
            .with_limits(-0.5, 0.5),
        );
        assert!(id.is_valid());
        assert!(world.set_constraint_collisions_enabled(id, false));
        world.set_body_linear_velocity(slider, Vector4::UNIT_X * 5.0);
        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }
        let x = world.body_position(slider).unwrap().x;
        assert!(x <= 0.8, "slider must stop at the limit: x {x}");
        assert!(
            world.body_linear_velocity(slider).unwrap().x.abs() < 0.2,
            "slider must stop: {:?}",
            world.body_linear_velocity(slider)
        );
    }

    #[test]
    fn spring_holds_the_pivot_distance() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let anchor = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5)).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)),
        );
        let bob = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        );
        let id = world.create_spring_constraint(&SpringCinfo::new(
            bob,
            anchor,
            Vector4::ZERO,
            Vector4::ZERO,
            1.0,
        ));
        assert!(id.is_valid());
        for _ in 0..240 {
            world.step(1.0 / 60.0);
        }
        let distance =
            (world.body_position(bob).unwrap() - Vector4::from_xyz(0.0, 2.0, 0.0)).length3();
        assert!(
            (distance - 1.0).abs() < 0.35,
            "spring distance {distance} must stay near 1.0"
        );
    }

    #[test]
    fn point_to_plane_keeps_the_pivot_on_the_plane() {
        let mut world = World::new(WorldCinfo::new());
        let plane_body = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        let body = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(0.0, 1.0, 0.0)),
        );
        let id = world.create_point_to_plane_constraint(&PointToPlaneCinfo::new(
            body,
            plane_body,
            Vector4::from_xyz(0.0, -1.0, 0.0),
            Vector4::ZERO,
            Vector4::UNIT_Y,
        ));
        assert!(id.is_valid());
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let y = world.body_position(body).unwrap().y;
        assert!(
            (y - 1.0).abs() < 0.25,
            "pivoted body must hang at y≈1 (pivot on the plane): y {y}"
        );
    }

    #[test]
    fn joint_collision_suppression_filters_contacts() {
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
        let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            a,
            b,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        assert!(
            !world.collect_contacts().is_empty(),
            "the overlapping bodies generate contacts by default"
        );

        assert!(world.set_constraint_collisions_enabled(id, false));
        assert!(!world.constraint_collisions_enabled(id).unwrap());
        assert!(
            world.collect_contacts().is_empty(),
            "overlapping jointed bodies must not generate contacts"
        );

        assert!(world.set_constraint_collisions_enabled(id, true));
        assert!(
            !world.collect_contacts().is_empty(),
            "re-enabling collisions must restore the contact"
        );
    }

    #[test]
    fn constraints_of_body_lists_attached_joints() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let a = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        let b = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        );
        let c = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(2.0, 0.0, 0.0)),
        );
        let first = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            a,
            b,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        let second = world.create_spring_constraint(&SpringCinfo::new(
            b,
            c,
            Vector4::ZERO,
            Vector4::ZERO,
            1.0,
        ));
        let attached: Vec<_> = world.constraints_of_body(b).map(|(id, _)| id).collect();
        assert_eq!(attached, vec![first, second]);
        assert_eq!(world.constraints_of_body(a).count(), 1);
        assert_eq!(world.constraints_of_body(c).count(), 1);
        assert!(!world.set_ragdoll_limits(first, 0.3, -0.1, 0.1));
        assert!(!world.set_prismatic_limits(first, -1.0, 1.0));
    }

    #[test]
    fn islands_split_bodies_on_a_shared_ground() {
        let mut world = World::new(WorldCinfo::new());
        let ground = world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(10.0, 0.5, 10.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        let left =
            world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(-3.0, 0.5, 0.0)));
        let right =
            world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(3.0, 0.5, 0.0)));

        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }

        assert_eq!(world.last_island_count(), 2);
        assert_eq!(world.stats().last_island_count, 2);
        let islands = world.islands();
        for island in islands {
            assert_eq!(island.dynamic_bodies, 1);
            assert!(island.contacts >= 1, "island without contacts");
            assert_eq!(island.joints, 0);
            assert!(
                island.bodies.contains(&ground),
                "the shared ground is listed in every island it touches"
            );
        }
        // Island order is by lowest dynamic body, so the negative-X sphere's
        // island comes first and body lists are ascending.
        assert_eq!(islands[0].bodies, vec![ground, left]);
        assert_eq!(islands[1].bodies, vec![ground, right]);
    }

    #[test]
    fn contact_chain_forms_one_island() {
        let mut world = World::new(WorldCinfo::new());
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(4.0, 0.5, 4.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::splat(0.5)))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(0.0, 0.6, 0.0)),
        );
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::splat(0.5)))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(0.0, 1.7, 0.0)),
        );

        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }
        assert_eq!(world.last_island_count(), 1);
        let island = &world.islands()[0];
        assert_eq!(island.dynamic_bodies, 2);
        assert!(island.contacts >= 2);
    }

    #[test]
    fn contact_cache_is_pruned_when_bodies_disappear() {
        let mut world = World::new(WorldCinfo::new());
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(3.0, 0.5, 3.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        let sphere =
            world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)));

        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        assert!(world.contact_cache.len() > 0, "no warm-start rows cached");

        assert!(world.destroy_body(sphere));
        world.step(1.0 / 60.0);
        assert_eq!(world.contact_cache.len(), 0);
        assert_eq!(world.last_island_count(), 0);

        world.clear_contact_cache();
        assert_eq!(world.contact_cache.len(), 0);
    }

    #[test]
    fn warm_starting_can_be_disabled() {
        let mut world = World::new(WorldCinfo::new().with_warm_starting(false));
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(3.0, 0.5, 3.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)));
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        assert!(world.last_contact_count() > 0);
        assert!(world.last_island_count() > 0);
        assert_eq!(world.contact_cache.len(), 0);
    }

    // Joint warm starting and hinge/prismatic motors.

    #[test]
    fn joint_warm_start_impulses_persist_in_slots() {
        let mut world = World::new(WorldCinfo::new());
        let a = world.create_body(&dynamic_cinfo(0.5));
        let b = world.create_body(&dynamic_cinfo(0.5));
        let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            a,
            b,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        let slot = id.as_constraint_index().value() as usize;

        // The overlapping pair is pushed apart by the joint rows, so rows
        // accumulate impulses.
        world.step(1.0 / 60.0);
        world.step(1.0 / 60.0);
        assert!(
            world.joints[slot]
                .as_ref()
                .unwrap()
                .impulses
                .rows
                .iter()
                .any(|&row| row != 0.0),
            "joint rows must keep their accumulated impulses in the slot"
        );

        // Destroying the constraint drops the slot with its impulse cache.
        assert!(world.destroy_constraint(id));
        assert!(world.joints[slot].is_none());
    }

    #[test]
    fn disabling_a_constraint_clears_its_warm_start_impulses() {
        let mut world = World::new(WorldCinfo::new());
        let a = world.create_body(&dynamic_cinfo(0.5));
        let b = world.create_body(&dynamic_cinfo(0.5));
        let id = world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            a,
            b,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        let slot = id.as_constraint_index().value() as usize;
        world.step(1.0 / 60.0);
        world.step(1.0 / 60.0);

        assert!(world.set_constraint_enabled(id, false));
        assert_eq!(
            world.joints[slot].as_ref().unwrap().impulses,
            phys_solver::JointImpulse::default(),
            "disabling must clear the warm-start impulses"
        );
        assert!(world.set_constraint_enabled(id, true));
    }

    #[test]
    fn hinge_position_motor_drives_the_arm_to_the_target_angle() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let (_, arm, id) = hinged_arm(&mut world);
        assert!(world.set_hinge_motor(
            id,
            Some(HingeMotorCinfo::Position(HingePositionMotorCinfo::new(
                1.0, 0.15, 20.0
            )))
        ));

        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let angle = current_hinge_angle(&world, hinged_arm_hub(&world), arm);
        assert!(
            (angle - 1.0).abs() < 0.25,
            "angle {angle} must converge to the motor target 1.0"
        );
    }

    #[test]
    fn hinge_spring_motor_oscillates_and_settles() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let (_, arm, id) = hinged_arm(&mut world);
        assert!(world.set_hinge_motor(
            id,
            Some(HingeMotorCinfo::Spring(HingeSpringMotorCinfo::new(
                0.5, 8.0, 2.0, 50.0
            )))
        ));

        let mut max_deviation = 0.0f32;
        for _ in 0..240 {
            world.step(1.0 / 60.0);
            let angle = current_hinge_angle(&world, hinged_arm_hub(&world), arm);
            max_deviation = max_deviation.max((angle - 0.5).abs());
        }
        let angle = current_hinge_angle(&world, hinged_arm_hub(&world), arm);
        assert!(
            (angle - 0.5).abs() < 0.25,
            "spring must settle near the rest angle: {angle}"
        );
        assert!(
            max_deviation <= 0.5 + 1e-3,
            "spring motion must stay bounded: {max_deviation}"
        );
    }

    #[test]
    fn prismatic_position_motor_drives_the_slider_to_the_target() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let parent = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        let slider = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::ZERO),
        );
        let id = world.create_prismatic_constraint(&PrismaticCinfo::between_bodies(
            slider,
            parent,
            Vector4::ZERO,
            &world.body_transform(slider).unwrap(),
            &world.body_transform(parent).unwrap(),
            Vector4::UNIT_X,
        ));
        assert!(world.set_constraint_collisions_enabled(id, false));
        assert!(world.set_prismatic_motor(
            id,
            Some(PrismaticMotorCinfo::Position(
                PrismaticPositionMotorCinfo::new(2.0, 0.2, 20.0)
            ))
        ));

        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }
        let x = world.body_position(slider).unwrap().x;
        assert!(
            (x - 2.0).abs() < 0.35,
            "slider x {x} must converge to the motor target 2.0"
        );
        assert!(
            world.body_position(slider).unwrap().y.abs() < 0.15,
            "slider must not drift off the axis"
        );
    }

    #[test]
    fn prismatic_spring_motor_pulls_the_slider_to_the_rest_length() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let parent = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        let slider = world.create_body(
            &BodyCinfo::new(sphere_shape(0.5))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::ZERO),
        );
        let id = world.create_prismatic_constraint(&PrismaticCinfo::between_bodies(
            slider,
            parent,
            Vector4::ZERO,
            &world.body_transform(slider).unwrap(),
            &world.body_transform(parent).unwrap(),
            Vector4::UNIT_X,
        ));
        assert!(world.set_constraint_collisions_enabled(id, false));
        assert!(world.set_prismatic_motor(
            id,
            Some(PrismaticMotorCinfo::Spring(PrismaticSpringMotorCinfo::new(
                0.0, 20.0, 4.0, 50.0
            )))
        ));

        // `between_bodies` binds the current pose (translation 0), so the
        // slider must be offset after creation to give the spring a
        // compressed start.
        world.set_body_transform(
            slider,
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(-2.0, 0.0, 0.0)),
        );

        for _ in 0..300 {
            world.step(1.0 / 60.0);
        }
        let x = world.body_position(slider).unwrap().x;
        assert!(
            x.abs() < 0.35,
            "spring must settle near the rest translation: x {x}"
        );
    }

    #[test]
    fn joint_motors_are_bitwise_deterministic() {
        let run = || {
            let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
            let (_, arm, id) = hinged_arm(&mut world);
            world.set_hinge_motor(
                id,
                Some(HingeMotorCinfo::Spring(HingeSpringMotorCinfo::new(
                    0.5, 8.0, 2.0, 50.0,
                ))),
            );
            world.set_body_transform(
                arm,
                Transform::from_quaternion(
                    Quaternion::from_axis_angle(Vector4::UNIT_Z, 0.3),
                    Vector4::from_xyz(2.0, 0.0, 0.0),
                ),
            );
            for _ in 0..120 {
                world.step(1.0 / 60.0);
            }
            world.snapshot()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn joint_warm_starting_is_bitwise_deterministic() {
        let run = || {
            let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
            let a = world.create_body(
                &BodyCinfo::new(sphere_shape(0.5))
                    .with_motion_type(MotionType::Dynamic)
                    .with_mass(1.0)
                    .with_position(Vector4::from_xyz(0.0, 4.0, 0.0))
                    .with_linear_velocity(Vector4::UNIT_X),
            );
            let b = world.create_body(
                &BodyCinfo::new(sphere_shape(0.5))
                    .with_motion_type(MotionType::Dynamic)
                    .with_mass(1.0)
                    .with_position(Vector4::from_xyz(1.0, 4.0, 0.0)),
            );
            world.create_fixed_constraint(&FixedCinfo::between_bodies(
                a,
                b,
                Vector4::from_xyz(0.5, 4.0, 0.0),
                &world.body_transform(a).unwrap(),
                &world.body_transform(b).unwrap(),
            ));
            for _ in 0..120 {
                world.step(1.0 / 60.0);
            }
            world.snapshot()
        };
        assert_eq!(run(), run());
    }

    // Island sleeping (deactivation subset).

    fn sleeping_world() -> World {
        World::new(
            WorldCinfo::new()
                .with_sleeping(true)
                .with_sleeping_settings(0.2, 0.1, 0.1),
        )
    }

    #[test]
    fn quiet_island_falls_asleep_and_skips_narrow_phase() {
        let mut world = sleeping_world();
        let ground = world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(3.0, 0.5, 3.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        let ball =
            world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 0.5, 0.0)));

        for _ in 0..30 {
            world.step(1.0 / 60.0);
        }

        assert_eq!(world.body_is_sleeping(ball), Some(true));
        assert_eq!(world.body_is_sleeping(ground), Some(false));
        assert_eq!(world.stats().sleeping_body_count, 1);
        // Frozen state: zero velocity, no narrow phase against the ground.
        assert_eq!(
            world.body_linear_velocity(ball),
            Some(Vector4::ZERO),
            "sleeping motions must be zeroed"
        );
        assert_eq!(world.last_contact_count(), 0);
        assert!(world.collect_contacts().is_empty());
        assert_eq!(world.last_island_count(), 0);
        // The pose is frozen (no integration), so body states are stable.
        let before = world.snapshot();
        world.step(1.0 / 60.0);
        assert_eq!(world.snapshot().bodies, before.bodies);
    }

    #[test]
    fn falling_body_wakes_a_sleeping_island() {
        let mut world = sleeping_world();
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(3.0, 0.5, 3.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        let resting =
            world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 0.5, 0.0)));
        for _ in 0..30 {
            world.step(1.0 / 60.0);
        }
        assert_eq!(world.body_is_sleeping(resting), Some(true));

        // Drop an active ball onto the sleeping island: the contact with an
        // awake body must wake it.
        world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 2.5, 0.0)));
        for _ in 0..30 {
            world.step(1.0 / 60.0);
            if !world.body_is_sleeping(resting).unwrap() {
                break;
            }
        }
        assert_eq!(world.body_is_sleeping(resting), Some(false));
        assert!(world.last_contact_count() > 0);
        // The woken island is solveable again, so both balls rest on the ground.
        let x = world.body_position(resting).unwrap();
        assert!(
            x.y > 0.0 && x.y < 0.75,
            "woken ball must stay on ground: {x:?}"
        );
    }

    #[test]
    fn wake_body_covers_the_joint_graph() {
        let mut world = World::new(
            WorldCinfo::new()
                .with_gravity(Vector4::ZERO)
                .with_sleeping(true)
                .with_sleeping_settings(0.1, 0.1, 0.1),
        );
        let a = world.create_body(&dynamic_cinfo(0.5));
        let b = world.create_body(&dynamic_cinfo(0.5));
        world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            a,
            b,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        for _ in 0..12 {
            world.step(1.0 / 60.0);
        }
        assert_eq!(world.body_is_sleeping(a), Some(true));
        assert_eq!(world.body_is_sleeping(b), Some(true));

        assert!(world.wake_body(a));
        assert_eq!(world.body_is_sleeping(a), Some(false));
        assert_eq!(
            world.body_is_sleeping(b),
            Some(false),
            "jointed partner wakes too"
        );
        assert!(!world.wake_body(BodyId::invalid()));
    }

    #[test]
    fn setting_velocity_or_transform_wakes_the_body() {
        let mut world = sleeping_world();
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(3.0, 0.5, 3.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        let ball =
            world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 0.5, 0.0)));
        for _ in 0..30 {
            world.step(1.0 / 60.0);
        }
        assert_eq!(world.body_is_sleeping(ball), Some(true));

        assert!(world.set_body_linear_velocity(ball, Vector4::UNIT_X));
        assert_eq!(world.body_is_sleeping(ball), Some(false));

        // Sleep again, then teleport.
        for _ in 0..30 {
            world.step(1.0 / 60.0);
        }
        assert_eq!(world.body_is_sleeping(ball), Some(true));
        assert!(world.set_body_position(ball, Vector4::from_xyz(0.0, 3.0, 0.0)));
        assert_eq!(world.body_is_sleeping(ball), Some(false));
    }

    #[test]
    fn keyframed_partner_keeps_an_island_awake() {
        let mut world = World::new(
            WorldCinfo::new()
                .with_gravity(Vector4::ZERO)
                .with_sleeping(true)
                .with_sleeping_settings(0.1, 0.1, 0.1),
        );
        let anchor = world.create_body(&BodyCinfo::new(sphere_shape(0.5)));
        let pendulum =
            world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 2.0, 0.0)));
        world.create_ball_socket_constraint(&BallAndSocketCinfo::new(
            pendulum,
            anchor,
            Vector4::ZERO,
            Vector4::ZERO,
        ));
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        assert_eq!(
            world.body_is_sleeping(pendulum),
            Some(false),
            "a joint to a keyframed body must keep the island awake"
        );
    }

    // Wheel joints (suspension + driven axle).

    /// A chassis box resting on two sphere wheels via wheel joints.
    ///
    /// Returns `(chassis, [front, rear], [front_id, rear_id])`.
    fn wheeled_cart(world: &mut World) -> (BodyId, [BodyId; 2], [ConstraintId; 2]) {
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(10.0, 0.5, 10.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        let chassis = world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(1.0, 0.25, 0.5)))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(2.0)
                .with_position(Vector4::from_xyz(0.0, 1.0, 0.0)),
        );
        let mut wheels = [BodyId::invalid(); 2];
        let mut ids = [ConstraintId::INVALID; 2];
        for (index, x) in [0.7, -0.7].into_iter().enumerate() {
            let wheel = world.create_body(
                &BodyCinfo::new(sphere_shape(0.3))
                    .with_motion_type(MotionType::Dynamic)
                    .with_mass(0.2)
                    .with_position(Vector4::from_xyz(x, 0.3, 0.0)),
            );
            let motion_id = world.body(wheel).unwrap().motion_id;
            world.motion_mut(motion_id).unwrap().inverse_inertia = Vector4::splat(0.05);
            // Hardpoint under the chassis bottom (y = 0.75 in chassis-local
            // space: 1.0 - 0.25), suspension axis down, wheel center 0.45
            // below the hardpoint at setup.
            let mut info = WheelCinfo::new(
                wheel,
                chassis,
                Vector4::ZERO,
                Vector4::from_xyz(x, -0.25, 0.0),
                Vector4::from_xyz(0.0, -1.0, 0.0),
                Vector4::UNIT_X,
                Vector4::UNIT_Z,
            )
            .with_suspension(WheelSuspensionCinfo::new(0.45, 120.0, 8.0, 50.0))
            .with_travel(0.25, 0.65);
            info.axle_b = Vector4::UNIT_X;
            ids[index] = world.create_wheel_constraint(&info);
            wheels[index] = wheel;
        }
        (chassis, wheels, ids)
    }

    #[test]
    fn wheel_suspension_supports_the_chassis_on_the_ground() {
        let mut world = World::new(WorldCinfo::new());
        let (chassis, wheels, ids) = wheeled_cart(&mut world);
        for id in ids {
            assert!(id.is_valid());
            assert!(matches!(world.constraint(id), Some(WorldJoint::Wheel(_))));
        }
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let chassis_y = world.body_position(chassis).unwrap().y;
        assert!(
            chassis_y > 0.5 && chassis_y < 1.5,
            "chassis must settle on its suspension: y {chassis_y}"
        );
        for wheel in wheels {
            let position = world.body_position(wheel).unwrap();
            assert!(
                position.y > 0.0 && position.y < 0.9,
                "wheel must stay under the hardpoint: {position:?}"
            );
        }
    }

    #[test]
    fn wheel_drive_propels_the_chassis_forward() {
        let mut world = World::new(WorldCinfo::new());
        let (chassis, _, ids) = wheeled_cart(&mut world);
        for id in ids {
            assert!(world.set_wheel_drive(id, Some(WheelDriveCinfo::new(20.0, 2.0))));
        }
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let position = world.body_position(chassis).unwrap();
        // The axles point along X, so the cart drives along +Z for a positive
        // spin velocity (contact-point friction pushes it forward).
        assert!(
            position.z > 0.5,
            "driven wheels must move the chassis forward: {position:?}"
        );
        assert!(
            position.y > 0.0 && position.y < 1.6,
            "chassis must stay above the ground: {position:?}"
        );
    }

    #[test]
    fn wheel_api_validates_ids_and_clears_suspension() {
        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let (chassis, _, ids) = wheeled_cart(&mut world);
        let id = ids[0];
        assert!(world.set_steering_angle(id, 0.2));
        assert!(world.set_wheel_suspension(id, None));
        assert!(world.set_wheel_drive(id, None));
        assert!(!world.set_steering_angle(ConstraintId::INVALID, 0.2));
        assert!(world.body_position(chassis).is_some());

        // Joint type lookups stay in sync.
        assert_eq!(world.constraints_of_body(chassis).count(), 2);
        assert!(matches!(world.constraint(id), Some(WorldJoint::Wheel(_))));
    }
}
