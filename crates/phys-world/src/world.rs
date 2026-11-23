//! `hknpWorld` — world assembly and single-threaded step pipeline.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::fmt;
use std::sync::{Arc, Mutex};

use base_math::{Matrix3, Quaternion, Transform, Vector4};
use phys_body::{
    Body, BodyCinfo, BodyManager, Motion, MotionManager, inverse_inertia_diagonal_matrix,
    inverse_inertia_is_diagonal, world_inverse_inertia_from_mass_properties,
};
use phys_collide::{
    BodyAabb, BroadPhase, BroadPhaseConfig, CollisionFilter, closest_points,
    collide_with_tolerance, leaf_key_at_local_point,
};
use phys_shape::{MassConfig, MassDistribution, MassProperties, Shape, ShapeMassCache};
use phys_solver::{
    BallAndSocketJoint, ContactConstraint, ContactImpulse, FixedJoint, HingeJoint, HingeLimits,
    HingeMotor, JointConstraint, JointImpulse, PointToPlaneJoint, PrismaticJoint, PrismaticMotor,
    RagdollJoint, SixDofJoint, Solver, SolverBody, SolverSettings, SpringJoint, WheelDrive,
    WheelJoint, WheelSuspension, contact_tangents,
};
use phys_types::{BodyId, CollisionFlags, ConstraintId, MotionId, MotionType, ShapeKey};

use crate::cache::{ContactImpulseCache, ContactKey};
use crate::constraint::{
    BallAndSocketCinfo, FixedCinfo, HingeCinfo, HingeLimitsCinfo, HingeMotorCinfo, JointSlot,
    LimitedHingeCinfo, PointToPlaneCinfo, PrismaticCinfo, PrismaticMotorCinfo, RagdollCinfo,
    SixDofAxisMode, SixDofCinfo, SixDofLimitsCinfo, SpringCinfo, WheelCinfo, WheelDriveCinfo,
    WheelSuspensionCinfo, WorldJoint, six_dof_constraint_frame_axes,
};
use crate::contact_modifier::{
    ContactEditAction, ContactModifier, ContactModifierId, ContactModifierRegistry, EditableContact,
};
use crate::events::{PostCollideContactPoint, PostCollideEvent, TriggerEvent, TriggerEventMerger};
use crate::island::{IslandInfo, build_islands};
use crate::signals::{
    MassChangeEvent, MassChangeReason, MassConnectionId, MassSignal, MassSummary,
    PostCollideConnectionId, PostCollideSignal, PostSolveConnectionId, PostSolveEvent,
    PostSolveSignal, TriggerConnectionId, TriggerSignal,
};
use crate::solver_data::{
    SolverDataHook, SolverDataHookId, SolverDataHookRegistry, SolverDataView,
};
use crate::{BodySnapshot, ContactManifold, WorldCinfo, WorldSnapshot, WorldStats};

/// Per-body deactivation state (the world's deactivation-manager subset).
#[derive(Clone, Copy, Debug, Default)]
struct SleepState {
    /// Seconds the body has continuously stayed below the sleeping thresholds.
    timer: f32,
    /// Whether the body is currently asleep (velocity zero, no narrow phase).
    sleeping: bool,
}

/// How `attachBodies`/`detachBodies` update the shared motion's mass
/// properties (`hknpWorld::UpdateMotionMode` subset).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionMassUpdateMode {
    /// Leave the motion's mass properties untouched (SDK default).
    Keep,
    /// Incrementally update the compound mass properties. This subset uses
    /// the same deterministic rebuild as [`Self::Rebuild`].
    Update,
    /// Recompute mass, center of mass and diagonal inertia from all members.
    Rebuild,
}

/// A group of bodies sharing one non-static motion (`hknpMotion` attachment
/// list subset). Members are stored in insertion order so all iteration that
/// affects simulation state is deterministic.
#[derive(Clone, Debug)]
struct MotionGroup {
    motion_id: MotionId,
    members: Vec<BodyId>,
}

type TriggerPairKey = (BodyId, BodyId, ShapeKey, ShapeKey);

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
    /// Shared-motion groups; indices are stable and never reused.
    motion_groups: Vec<MotionGroup>,
    /// Body → shared-motion group lookup (never iterated).
    group_of_body: HashMap<BodyId, usize>,
    /// Insertion-ordered shape mass-property cache.
    mass_cache: ShapeMassCache,
    /// Mass-property change notifications (post-rebuild dispatch).
    mass_signal: MassSignal,
    /// Post-solve notifications (`hknpWorld::m_signals.m_postSolve` subset).
    post_solve_signal: PostSolveSignal,
    /// Bodies flagged as trigger volumes (`hknpCollisionFlags::IS_TRIGGER` subset).
    trigger_bodies: BTreeSet<BodyId>,
    /// Trigger pairs that overlapped at the end of the previous step.
    active_trigger_pairs: BTreeSet<TriggerPairKey>,
    /// Per-step trigger enter/exit accumulator (`hknpEventMergeAndDispatcher`).
    trigger_merge: TriggerEventMerger,
    /// World trigger event dispatch.
    trigger_signal: TriggerSignal,
    /// Pre-solve contact snapshots for character rigid-body listeners.
    post_collide_signal: PostCollideSignal,
    /// Deterministic contact-row editors applied before islands are built.
    contact_modifiers: ContactModifierRegistry,
    /// Deterministic Jacobian-like row viewers applied after contact editors.
    solver_data_hooks: SolverDataHookRegistry,
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
            .field("mass_cache", &self.mass_cache)
            .field("mass_signal", &self.mass_signal)
            .field("post_solve_signal", &self.post_solve_signal)
            .field("trigger_bodies", &self.trigger_bodies.len())
            .field("active_trigger_pairs", &self.active_trigger_pairs.len())
            .field("trigger_signal", &self.trigger_signal)
            .field("post_collide_signal", &self.post_collide_signal)
            .field("contact_modifiers", &self.contact_modifiers)
            .field("solver_data_hooks", &self.solver_data_hooks)
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
            motion_groups: Vec::new(),
            group_of_body: HashMap::new(),
            mass_cache: ShapeMassCache::new(),
            mass_signal: MassSignal::default(),
            post_solve_signal: PostSolveSignal::default(),
            trigger_bodies: BTreeSet::new(),
            active_trigger_pairs: BTreeSet::new(),
            trigger_merge: TriggerEventMerger::default(),
            trigger_signal: TriggerSignal::default(),
            post_collide_signal: PostCollideSignal::default(),
            contact_modifiers: ContactModifierRegistry::default(),
            solver_data_hooks: SolverDataHookRegistry::default(),
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
        let shape_properties = if cinfo.mass < 0.0 {
            self.mass_cache
                .compute(&cinfo.shape, cinfo.mass_config)
                .ok()
        } else {
            None
        };
        let resolved_mass = shape_properties
            .map(|properties| properties.mass)
            .unwrap_or(cinfo.mass);
        let transform = cinfo.transform();

        let (motion_id, motion_flags) = match cinfo.motion_type {
            MotionType::Static => (MotionId::PRESET_STATIC, CollisionFlags::IS_STATIC),
            MotionType::Keyframed => {
                let motion = Motion::new(
                    MotionType::Keyframed,
                    cinfo.position,
                    cinfo.orientation,
                    cinfo.linear_velocity,
                    cinfo.angular_velocity,
                    resolved_mass,
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
                    resolved_mass,
                );
                (
                    self.motions.allocate(motion),
                    CollisionFlags::IS_DYNAMIC | CollisionFlags::IS_ACTIVE,
                )
            }
        };

        if let Some(properties) = shape_properties {
            if motion_id != MotionId::PRESET_STATIC && cinfo.motion_type.is_dynamic() {
                let inertia_world =
                    world_inverse_inertia_from_mass_properties(&properties, &transform);
                if let Some(motion) = self.motions.get_mut(motion_id) {
                    motion.set_inverse_inertia_world(inertia_world);
                }
            }
        }

        let mut body = Body::new(cinfo.shape.clone(), transform, motion_id);
        if let Some(motion) = self.motions.get(motion_id) {
            body.mass = motion.mass;
            body.inverse_inertia = motion.inverse_inertia;
            body.inverse_inertia_world = motion.inverse_inertia_world;
        }
        body.mass_properties = shape_properties;
        body.mass_config = cinfo.mass_config;
        body.derive_mass_from_shape = cinfo.mass < 0.0;
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

    /// Create a body that shares an existing body's motion
    /// (`hknpWorld::allocateAttachedBody` subset).
    ///
    /// The target must be non-static. The created body gets the target's
    /// motion type and motion slot; the temporarily allocated motion is
    /// released. Cinfo mass/velocity settings are therefore ignored, matching
    /// the SDK's attached-body rule.
    pub fn create_attached_body(&mut self, target: BodyId, cinfo: &BodyCinfo) -> Option<BodyId> {
        let target_motion = self.bodies.get(target)?.motion_id;
        if target_motion == MotionId::PRESET_STATIC || self.motions.get(target_motion).is_none() {
            return None;
        }
        let motion_type = self.motions.get(target_motion)?.motion_type;
        let mut cinfo = cinfo.clone();
        cinfo.motion_type = motion_type;
        let body = self.create_body(&cinfo);
        let created_motion = self.bodies.get(body)?.motion_id;
        if created_motion != MotionId::PRESET_STATIC {
            self.motions.destroy(created_motion);
        }
        self.bodies.get_mut(body)?.motion_id = target_motion;
        let index = self.ensure_motion_group(target)?;
        self.motion_groups[index].members.push(body);
        self.group_of_body.insert(body, index);
        Some(body)
    }

    /// Attach existing bodies to `target`'s shared motion (`attachBodies`
    /// subset). Bodies already using the target's motion are skipped.
    ///
    /// Mass properties are retained by default (`KEEP_MASS_PROPERTIES`);
    /// [`Self::attach_bodies_with_mode`] can rebuild them from all members.
    pub fn attach_bodies(&mut self, target: BodyId, bodies: &[BodyId]) -> bool {
        self.attach_bodies_with_mode(target, bodies, MotionMassUpdateMode::Keep)
    }

    /// [`Self::attach_bodies`] with an explicit mass-property update mode.
    pub fn attach_bodies_with_mode(
        &mut self,
        target: BodyId,
        bodies: &[BodyId],
        mode: MotionMassUpdateMode,
    ) -> bool {
        let Some(target_motion) = self.bodies.get(target).map(|body| body.motion_id) else {
            return false;
        };
        if target_motion == MotionId::PRESET_STATIC || self.motions.get(target_motion).is_none() {
            return false;
        }
        for &body in bodies {
            let Some(entry) = self.bodies.get(body) else {
                return false;
            };
            if entry.motion_id == MotionId::PRESET_STATIC {
                return false;
            }
        }
        let Some(group_index) = self.ensure_motion_group(target) else {
            return false;
        };
        for &body in bodies {
            let Some(entry) = self.bodies.get(body) else {
                continue;
            };
            if entry.motion_id == target_motion {
                continue;
            }
            self.detach_motion_membership(body);
            let Some(motion_type) = self
                .motions
                .get(target_motion)
                .map(|motion| motion.motion_type)
            else {
                continue;
            };
            if let Some(body) = self.bodies.get_mut(body) {
                body.motion_id = target_motion;
                body.flags
                    .set(CollisionFlags::IS_DYNAMIC, motion_type.is_dynamic());
                body.flags
                    .set(CollisionFlags::IS_KEYFRAMED, !motion_type.is_dynamic());
                body.flags
                    .set(CollisionFlags::IS_ACTIVE, motion_type.is_dynamic());
            }
            self.sleep_states.remove(&body);
            self.motion_groups[group_index].members.push(body);
            self.group_of_body.insert(body, group_index);
        }
        if mode != MotionMassUpdateMode::Keep {
            let members = self.motion_groups[group_index].members.clone();
            self.recompute_motion_properties(target_motion, &members);
        }
        true
    }

    /// Detach bodies from their shared motion, allocating a fresh motion for
    /// each (`detachBodies` subset). The new motion keeps the group's motion
    /// type, mass and inverse inertia, and translates the linear velocity to
    /// the body's center (`v + ω × r`).
    pub fn detach_bodies(&mut self, bodies: &[BodyId]) -> bool {
        self.detach_bodies_with_mode(bodies, MotionMassUpdateMode::Keep)
    }

    /// [`Self::detach_bodies`] with an explicit mass-property update mode.
    pub fn detach_bodies_with_mode(
        &mut self,
        bodies: &[BodyId],
        mode: MotionMassUpdateMode,
    ) -> bool {
        let mut affected: Vec<(MotionId, Vec<BodyId>)> = Vec::new();
        for &body in bodies {
            let Some(group_index) = self.group_of_body.get(&body).copied() else {
                continue;
            };
            let group = &self.motion_groups[group_index];
            if !affected.iter().any(|(id, _)| *id == group.motion_id) {
                let remaining: Vec<_> = group
                    .members
                    .iter()
                    .copied()
                    .filter(|member| !bodies.contains(member))
                    .collect();
                affected.push((group.motion_id, remaining));
            }
        }
        for &body in bodies {
            let Some(group_index) = self.group_of_body.get(&body).copied() else {
                continue;
            };
            let motion_id = self.motion_groups[group_index].motion_id;
            let Some(motion) = self.motions.get(motion_id).copied() else {
                continue;
            };
            let Some(body_properties) = self.bodies.get(body).map(|entry| {
                (
                    entry.transform,
                    entry.mass,
                    entry.inverse_inertia,
                    entry.inverse_inertia_world,
                )
            }) else {
                continue;
            };
            let transform = body_properties.0;
            let new_position = transform.translation;
            let mut detached = Motion::new(
                motion.motion_type,
                new_position,
                transform.rotation_quaternion(),
                motion.linear_velocity
                    + motion
                        .angular_velocity
                        .cross(new_position - motion.position),
                motion.angular_velocity,
                if body_properties.1 > 0.0 {
                    body_properties.1
                } else {
                    motion.mass
                },
            );
            if body_properties.3 != Matrix3::ZERO {
                detached.set_inverse_inertia_world(body_properties.3);
            } else {
                detached.set_inverse_inertia_diagonal(body_properties.2);
            }
            let new_id = self.motions.allocate(detached);
            self.detach_motion_membership(body);
            if let Some(entry) = self.bodies.get_mut(body) {
                entry.motion_id = new_id;
            }
        }
        if mode != MotionMassUpdateMode::Keep {
            for (motion_id, members) in affected {
                self.recompute_motion_properties(motion_id, &members);
            }
        }
        true
    }

    /// Bodies sharing `motion_id`, in attachment order. Empty for standalone
    /// bodies (including the shared static motion).
    pub fn bodies_sharing_motion(&self, motion_id: MotionId) -> &[BodyId] {
        self.motion_groups
            .iter()
            .find(|group| group.motion_id == motion_id)
            .map(|group| group.members.as_slice())
            .unwrap_or_default()
    }

    /// The motion slot of a body.
    pub fn body_motion_id(&self, body_id: BodyId) -> Option<MotionId> {
        self.bodies.get(body_id).map(|body| body.motion_id)
    }

    /// Start (or reuse) the shared-motion group anchored at `target`.
    fn ensure_motion_group(&mut self, target: BodyId) -> Option<usize> {
        if let Some(index) = self.group_of_body.get(&target).copied() {
            return Some(index);
        }
        let motion_id = self.bodies.get(target)?.motion_id;
        if motion_id == MotionId::PRESET_STATIC || self.motions.get(motion_id).is_none() {
            return None;
        }
        let index = self.motion_groups.len();
        self.motion_groups.push(MotionGroup {
            motion_id,
            members: vec![target],
        });
        self.group_of_body.insert(target, index);
        Some(index)
    }

    /// Remove a body from its motion group. Groups collapse at one member:
    /// the remaining body keeps the motion slot, so no slot is freed here.
    fn detach_motion_membership(&mut self, body: BodyId) -> Option<MotionId> {
        let index = self.group_of_body.remove(&body)?;
        let group = &mut self.motion_groups[index];
        group.members.retain(|&member| member != body);
        if group.members.len() < 2 {
            group.members.clear();
        }
        Some(group.motion_id)
    }

    /// World-space mass properties of one body for shared-motion aggregation.
    fn body_world_mass_properties(body: &Body) -> Option<MassProperties> {
        if body.mass <= 0.0 || !body.mass.is_finite() {
            return None;
        }
        if let Some(properties) = body.mass_properties {
            return Some(properties.transformed(&body.transform));
        }
        let inverse_world = if body.inverse_inertia_world != Matrix3::ZERO
            && (!inverse_inertia_is_diagonal(body.inverse_inertia_world, 1e-8)
                || body.inverse_inertia == Vector4::ZERO)
        {
            body.inverse_inertia_world
        } else {
            inverse_inertia_diagonal_matrix(body.inverse_inertia)
        };
        let (inertia_tensor, _) = inverse_world.inverse();
        Some(MassProperties {
            volume: 0.0,
            mass: body.mass,
            center_of_mass: body.transform.translation,
            inertia_tensor,
            major_axis_space: Quaternion::IDENTITY,
        })
    }

    /// Recompute a shared dynamic motion's mass, center of mass and full world
    /// inverse inertia from the given members (`rebuildMotionMassProperties`).
    /// Non-diagonal inertia terms are preserved through the parallel-axis theorem.
    fn recompute_motion_properties(&mut self, motion_id: MotionId, members: &[BodyId]) {
        let Some(motion) = self.motions.get(motion_id).copied() else {
            return;
        };
        if !motion.motion_type.is_dynamic() {
            return;
        }
        let combined = MassProperties::combine_all(
            members
                .iter()
                .filter_map(|&member| self.bodies.get(member))
                .filter_map(Self::body_world_mass_properties),
        );
        if combined.mass <= 0.0 || !combined.mass.is_finite() {
            return;
        }
        let old_position = motion.position;
        let (inverse_world, _) = combined.inertia_tensor.inverse();
        if let Some(motion) = self.motions.get_mut(motion_id) {
            motion.mass = combined.mass;
            motion.position = combined.center_of_mass;
            motion.set_inverse_inertia_world(inverse_world);
            motion.linear_velocity += motion
                .angular_velocity
                .cross(combined.center_of_mass - old_position);
        }
    }

    /// Set a body's local inverse inertia, keeping the stored mass-merge
    /// source in sync with its motion.
    pub fn set_body_inverse_inertia(&mut self, body_id: BodyId, inverse: Vector4) -> bool {
        let old = self.body_mass_summary(body_id);
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        let motion_id = body.motion_id;
        if let Some(motion) = self.motions.get_mut(motion_id) {
            motion.set_inverse_inertia_diagonal(inverse);
        }
        if let Some(body) = self.bodies.get_mut(body_id) {
            body.inverse_inertia = inverse;
            body.inverse_inertia_world = inverse_inertia_diagonal_matrix(inverse);
            body.mass_version = body.mass_version.wrapping_add(1);
        }
        self.emit_mass_change(body_id, MassChangeReason::InverseInertiaSet, old);
        true
    }

    /// Set a body's full world-space inverse inertia matrix, keeping the
    /// legacy diagonal view in sync.
    pub fn set_body_inverse_inertia_world(&mut self, body_id: BodyId, matrix: Matrix3) -> bool {
        let old = self.body_mass_summary(body_id);
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        let motion_id = body.motion_id;
        let diagonal = Vector4::from_xyz(
            matrix.element(0, 0),
            matrix.element(1, 1),
            matrix.element(2, 2),
        );
        if let Some(motion) = self.motions.get_mut(motion_id) {
            motion.set_inverse_inertia_world(matrix);
        }
        if let Some(body) = self.bodies.get_mut(body_id) {
            body.inverse_inertia = diagonal;
            body.inverse_inertia_world = matrix;
            body.mass_version = body.mass_version.wrapping_add(1);
        }
        self.emit_mass_change(body_id, MassChangeReason::InverseInertiaSet, old);
        true
    }

    fn body_mass_summary(&self, body_id: BodyId) -> Option<MassSummary> {
        let body = self.bodies.get(body_id)?;
        let mut summary = MassSummary::from_properties(body.mass_properties.as_ref())?;
        summary.inverse_inertia_world = body.inverse_inertia_world;
        Some(summary)
    }

    fn emit_mass_change(
        &self,
        body_id: BodyId,
        reason: MassChangeReason,
        old: Option<MassSummary>,
    ) {
        let event = MassChangeEvent {
            body: body_id,
            reason,
            old,
            new: self.body_mass_summary(body_id),
        };
        self.mass_signal.emit(&event);
    }

    fn rebuild_body_mass_properties_inner(
        &mut self,
        body_id: BodyId,
        reason: MassChangeReason,
    ) -> bool {
        let old = self.body_mass_summary(body_id);
        let (motion_id, transform, derive_from_shape, mass_config, shape, existing_properties) = {
            let Some(body) = self.bodies.get(body_id) else {
                return false;
            };
            (
                body.motion_id,
                body.transform,
                body.derive_mass_from_shape,
                body.mass_config,
                body.shape.clone(),
                body.mass_properties,
            )
        };
        let properties = if derive_from_shape {
            self.mass_cache.compute(&shape, mass_config).ok()
        } else {
            existing_properties
        };

        if let Some(motion) = self.motions.get_mut(motion_id) {
            if motion.motion_type.is_dynamic() {
                let matrix = properties.map(|properties| {
                    world_inverse_inertia_from_mass_properties(&properties, &transform)
                });
                if let Some(matrix) = matrix {
                    motion.mass = properties.map_or(motion.mass, |p| p.mass);
                    motion.set_inverse_inertia_world(matrix);
                }
            }
        }

        if let Some(body) = self.bodies.get_mut(body_id) {
            body.mass_properties = properties;
            if motion_id != MotionId::PRESET_STATIC {
                if let Some(motion) = self.motions.get(motion_id) {
                    body.mass = motion.mass;
                    body.inverse_inertia = motion.inverse_inertia;
                    body.inverse_inertia_world = motion.inverse_inertia_world;
                }
            }
            body.mass_version = body.mass_version.wrapping_add(1);
        }

        if let Some(&group_index) = self.group_of_body.get(&body_id) {
            let members = self.motion_groups[group_index].members.clone();
            self.recompute_motion_properties(motion_id, &members);
        }
        self.emit_mass_change(body_id, reason, old);
        true
    }

    /// Replace a body's shape and rebuild cached mass properties
    /// (`hknpWorld` shape-mutation subset).
    pub fn set_body_shape(&mut self, body_id: BodyId, shape: Arc<dyn Shape>) -> bool {
        let old_shape = {
            let Some(body) = self.bodies.get(body_id) else {
                return false;
            };
            body.shape.clone()
        };
        self.mass_cache.invalidate(&old_shape);
        self.bodies.get_mut(body_id).unwrap().set_shape(shape);
        self.rebuild_body_mass_properties_inner(body_id, MassChangeReason::ShapeReplaced)
    }

    /// Replace a body's mass configuration and rebuild derived mass properties.
    pub fn set_body_mass_config(&mut self, body_id: BodyId, config: MassConfig) -> bool {
        let Some(body) = self.bodies.get_mut(body_id) else {
            return false;
        };
        body.set_mass_config(config);
        self.rebuild_body_mass_properties_inner(body_id, MassChangeReason::MassConfigChanged)
    }

    /// Replace a body's explicit mass. Negative values switch back to
    /// shape-derived mass properties.
    pub fn set_body_mass(&mut self, body_id: BodyId, mass: f32) -> bool {
        if !mass.is_finite() {
            return false;
        }
        let old = self.body_mass_summary(body_id);
        let explicit = mass >= 0.0;
        {
            let Some(body) = self.bodies.get_mut(body_id) else {
                return false;
            };
            let motion_id = body.motion_id;
            body.mass = mass;
            body.derive_mass_from_shape = !explicit;
            body.mass_version = body.mass_version.wrapping_add(1);
            if explicit {
                body.mass_properties = body
                    .mass_properties
                    .and_then(|properties| properties.with_mass(mass).ok());
                if let Some(motion) = self.motions.get_mut(motion_id) {
                    motion.mass = mass;
                    if mass == 0.0 {
                        motion.set_inverse_inertia_diagonal(Vector4::ZERO);
                    }
                }
                if let Some(motion) = self.motions.get(motion_id) {
                    body.inverse_inertia = motion.inverse_inertia;
                    body.inverse_inertia_world = motion.inverse_inertia_world;
                }
            }
        };
        if explicit {
            self.emit_mass_change(body_id, MassChangeReason::MassConfigChanged, old);
        } else {
            self.rebuild_body_mass_properties_inner(body_id, MassChangeReason::MassConfigChanged);
        }
        true
    }

    /// Rebuild one body's mass properties from its current shape/config.
    pub fn rebuild_body_mass_properties(&mut self, body_id: BodyId) -> bool {
        self.rebuild_body_mass_properties_inner(body_id, MassChangeReason::MotionRebuilt)
    }

    /// Mass distribution of one body (`hknpMassDistribution` diagnostic API).
    pub fn body_mass_distribution(&self, body_id: BodyId) -> Option<MassDistribution> {
        let body = self.bodies.get(body_id)?;
        if let Some(properties) = body.mass_properties.as_ref() {
            return MassDistribution::from_mass_properties(properties).ok();
        }
        MassDistribution::from_shape(&body.shape, body.mass_config).ok()
    }

    /// Build a unit-mass distribution from transformed shapes (`setFromShapes`).
    pub fn mass_distribution_from_shapes(
        &self,
        shapes: &[(Transform, Arc<dyn Shape>)],
        config: MassConfig,
    ) -> Result<MassDistribution, phys_shape::MassError> {
        MassDistribution::from_shapes(shapes, config)
    }

    /// Subscribe to post-rebuild mass-property change notifications.
    pub fn subscribe_mass_changes<F>(&self, listener: F) -> MassConnectionId
    where
        F: Fn(&MassChangeEvent) + Send + Sync + 'static,
    {
        self.mass_signal.subscribe(listener)
    }

    /// Remove a mass-change subscription.
    pub fn unsubscribe_mass_change(&self, id: MassConnectionId) -> bool {
        self.mass_signal.unsubscribe(id)
    }

    /// Read-only view of the world's mass-change signal.
    pub fn mass_signal(&self) -> &MassSignal {
        &self.mass_signal
    }

    /// Subscribe to the world's post-solve signal (`hknpWorld::m_postSolve`).
    ///
    /// The listener receives a copyable [`PostSolveEvent`] after integration;
    /// it cannot borrow the world mutably. Callbacks run at the end of every
    /// [`Self::step`] / [`Self::step_with_threads`] in subscription order.
    pub fn subscribe_post_solve<F>(&self, listener: F) -> PostSolveConnectionId
    where
        F: Fn(&PostSolveEvent) + Send + Sync + 'static,
    {
        self.post_solve_signal.subscribe(listener)
    }

    /// Remove a previously registered post-solve subscription.
    pub fn unsubscribe_post_solve(&self, id: PostSolveConnectionId) -> bool {
        self.post_solve_signal.unsubscribe(id)
    }

    /// Read-only view of the world's post-solve signal.
    pub fn post_solve_signal(&self) -> &PostSolveSignal {
        &self.post_solve_signal
    }

    /// Mark or unmark a body as a trigger volume.
    ///
    /// Trigger pairs are detected from cached body AABBs each step (deterministic
    /// broad-phase order), so static and sleeping bodies participate as well.
    /// Returns `false` when the id is not a live body.
    pub fn set_body_trigger(&mut self, body_id: BodyId, enabled: bool) -> bool {
        if self.bodies.get(body_id).is_none() {
            return false;
        }
        if enabled {
            self.trigger_bodies.insert(body_id);
        } else {
            self.trigger_bodies.remove(&body_id);
            self.active_trigger_pairs
                .retain(|&(a, b, _, _)| a != body_id && b != body_id);
        }
        true
    }

    /// Whether a body is currently marked as a trigger.
    pub fn is_body_trigger(&self, body_id: BodyId) -> bool {
        self.trigger_bodies.contains(&body_id)
    }

    /// Iterate live trigger bodies in ascending id order.
    pub fn trigger_bodies(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.trigger_bodies.iter().copied()
    }

    /// Queue an explicit trigger enter/exit transition for the current step.
    ///
    /// Automatic AABB diffs already feed the same per-step merger; this API
    /// lets a narrower collision source (or a test) merge multiple transitions
    /// for one pair. An enter followed by an exit in the same step produces
    /// `TriggerStatus::Updated` when the world cinfo enables it.
    pub fn record_trigger_transition(&mut self, a: BodyId, b: BodyId, entered: bool) -> bool {
        if self.bodies.get(a).is_none() || self.bodies.get(b).is_none() {
            return false;
        }
        let (body_a, body_b) = if a < b { (a, b) } else { (b, a) };
        if entered {
            self.trigger_merge
                .enter(body_a, body_b, ShapeKey::INVALID, ShapeKey::INVALID);
        } else {
            self.trigger_merge
                .exit(body_a, body_b, ShapeKey::INVALID, ShapeKey::INVALID);
        }
        true
    }

    /// Record a trigger transition with explicit leaf shape keys.
    ///
    /// Body ids are canonicalized together with their keys, so callers may pass
    /// either ordering. Returns `false` if either body is not live.
    pub fn record_trigger_transition_with_keys(
        &mut self,
        a: BodyId,
        b: BodyId,
        key_a: ShapeKey,
        key_b: ShapeKey,
        entered: bool,
    ) -> bool {
        if self.bodies.get(a).is_none() || self.bodies.get(b).is_none() {
            return false;
        }
        let (body_a, body_b, first_key, second_key) = if a < b {
            (a, b, key_a, key_b)
        } else {
            (b, a, key_b, key_a)
        };
        if entered {
            self.trigger_merge
                .enter(body_a, body_b, first_key, second_key);
        } else {
            self.trigger_merge
                .exit(body_a, body_b, first_key, second_key);
        }
        true
    }

    /// Subscribe to world-level trigger events.
    ///
    /// Events are emitted once per step after contact generation and before the
    /// solver runs, in canonical `(body_a, body_b, shape key)` order.
    pub fn subscribe_trigger_events<F>(&self, listener: F) -> TriggerConnectionId
    where
        F: Fn(&TriggerEvent) + Send + Sync + 'static,
    {
        self.trigger_signal.subscribe(listener)
    }

    /// Remove a previously registered trigger subscription.
    pub fn unsubscribe_trigger_event(&self, id: TriggerConnectionId) -> bool {
        self.trigger_signal.unsubscribe(id)
    }

    /// Read-only view of the world's trigger signal.
    pub fn trigger_signal(&self) -> &TriggerSignal {
        &self.trigger_signal
    }

    /// Subscribe to pre-solve contact snapshots for a body pair.
    ///
    /// These callbacks observe the contacts *after* [`crate::ContactModifier`]
    /// hooks ran and before islands and solving; they cannot mutate the world.
    pub fn subscribe_post_collide<F>(&self, listener: F) -> PostCollideConnectionId
    where
        F: Fn(&crate::events::PostCollideEvent) + Send + Sync + 'static,
    {
        self.post_collide_signal.subscribe(listener)
    }

    /// Remove a previously registered post-collide subscription.
    pub fn unsubscribe_post_collide(&self, id: PostCollideConnectionId) -> bool {
        self.post_collide_signal.unsubscribe(id)
    }

    /// Read-only view of the world's post-collide signal.
    pub fn post_collide_signal(&self) -> &PostCollideSignal {
        &self.post_collide_signal
    }

    /// Register a contact modifier used by both single- and multi-threaded steps.
    pub fn register_contact_modifier(
        &self,
        modifier: Arc<dyn ContactModifier>,
    ) -> ContactModifierId {
        self.contact_modifiers.register(modifier)
    }

    /// Remove a registered contact modifier.
    pub fn unregister_contact_modifier(&self, id: ContactModifierId) -> bool {
        self.contact_modifiers.unregister(id)
    }

    /// Remove all registered contact modifiers.
    pub fn clear_contact_modifiers(&self) {
        self.contact_modifiers.clear();
    }

    /// Number of registered contact modifiers.
    pub fn contact_modifier_count(&self) -> usize {
        self.contact_modifiers.len()
    }

    /// Register a pre-solve Jacobian-like row hook.
    ///
    /// Hooks run after all [`ContactModifier`] callbacks and before islands are
    /// built, in registration order. They observe the row's cached impulse and
    /// can edit the normal, tangent basis, penetration, friction or impulse.
    pub fn register_solver_data_hook(&self, hook: Arc<dyn SolverDataHook>) -> SolverDataHookId {
        self.solver_data_hooks.register(hook)
    }

    /// Remove a registered solver-data hook.
    pub fn unregister_solver_data_hook(&self, id: SolverDataHookId) -> bool {
        self.solver_data_hooks.unregister(id)
    }

    /// Remove all registered solver-data hooks.
    pub fn clear_solver_data_hooks(&self) {
        self.solver_data_hooks.clear();
    }

    /// Number of registered solver-data hooks.
    pub fn solver_data_hook_count(&self) -> usize {
        self.solver_data_hooks.len()
    }

    /// Destroy a body and its motion (`destroyBodies`).
    pub fn destroy_body(&mut self, body_id: BodyId) -> bool {
        let Some(body) = self.bodies.get(body_id) else {
            return false;
        };
        let motion_id = body.motion_id;
        if self.group_of_body.contains_key(&body_id) {
            // Shared motion: remaining members keep the slot; the group
            // collapses when only one member is left.
            self.detach_motion_membership(body_id);
        } else if motion_id != MotionId::PRESET_STATIC {
            self.motions.destroy(motion_id);
        }
        let destroyed = self.bodies.destroy(body_id);
        if destroyed {
            self.remove_joints_of(body_id);
            self.sleep_states.remove(&body_id);
            self.trigger_bodies.remove(&body_id);
            self.active_trigger_pairs
                .retain(|&(a, b, _, _)| a != body_id && b != body_id);
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

    /// Add a limited-hinge joint (`hknpLimitedHingeConstraintData` subset).
    ///
    /// Shares the hinge geometry and angular limits; the joint is solved with
    /// the hinge rows and has no motor.
    pub fn create_limited_hinge_constraint(&mut self, info: &LimitedHingeCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::LimitedHinge(*info))
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

    /// Add a 6DOF joint (`hknp6DofConstraintData` limits subset).
    pub fn create_6dof_constraint(&mut self, info: &SixDofCinfo) -> ConstraintId {
        self.push_joint(WorldJoint::SixDof(*info))
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

    /// Set or clear the angular limits of a limited-hinge joint.
    pub fn set_limited_hinge_limits(
        &mut self,
        id: ConstraintId,
        limits: Option<HingeLimitsCinfo>,
    ) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::LimitedHinge(info) => {
                    info.limits = limits;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    /// Set the twist/swing/linear limits of a 6DOF joint.
    /// Enable, update or disable a ragdoll's plane-angle limits (deviation
    /// from 90°, in radians).
    pub fn set_ragdoll_plane_limits(
        &mut self,
        id: ConstraintId,
        plane_limits: Option<(f32, f32)>,
    ) -> bool {
        if let Some((min, max)) = plane_limits {
            if !min.is_finite() || !max.is_finite() || min > max {
                return false;
            }
        }
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::Ragdoll(info) => {
                    info.plane_limits = plane_limits;
                    true
                }
                _ => false,
            },
            None => false,
        }
    }

    pub fn set_6dof_limits(&mut self, id: ConstraintId, limits: SixDofLimitsCinfo) -> bool {
        match self.joint_slot_mut(id) {
            Some(slot) => match &mut slot.joint {
                WorldJoint::SixDof(info) => {
                    info.limits = SixDofLimitsCinfo {
                        twist_min: limits.twist_min,
                        twist_max: limits.twist_max.max(limits.twist_min),
                        swing0_max: limits.swing0_max.max(0.0),
                        swing1_max: limits.swing1_max.max(0.0),
                        linear_limit: limits.linear_limit.max(0.0),
                        linear_axes: limits.linear_axes,
                        linear_min: limits.linear_min,
                        linear_max: [
                            limits.linear_max[0].max(limits.linear_min[0]),
                            limits.linear_max[1].max(limits.linear_min[1]),
                            limits.linear_max[2].max(limits.linear_min[2]),
                        ],
                    };
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
                WorldJoint::Hinge(info) | WorldJoint::LimitedHinge(info) => {
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
                    // A limited hinge carries no motor even when the shared
                    // construction-info type has one configured.
                    let motor = if matches!(slot.joint, WorldJoint::LimitedHinge(_)) {
                        None
                    } else {
                        info.motor.map(|motor| match motor {
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
                            crate::constraint::HingeMotorCinfo::Spring(motor) => {
                                HingeMotor::Spring {
                                    target_angle: motor.target_angle,
                                    stiffness: motor.stiffness,
                                    damping: motor.damping,
                                    max_impulse: motor.max_impulse,
                                }
                            }
                        })
                    };
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
                    let plane_axis_b = unit_vector(transform_b.rotation.rotate(info.plane_axis_b));
                    let (plane_enabled, plane_angle, plane_min, plane_max) = match info.plane_limits
                    {
                        Some((sdk_min, sdk_max)) => (
                            true,
                            twist_a.dot3(plane_axis_b).clamp(-1.0, 1.0).acos()
                                - core::f32::consts::FRAC_PI_2,
                            -sdk_max,
                            -sdk_min,
                        ),
                        None => (false, 0.0, f32::NEG_INFINITY, f32::INFINITY),
                    };
                    result.push((
                        slot_index,
                        JointConstraint::Ragdoll(RagdollJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a: transform_a.transform_point(info.pivot_a),
                            pivot_b: transform_b.transform_point(info.pivot_b),
                            twist_axis_a: twist_a,
                            twist_axis_b: twist_b,
                            plane_axis_b,
                            swing_angle,
                            cone_angle: info.cone_angle,
                            twist_angle,
                            twist_min: info.twist_min,
                            twist_max: info.twist_max,
                            plane_enabled,
                            plane_angle,
                            plane_min,
                            plane_max,
                        }),
                    ));
                }
                WorldJoint::SixDof(info) => {
                    let twist_a = unit_vector(transform_a.rotation.rotate(info.twist_axis_a));
                    let twist_b = unit_vector(transform_b.rotation.rotate(info.twist_axis_b));
                    let pivot_a = transform_a.transform_point(info.pivot_a);
                    let pivot_b = transform_b.transform_point(info.pivot_b);
                    // Constraint-frame columns are `[twist, plane, twist×plane]`,
                    // matching `hknp6DofConstraintData::getConstraintFrameA/B`.
                    let frame_axes = six_dof_constraint_frame_axes(
                        twist_a,
                        transform_a.rotation.rotate(info.plane_axis_a),
                    );
                    let u = frame_axes[1];
                    let v = frame_axes[2];
                    let error = twist_a.cross(twist_b);
                    let plane_a = frame_axes[1];
                    let plane_b = perpendicular_reference(
                        twist_a,
                        transform_b.rotation.rotate(info.plane_axis_b),
                    );
                    let twist_angle = twist_a
                        .dot3(plane_a.cross(plane_b))
                        .atan2(plane_a.dot3(plane_b));
                    let axis_vectors = frame_axes;
                    let mut linear_axis_enabled = [false; 3];
                    let mut linear_axis_translation = [0.0; 3];
                    let mut linear_axis_min = [f32::NEG_INFINITY; 3];
                    let mut linear_axis_max = [f32::INFINITY; 3];
                    for index in 0..3 {
                        match info.limits.linear_axes[index] {
                            SixDofAxisMode::Unlimited => {}
                            SixDofAxisMode::Limited => {
                                linear_axis_enabled[index] = true;
                                linear_axis_translation[index] =
                                    (pivot_a - pivot_b).dot3(axis_vectors[index]);
                                linear_axis_min[index] = info.limits.linear_min[index];
                                linear_axis_max[index] = info.limits.linear_max[index];
                            }
                            SixDofAxisMode::Fixed => {
                                linear_axis_enabled[index] = true;
                                linear_axis_translation[index] =
                                    (pivot_a - pivot_b).dot3(axis_vectors[index]);
                                linear_axis_min[index] = 0.0;
                                linear_axis_max[index] = 0.0;
                            }
                        }
                    }
                    result.push((
                        slot_index,
                        JointConstraint::SixDof(SixDofJoint {
                            body_a: index_a,
                            body_b: index_b,
                            pivot_a,
                            pivot_b,
                            twist_axis_a: twist_a,
                            twist_axis_b: twist_b,
                            twist_angle,
                            twist_min: info.limits.twist_min,
                            twist_max: info.limits.twist_max,
                            swing0_angle: error.dot3(u),
                            swing0_max: info.limits.swing0_max,
                            swing1_angle: error.dot3(v),
                            swing1_max: info.limits.swing1_max,
                            distance: (pivot_a - pivot_b).length3(),
                            max_distance: info.limits.linear_limit,
                            linear_axis_enabled,
                            linear_axis_translation,
                            linear_axis_min,
                            linear_axis_max,
                            linear_axes: frame_axes,
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
        if let Some(index) = self.group_of_body.get(&body_id).copied() {
            let members = self.motion_groups[index].members.clone();
            if members.len() >= 2 {
                if let Some(motion) = self.motions.get_mut(motion_id) {
                    motion.set_transform(transform);
                }
                for member in &members {
                    if let Some(entry) = self.bodies.get_mut(*member) {
                        entry.set_transform(transform);
                    }
                }
                for member in members {
                    self.wake_if_dynamic(member);
                }
                return true;
            }
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
        if let Some(index) = self.group_of_body.get(&body_id).copied() {
            let members = self.motion_groups[index].members.clone();
            if members.len() >= 2 {
                // One motion is shared, so the type change applies to the
                // whole group rather than mutating one body's flags.
                if let Some(motion) = self.motions.get_mut(motion_id) {
                    if motion.motion_type == motion_type {
                        return true;
                    }
                    motion.motion_type = motion_type;
                }
                for member in members {
                    if let Some(body) = self.bodies.get_mut(member) {
                        body.flags
                            .set(CollisionFlags::IS_DYNAMIC, motion_type.is_dynamic());
                        body.flags
                            .set(CollisionFlags::IS_KEYFRAMED, !motion_type.is_dynamic());
                        body.flags
                            .set(CollisionFlags::IS_ACTIVE, motion_type.is_dynamic());
                    }
                    self.sleep_states.remove(&member);
                    if motion_type.is_dynamic() {
                        self.wake_if_dynamic(member);
                    }
                }
                return true;
            }
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

    /// Flatten manifolds into solver rows, applying contact modifiers first.
    ///
    /// The returned rows, warm-start keys and impulses stay index-aligned: a
    /// dropped point is excluded from all three. One post-collide event is
    /// produced per manifold (including those whose points were all dropped)
    /// so listeners can observe the filtering.
    fn build_contact_rows(
        &self,
        contacts: &[ContactManifold],
        index_map: &HashMap<BodyId, usize>,
        modifiers: &[Arc<dyn ContactModifier>],
        hooks: &[Arc<dyn SolverDataHook>],
        step_number: u64,
        delta_time: f32,
    ) -> (
        Vec<ContactConstraint>,
        Vec<ContactKey>,
        Vec<ContactImpulse>,
        Vec<PostCollideEvent>,
    ) {
        let point_count: usize = contacts
            .iter()
            .map(|contact| contact.manifold.num_points())
            .sum();
        let mut constraints = Vec::with_capacity(point_count);
        let mut contact_keys = Vec::with_capacity(point_count);
        let mut contact_impulses = Vec::with_capacity(point_count);
        let mut events = Vec::with_capacity(contacts.len());

        for contact in contacts {
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

            let mut points = Vec::with_capacity(contact.manifold.points.len());
            let mut dropped = 0usize;
            for point in &contact.manifold.points {
                let initial_normal = contact.manifold.normal;
                let (initial_tangent_1, initial_tangent_2) = contact_tangents(initial_normal);
                let mut editable = EditableContact {
                    body_a: contact.body_a,
                    body_b: contact.body_b,
                    normal: initial_normal,
                    position: point.position,
                    penetration: (-point.distance).max(0.0),
                    friction: 0.5,
                    tangent_1: initial_tangent_1,
                    tangent_2: initial_tangent_2,
                    normal_impulse: 0.0,
                    tangent_impulse_1: 0.0,
                    tangent_impulse_2: 0.0,
                };
                let mut keep = true;
                for modifier in modifiers {
                    if modifier.modify(&mut editable) == ContactEditAction::Drop {
                        keep = false;
                        break;
                    }
                }
                if !keep {
                    dropped += 1;
                    continue;
                }

                let lookup_key = ContactKey {
                    body_a: contact.body_a,
                    body_b: contact.body_b,
                    local_point_b: transform_b.inverse_transform_point(editable.position),
                    normal: editable.normal,
                };
                let cached = if self.cinfo.enable_warm_starting {
                    self.contact_cache.match_impulse(lookup_key)
                } else {
                    ContactImpulse::default()
                };
                editable.normal_impulse = cached.normal;
                editable.tangent_impulse_1 = cached.tangent_1;
                editable.tangent_impulse_2 = cached.tangent_2;

                if !hooks.is_empty() {
                    let mut view = SolverDataView {
                        body_a: contact.body_a,
                        body_b: contact.body_b,
                        normal: &mut editable.normal,
                        tangent_1: &mut editable.tangent_1,
                        tangent_2: &mut editable.tangent_2,
                        penetration: &mut editable.penetration,
                        friction: &mut editable.friction,
                        normal_impulse: &mut editable.normal_impulse,
                        tangent_impulse_1: &mut editable.tangent_impulse_1,
                        tangent_impulse_2: &mut editable.tangent_impulse_2,
                    };
                    for hook in hooks {
                        hook.modify(&mut view);
                    }
                }

                if editable.normal.length3() > 1e-10 {
                    editable.normal = editable.normal.normalized3();
                }
                let (tangent_1, tangent_2) = contact_tangents(editable.normal);
                editable.tangent_1 = tangent_1;
                editable.tangent_2 = tangent_2;
                editable.normal_impulse = editable.normal_impulse.max(0.0);
                let limit = editable.friction.max(0.0) * editable.normal_impulse;
                editable.tangent_impulse_1 = editable.tangent_impulse_1.clamp(-limit, limit);
                editable.tangent_impulse_2 = editable.tangent_impulse_2.clamp(-limit, limit);

                let penetration = editable.penetration.max(0.0);
                let constraint_point = editable.position - editable.normal * (penetration * 0.5);
                constraints.push(ContactConstraint::new(
                    body_a,
                    body_b,
                    editable.normal,
                    constraint_point,
                    penetration,
                    editable.friction,
                ));
                let key = ContactKey {
                    body_a: contact.body_a,
                    body_b: contact.body_b,
                    local_point_b: transform_b.inverse_transform_point(editable.position),
                    normal: editable.normal,
                };
                contact_impulses.push(ContactImpulse {
                    normal: editable.normal_impulse,
                    tangent_1: editable.tangent_impulse_1,
                    tangent_2: editable.tangent_impulse_2,
                });
                contact_keys.push(key);
                points.push(PostCollideContactPoint {
                    position: editable.position,
                    normal: editable.normal,
                    penetration,
                    friction: editable.friction,
                });
            }

            events.push(PostCollideEvent {
                step_count: step_number,
                delta_time,
                body_a: contact.body_a,
                body_b: contact.body_b,
                points,
                dropped_point_count: dropped,
            });
        }

        (constraints, contact_keys, contact_impulses, events)
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
        // Trigger overlap transitions are collected from broad-phase AABBs and
        // emitted before contact rows are built; subscribers cannot mutate the
        // world, so ordering relative to the solver is stable.
        self.update_trigger_events();

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
            let (inverse_mass, inverse_inertia_world) = if body.is_dynamic() {
                let matrix = motion
                    .map(|motion| {
                        if motion.inverse_inertia_world != Matrix3::ZERO
                            && (!inverse_inertia_is_diagonal(motion.inverse_inertia_world, 1e-8)
                                || motion.inverse_inertia == Vector4::ZERO)
                        {
                            motion.inverse_inertia_world
                        } else {
                            inverse_inertia_diagonal_matrix(motion.inverse_inertia)
                        }
                    })
                    .unwrap_or(Matrix3::ZERO);
                (motion.map(Motion::inverse_mass).unwrap_or(0.0), matrix)
            } else {
                (0.0, Matrix3::ZERO)
            };
            let linear_velocity = motion
                .map(|motion| motion.linear_velocity)
                .unwrap_or(Vector4::ZERO);
            let angular_velocity = motion
                .map(|motion| motion.angular_velocity)
                .unwrap_or(Vector4::ZERO);
            solver_bodies.push(SolverBody::new_with_inertia_matrix(
                inverse_mass,
                inverse_inertia_world,
                body.transform.translation,
                linear_velocity,
                angular_velocity,
            ));
        }

        // Contact rows are shared by both step paths; modifiers run once per
        // step over a snapshot so parallel workers cannot see partial edits.
        let modifier_snapshot = self.contact_modifiers.snapshot();
        let hook_snapshot = self.solver_data_hooks.snapshot();
        let (constraints, contact_keys, mut contact_impulses, post_collide_events) = self
            .build_contact_rows(
                &contacts,
                &index_map,
                &modifier_snapshot,
                &hook_snapshot,
                self.step_count + 1,
                delta_time,
            );
        self.last_manifold_count = post_collide_events.len();
        self.last_contact_count = constraints.len();
        for event in &post_collide_events {
            self.post_collide_signal.emit(event);
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
        // A shared motion must be integrated once even when several bodies
        // reference it; all members then get the same refreshed transform.
        let mut integrated: Vec<MotionId> = Vec::new();
        for (_, motion_id) in &dynamic_bodies {
            if integrated.contains(motion_id) {
                continue;
            }
            integrated.push(*motion_id);
            let Some(motion) = self.motions.get_mut(*motion_id) else {
                continue;
            };
            motion.integrate(delta_time);
            let transform = motion.transform();
            for body in self.bodies.iter_mut() {
                if body.motion_id == *motion_id {
                    body.set_transform(transform);
                }
            }
        }

        self.step_count += 1;
        self.last_delta_time = delta_time;

        // `hknpWorldEx::postSolve`: notify after the step's state is stable.
        let event = PostSolveEvent {
            step_count: self.step_count,
            delta_time,
            manifold_count: self.last_manifold_count,
            contact_count: self.last_contact_count,
            island_count: self.last_islands.len(),
        };
        self.post_solve_signal.emit(&event);
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

    /// Leaf keys for one narrow-phase trigger pair.
    ///
    /// Witness points are resolved through the shared `phys-collide` helper so
    /// world-level events and query-side diagnostics use the same leaf key
    /// encoding. Convex hulls, meshes, height fields and compounds can produce
    /// leaf keys; shapes without ported subdivision fall back to `INVALID`.
    fn trigger_pair_keys(&self, a: BodyId, b: BodyId) -> (ShapeKey, ShapeKey) {
        let (Some(body_a), Some(body_b)) = (self.bodies.get(a), self.bodies.get(b)) else {
            return (ShapeKey::INVALID, ShapeKey::INVALID);
        };
        if let Some(result) = closest_points(
            body_a.shape.as_ref(),
            &body_a.transform,
            body_b.shape.as_ref(),
            &body_b.transform,
        ) {
            return (
                leaf_key_at_local_point(
                    body_a.shape.as_ref(),
                    body_a.transform.inverse_transform_point(result.point_a),
                ),
                leaf_key_at_local_point(
                    body_b.shape.as_ref(),
                    body_b.transform.inverse_transform_point(result.point_b),
                ),
            );
        }

        // Non-convex shapes have no GJK witness. Their centres still give a
        // deterministic local point from which the shared resolver can choose
        // the nearest leaf for diagnostic trigger events.
        let centre_a = body_a.transform.transform_point(Vector4::ZERO);
        let centre_b = body_b.transform.transform_point(Vector4::ZERO);
        (
            leaf_key_at_local_point(
                body_a.shape.as_ref(),
                body_a.transform.inverse_transform_point(centre_b),
            ),
            leaf_key_at_local_point(
                body_b.shape.as_ref(),
                body_b.transform.inverse_transform_point(centre_a),
            ),
        )
    }

    /// Refresh trigger overlaps and dispatch merged events for this step.
    ///
    /// The detector uses broad-phase AABB pairs (canonical `(low, high)` order)
    /// so static, sleeping and non-convex trigger bodies all participate.
    /// `DONT_COLLIDE` and the world collision filter are honoured. Narrow-phase
    /// witnesses are resolved via [`leaf_key_at_local_point`], so hull faces,
    /// mesh triangles, height-field cells and compound children can produce
    /// leaf keys; other shapes fall back to `ShapeKey::INVALID`. Non-convex
    /// shapes have no GJK witness, so their body centre is projected into the
    /// other shape's local space as a deterministic diagnostic fallback.
    fn update_trigger_events(&mut self) {
        if self.trigger_bodies.is_empty() {
            self.active_trigger_pairs.clear();
        } else {
            let entries: Vec<BodyAabb> = self
                .bodies
                .iter()
                .map(|body| BodyAabb::new(body.id, body.aabb))
                .collect();
            let mut current = BTreeSet::new();
            for pair in self.broad_phase.find_pairs(&entries) {
                let (low, high) = if pair.a < pair.b {
                    (pair.a, pair.b)
                } else {
                    (pair.b, pair.a)
                };
                if !self.trigger_bodies.contains(&low) && !self.trigger_bodies.contains(&high) {
                    continue;
                }
                let (Some(body_a), Some(body_b)) =
                    (self.bodies.get(pair.a), self.bodies.get(pair.b))
                else {
                    continue;
                };
                if body_a.flags.contains(CollisionFlags::DONT_COLLIDE)
                    || body_b.flags.contains(CollisionFlags::DONT_COLLIDE)
                {
                    continue;
                }
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
                let (mut key_low, mut key_high) = self.trigger_pair_keys(pair.a, pair.b);
                if pair.a != low {
                    core::mem::swap(&mut key_low, &mut key_high);
                }
                current.insert((low, high, key_low, key_high));
            }

            let enters: Vec<TriggerPairKey> = current
                .iter()
                .copied()
                .filter(|pair| !self.active_trigger_pairs.contains(pair))
                .collect();
            let exits: Vec<TriggerPairKey> = self
                .active_trigger_pairs
                .iter()
                .copied()
                .filter(|pair| !current.contains(pair))
                .collect();
            for (body_a, body_b, key_a, key_b) in enters {
                self.trigger_merge.enter(body_a, body_b, key_a, key_b);
            }
            for (body_a, body_b, key_a, key_b) in exits {
                self.trigger_merge.exit(body_a, body_b, key_a, key_b);
            }
            self.active_trigger_pairs = current;
        }

        let events = self
            .trigger_merge
            .flush(self.cinfo.raise_trigger_updated_events);
        for event in events {
            self.trigger_signal.emit(&event);
        }
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

    #[test]
    fn trigger_bodies_emit_enter_and_exit_from_aabb_overlaps() {
        use crate::events::TriggerStatus;

        let mut world = World::new(WorldCinfo::new().with_gravity(Vector4::ZERO));
        let trigger = world.create_body(
            &BodyCinfo::new(box_shape(Vector4::splat(1.0))).with_position(Vector4::ZERO),
        );
        let other = world.create_body(
            &BodyCinfo::new(box_shape(Vector4::splat(0.5)))
                .with_position(Vector4::from_xyz(0.5, 0.0, 0.0)),
        );
        assert!(world.set_body_trigger(trigger, true));
        assert!(world.is_body_trigger(trigger));
        assert!(!world.is_body_trigger(other));

        let log = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        let connection = world.subscribe_trigger_events(move |event| {
            sink.lock().unwrap().push(event.status);
        });

        world.step(0.0);
        assert_eq!(*log.lock().unwrap(), vec![TriggerStatus::Entered]);
        world.set_body_position(other, Vector4::from_xyz(8.0, 0.0, 0.0));
        world.step(0.0);
        assert_eq!(
            *log.lock().unwrap(),
            vec![TriggerStatus::Entered, TriggerStatus::Exited]
        );
        assert!(world.unsubscribe_trigger_event(connection));
        world.set_body_position(other, Vector4::from_xyz(0.5, 0.0, 0.0));
        world.step(0.0);
        assert_eq!(log.lock().unwrap().len(), 2);
    }

    struct DropContacts;
    impl ContactModifier for DropContacts {
        fn modify(&self, _: &mut EditableContact) -> ContactEditAction {
            ContactEditAction::Drop
        }
    }

    struct SetFriction(f32);
    impl ContactModifier for SetFriction {
        fn modify(&self, contact: &mut EditableContact) -> ContactEditAction {
            contact.friction = self.0;
            ContactEditAction::Keep
        }
    }

    #[test]
    fn contact_modifier_drops_points_and_post_collide_reports_them() {
        let mut world = World::new(WorldCinfo::new());
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(10.0, 0.5, 10.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 0.5, 0.0)));

        let seen = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&seen);
        world.subscribe_post_collide(move |event| {
            sink.lock()
                .unwrap()
                .push((event.points.len(), event.dropped_point_count));
        });
        let id = world.register_contact_modifier(Arc::new(DropContacts));
        for _ in 0..20 {
            world.step(1.0 / 60.0);
        }
        assert_eq!(world.stats().last_contact_count, 0);
        assert!(
            seen.lock()
                .unwrap()
                .iter()
                .any(|&(retained, dropped)| retained == 0 && dropped > 0)
        );
        assert!(world.unregister_contact_modifier(id));
        assert_eq!(world.contact_modifier_count(), 0);
    }

    #[test]
    fn contact_modifier_edits_friction_seen_by_post_collide() {
        let mut world = World::new(WorldCinfo::new());
        world.create_body(
            &BodyCinfo::new(box_shape(Vector4::from_xyz(10.0, 0.5, 10.0)))
                .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
        );
        world.create_body(&dynamic_cinfo(0.5).with_position(Vector4::from_xyz(0.0, 0.5, 0.0)));

        let seen = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&seen);
        world.subscribe_post_collide(move |event| {
            for point in &event.points {
                sink.lock().unwrap().push(point.friction);
            }
        });
        let id = world.register_contact_modifier(Arc::new(SetFriction(0.9)));
        for _ in 0..8 {
            world.step(1.0 / 60.0);
        }
        assert!(seen.lock().unwrap().contains(&0.9));
        world.clear_contact_modifiers();
        assert!(!world.contact_modifiers.unregister(id));
    }

    #[test]
    fn explicit_trigger_transitions_merge_into_updated_only_when_enabled() {
        use crate::events::TriggerStatus;

        let mut world = World::new(WorldCinfo::new().with_raise_trigger_updated_events(true));
        let a = world.create_body(&BodyCinfo::new(box_shape(Vector4::splat(0.5))));
        let b = world.create_body(
            &BodyCinfo::new(box_shape(Vector4::splat(0.5)))
                .with_position(Vector4::from_xyz(8.0, 0.0, 0.0)),
        );
        let log = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        world.subscribe_trigger_events(move |event| {
            sink.lock().unwrap().push(event.status);
        });

        assert!(world.record_trigger_transition(a, b, true));
        assert!(world.record_trigger_transition(b, a, false));
        world.step(0.0);
        assert_eq!(*log.lock().unwrap(), vec![TriggerStatus::Updated]);
        assert!(!world.record_trigger_transition(BodyId::INVALID, b, true));
    }
}
