//! Rigid-body character route (`hknpCharacterRigidBody` subset).
//!
//! Unlike the ghost [`super::proxy::CharacterProxy`], this route owns a real
//! dynamic body and steers it with velocity/position control. Support checks
//! use the world's convex closest-point queries, and listeners receive the
//! SDK's three logical entry points: contact filtering, post-collide and
//! post-solve. When managed by [`super::manager::CharacterProxyManager`], the
//! listener is also wired to the world's deterministic contact modifier and
//! post-collide signal, so `filter_contact_point` can drop real solver contacts.
//! Pre-solve row editing uses the shared listener slot; VDB streams are replaced
//! by the local [`super::monitor`] callback.

use std::sync::{Arc, Mutex};

use base_math::{Aabb, Quaternion, Transform, Vector4};
use phys_body::BodyCinfo;
use phys_collide::closest_points;
use phys_shape::Shape;
use phys_types::{BodyId, MotionType};
use phys_world::{QueryFilter, World};

use super::monitor::{CharacterMonitorEvent, CharacterMonitorPhase};
use super::rigid_body_listener::{
    CharacterRigidBodyListener, CharacterRigidBodyState, ContactPointInfo,
};
use super::surface::{SupportedState, SurfaceInfo};

/// Shared listener slot used by the world contact bridge.
pub(crate) type RigidBodyListenerCell =
    Arc<Mutex<Option<Arc<Mutex<Box<dyn CharacterRigidBodyListener>>>>>>;

/// Construction info for [`CharacterRigidBody`].
///
/// Defaults mirror the SDK constructor (`hknpCharacterRigidBody.cpp:72-93`),
/// including the effective mass of 75 (the header's `+default(100)` is not
/// what the constructor assigns).
#[derive(Clone)]
pub struct CharacterRigidBodyCinfo {
    /// Character shape.
    pub shape: Arc<dyn Shape>,
    /// Initial world position.
    pub position: Vector4,
    /// Initial orientation.
    pub orientation: Quaternion,
    /// Body mass (must be finite and positive).
    pub mass: f32,
    /// Maximum force used for velocity control.
    pub max_force: f32,
    /// Maximum walkable slope in radians.
    pub max_slope: f32,
    /// Support surface distance.
    pub support_distance: f32,
    /// Up direction.
    pub up: Vector4,
    /// Query-side collision filter info.
    pub collision_filter_info: u32,
    /// Optional debug name.
    pub name: &'static str,
}

impl CharacterRigidBodyCinfo {
    /// Default construction info for `shape`.
    pub fn new(shape: Arc<dyn Shape>) -> Self {
        Self {
            shape,
            position: Vector4::ZERO,
            orientation: Quaternion::IDENTITY,
            mass: 75.0,
            max_force: 1000.0,
            max_slope: core::f32::consts::PI / 3.0,
            support_distance: 0.1,
            up: Vector4::UNIT_Y,
            collision_filter_info: 0,
            name: "hknpCharacterRigidBody",
        }
    }

    /// Builder for the initial position.
    pub fn with_position(mut self, position: Vector4) -> Self {
        self.position = position;
        self
    }

    /// Builder for the body mass.
    pub fn with_mass(mut self, mass: f32) -> Self {
        self.mass = mass;
        self
    }

    /// Builder for the maximum control force.
    pub fn with_max_force(mut self, max_force: f32) -> Self {
        self.max_force = max_force;
        self
    }

    /// Builder for the up direction.
    pub fn with_up(mut self, up: Vector4) -> Self {
        self.up = up;
        self
    }
}

/// Rigid-body character controller.
pub struct CharacterRigidBody {
    body: BodyId,
    up: Vector4,
    max_force: f32,
    max_slope_cosine: f32,
    support_distance: f32,
    supported: bool,
    cached_position: Vector4,
    cached_velocity: Vector4,
    listener_slot: RigidBodyListenerCell,
    monitor_step_count: u64,
}

impl CharacterRigidBody {
    /// Create the character body in `world`.
    ///
    /// # Panics
    /// Panics for non-finite/non-positive mass or slope outside `(0, π/2]`.
    pub fn new(world: &mut World, info: &CharacterRigidBodyCinfo) -> Self {
        assert!(
            info.mass.is_finite() && info.mass > 0.0,
            "invalid character mass"
        );
        assert!(
            info.max_slope > 0.0 && info.max_slope <= 0.5 * core::f32::consts::PI,
            "maxSlope must be between 0 and pi/2 radians"
        );
        let body_info = BodyCinfo::new(info.shape.clone())
            .with_motion_type(MotionType::Dynamic)
            .with_mass(info.mass)
            .with_transform(Transform::from_quaternion(info.orientation, info.position))
            .with_collision_filter_info(info.collision_filter_info)
            .with_name(info.name);
        let body = world.create_body(&body_info);
        Self {
            body,
            up: info.up.normalized3(),
            max_force: info.max_force.max(0.0),
            max_slope_cosine: info.max_slope.cos(),
            support_distance: info.support_distance.max(0.0),
            supported: false,
            cached_position: info.position,
            cached_velocity: Vector4::ZERO,
            listener_slot: Arc::new(Mutex::new(None)),
            monitor_step_count: 0,
        }
    }

    /// Attach a listener (replaces any previous one).
    ///
    /// The listener is kept behind a shared mutex so a manager can wire the
    /// same callback into the world's contact modifier and post-collide signal.
    pub fn set_listener(&mut self, listener: Box<dyn CharacterRigidBodyListener>) {
        *self
            .listener_slot
            .lock()
            .expect("character listener lock poisoned") = Some(Arc::new(Mutex::new(listener)));
    }

    /// Shared listener slot for the manager's world contact bridge.
    pub(crate) fn listener_slot(&self) -> RigidBodyListenerCell {
        Arc::clone(&self.listener_slot)
    }

    /// Character body id.
    pub fn body(&self) -> BodyId {
        self.body
    }

    /// World position (`getPosition`).
    pub fn position(&self, world: &World) -> Vector4 {
        world.body_position(self.body).unwrap_or(Vector4::ZERO)
    }

    /// Linear velocity (`getLinearVelocity`).
    pub fn linear_velocity(&self, world: &World) -> Vector4 {
        world
            .body_linear_velocity(self.body)
            .unwrap_or(Vector4::ZERO)
    }

    /// Set linear velocity (`setLinearVelocity`).
    pub fn set_linear_velocity(&self, world: &mut World, velocity: Vector4) -> bool {
        let _ = world.wake_body(self.body);
        world.set_body_linear_velocity(self.body, velocity)
    }

    /// Set world position (`setPosition`).
    pub fn set_position(&self, world: &mut World, position: Vector4) -> bool {
        world.set_body_transform(
            self.body,
            Transform::from_quaternion(Quaternion::IDENTITY, position),
        )
    }

    /// Classify the current support surface (`checkSupport` subset).
    pub fn check_support(
        &mut self,
        world: &World,
        direction: Vector4,
        _delta_time: f32,
    ) -> SurfaceInfo {
        let direction = direction.normalized3();
        let contacts = self.collect_contacts(world);
        let mut normal = Vector4::ZERO;
        let mut velocity = Vector4::ZERO;
        let mut count = 0usize;
        for point in &contacts {
            if point.normal.dot3(direction) > -0.08 {
                continue;
            }
            let cosine = point.normal.dot3(self.up);
            if cosine >= self.max_slope_cosine {
                normal += point.normal;
                count += 1;
                if let Some(body) = world.body(point.body) {
                    if let Some(motion) = world.motion(body.motion_id) {
                        velocity += motion.linear_velocity;
                    }
                }
            }
        }
        let state = if count == 0 {
            SurfaceInfo::UNSUPPORTED
        } else {
            normal.normalize3();
            SurfaceInfo {
                supported_state: SupportedState::Supported,
                surface_normal: normal,
                surface_velocity: velocity * (1.0 / count as f32),
                surface_angular_velocity: Vector4::ZERO,
            }
        };
        self.supported = state.supported_state == SupportedState::Supported;
        state
    }

    /// Run one character control step.
    ///
    /// Contacts are collected and filtered (post-collide), then the body's
    /// velocity is clamped by `max_force` when supported. `on_post_solve` is
    /// delivered by [`CharacterProxyManager`](super::manager::CharacterProxyManager)
    /// after the world step.
    pub fn step(&mut self, world: &mut World, delta_time: f32, surface: &SurfaceInfo) {
        self.monitor_step_count += 1;
        let begin_position = self.position(world);
        let begin_velocity = self.linear_velocity(world);
        let mut points = self.collect_contacts(world);
        self.fire_monitor(
            CharacterMonitorPhase::IntegrateBegin,
            begin_position,
            begin_velocity,
            points.len(),
        );
        self.with_listener(|listener| {
            points.retain(|point| {
                let mut point = *point;
                listener.filter_contact_point(&mut point)
            });
            listener.on_post_collide(&points);
        });

        let mut velocity = self.linear_velocity(world);
        if surface.supported_state == SupportedState::Supported && self.max_force > 0.0 {
            let mass = world
                .body(self.body)
                .and_then(|body| world.motion(body.motion_id))
                .map(|motion| motion.mass)
                .unwrap_or(0.0);
            if mass > 0.0 {
                let max_delta = self.max_force * delta_time / mass;
                let speed = velocity.length3();
                if speed > max_delta && speed > 1e-10 {
                    velocity *= max_delta / speed;
                }
            }
        }
        let _ = world.wake_body(self.body);
        let _ = world.set_body_linear_velocity(self.body, velocity);
        self.cached_position = self.position(world);
        self.cached_velocity = self.linear_velocity(world);
        self.fire_monitor(
            CharacterMonitorPhase::IntegrateEnd,
            self.cached_position,
            self.cached_velocity,
            points.len(),
        );
    }

    /// Dispatch the post-solve listener from the state cached by [`Self::step`].
    pub(crate) fn fire_post_solve_from_snapshot(&mut self) {
        let state = CharacterRigidBodyState {
            body: self.body,
            position: self.cached_position,
            velocity: self.cached_velocity,
            supported: self.supported,
        };
        self.with_listener(|listener| listener.on_post_solve(&state));
        self.fire_monitor(
            CharacterMonitorPhase::PostSolve,
            state.position,
            state.velocity,
            0,
        );
    }

    fn fire_monitor(
        &self,
        phase: CharacterMonitorPhase,
        position: Vector4,
        linear_velocity: Vector4,
        contact_count: usize,
    ) {
        let event = CharacterMonitorEvent {
            step_count: self.monitor_step_count,
            phase,
            body: self.body,
            position,
            linear_velocity,
            contact_count,
        };
        self.with_listener(|listener| listener.monitor(&event));
    }

    fn with_listener<R>(
        &self,
        visit: impl FnOnce(&mut dyn CharacterRigidBodyListener) -> R,
    ) -> Option<R> {
        let listener = self
            .listener_slot
            .lock()
            .expect("character listener lock poisoned")
            .as_ref()
            .cloned()?;
        let mut listener = listener.lock().unwrap_or_else(|error| error.into_inner());
        Some(visit(listener.as_mut()))
    }

    fn collect_contacts(&self, world: &World) -> Vec<ContactPointInfo> {
        let Some(body) = world.body(self.body) else {
            return Vec::new();
        };
        let query_aabb = body.aabb;
        let mut points = Vec::new();
        let filter = QueryFilter::default().with_ignore_body(self.body);
        for other in world.query_aabb(&query_aabb, &filter) {
            let Some(other_body) = world.body(other) else {
                continue;
            };
            let Some(result) = closest_points(
                body.shape.as_ref(),
                &body.transform,
                other_body.shape.as_ref(),
                &other_body.transform,
            ) else {
                continue;
            };
            if result.distance > self.support_distance {
                continue;
            }
            points.push(ContactPointInfo {
                body: other,
                position: result.point_b,
                normal: result.normal,
                distance: if result.intersects {
                    -result.distance.abs()
                } else {
                    result.distance
                },
            });
        }
        points.sort_by_key(|point| point.body.value());
        points
    }

    /// AABB helper retained for diagnostics/tests of support distances.
    pub fn support_aabb(&self, world: &World) -> Option<Aabb> {
        world.body(self.body).map(|body| {
            let mut aabb = body.aabb;
            aabb.expand_by(self.support_distance);
            aabb
        })
    }
}
