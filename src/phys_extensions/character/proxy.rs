//! Character proxy: a non-penetrating "ghost" character controller
//! (`hknpCharacterProxy` subset).
//!
//! Port of `original/Physics/Source/Physics/Physics/Extensions/
//! CharacterControl/Proxy/` (`hknpCharacterProxy.{h,cpp,inl}`,
//! `hknpCharacterProxy_Manifold.cpp`, `hknpCharacterProxyCinfo.h`). The proxy
//! is not a rigid body: it owns a shape plus a world-space transform, keeps a
//! self-maintained contact manifold of nearby surfaces and moves itself with
//! shape casts, using the simplex solver
//! ([`crate::phys_extensions::character::simplex_solver`]) to resolve allowed displacements
//! against the touched planes.
//!
//! # Flow (mirrors the original)
//!
//! * [`CharacterProxy::check_support`] classifies the support state
//!   (supported / sliding / unsupported) by running the simplex solver with
//!   the constraint velocities zeroed ("static geometry query").
//! * [`CharacterProxy::integrate`] moves the character: it casts the shape
//!   along the desired displacement, updates the manifold, adds the maximum
//!   slope blocking planes and lets the simplex solver walk the character
//!   along the constraint surfaces until the time slice is consumed.
//!
//! # Subset decisions
//!
//! * **World queries**: the original uses `hknpAllHitsCollector`s for both the
//!   start contacts and the cast. This port rebuilds the start contacts from
//!   per-body closest-point queries (all convex bodies within
//!   `keepDistance + keepContactTolerance`) and uses the single closest
//!   `World::cast_shape` hit for the cast contact.
//! * **Convex world only**: manifold collection ignores mesh/height-field/
//!   compound bodies (the GJK closest-point query is convex-only), mirroring
//!   the `World::closest_points_to_shape` limitation.
//! * **Listeners/phantom are local**: listeners are plain trait objects and
//!   the keyframed phantom is created/synced through `&mut World`; there is no
//!   signal manager, LOD shape selection or multi-threaded task graph.
//! * **`resolveContacts`** pushes dynamic bodies with a linear-only impulse
//!   (inverse mass only; the angular inertia term of the original is skipped)
//!   applied through `World::set_body_linear_velocity`.
//! * The plane-distance convention of the simplex solver is preserved: the
//!   solver reads `plane.xyz · pos.xyz + pos.w`, i.e. the point's `w` slot
//!   holds the plane constant (see the solver module docs).
//! * Contact deduplication (`compareContacts`) keeps the normal/distance
//!   terms; the point-velocity term is only used when comparing manifold
//!   contacts against freshly collected ones.

use std::sync::Arc;

use crate::base_math::{Quaternion, Transform, Vector4};
use crate::phys_body::{Body, BodyCinfo};
use crate::phys_collide::{closest_points as gjk_closest_points, collide as collide_manifold};
use crate::phys_shape::Shape;
use crate::phys_types::{BodyId, CollisionFlags, MotionType, ShapeKey};
use crate::phys_world::{QueryFilter, ShapeCastHitInfo, World};

use super::listener::{
    CharacterInteractionInfo, CharacterObjectInteractionEvent, CharacterObjectInteractionResult,
    CharacterProxyListener, CharacterProxyState,
};
use super::monitor::{CharacterMonitorEvent, CharacterMonitorPhase};
use super::simplex_solver::{SimplexSolverInput, SurfaceConstraint, solve};
use super::surface::{SupportedState, SurfaceInfo};
use super::triggers::TriggerEvent;

/// `HK_REAL_EPSILON` step guard for the integration loop.
const REAL_EPSILON: f32 = 1e-5;
/// `hkSimdReal_Eps` degeneracy threshold.
const SIMD_EPSILON: f32 = 1e-5;
/// `hkSimdReal_EpsSqrd`.
const SIMD_EPSILON_SQRD: f32 = 1e-10;
/// Displacement agreement threshold in [`CharacterProxy::integrate`]
/// (`hknpCharacterProxy.cpp:446`).
const DISPLACEMENT_EPSILON: f32 = 0.001;
/// Damping of the push impulse on contacted dynamic bodies
/// (`hknpCharacterProxyInternals::resolveContacts`).
const DAMP_FACTOR: f32 = 0.9;
/// Penetration recovery tau for push impulses.
const RECOVERY_TAU: f32 = 0.4;
/// `HK_REAL_MAX`.
const REAL_MAX: f32 = f32::MAX;

/// Plane vector from a unit normal and a signed distance.
fn plane_vec(normal: Vector4, distance: f32) -> Vector4 {
    Vector4::new(normal.x, normal.y, normal.z, distance)
}

/// The `xyz` part of a vector with `w = 0`.
fn xyz(v: Vector4) -> Vector4 {
    Vector4::new(v.x, v.y, v.z, 0.0)
}

/// One contact in the proxy's self-maintained manifold
/// (`hknpCharacterProxy::Contact`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    /// Contact point on the hit body (world space).
    pub position: Vector4,
    /// Separating direction from the hit body towards the proxy (unit).
    pub normal: Vector4,
    /// The hit body.
    pub body: BodyId,
    /// Fraction of the cast displacement at which the contact was picked up
    /// (`0` for closest-point contacts).
    pub fraction: f32,
    /// Signed distance between the proxy shape and the hit body along
    /// `normal` (negative when penetrating).
    pub distance: f32,
    /// Penetration budget accumulated for max-slope plane handling
    /// (`m_allowedPenetration`).
    pub allowed_penetration: f32,
    /// Leaf shape key of the hit body (`m_shapeKeyB` subset).
    pub shape_key: ShapeKey,
}

/// A deferred object push collected during a character step
/// (`hknpCharacterProxy::ObjectInteraction` subset).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeferredInteraction {
    /// Body receiving the impulse.
    pub body: BodyId,
    /// Vector impulse to apply.
    pub impulse: Vector4,
    /// Point of application (diagnostic; the linear-only port applies at COM).
    pub impulse_position: Vector4,
}

impl Contact {
    /// Build a contact from a closest-points result
    /// (`Contact::set(closestPointsQueryResult, keepDistance)`).
    fn from_closest_points(
        position: Vector4,
        normal: Vector4,
        body: BodyId,
        distance: f32,
        keep_distance: f32,
    ) -> Self {
        Self {
            position,
            normal,
            body,
            fraction: 0.0,
            distance,
            allowed_penetration: (keep_distance - distance).clamp(0.0, keep_distance),
            shape_key: ShapeKey::INVALID,
        }
    }

    /// Build a contact from a shape-cast result
    /// (`Contact::set(shapeCastQueryResult, castPath, keepDistance)`).
    fn from_shape_cast(hit: ShapeCastHitInfo, cast_path: Vector4, keep_distance: f32) -> Self {
        let cast_length = cast_path.length3();
        let fraction = if cast_length > SIMD_EPSILON {
            hit.distance / cast_length
        } else {
            0.0
        };
        // Distance travelled normal to the contact plane until the hit
        // (the cast path is assumed to include the cast length).
        let distance = -fraction * cast_path.dot3(hit.normal);
        Self {
            position: hit.position,
            normal: hit.normal,
            body: hit.body,
            fraction,
            distance,
            allowed_penetration: (keep_distance - distance).clamp(0.0, keep_distance),
            shape_key: hit.shape_key,
        }
    }
}

/// Construction info for [`CharacterProxy`] (`hknpCharacterProxyCinfo`
/// defaults).
#[derive(Clone)]
pub struct CharacterProxyCinfo {
    /// Initial world-space position.
    pub position: Vector4,
    /// Initial world-space orientation.
    pub orientation: Quaternion,
    /// Initial world-space velocity.
    pub velocity: Vector4,
    /// Default dynamic friction for touched surfaces (1.0 clips momentum
    /// against the surface normal, 0.0 keeps it).
    pub dynamic_friction: f32,
    /// Default static friction for touched surfaces.
    pub static_friction: f32,
    /// Distance above `keep_distance` within which points join the manifold.
    pub keep_contact_tolerance: f32,
    /// Up direction (normalized copy; generally opposite the gravity).
    pub up: Vector4,
    /// Character collision shape.
    pub shape: Arc<dyn Shape>,
    /// Query-side collision filter info for all proxy queries.
    pub collision_filter_info: u32,
    /// Minimum distance kept between the proxy shape and surfaces.
    pub keep_distance: f32,
    /// Angle sensitivity used when deciding whether two contacts are
    /// duplicates.
    pub contact_angle_sensitivity: f32,
    /// Velocity clamp for simultaneous plane solves in the simplex solver.
    pub max_character_speed_for_solver: f32,
    /// Maximum push impulse the proxy can impart on dynamic bodies per second
    /// (`f32::MAX` = infinitely strong).
    pub character_strength: f32,
    /// Extra downward force applied to dynamic bodies the character stands on
    /// (0 = none).
    pub character_mass: f32,
    /// Maximum walkable slope in radians from horizontal (`π/2` = vertical,
    /// disabling the feature).
    pub max_slope: f32,
    /// Speed at which penetrating planes push the proxy back out.
    pub penetration_recovery_speed: f32,
    /// Maximum shape-cast iterations per integrate call.
    pub max_cast_iterations: usize,
    /// Whether `check_support` refreshes the manifold first.
    pub refresh_manifold_in_check_support: bool,
}

impl CharacterProxyCinfo {
    /// Cinfo with the original defaults, requiring the character shape.
    pub fn new(shape: Arc<dyn Shape>) -> Self {
        Self {
            position: Vector4::ZERO,
            orientation: Quaternion::IDENTITY,
            velocity: Vector4::ZERO,
            dynamic_friction: 1.0,
            static_friction: 0.0,
            keep_contact_tolerance: 0.1,
            up: Vector4::UNIT_Y,
            shape,
            collision_filter_info: 0,
            keep_distance: 0.05,
            contact_angle_sensitivity: 10.0,
            max_character_speed_for_solver: 10.0,
            character_strength: REAL_MAX,
            character_mass: 0.0,
            max_slope: core::f32::consts::FRAC_PI_2,
            penetration_recovery_speed: 1.0,
            max_cast_iterations: 10,
            refresh_manifold_in_check_support: false,
        }
    }

    /// Set the initial position.
    pub fn with_position(mut self, position: Vector4) -> Self {
        self.position = position;
        self
    }

    /// Set the initial velocity.
    pub fn with_velocity(mut self, velocity: Vector4) -> Self {
        self.velocity = velocity;
        self
    }

    /// Set the up direction (normalized).
    pub fn with_up(mut self, up: Vector4) -> Self {
        self.up = up.normalized3();
        self
    }

    /// Set the maximum walkable slope (radians from horizontal).
    pub fn with_max_slope(mut self, max_slope: f32) -> Self {
        self.max_slope = max_slope;
        self
    }

    /// Set the kept distance to surfaces.
    pub fn with_keep_distance(mut self, keep_distance: f32) -> Self {
        self.keep_distance = keep_distance;
        self
    }

    /// Set the query-side collision filter info.
    pub fn with_collision_filter_info(mut self, filter_info: u32) -> Self {
        self.collision_filter_info = filter_info;
        self
    }

    /// Set the static/dynamic friction defaults.
    pub fn with_friction(mut self, static_friction: f32, dynamic_friction: f32) -> Self {
        self.static_friction = static_friction;
        self.dynamic_friction = dynamic_friction;
        self
    }

    /// Set the maximum shape-cast iterations per integrate call.
    pub fn with_max_cast_iterations(mut self, iterations: usize) -> Self {
        self.max_cast_iterations = iterations;
        self
    }
}

/// The character proxy controller (`hknpCharacterProxy` subset).
pub struct CharacterProxy {
    shape: Arc<dyn Shape>,
    transform: Transform,
    velocity: Vector4,
    manifold: Vec<Contact>,
    up: Vector4,
    max_slope_cosine: f32,
    keep_distance: f32,
    keep_contact_tolerance: f32,
    dynamic_friction: f32,
    static_friction: f32,
    contact_angle_sensitivity: f32,
    max_character_speed_for_solver: f32,
    character_strength: f32,
    character_mass: f32,
    penetration_recovery_speed: f32,
    max_cast_iterations: usize,
    refresh_manifold_in_check_support: bool,
    collision_filter_info: u32,
    last_displacement: Vector4,
    last_velocity: Vector4,
    last_inv_delta_time: f32,
    listeners: Vec<(u64, Box<dyn CharacterProxyListener>)>,
    next_listener_id: u64,
    phantom: Option<BodyId>,
    id: u64,
    post_solve_callback: bool,
    monitor_step_count: u64,
    deferred_interactions: Vec<DeferredInteraction>,
}

impl CharacterProxy {
    /// Construct the proxy from the cinfo
    /// (`hknpCharacterProxy::hknpCharacterProxy`, without the world
    /// registration; call [`Self::create_phantom`] to add the phantom body).
    ///
    /// # Panics
    /// Panics when `max_slope` is not in `(0, π/2]`.
    pub fn new(info: &CharacterProxyCinfo) -> Self {
        assert!(
            info.max_slope > 0.0 && info.max_slope <= 0.5 * core::f32::consts::PI,
            "maxSlope must be between 0 and pi/2 radians"
        );
        Self {
            shape: info.shape.clone(),
            transform: Transform::from_quaternion(info.orientation, info.position),
            velocity: info.velocity,
            manifold: Vec::new(),
            up: info.up.normalized3(),
            max_slope_cosine: info.max_slope.cos(),
            keep_distance: info.keep_distance,
            keep_contact_tolerance: info.keep_contact_tolerance,
            dynamic_friction: info.dynamic_friction,
            static_friction: info.static_friction,
            contact_angle_sensitivity: info.contact_angle_sensitivity,
            max_character_speed_for_solver: info.max_character_speed_for_solver,
            character_strength: info.character_strength,
            character_mass: info.character_mass,
            penetration_recovery_speed: info.penetration_recovery_speed,
            max_cast_iterations: info.max_cast_iterations,
            refresh_manifold_in_check_support: info.refresh_manifold_in_check_support,
            collision_filter_info: info.collision_filter_info,
            last_displacement: Vector4::ZERO,
            last_velocity: Vector4::ZERO,
            last_inv_delta_time: 0.0,
            listeners: Vec::new(),
            next_listener_id: 0,
            phantom: None,
            id: 0,
            post_solve_callback: false,
            monitor_step_count: 0,
            deferred_interactions: Vec::new(),
        }
    }

    /// The character shape.
    pub fn shape(&self) -> &Arc<dyn Shape> {
        &self.shape
    }

    /// World-space position.
    pub fn position(&self) -> Vector4 {
        self.transform.translation
    }

    /// Set the world-space position.
    pub fn set_position(&mut self, position: Vector4) {
        self.transform.translation = position;
    }

    /// World-space transform.
    pub fn transform(&self) -> &Transform {
        &self.transform
    }

    /// Replace the world-space transform.
    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
    }

    /// Linear velocity of the proxy.
    pub fn linear_velocity(&self) -> Vector4 {
        self.velocity
    }

    /// Set the linear velocity of the proxy.
    pub fn set_linear_velocity(&mut self, velocity: Vector4) {
        self.velocity = velocity;
    }

    /// Manager-assigned proxy id (zero for standalone proxies).
    pub fn id(&self) -> u64 {
        self.id
    }

    pub(crate) fn set_id(&mut self, id: u64) {
        self.id = id;
    }

    /// Whether `character_callback` is owned by the world post-solve signal.
    pub(crate) fn set_post_solve_callback(&mut self, enabled: bool) {
        self.post_solve_callback = enabled;
    }

    /// Copyable state snapshot used by post-solve listeners.
    pub fn state(&self) -> CharacterProxyState {
        CharacterProxyState {
            id: self.id,
            position: self.transform.translation,
            velocity: self.velocity,
            contact_count: self.manifold.len(),
        }
    }

    /// Interaction pushes collected during the last step.
    pub fn deferred_interactions(&self) -> &[DeferredInteraction] {
        &self.deferred_interactions
    }

    /// Apply collected interaction impulses serially and clear the buffer
    /// (`hknpCharacterProxy::applyDeferredUpdates` subset).
    pub fn apply_deferred_updates(&mut self, world: &mut World) {
        for interaction in &self.deferred_interactions {
            let current = world
                .body_linear_velocity(interaction.body)
                .unwrap_or(Vector4::ZERO);
            let _ = world
                .set_body_linear_velocity(interaction.body, current + xyz(interaction.impulse));
            let _ = world.wake_body(interaction.body);
        }
        self.deferred_interactions.clear();
        if let Some(phantom) = self.phantom {
            world.set_body_transform(phantom, self.transform);
        }
    }

    /// Up direction.
    pub fn up(&self) -> Vector4 {
        self.up
    }

    /// The self-maintained contact manifold.
    pub fn manifold(&self) -> &[Contact] {
        &self.manifold
    }

    /// Attach a listener and immediately notify it of the current shape.
    ///
    /// Returns an id for [`Self::remove_listener`]. Like the SDK's
    /// `addListener`, notification and later dispatch run in reverse
    /// registration order.
    pub fn add_listener(&mut self, mut listener: Box<dyn CharacterProxyListener>) -> u64 {
        let id = self.next_listener_id;
        self.next_listener_id += 1;
        listener.shape_changed();
        self.listeners.push((id, listener));
        id
    }

    /// Detach a listener by id; returns `false` when it was not registered.
    pub fn remove_listener(&mut self, id: u64) -> bool {
        let Some(index) = self.listeners.iter().position(|(other, _)| *other == id) else {
            return false;
        };
        self.listeners.remove(index);
        true
    }

    /// Forward trigger events from a `TriggerTracker` to all listeners.
    pub fn dispatch_trigger_events(&mut self, events: &[TriggerEvent]) {
        for event in events {
            for (_, listener) in self.listeners.iter_mut().rev() {
                listener.trigger_volume_interaction(*event);
            }
        }
    }

    /// Phantom body id, when presence in the world is enabled.
    pub fn phantom(&self) -> Option<BodyId> {
        self.phantom
    }

    /// Create the keyframed, non-colliding phantom body used for world presence.
    ///
    /// Queries exclude it via `QueryFilter::ignore_body`, matching the SDK's
    /// use of the phantom as the query body.
    pub fn create_phantom(&mut self, world: &mut World) -> Option<BodyId> {
        if self.phantom.is_some() {
            return self.phantom;
        }
        let info = BodyCinfo::new(self.shape.clone())
            .with_motion_type(MotionType::Keyframed)
            .with_transform(self.transform)
            .with_flags(CollisionFlags::DONT_COLLIDE)
            .with_collision_filter_info(self.collision_filter_info)
            .with_name("hknpCharacterProxy");
        let id = world.create_body(&info);
        self.phantom = Some(id);
        Some(id)
    }

    /// Destroy the phantom body, if present.
    pub fn destroy_phantom(&mut self, world: &mut World) -> bool {
        let Some(id) = self.phantom.take() else {
            return false;
        };
        world.destroy_body(id)
    }

    /// Emit contact events by diffing the manifold around a proxy operation.
    ///
    /// The SDK fires callbacks immediately around each manifold mutation;
    /// this subset emits one deterministic batch per public proxy call.
    fn dispatch_manifold_events(&mut self, before: &[Contact]) {
        let after = self.manifold.clone();
        let mut removed: Vec<Contact> = before
            .iter()
            .copied()
            .filter(|contact| !after.contains(contact))
            .collect();
        let mut added: Vec<Contact> = after
            .iter()
            .copied()
            .filter(|contact| !before.contains(contact))
            .collect();
        removed.sort_by_key(|contact| (contact.body.value(), contact.shape_key.raw()));
        added.sort_by_key(|contact| (contact.body.value(), contact.shape_key.raw()));
        for contact in &removed {
            for (_, listener) in self.listeners.iter_mut().rev() {
                listener.contact_removed(contact);
            }
        }
        for contact in &added {
            for (_, listener) in self.listeners.iter_mut().rev() {
                listener.contact_added(contact);
            }
        }
    }

    fn fire_monitor(&mut self, phase: CharacterMonitorPhase) {
        let state = self.state();
        let event = CharacterMonitorEvent {
            step_count: self.monitor_step_count,
            phase,
            body: BodyId::INVALID,
            position: state.position,
            linear_velocity: state.velocity,
            contact_count: state.contact_count,
        };
        for (_, listener) in self.listeners.iter_mut().rev() {
            listener.monitor(&event);
        }
    }

    fn fire_character_callback(&mut self) {
        if self.post_solve_callback {
            return;
        }
        let state = self.state();
        for (_, listener) in self.listeners.iter_mut().rev() {
            listener.character_callback(&state);
        }
    }

    /// Dispatch the post-solve callback for manager-owned proxies.
    pub(crate) fn fire_post_solve_callback(&mut self) {
        if !self.post_solve_callback {
            return;
        }
        let state = self.state();
        for (_, listener) in self.listeners.iter_mut().rev() {
            listener.character_callback(&state);
        }
        self.fire_monitor(CharacterMonitorPhase::PostSolve);
    }

    /// Dispatch a character↔character interaction in reverse listener order.
    pub(crate) fn fire_character_interaction(&mut self, info: &CharacterInteractionInfo) {
        for (_, listener) in self.listeners.iter_mut().rev() {
            listener.character_interaction(info);
        }
    }

    /// Dispatch the simplex constraint-processed callback.
    pub(crate) fn fire_constraints_processed(&mut self, count: usize) {
        for (_, listener) in self.listeners.iter_mut().rev() {
            listener.process_constraints(count);
        }
    }

    /// Maximum walkable slope (radians).
    pub fn max_slope(&self) -> f32 {
        self.max_slope_cosine.acos()
    }

    /// Set the maximum walkable slope (radians).
    pub fn set_max_slope(&mut self, max_slope: f32) {
        self.max_slope_cosine = max_slope.cos();
    }

    /// Shift the proxy and its manifold contacts by `offset`
    /// (`hknpCharacterProxy::shift`).
    pub fn shift(&mut self, offset: Vector4) {
        self.transform.translation += offset;
        for contact in &mut self.manifold {
            contact.position += offset;
        }
    }

    /// The query filter built from the proxy's collision filter info.
    fn filter(&self) -> QueryFilter {
        QueryFilter {
            filter_info: self.collision_filter_info,
            ignore_body: self.phantom,
            ..Default::default()
        }
    }

    /// Blocking cosine for max-slope planes (`addMaxSlopePlane` uses the raw
    /// `m_maxSlopeCosine`; the 0.1 floor in the original belongs to the
    /// contact-position projection in `createSurfaceConstraint`, which is
    /// part of the not-ported rotating-surface correction).
    fn blocking_max_slope_cosine(&self) -> f32 {
        self.max_slope_cosine
    }

    /// Remove manifold contacts whose bodies are no longer in the world
    /// (`validateManifold`).
    fn validate_manifold(&mut self, world: &World) {
        self.manifold
            .retain(|contact| world.body(contact.body).is_some());
    }

    /// Whether a body participates in proxy queries (motion-type flags only;
    /// the world's collision filter is a body-pair filter and does not gate
    /// closest-point collection).
    fn query_allows(filter: &QueryFilter, body: &Body) -> bool {
        if filter.ignore_body == Some(body.id) {
            return false;
        }
        if body.is_static() {
            filter.include_static
        } else if body.is_keyframed() {
            filter.include_keyframed
        } else {
            filter.include_dynamic
        }
    }

    /// Collect all closest-point contacts within
    /// `keepDistance + keepContactTolerance` (the "start collector").
    fn collect_start_contacts(&self, world: &World, filter: &QueryFilter) -> Vec<Contact> {
        let tolerance = self.keep_distance + self.keep_contact_tolerance;
        let query_aabb = self.shape.calc_aabb(&self.transform);
        let mut expanded = query_aabb;
        expanded.expand_by(tolerance);

        let mut contacts = Vec::new();
        for body in world.bodies() {
            if !Self::query_allows(filter, body) || !body.shape.is_convex() {
                continue;
            }
            if !body.aabb.overlaps(&expanded) {
                continue;
            }
            let Some(result) = gjk_closest_points(
                self.shape.as_ref(),
                &self.transform,
                body.shape.as_ref(),
                &body.transform,
            ) else {
                continue;
            };
            if result.intersects {
                // Overlapping pairs take the penetration depth from the
                // EPA-backed collision query (mirrors
                // `World::closest_points_to_shape`).
                if let Some(manifold) = collide_manifold(
                    self.shape.as_ref(),
                    &self.transform,
                    body.shape.as_ref(),
                    &body.transform,
                ) {
                    if let Some(point) = manifold.points.first() {
                        if point.distance <= tolerance {
                            contacts.push(Contact::from_closest_points(
                                point.position,
                                manifold.normal,
                                body.id,
                                point.distance,
                                self.keep_distance,
                            ));
                        }
                    }
                }
                continue;
            }
            if result.distance <= tolerance {
                contacts.push(Contact::from_closest_points(
                    result.point_b,
                    result.normal,
                    body.id,
                    result.distance,
                    self.keep_distance,
                ));
            }
        }
        contacts
    }

    /// Point velocity of the contacted body at the contact point
    /// (`hknpMotion::getPointVelocity`; static bodies report zero).
    fn contact_point_velocity(&self, world: &World, contact: &Contact) -> Vector4 {
        let Some(body) = world.body(contact.body) else {
            return Vector4::ZERO;
        };
        let Some(motion) = world.motion(body.motion_id) else {
            return Vector4::ZERO;
        };
        if motion.motion_type == MotionType::Static {
            return Vector4::ZERO;
        }
        let arm = contact.position - motion.position;
        motion.linear_velocity + motion.angular_velocity.cross(arm)
    }

    /// Similarity metric for two contacts (`compareContacts`); closer to zero
    /// means more similar. The point-velocity term requires the world.
    fn compare_contacts(&self, world: &World, a: &Contact, b: &Contact) -> f32 {
        let angle_squared = (1.0 - a.normal.dot3(b.normal))
            * (self.contact_angle_sensitivity * self.contact_angle_sensitivity);
        let plane_distance_sqrd = {
            let d = a.distance - b.distance;
            d * d
        };
        let velocity_diff_sqrd = {
            let v0 = self.contact_point_velocity(world, a);
            let v1 = self.contact_point_velocity(world, b);
            (v0 - v1).length_squared3()
        };
        angle_squared * 10.0 + velocity_diff_sqrd * 0.1 + plane_distance_sqrd
    }

    /// Best match of `reference` in `contact_list` under `threshold`
    /// (`findContact`).
    fn find_contact(
        &self,
        world: &World,
        reference: &Contact,
        contact_list: &[Contact],
        threshold: f32,
    ) -> Option<usize> {
        let mut best: Option<(usize, f32)> = None;
        for (j, candidate) in contact_list.iter().enumerate() {
            let fitness = self.compare_contacts(world, reference, candidate);
            if fitness < threshold && best.is_none_or(|(_, f)| fitness < f) {
                best = Some((j, fitness));
            }
        }
        best.map(|(j, _)| j)
    }

    /// Remove contacts too similar to survive the simplex solver
    /// (tail of `updateManifold`, geometry terms only — the compared contacts
    /// share the proxy's velocity terms). Order is preserved like the
    /// original's `removeAt` with copy.
    fn remove_similar_contacts(&mut self) {
        let mut e1 = self.manifold.len();
        while e1 > 1 {
            e1 -= 1;
            let current = self.manifold[e1];
            let similar = self.manifold[..e1].iter().any(|other| {
                let angle_squared = (1.0 - current.normal.dot3(other.normal))
                    * (self.contact_angle_sensitivity * self.contact_angle_sensitivity);
                let d = current.distance - other.distance;
                angle_squared * 10.0 + d * d < 0.1
            });
            if similar {
                self.manifold.remove(e1);
            }
        }
    }

    /// Update the manifold from start contacts plus the cast contact
    /// (`updateManifold`).
    fn update_manifold(
        &mut self,
        world: &World,
        start_contacts: &[Contact],
        cast_contact: Option<Contact>,
    ) {
        if start_contacts.is_empty() {
            // No start hits: clear the manifold completely.
            self.manifold.clear();
        } else {
            let mut new_contacts = start_contacts.to_vec();
            for i in (0..self.manifold.len()).rev() {
                let current = self.manifold[i];
                let best_match = self.find_contact(world, &current, &new_contacts, 1.1);
                match best_match {
                    Some(j) => {
                        let new_contact = new_contacts[j];
                        self.manifold[i] = new_contact;
                        new_contacts.swap_remove(j);
                    }
                    None => {
                        self.manifold.swap_remove(i);
                    }
                }
            }
            // Add the closest remaining new contact; it can never be an
            // unwanted edge.
            let min_distance = new_contacts
                .iter()
                .map(|c| c.distance)
                .fold(REAL_MAX, f32::min);
            if let Some(index) = new_contacts.iter().position(|c| c.distance == min_distance) {
                let new_contact = new_contacts[index];
                let best_match = self.find_contact(world, &new_contact, &self.manifold, 0.1);
                match best_match {
                    Some(j) => self.manifold[j] = new_contact,
                    None => self.manifold.push(new_contact),
                }
            }
        }

        // Add the closest cast contact when it is not already in the manifold.
        if let Some(cast_contact) = cast_contact {
            let best_match = self.find_contact(world, &cast_contact, &self.manifold, 0.1);
            if best_match.is_none() {
                self.manifold.push(cast_contact);
            }
        }

        self.remove_similar_contacts();
    }

    /// Build a surface constraint from a manifold contact
    /// (`createSurfaceConstraint`).
    fn create_surface_constraint(
        &self,
        world: &World,
        contact: &Contact,
        time_travelled: f32,
    ) -> SurfaceConstraint {
        // Subtract the keep distance from the plane distance so the
        // constraint enforces that separation.
        let mut constraint = SurfaceConstraint {
            plane: plane_vec(contact.normal, contact.distance - self.keep_distance),
            velocity: Vector4::ZERO,
            angular_velocity: Vector4::ZERO,
            static_friction: self.static_friction,
            dynamic_friction: self.dynamic_friction,
            extra_up_static_friction: 0.0,
            extra_down_static_friction: 0.0,
            priority: 0,
        };

        let Some(body) = world.body(contact.body) else {
            return constraint;
        };
        // Increase the priority of fixed and keyframed bodies.
        if body.is_static() {
            constraint.priority = 2;
        } else if body.is_keyframed() {
            constraint.priority = 1;
        }
        if !body.is_static() {
            constraint.velocity = self.contact_point_velocity(world, contact);

            // Move the plane back based on the velocity and the time already
            // travelled (HVK-1477).
            let shift = constraint.velocity.dot3(constraint.plane) * time_travelled;
            constraint.plane = plane_vec(contact.normal, constraint.plane.w - shift);
        }
        constraint
    }

    /// Add extra velocity pushing the proxy out of a penetrating plane
    /// (`resolveConstraintPenetration`).
    fn resolve_constraint_penetration(&self, constraint: &mut SurfaceConstraint) {
        let distance = constraint.plane.w;
        if distance < -SIMD_EPSILON {
            let push = xyz(constraint.plane) * (distance * self.penetration_recovery_speed);
            constraint.velocity -= push;
            constraint.plane = plane_vec(xyz(constraint.plane), 0.0);
        }
    }

    /// Append a vertical blocking plane for surfaces steeper than the maximum
    /// slope (`addMaxSlopePlane`). Returns the extra plane when one was added;
    /// may disable penetration recovery on the original constraint.
    fn add_max_slope_plane(
        &self,
        constraint: &mut SurfaceConstraint,
        allowed_penetration: f32,
    ) -> Option<SurfaceConstraint> {
        let vertical_component = constraint.plane.dot3(self.up);
        if !(vertical_component > 0.01 && vertical_component < self.blocking_max_slope_cosine()) {
            return None;
        }
        let mut new_constraint = *constraint;
        let distance = new_constraint.plane.w;
        // Copy the constraint and zero its vertical normal component.
        let horizontal = xyz(xyz(constraint.plane) - self.up * vertical_component);
        let horizontal = horizontal.normalized3();
        if distance >= 0.0 {
            // Distance to the blocking plane along the new normal.
            let w = distance * horizontal.dot3(constraint.plane);
            new_constraint.plane = plane_vec(horizontal, w);
        } else {
            // Resolve the penetration along the new normal and disable
            // penetration recovery for the original plane.
            let penetration_to_resolve = 0.0f32.min(distance + allowed_penetration);
            let w = penetration_to_resolve / horizontal.dot3(constraint.plane);
            new_constraint.plane = plane_vec(horizontal, w);
            constraint.plane = plane_vec(xyz(constraint.plane), 0.0);
            self.resolve_constraint_penetration(&mut new_constraint);
        }
        Some(new_constraint)
    }

    /// Build the simplex constraints from the manifold
    /// (`createConstraintsFromManifold`, including max slope planes and
    /// penetration recovery). Max slope planes are appended right after the
    /// original constraint, matching the original array layout.
    fn create_constraints_from_manifold(
        &self,
        world: &World,
        time_travelled: f32,
    ) -> Vec<SurfaceConstraint> {
        let mut constraints = Vec::with_capacity(self.manifold.len() * 2);
        for contact in &self.manifold {
            let mut constraint = self.create_surface_constraint(world, contact, time_travelled);
            let slope = self.add_max_slope_plane(&mut constraint, contact.allowed_penetration);
            self.resolve_constraint_penetration(&mut constraint);
            constraints.push(constraint);
            if let Some(slope) = slope {
                constraints.push(slope);
            }
        }
        constraints
    }

    /// Classify the support state (`checkSupportWithCollector`).
    ///
    /// `direction` should be normalized.
    pub fn check_support(
        &mut self,
        world: &World,
        direction: Vector4,
        _delta_time: f32,
    ) -> SurfaceInfo {
        if self.refresh_manifold_in_check_support {
            self.refresh_manifold(world);
        } else {
            self.validate_manifold(world);
        }
        let before = self.manifold.clone();

        // Empty manifold: definitely unsupported.
        if self.manifold.is_empty() {
            self.dispatch_manifold_events(&before);
            return SurfaceInfo::unsupported();
        }

        let mut constraints = self.create_constraints_from_manifold(world, 0.0);

        // Static geometry query: strip velocities, remember them for output.
        let stored_velocities: Vec<Vector4> = constraints.iter().map(|c| c.velocity).collect();
        for constraint in &mut constraints {
            constraint.velocity = Vector4::ZERO;
        }

        let input = SimplexSolverInput {
            position: Vector4::ZERO,
            velocity: direction,
            max_surface_velocity: Vector4::splat(self.max_character_speed_for_solver),
            up: self.up,
            delta_time: 1.0 / 60.0,
            min_delta_time: 1.0 / 60.0,
            constraints,
        };
        let output = solve(&input);

        // If the constraints did not affect the movement => unsupported.
        let epsilon = 0.001;
        if output.velocity.all_equal3(direction, epsilon) {
            self.dispatch_manifold_events(&before);
            return SurfaceInfo::unsupported();
        }

        let mut ground = SurfaceInfo::unsupported();
        if output.velocity.length_squared3() < epsilon {
            ground.supported_state = SupportedState::Supported;
        } else {
            let normalized = output.velocity.normalized3();
            let angle_sin = normalized.dot3(direction);
            let cos_sqr = 1.0 - angle_sin * angle_sin;
            if cos_sqr < self.max_slope_cosine * self.max_slope_cosine {
                ground.supported_state = SupportedState::Sliding;
            } else {
                ground.supported_state = SupportedState::Supported;
            }
        }

        // Average the touched supporting planes facing the input direction.
        let mut num_touching = 0usize;
        for (i, interaction) in output.interactions.iter().enumerate() {
            let Some(constraint) = input.constraints.get(i) else {
                continue;
            };
            if interaction.touched && constraint.plane.dot3(direction) < -0.08 {
                ground.surface_normal += constraint.plane;
                ground.surface_velocity += stored_velocities[i];
                ground.surface_angular_velocity += constraint.angular_velocity;
                num_touching += 1;
            }
        }
        if num_touching == 0 {
            self.dispatch_manifold_events(&before);
            return SurfaceInfo::unsupported();
        }
        ground.surface_normal.normalize3();
        ground.surface_velocity *= 1.0 / num_touching as f32;
        ground.surface_angular_velocity *= 1.0 / num_touching as f32;
        self.dispatch_manifold_events(&before);
        ground
    }

    /// Rebuild the manifold from current closest points (`refreshManifold`).
    pub fn refresh_manifold(&mut self, world: &World) {
        let before = self.manifold.clone();
        self.validate_manifold(world);
        let filter = self.filter();
        let mut new_contacts = self.collect_start_contacts(world, &filter);
        for i in (0..self.manifold.len()).rev() {
            let current = self.manifold[i];
            let best_match = self.find_contact(world, &current, &new_contacts, 1.1);
            match best_match {
                Some(j) => {
                    self.manifold[i] = new_contacts[j];
                    new_contacts.swap_remove(j);
                }
                None => {
                    self.manifold.swap_remove(i);
                }
            }
        }
        if let Some(index) = new_contacts
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.distance.total_cmp(&b.1.distance))
            .map(|(i, _)| i)
        {
            let new_contact = new_contacts[index];
            if self
                .find_contact(world, &new_contact, &self.manifold, 0.1)
                .is_none()
            {
                self.manifold.push(new_contact);
            }
        }
        self.remove_similar_contacts();
        self.dispatch_manifold_events(&before);
    }

    /// Move the character for one time slice (`integrateWithCollectors`).
    ///
    /// `delta_time` must exceed `HK_REAL_EPSILON`.
    pub fn integrate(&mut self, world: &mut World, delta_time: f32, surface_info: &SurfaceInfo) {
        self.deferred_interactions.clear();
        self.monitor_step_count += 1;
        self.fire_monitor(CharacterMonitorPhase::IntegrateBegin);
        let before = self.manifold.clone();
        let mut remaining_time = delta_time;
        debug_assert!(
            delta_time > REAL_EPSILON,
            "Integration with delta time must be larger than HK_REAL_EPSILON"
        );
        let mut new_velocity = Vector4::ZERO;

        // Choose the first cast direction: reuse the previous displacement
        // when the velocity did not change, otherwise guess from the current
        // velocity (projected onto a supporting surface when appropriate).
        {
            let tolerance = DISPLACEMENT_EPSILON * self.last_inv_delta_time;
            if self.velocity.all_equal3(self.last_velocity, tolerance) {
                self.last_displacement *= remaining_time * self.last_inv_delta_time;
            } else {
                let mut displacement_velocity = self.velocity;
                if surface_info.supported_state == SupportedState::Supported {
                    let mut relative_velocity = self.velocity - surface_info.surface_velocity;
                    let normal_dot = surface_info.surface_normal.dot3(relative_velocity);
                    if normal_dot < 0.0 {
                        relative_velocity -= surface_info.surface_normal * normal_dot;
                        displacement_velocity = relative_velocity + surface_info.surface_velocity;
                    }
                }
                self.last_displacement = displacement_velocity * remaining_time;
            }
            self.last_velocity = self.velocity;
            self.last_inv_delta_time = 1.0 / delta_time;
        }

        self.validate_manifold(world);
        let filter = self.filter();

        let mut iter = 0usize;
        while remaining_time > REAL_EPSILON && iter < self.max_cast_iterations {
            iter += 1;

            // Linear cast with the last displacement.
            let cast_path = self.last_displacement;
            let cast_length = cast_path.length3();
            let cast_contact = if cast_length > SIMD_EPSILON {
                world
                    .cast_shape(
                        self.shape.as_ref(),
                        &self.transform,
                        cast_path * (1.0 / cast_length),
                        cast_length,
                        &filter,
                    )
                    .map(|hit| Contact::from_shape_cast(hit, cast_path, self.keep_distance))
            } else {
                None
            };

            // Update the manifold with the start contacts and the cast hit.
            let start_contacts = self.collect_start_contacts(world, &filter);
            self.update_manifold(world, &start_contacts, cast_contact);

            // Constraints from the manifold.
            let constraints =
                self.create_constraints_from_manifold(world, delta_time - remaining_time);
            self.fire_constraints_processed(constraints.len());

            // Simplex solve for the allowed displacement.
            let (solver_delta_time, new_displacement, solved_velocity) = {
                let velocity_length2 = self.velocity.length_squared3();
                let min_delta_time = if velocity_length2 == 0.0 {
                    0.0
                } else {
                    0.5 * self.keep_distance / velocity_length2.sqrt()
                };
                let input = SimplexSolverInput {
                    position: Vector4::ZERO,
                    velocity: self.velocity,
                    max_surface_velocity: Vector4::new(
                        self.max_character_speed_for_solver,
                        self.max_character_speed_for_solver,
                        self.max_character_speed_for_solver,
                        0.0,
                    ),
                    up: self.up,
                    delta_time: remaining_time,
                    min_delta_time,
                    constraints,
                };
                let output = solve(&input);
                (output.delta_time, output.position, output.velocity)
            };
            new_velocity = solved_velocity;

            // Push contacted dynamic bodies.
            self.resolve_contacts(world, delta_time);

            // Check whether we can walk to the position the solver suggests.
            let mut new_contact_index: Option<usize> = None;
            if new_displacement.length_squared3() > SIMD_EPSILON_SQRD
                && !self
                    .last_displacement
                    .all_equal3(new_displacement, DISPLACEMENT_EPSILON)
            {
                // Re-cast along the solver displacement to collect any new
                // contact on the way.
                let recast_length = new_displacement.length3();
                let recast_hit = world.cast_shape(
                    self.shape.as_ref(),
                    &self.transform,
                    new_displacement * (1.0 / recast_length),
                    recast_length,
                    &filter,
                );
                if let Some(hit) = recast_hit {
                    let contact =
                        Contact::from_shape_cast(hit, new_displacement, self.keep_distance);
                    if self
                        .find_contact(world, &contact, &self.manifold, 0.1)
                        .is_none()
                    {
                        new_contact_index = Some(self.manifold.len());
                        self.manifold.push(contact);
                    }
                }
            }

            match new_contact_index {
                Some(index) => {
                    // Move along newDisplacement until the character is at
                    // keepDistance from the new contact.
                    let contact = self.manifold[index];
                    let displacement_length_inv = 1.0 / new_displacement.length3();
                    let angle = new_displacement.dot3(contact.normal) * displacement_length_inv;
                    let keep_distance_along_movement = self.keep_distance / -angle;
                    let mut fraction =
                        contact.fraction - keep_distance_along_movement * displacement_length_inv;
                    fraction = fraction.clamp(0.0, 1.0);
                    self.transform.translation += new_displacement * fraction;
                    remaining_time -= solver_delta_time * fraction;
                }
                None => {
                    // Move all the way to the end of the allowed displacement.
                    self.transform.translation += new_displacement;
                    remaining_time -= solver_delta_time;
                }
            }
            self.last_displacement = new_displacement;
        }

        self.velocity = new_velocity;
        if let Some(phantom) = self.phantom {
            world.set_body_transform(phantom, self.transform);
        }
        self.dispatch_manifold_events(&before);
        self.fire_monitor(CharacterMonitorPhase::IntegrateEnd);
        self.fire_character_callback();
    }

    /// Push dynamic bodies out of the character's way
    /// (`resolveContacts`, linear-impulse subset).
    fn resolve_contacts(&mut self, world: &mut World, delta_time: f32) {
        let inv_delta_time = 1.0 / delta_time;
        let max_push_impulse = -self.character_strength * delta_time;

        for contact in self.manifold.clone() {
            let Some(body) = world.body(contact.body) else {
                continue;
            };
            if !body.is_dynamic() {
                continue;
            }
            let Some(motion) = world.motion(body.motion_id) else {
                continue;
            };
            if motion.mass <= 0.0 {
                continue;
            }
            let inverse_mass = 1.0 / motion.mass;

            // Relative normal velocity of the contact point.
            let point_velocity = self.contact_point_velocity(world, &contact);
            let projected_velocity = (point_velocity - self.velocity).dot3(contact.normal);

            let mut delta_velocity = -projected_velocity * DAMP_FACTOR;
            if contact.distance < 0.0 {
                delta_velocity += contact.distance * inv_delta_time * RECOVERY_TAU;
            }

            let mut impulse = 0.0;
            if delta_velocity < 0.0 {
                impulse = delta_velocity / inverse_mass;
                if impulse < max_push_impulse {
                    impulse = max_push_impulse;
                }
            }

            // Extra downward force when the character has a mass
            // (`m_characterMass` term).
            if self.character_mass > 0.0 {
                let char_vel_down = world.gravity() * delta_time;
                let mut rel_vel_n = char_vel_down.dot3(contact.normal);
                if projected_velocity < 0.0 {
                    rel_vel_n -= projected_velocity;
                }
                if rel_vel_n < -SIMD_EPSILON {
                    impulse += rel_vel_n * self.character_mass;
                }
            }

            if impulse != 0.0 {
                let event = CharacterObjectInteractionEvent {
                    body: contact.body,
                    position: contact.position,
                    normal: contact.normal,
                    object_impulse: impulse,
                    timestep: delta_time,
                    projected_velocity,
                    object_mass_inv: inverse_mass,
                };
                let mut result = CharacterObjectInteractionResult {
                    object_impulse: contact.normal * impulse,
                    impulse_position: contact.position,
                };
                for (_, listener) in self.listeners.iter_mut().rev() {
                    listener.object_interaction(&event, &mut result);
                }
                let deferred = DeferredInteraction {
                    body: contact.body,
                    impulse: result.object_impulse,
                    impulse_position: result.impulse_position,
                };
                if self.post_solve_callback {
                    // Manager-owned proxies apply pushes serially after all
                    // characters have stepped.
                    self.deferred_interactions.push(deferred);
                } else {
                    let current = world
                        .body_linear_velocity(contact.body)
                        .unwrap_or(Vector4::ZERO);
                    let _ = world
                        .set_body_linear_velocity(contact.body, current + xyz(deferred.impulse));
                    let _ = world.wake_body(contact.body);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_math::Vector4 as V4;
    use crate::phys_shape::{BoxShape, SphereShape};
    use crate::phys_world::WorldCinfo;
    use std::sync::Arc;

    fn ground_world() -> (World, crate::phys_types::BodyId) {
        let mut world = World::new(WorldCinfo::new());
        let ground = Arc::new(BoxShape::with_convex_radius(
            V4::from_xyz(10.0, 0.5, 10.0),
            0.0,
        )) as Arc<dyn Shape>;
        let id = world.create_body(
            &crate::phys_body::BodyCinfo::new(ground).with_position(V4::from_xyz(0.0, -0.5, 0.0)),
        );
        (world, id)
    }

    fn capsule_proxy(position: V4) -> CharacterProxy {
        let shape = Arc::new(crate::phys_shape::CapsuleShape::new(
            V4::from_xyz(0.0, -0.6, 0.0),
            V4::from_xyz(0.0, 0.6, 0.0),
            0.4,
        )) as Arc<dyn Shape>;
        let mut proxy = CharacterProxy::new(&CharacterProxyCinfo::new(shape));
        proxy.set_position(position);
        proxy
    }

    #[test]
    fn standing_on_ground_is_supported() {
        let (world, _ground) = ground_world();
        let mut proxy = capsule_proxy(V4::from_xyz(0.0, 1.05, 0.0));
        proxy.refresh_manifold(&world);
        assert!(
            !proxy.manifold().is_empty(),
            "capsule must touch the ground"
        );
        let info = proxy.check_support(&world, V4::from_xyz(0.0, -1.0, 0.0), 1.0 / 60.0);
        assert_eq!(info.supported_state, SupportedState::Supported);
        assert!(info.surface_normal.y > 0.9, "{:?}", info.surface_normal);
    }

    #[test]
    fn in_air_is_unsupported() {
        let (world, _ground) = ground_world();
        let mut proxy = capsule_proxy(V4::from_xyz(0.0, 5.0, 0.0));
        proxy.refresh_manifold(&world);
        assert!(proxy.manifold().is_empty());
        let info = proxy.check_support(&world, V4::from_xyz(0.0, -1.0, 0.0), 1.0 / 60.0);
        assert_eq!(info.supported_state, SupportedState::Unsupported);
    }

    #[test]
    fn integrate_downwards_lands_on_ground() {
        let (mut world, _ground) = ground_world();
        let mut proxy = capsule_proxy(V4::from_xyz(0.0, 2.0, 0.0));
        proxy.set_linear_velocity(V4::from_xyz(0.0, -3.0, 0.0));
        let dt = 1.0 / 60.0;
        for _ in 0..120 {
            let info = proxy.check_support(&world, V4::from_xyz(0.0, -1.0, 0.0), dt);
            proxy.integrate(&mut world, dt, &info);
        }
        // The capsule bottom (0.4 radius below the lowest segment point) rests
        // on the ground top (y = 0) plus the keep distance.
        let y = proxy.position().y;
        assert!(
            y > 0.9 && y < 1.15,
            "character should rest near y=1.0, got {}",
            y
        );
        assert!(proxy.linear_velocity().y.abs() < 1e-4);
    }

    #[test]
    fn integrate_into_wall_stops() {
        let (mut world, _ground) = ground_world();
        let wall = Arc::new(BoxShape::with_convex_radius(
            V4::from_xyz(0.5, 2.0, 2.0),
            0.0,
        )) as Arc<dyn Shape>;
        world.create_body(
            &crate::phys_body::BodyCinfo::new(wall).with_position(V4::from_xyz(2.0, 1.5, 0.0)),
        );
        let mut proxy = capsule_proxy(V4::from_xyz(0.0, 1.05, 0.0));
        proxy.set_linear_velocity(V4::from_xyz(4.0, 0.0, 0.0));
        let dt = 1.0 / 60.0;
        for _ in 0..90 {
            let info = proxy.check_support(&world, V4::from_xyz(0.0, -1.0, 0.0), dt);
            proxy.integrate(&mut world, dt, &info);
        }
        // The capsule (radius 0.4) stops before the wall face at x = 1.5.
        let x = proxy.position().x;
        assert!(
            x > 0.7 && x < 1.3,
            "character must stop at the wall, got x={}",
            x
        );
    }

    #[test]
    fn max_slope_plane_blocks_steep_ramp() {
        let (mut world, _ground) = ground_world();
        // A 60° ramp ascending towards +x (top face normal tilts towards
        // -x); max slope 30° (cos ≈ 0.866).
        let ramp_angle = 60.0f32.to_radians();
        let ramp = Arc::new(BoxShape::with_convex_radius(
            V4::from_xyz(2.0, 0.25, 2.0),
            0.0,
        )) as Arc<dyn Shape>;
        world.create_body(
            &crate::phys_body::BodyCinfo::new(ramp)
                .with_position(V4::from_xyz(4.0, 1.0, 0.0))
                .with_orientation(crate::base_math::Quaternion::from_axis_angle(
                    V4::from_xyz(0.0, 0.0, 1.0),
                    ramp_angle,
                )),
        );
        let mut proxy = capsule_proxy(V4::from_xyz(0.0, 1.05, 0.0));
        proxy.set_max_slope(30.0f32.to_radians());
        proxy.set_linear_velocity(V4::from_xyz(3.0, 0.0, 0.0));
        let dt = 1.0 / 60.0;
        for _ in 0..90 {
            let info = proxy.check_support(&world, V4::from_xyz(0.0, -1.0, 0.0), dt);
            proxy.integrate(&mut world, dt, &info);
        }
        // The character must not climb far up the steep ramp.
        assert!(
            proxy.position().x < 4.0,
            "character must be blocked by the steep ramp, got x={}",
            proxy.position().x
        );
    }

    #[test]
    fn resolve_contacts_pushes_dynamic_body() {
        let (mut world, _ground) = ground_world();
        let box_shape = Arc::new(BoxShape::with_convex_radius(
            V4::from_xyz(0.5, 0.5, 0.5),
            0.0,
        )) as Arc<dyn Shape>;
        let box_id = world.create_body(
            &crate::phys_body::BodyCinfo::new(box_shape)
                .with_motion_type(crate::phys_types::MotionType::Dynamic)
                .with_mass(2.0)
                .with_position(V4::from_xyz(0.9, 1.05, 0.0)),
        );
        let mut proxy = capsule_proxy(V4::from_xyz(0.0, 1.05, 0.0));
        proxy.refresh_manifold(&world);
        proxy.set_linear_velocity(V4::from_xyz(2.0, 0.0, 0.0));
        let dt = 1.0 / 60.0;
        let info = proxy.check_support(&world, V4::from_xyz(0.0, -1.0, 0.0), dt);
        proxy.integrate(&mut world, dt, &info);
        let velocity = world.body_linear_velocity(box_id).unwrap_or(V4::ZERO);
        assert!(
            velocity.x > 0.0,
            "dynamic box should be pushed in +x, got {:?}",
            velocity
        );
    }

    #[test]
    fn integrate_is_deterministic() {
        let run = || {
            let (mut world, _ground) = ground_world();
            let mut proxy = capsule_proxy(V4::from_xyz(0.0, 2.0, 0.0));
            proxy.set_linear_velocity(V4::from_xyz(1.0, -3.0, 0.5));
            let dt = 1.0 / 60.0;
            for _ in 0..60 {
                let info = proxy.check_support(&world, V4::from_xyz(0.0, -1.0, 0.0), dt);
                proxy.integrate(&mut world, dt, &info);
            }
            proxy.position().to_array()
        };
        let a = run();
        let b = run();
        assert_eq!(a, b);
    }

    #[test]
    fn cinfo_defaults_match_sdk() {
        let shape = Arc::new(SphereShape::new(V4::ZERO, 0.5)) as Arc<dyn Shape>;
        let info = CharacterProxyCinfo::new(shape);
        assert_eq!(info.dynamic_friction, 1.0);
        assert_eq!(info.static_friction, 0.0);
        assert_eq!(info.keep_contact_tolerance, 0.1);
        assert_eq!(info.keep_distance, 0.05);
        assert_eq!(info.contact_angle_sensitivity, 10.0);
        assert_eq!(info.max_character_speed_for_solver, 10.0);
        assert_eq!(info.character_mass, 0.0);
        assert_eq!(info.max_slope, core::f32::consts::FRAC_PI_2);
        assert_eq!(info.penetration_recovery_speed, 1.0);
        assert_eq!(info.max_cast_iterations, 10);
        assert!(!info.refresh_manifold_in_check_support);
    }
}
