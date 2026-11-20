//! Ragdoll penetration detection and correction subset.
//!
//! Rust port of `Extensions/Ragdoll/Utils/hknpRagdollPenetrationUtil`: root
//! penetration detection (closest-point + backface ray), batched bone-segment
//! rays, ancestor status propagation, hierarchical `DONT_COLLIDE` masking and
//! corrective impulses with gravity cancellation.
//!
//! Deviations forced by the current port: the closest-point query uses
//! `World::closest_points_to_shape` with a positive search radius and a
//! penetration-depth test; ray queries cannot express backface-only hits or
//! ignore a whole ragdoll at once, so the filter ignores the queried body and
//! uses the supplied `QueryFilter::filter_info`. Corrective impulses update
//! both linear and angular motion directly (there is no world-level
//! `applyBodyImpulseAt`).

use base_math::{Transform, Vector4};
use phys_types::{BodyId, CollisionFlags};
use phys_world::{QueryFilter, World};

use crate::ragdoll::Ragdoll;

/// `PenetrationStatusFlagsEnum` bit values.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PenetrationStatus(u8);

impl PenetrationStatus {
    /// A ray from a bone's parent to the bone hit geometry.
    pub const RAY: Self = Self(1);
    /// An ancestor of this bone was flagged by a collision query.
    pub const ANCESTOR: Self = Self(2);
    /// Closest-point query reported a penetration (declared by the SDK).
    pub const CLOSEST_POINT: Self = Self(4);
    /// All three status bits, used for ancestor propagation.
    pub const ANY: Self = Self(7);

    /// Raw flag bits.
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Whether any flag in `other` is set.
    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 != 0
    }

    /// Add `other` to this status.
    pub fn insert(&mut self, other: Self) {
        self.0 |= other.0;
    }

    /// Clear all flags.
    pub fn clear(&mut self) {
        self.0 = 0;
    }
}

/// `ResolveModeFlagsEnum` bit values.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ResolveMode(u8);

impl ResolveMode {
    /// Apply corrective impulses.
    pub const RESOLVE: Self = Self(1);
    /// Resolve rootward penetrations before leafward ones.
    pub const RESOLVE_HIERARCHICALLY: Self = Self(2);
    /// Treat the resolution vector as a world position instead of a direction.
    pub const RESOLVE_TOWARD_POSITION: Self = Self(4);
    /// SDK default: resolve + hierarchical.
    pub const DEFAULT: Self = Self(3);

    /// Raw flag bits.
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Whether all flags in `other` are set.
    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
}

/// Root/bone index constants from `BodyHierarchy::ParentIndex`.
pub const ROOT: i32 = 0;
/// The root has no parent.
pub const NO_PARENT: i32 = -1;
/// The bone is excluded from penetration resolution.
pub const EXCLUDED: i32 = -2;

/// `hknpRagdollPenetrationUtil::Input` with SDK defaults.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PenetrationInput {
    /// Total ragdoll mass; negative values are replaced by the sum of body masses.
    pub total_mass: f32,
    /// Resolution mode flags (default `RESOLVE | RESOLVE_HIERARCHICALLY`).
    pub mode_flags: ResolveMode,
    /// World query filter (use `filter_info` to ignore ragdoll bodies).
    pub query_filter: QueryFilter,
    /// Root penetration depth threshold; negative means require real penetration.
    pub root_penetration_tolerance: f32,
    /// World-space resolution direction, or target position with `RESOLVE_TOWARD_POSITION`.
    pub resolution_vector: Vector4,
    /// Target separating speed along the penetration normal.
    pub max_resolution_speed: f32,
    /// Root backface-ray length when `resolution_vector` is a direction.
    pub max_resolution_distance: f32,
}

impl Default for PenetrationInput {
    fn default() -> Self {
        Self {
            total_mass: -1.0,
            mode_flags: ResolveMode::DEFAULT,
            query_filter: QueryFilter::default(),
            root_penetration_tolerance: -0.015,
            resolution_vector: Vector4::ZERO,
            max_resolution_speed: 1.5,
            max_resolution_distance: 2.0,
        }
    }
}

impl PenetrationInput {
    /// Override the resolution direction (expected to be unit length).
    #[must_use]
    pub fn with_resolution_vector(mut self, direction: Vector4) -> Self {
        self.resolution_vector = direction;
        self
    }

    /// Override the total mass used for gravity cancellation.
    #[must_use]
    pub fn with_total_mass(mut self, mass: f32) -> Self {
        self.total_mass = mass;
        self
    }

    /// Override the query filter info used for penetration queries.
    #[must_use]
    pub fn with_filter_info(mut self, filter_info: u32) -> Self {
        self.query_filter = self.query_filter.with_filter_info(filter_info);
        self
    }
}

/// One bone in a topologically sorted penetration hierarchy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HierarchyNode {
    /// Parent node index, `ROOT`, `NO_PARENT` or `EXCLUDED`.
    pub parent_index: i32,
    /// Current world position of the bone segment.
    pub position: Vector4,
    /// Associated body, if any.
    pub body: Option<BodyId>,
}

/// `BodyHierarchy`: parent-before-child bone positions and bodies.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyHierarchy {
    /// Nodes in topological order.
    pub nodes: Vec<HierarchyNode>,
}

impl BodyHierarchy {
    /// Build a hierarchy from a ragdoll's current world poses.
    ///
    /// Bones whose strength is `<= 0` are marked `EXCLUDED`, matching the SDK's
    /// tag-mask filtering. Bone-less terminal nodes use their parent's
    /// position plus the reference translation.
    pub fn from_ragdoll(world: &World, ragdoll: &Ragdoll, bone_strengths: Option<&[f32]>) -> Self {
        let bones = ragdoll.bones();
        let mut nodes: Vec<HierarchyNode> = Vec::with_capacity(bones.len());
        for (index, bone) in bones.iter().enumerate() {
            let enabled = bone_strengths
                .and_then(|values| values.get(index).copied())
                .unwrap_or(1.0)
                > 0.0;
            let position = bone
                .body
                .and_then(|body| world.body_transform(body))
                .map(|transform| transform.translation)
                .or_else(|| {
                    bone.parent
                        .and_then(|parent| nodes.get(parent).map(|node| node.position))
                        .map(|parent_position| parent_position + bone.reference.translation)
                })
                .unwrap_or(bone.reference.translation);
            let parent_index = match bone.parent {
                None => {
                    if enabled && bone.body.is_some() {
                        NO_PARENT
                    } else {
                        EXCLUDED
                    }
                }
                Some(parent) => {
                    if enabled
                        && bone.body.is_some()
                        && nodes.get(parent).and_then(|node| node.body).is_some()
                    {
                        parent as i32
                    } else {
                        EXCLUDED
                    }
                }
            };
            nodes.push(HierarchyNode {
                parent_index,
                position,
                body: bone.body,
            });
        }
        Self { nodes }
    }

    /// Sum the masses of all non-static bodies in the hierarchy.
    pub fn total_mass(&self, world: &World) -> f32 {
        let mut total = 0.0;
        for node in &self.nodes {
            if let Some(body) = node.body {
                if let Some(motion_id) = world.body(body).map(|body| body.motion_id) {
                    total += world
                        .motion(motion_id)
                        .map(|motion| motion.mass)
                        .unwrap_or(0.0);
                }
            }
        }
        total
    }

    /// Clear `DONT_COLLIDE` on every body (`reenableAllCollision`).
    pub fn reenable_collisions(&self, world: &mut World) {
        for node in &self.nodes {
            if let Some(body) = node.body {
                if let Some(body) = world.body_mut(body) {
                    body.flags.remove(CollisionFlags::DONT_COLLIDE);
                }
            }
        }
    }
}

/// One detected penetration (`hknpRagdollPenetrationUtil::Penetration`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Penetration {
    /// Node whose body should receive the corrective impulse (the parent for hierarchy rays).
    pub penetrating_node: usize,
    /// Body that was hit by the query.
    pub penetrated_body: BodyId,
    /// World-space penetration point.
    pub point: Vector4,
    /// World-space separating direction, pointing out of the obstacle.
    pub normal: Vector4,
}

/// `DetectionOutput` with per-node status flags.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DetectionOutput {
    /// Whether the root is embedded in or behind geometry.
    pub has_root_penetration: bool,
    /// Root penetration details, if detected.
    pub root_penetration: Option<Penetration>,
    /// One entry per hierarchy ray hit.
    pub penetrations: Vec<Penetration>,
    /// Status flags in hierarchy node order.
    pub body_status: Vec<PenetrationStatus>,
}

impl DetectionOutput {
    /// Clear detections and statuses without resizing `body_status`.
    pub fn clear(&mut self) {
        self.has_root_penetration = false;
        self.root_penetration = None;
        self.penetrations.clear();
        for status in &mut self.body_status {
            status.clear();
        }
    }
}

/// Velocity-space correction: `maxResolutionSpeed - dot(relative, direction)`.
///
/// Returns zero when the bodies already separate at or above the target speed.
pub fn corrective_impulse(
    body_point_velocity: Vector4,
    collider_point_velocity: Vector4,
    direction: Vector4,
    max_resolution_speed: f32,
) -> Vector4 {
    let separating = body_point_velocity - collider_point_velocity;
    let velocity_diff = max_resolution_speed - separating.dot3(direction);
    if velocity_diff > 0.0 {
        direction * velocity_diff
    } else {
        Vector4::ZERO
    }
}

/// Propagate `RAY`/`CLOSEST_POINT` statuses to descendants.
///
/// Nodes must be parent-before-child; a child inherits `ANCESTOR` whenever its
/// parent has any of the three status bits set.
pub fn propagate_ancestor_status(hierarchy: &BodyHierarchy, output: &mut DetectionOutput) {
    for (index, node) in hierarchy.nodes.iter().enumerate() {
        let parent = node.parent_index;
        if parent >= ROOT
            && (parent as usize) < index
            && output.body_status[parent as usize].contains(PenetrationStatus::ANY)
        {
            output.body_status[index].insert(PenetrationStatus::ANCESTOR);
        }
    }
}

/// Detect root and hierarchy penetrations (`detectPenetration`).
pub fn detect_penetration(
    world: &World,
    hierarchy: &BodyHierarchy,
    input: &PenetrationInput,
    output: &mut DetectionOutput,
) -> bool {
    output
        .body_status
        .resize(hierarchy.nodes.len(), PenetrationStatus::default());
    output.clear();

    detect_root(world, hierarchy, input, output);
    detect_hierarchy(world, hierarchy, input, output);
    propagate_ancestor_status(hierarchy, output);

    output.has_root_penetration || !output.penetrations.is_empty()
}

fn detect_root(
    world: &World,
    hierarchy: &BodyHierarchy,
    input: &PenetrationInput,
    output: &mut DetectionOutput,
) {
    let Some(root) = hierarchy.nodes.first() else {
        return;
    };
    if root.parent_index == EXCLUDED {
        return;
    }
    let Some(body) = root.body else {
        return;
    };
    let filter = input.query_filter.with_ignore_body(body);
    let transform = world.body_transform(body).unwrap_or(Transform::IDENTITY);
    let shape = world.body(body).map(|body| body.shape.as_ref());

    // Phase A: closest-point query on the root shape, requiring penetration
    // deeper than `root_penetration_tolerance`.
    if let Some(shape) = shape {
        let radius = input.root_penetration_tolerance.abs().max(0.1);
        if let Some(hit) = world.closest_points_to_shape(shape, &transform, radius, &filter) {
            if hit.distance <= input.root_penetration_tolerance {
                output.has_root_penetration = true;
                output.root_penetration = Some(Penetration {
                    penetrating_node: ROOT as usize,
                    penetrated_body: hit.body,
                    point: hit.point_on_query,
                    normal: hit.normal,
                });
            }
        }
    }

    // Phase B: ray from the root toward the resolution point, catching a root
    // that is fully behind a surface. The SDK uses backface-only triangle hits;
    // our query is two-sided and returns the closest hit.
    if output.has_root_penetration {
        return;
    }
    let start = transform.translation;
    let direction = if input
        .mode_flags
        .contains(ResolveMode::RESOLVE_TOWARD_POSITION)
    {
        input.resolution_vector - start
    } else {
        input.resolution_vector * input.max_resolution_distance
    };
    let length = direction.length3();
    if length <= 1e-6 {
        return;
    }
    if let Some(hit) = world.raycast_filtered(start, direction, length, &filter) {
        output.has_root_penetration = true;
        output.root_penetration = Some(Penetration {
            penetrating_node: ROOT as usize,
            penetrated_body: hit.body,
            point: hit.position,
            normal: hit.normal,
        });
    }
}

fn detect_hierarchy(
    world: &World,
    hierarchy: &BodyHierarchy,
    input: &PenetrationInput,
    output: &mut DetectionOutput,
) {
    for node in hierarchy.nodes.iter().skip(1) {
        if node.parent_index == EXCLUDED || node.parent_index == NO_PARENT {
            continue;
        }
        let parent_index = node.parent_index as usize;
        let Some(parent) = hierarchy.nodes.get(parent_index) else {
            continue;
        };
        let direction = node.position - parent.position;
        if direction.length3() <= 1e-6 {
            continue;
        }
        let mut filter = input.query_filter;
        if let Some(body) = node.body {
            filter = filter.with_ignore_body(body);
        }
        if let Some(hit) =
            world.raycast_filtered(parent.position, direction, direction.length3(), &filter)
        {
            output.penetrations.push(Penetration {
                penetrating_node: parent_index,
                penetrated_body: hit.body,
                point: hit.position,
                normal: hit.normal,
            });
            output.body_status[parent_index].insert(PenetrationStatus::RAY);
        }
    }
}

fn point_velocity(world: &World, body: BodyId, point: Vector4) -> Vector4 {
    let Some(body) = world.body(body) else {
        return Vector4::ZERO;
    };
    let Some(motion) = world.motion(body.motion_id) else {
        return Vector4::ZERO;
    };
    motion.linear_velocity + motion.angular_velocity.cross(point - motion.position)
}

fn apply_impulse_at(world: &mut World, body: BodyId, impulse: Vector4, point: Vector4) {
    let Some(motion_id) = world.body(body).map(|body| body.motion_id) else {
        return;
    };
    let Some(motion) = world.motion_mut(motion_id) else {
        return;
    };
    if motion.mass <= 0.0 || !motion.mass.is_finite() {
        return;
    }
    motion.linear_velocity += impulse * (1.0 / motion.mass);
    let torque = (point - motion.position).cross(impulse);
    let local_torque = motion.orientation.rotate_inverse(torque);
    let local_delta = Vector4::from_xyz(
        local_torque.x * motion.inverse_inertia.x,
        local_torque.y * motion.inverse_inertia.y,
        local_torque.z * motion.inverse_inertia.z,
    );
    motion.angular_velocity += motion.orientation.rotate(local_delta);
    world.wake_body(body);
}

fn apply_corrective_impulse(
    world: &mut World,
    input: &PenetrationInput,
    body: BodyId,
    collider: BodyId,
    point: Vector4,
    direction: Vector4,
    additional: Vector4,
) {
    let mut normal = direction;
    if normal.length3() > 1e-6 {
        normal = normal.normalized3();
    }
    let correction = corrective_impulse(
        point_velocity(world, body, point),
        point_velocity(world, collider, point),
        normal,
        input.max_resolution_speed,
    );
    let mass = world
        .body(body)
        .and_then(|body| world.motion(body.motion_id))
        .map(|motion| motion.mass)
        .unwrap_or(0.0);
    let mut impulse = correction * mass + additional;
    impulse.w = 0.0;
    apply_impulse_at(world, body, impulse, point);
}

/// Apply `DONT_COLLIDE` masks for the current detection snapshot.
pub fn update_collision_flags(
    world: &mut World,
    hierarchy: &BodyHierarchy,
    output: &DetectionOutput,
) {
    if output.has_root_penetration {
        for node in hierarchy.nodes.iter().skip(1) {
            if node.parent_index >= ROOT {
                if let Some(parent_body) = node
                    .body
                    .and_then(|_| hierarchy.nodes.get(node.parent_index as usize))
                    .and_then(|parent| parent.body)
                {
                    if let Some(body) = world.body_mut(parent_body) {
                        body.flags.insert(CollisionFlags::DONT_COLLIDE);
                    }
                }
            }
        }
        return;
    }
    for (index, node) in hierarchy.nodes.iter().enumerate() {
        let Some(body) = node.body else {
            continue;
        };
        let Some(body) = world.body_mut(body) else {
            continue;
        };
        let flagged = output
            .body_status
            .get(index)
            .map(|status| {
                status.contains(PenetrationStatus::RAY)
                    || status.contains(PenetrationStatus::ANCESTOR)
            })
            .unwrap_or(false);
        body.flags.set(CollisionFlags::DONT_COLLIDE, flagged);
    }
}

/// Detect and correct penetrations (`detectAndResolvePenetration`).
///
/// Returns `true` when the detection snapshot was clean; the result is based
/// on the snapshot taken before impulses are applied, as in the SDK.
pub fn detect_and_resolve_penetration(
    world: &mut World,
    hierarchy: &BodyHierarchy,
    input: &PenetrationInput,
    delta_time: f32,
    output: &mut DetectionOutput,
) -> bool {
    let detected = detect_penetration(world, hierarchy, input, output);
    if !detected {
        hierarchy.reenable_collisions(world);
        return true;
    }

    if input
        .mode_flags
        .contains(ResolveMode::RESOLVE_HIERARCHICALLY)
    {
        update_collision_flags(world, hierarchy, output);
    }

    if input
        .mode_flags
        .contains(ResolveMode::RESOLVE_HIERARCHICALLY)
        && output.has_root_penetration
    {
        if let Some(root_penetration) = output.root_penetration {
            let root_body = hierarchy.nodes[ROOT as usize].body;
            if let Some(body) = root_body {
                let total_mass = if input.total_mass >= 0.0 {
                    input.total_mass
                } else {
                    hierarchy.total_mass(world)
                };
                let gravity_cancellation = world.gravity() * (-total_mass * delta_time);
                apply_corrective_impulse(
                    world,
                    input,
                    body,
                    body,
                    root_penetration.point,
                    root_penetration.normal,
                    gravity_cancellation,
                );
            }
        }
    } else {
        let mut impulses = Vec::new();
        for penetration in &output.penetrations {
            let node = &hierarchy.nodes[penetration.penetrating_node];
            let Some(body) = node.body else {
                continue;
            };
            let status = output.body_status[penetration.penetrating_node];
            if status.contains(PenetrationStatus::RAY)
                && !status.contains(PenetrationStatus::ANCESTOR)
            {
                impulses.push((body, *penetration));
            }
        }
        for (body, penetration) in impulses {
            apply_corrective_impulse(
                world,
                input,
                body,
                penetration.penetrated_body,
                penetration.point,
                penetration.normal,
                Vector4::ZERO,
            );
        }
    }

    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(parent_index: i32) -> HierarchyNode {
        HierarchyNode {
            parent_index,
            position: Vector4::ZERO,
            body: None,
        }
    }

    #[test]
    fn defaults_match_sdk() {
        let input = PenetrationInput::default();
        assert_eq!(input.total_mass, -1.0);
        assert_eq!(input.mode_flags, ResolveMode::DEFAULT);
        assert_eq!(input.root_penetration_tolerance, -0.015);
        assert_eq!(input.max_resolution_speed, 1.5);
        assert_eq!(input.max_resolution_distance, 2.0);
        assert_eq!(PenetrationStatus::ANY.bits(), 7);
        assert_eq!(ResolveMode::RESOLVE_HIERARCHICALLY.bits(), 2);
    }

    #[test]
    fn ancestor_status_cascades_in_topological_order() {
        let hierarchy = BodyHierarchy {
            nodes: vec![
                node(NO_PARENT),
                node(ROOT),
                node(1),
                node(EXCLUDED),
                node(3),
            ],
        };
        let mut output = DetectionOutput {
            body_status: vec![PenetrationStatus::default(); 5],
            ..DetectionOutput::default()
        };
        output.body_status[1].insert(PenetrationStatus::RAY);
        propagate_ancestor_status(&hierarchy, &mut output);
        assert!(output.body_status[2].contains(PenetrationStatus::ANCESTOR));
        assert!(!output.body_status[1].contains(PenetrationStatus::ANCESTOR));
        assert!(!output.body_status[3].contains(PenetrationStatus::ANCESTOR));
        assert!(!output.body_status[4].contains(PenetrationStatus::ANCESTOR));
    }

    #[test]
    fn corrective_impulse_targets_separating_speed() {
        let up = Vector4::UNIT_Y;
        let moving_in = Vector4::from_xyz(0.0, -2.0, 0.0);
        let impulse = corrective_impulse(moving_in, Vector4::ZERO, up, 1.5);
        assert!((impulse.y - 3.5).abs() < 1e-6);
        let separating_fast = Vector4::from_xyz(0.0, 4.0, 0.0);
        assert_eq!(
            corrective_impulse(separating_fast, Vector4::ZERO, up, 1.5),
            Vector4::ZERO
        );
    }
}
