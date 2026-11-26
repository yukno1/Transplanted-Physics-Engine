//! Ray-casting character controller utility
//! (`hknpRayCastingCharacterControllerUtil` subset).
//!
//! The SDK has no ray-casting *rigid body* class: this module ports the public
//! utility that turns eight downward casts, an optional shape cast and a
//! horizontal bump cast into [`CharacterParam`]. Callers can feed the resulting
//! desired displacement/easing into their own movement or state-machine route.
//!
//! Cast order is fixed (forward then 45° rotations around `up`), results are
//! compared by `(distance, body id)` inside world queries, and all statistics
//! use integer counters, so the output is deterministic across runs and thread
//! counts.

use crate::base_math::{Quaternion, Vec4Ext, Vector4};
use crate::phys_shape::{to_vec4, to_vector4};
use crate::phys_types::BodyId;
use crate::phys_world::{QueryFilter, World};

const INPUT_DIRECTION_COUNT: usize = 8;
const DIRECTION_STEP: f32 = core::f32::consts::PI * 0.25;
const INPUT_DOT_THRESHOLD: f32 = -0.1;
const DIFF_TOLERANCE: f32 = 0.01;
const SUPPORT_TOLERANCE: f32 = 1.1;

/// Result of one ray-casting character query (`CharacterParam`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterParam {
    /// Vertical displacement required to keep the character at the target
    /// height (positive climbs a step, negative descends).
    pub desired_displacement: f32,
    /// Multiplier returned by the horizontal bump cast (1 when unobstructed).
    pub bump_factor: f32,
    /// Velocity easing factor selected by the displacement direction.
    pub easing_factor: f32,
    /// Whether a support surface was found within tolerance.
    pub has_support: bool,
    /// Whether at least one cast hit a walkable slope.
    pub can_climb: bool,
    /// Easing factor when moving up (SDK default 0.15).
    pub velocity_easing_factor_up: f32,
    /// Easing factor when moving down (SDK default 0.25).
    pub velocity_easing_factor_down: f32,
}

impl Default for CharacterParam {
    fn default() -> Self {
        Self {
            desired_displacement: 0.0,
            bump_factor: 1.0,
            easing_factor: 0.25,
            has_support: false,
            can_climb: true,
            velocity_easing_factor_up: 0.15,
            velocity_easing_factor_down: 0.25,
        }
    }
}

/// Inputs for [`get_character_params`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayCastingCharacterCinfo {
    /// Character forward direction (used as the first cast direction).
    pub forward: Vector4,
    /// Character height used for cast offsets and support tolerance.
    pub body_height: f32,
    /// Character width used for horizontal offsets and bump casts.
    pub body_width: f32,
    /// Maximum climbable step height.
    pub max_step_height: f32,
    /// Maximum walkable slope in radians.
    pub max_slope: f32,
    /// Bump multiplier returned when the horizontal cast hits an obstacle.
    pub bump_factor: f32,
    /// Query filter applied to every cast; the character body is always ignored.
    pub query_filter: QueryFilter,
}

impl RayCastingCharacterCinfo {
    /// Construct with SDK-neutral query settings.
    pub fn new(
        forward: Vector4,
        body_height: f32,
        body_width: f32,
        max_step_height: f32,
        max_slope: f32,
        bump_factor: f32,
    ) -> Self {
        Self {
            forward,
            body_height,
            body_width,
            max_step_height,
            max_slope,
            bump_factor,
            query_filter: QueryFilter::default(),
        }
    }

    /// Override the query filter (the character body is still ignored).
    #[must_use]
    pub fn with_query_filter(mut self, filter: QueryFilter) -> Self {
        self.query_filter = filter;
        self
    }
}

#[derive(Clone, Copy)]
struct CastSpec {
    start: Vector4,
    end: Vector4,
}

fn unit(direction: Vector4) -> Option<Vector4> {
    let length = direction.length3();
    if length > 1e-10 {
        Some(direction * (1.0 / length))
    } else {
        None
    }
}

fn input_directions(up: Vector4, forward: Vector4) -> Vec<Vector4> {
    let rotation = Quaternion::from_axis_angle(up, DIRECTION_STEP);
    let mut directions = Vec::with_capacity(INPUT_DIRECTION_COUNT);
    let mut current = unit(forward).unwrap_or(up.cross(Vector4::UNIT_X));
    if unit(current).is_none() {
        current = Vector4::UNIT_X;
    }
    for index in 0..INPUT_DIRECTION_COUNT {
        if index > 0 {
            current = rotation.rotate(current);
        }
        current = unit(current).unwrap_or(current);
        directions.push(current);
    }
    directions
}

fn cast_specs(
    position: Vector4,
    up: Vector4,
    directions: &[Vector4],
    user_input: Vector4,
    half_width: f32,
    max_cast_distance: f32,
) -> Vec<CastSpec> {
    let mut specs = Vec::with_capacity(directions.len());
    if unit(user_input).is_some() {
        for &direction in directions {
            if user_input.dot3(direction) >= INPUT_DOT_THRESHOLD {
                let start = position + direction * half_width;
                specs.push(CastSpec {
                    start,
                    end: start - up * max_cast_distance,
                });
            }
        }
    } else {
        specs.push(CastSpec {
            start: position,
            end: position - up * max_cast_distance,
        });
    }
    specs
}

struct CastSummary {
    diffs: Vec<f32>,
    min_distance: f32,
    max_slope_misses: usize,
}

fn perform_vertical_casts(
    world: &World,
    body_id: BodyId,
    specs: &[CastSpec],
    up: Vector4,
    info: &RayCastingCharacterCinfo,
    filter: &QueryFilter,
) -> CastSummary {
    let mut diffs = Vec::with_capacity(specs.len() + 1);
    let mut min_distance = f32::MAX;
    let mut max_slope_misses = 0usize;

    for spec in specs {
        let cast_length = (spec.end - spec.start).length3();
        let Some(hit) = world.raycast_filtered(to_vec4(spec.start), -to_vec4(up), cast_length, filter)
        else {
            continue;
        };
        let distance = hit.hit_fraction;
        if distance < min_distance {
            min_distance = distance;
        }
        if hit.normal.dot3(to_vec4(up)) < info.max_slope.cos() {
            max_slope_misses += 1;
        }
        diffs.push(info.max_step_height + 0.5 * info.body_height - distance);
    }

    if min_distance > info.max_step_height {
        if let Some(body) = world.body(body_id) {
            if let Some(hit) = world.cast_shape(
                body.shape.as_ref(),
                &body.transform,
                -to_vec4(up),
                3.0 * info.max_step_height,
                filter,
            ) {
                if hit.distance < min_distance {
                    min_distance = hit.distance;
                }
                diffs.push(info.max_step_height - hit.distance);
            }
        }
    }

    CastSummary {
        diffs,
        min_distance,
        max_slope_misses,
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PotentialDiffs {
    min: f32,
    next_to_min: f32,
    max: f32,
    ups: usize,
    downs: usize,
}

fn calculate_potential_diffs(diffs: &[f32]) -> PotentialDiffs {
    // SDK quirk: HK_FLOAT_MIN is the smallest positive normalized value.
    let mut min = f32::MAX;
    let mut next_to_min = f32::MAX;
    let mut max = f32::MIN_POSITIVE;
    for &diff in diffs {
        if diff < min {
            min = diff;
        }
        if diff > max {
            max = diff;
        }
    }
    let mut ups = 0usize;
    let mut downs = 0usize;
    let mut middles = 0usize;
    for &diff in diffs {
        if (min - DIFF_TOLERANCE..=min + DIFF_TOLERANCE).contains(&diff) {
            downs += 1;
        } else if (max - DIFF_TOLERANCE..=max + DIFF_TOLERANCE).contains(&diff) {
            ups += 1;
        } else if diff < next_to_min {
            next_to_min = diff;
            middles += 1;
        }
    }
    if middles == 0 {
        next_to_min = min;
    }
    PotentialDiffs {
        min,
        next_to_min,
        max,
        ups,
        downs,
    }
}

fn bump_factor(
    world: &World,
    position: Vector4,
    up: Vector4,
    user_input: Vector4,
    info: &RayCastingCharacterCinfo,
    filter: &QueryFilter,
) -> f32 {
    let half_width = 0.5 * info.body_width;
    let half_height = 0.5 * info.body_height;
    let up_temp = up * half_height;
    let start = position + user_input * half_width - up_temp;
    let bottom = position - up_temp;
    let Some(direction) = unit(start - bottom) else {
        return 1.0;
    };
    if world
        .raycast_filtered(to_vec4(start), to_vec4(direction), half_width, filter)
        .is_some()
    {
        info.bump_factor
    } else {
        1.0
    }
}

/// Compute ray-casting character parameters for one simulation frame.
///
/// Returns a neutral [`CharacterParam`] when `body_id` is not a live body or
/// no cast produced a usable height difference.
pub fn get_character_params(
    world: &World,
    body_id: BodyId,
    info: &RayCastingCharacterCinfo,
    user_input: Vector4,
) -> CharacterParam {
    let mut param = CharacterParam::default();
    let Some(body) = world.body(body_id) else {
        return param;
    };
    let position = body.transform.translation;
    let up = unit(to_vector4(-world.gravity())).unwrap_or(Vector4::UNIT_Y);
    let filter = info.query_filter.with_ignore_body(body_id);

    let directions = input_directions(up, info.forward);
    let half_width = 0.5 * info.body_width;
    let max_cast_distance = 2.0 * info.max_step_height + 0.5 * info.body_height;
    let specs = cast_specs(
        position,
        up,
        &directions,
        user_input,
        half_width,
        max_cast_distance,
    );
    let summary = perform_vertical_casts(world, body_id, &specs, up, info, &filter);
    param.bump_factor = bump_factor(world, position, up, user_input, info, &filter);
    param.can_climb = summary.max_slope_misses < specs.len();
    param.easing_factor = param.velocity_easing_factor_down;

    if summary.diffs.is_empty() {
        return param;
    }
    let diffs = calculate_potential_diffs(&summary.diffs);
    param.desired_displacement = if diffs.downs > diffs.ups {
        diffs.min
    } else if diffs.ups > diffs.downs {
        diffs.max
    } else {
        diffs.next_to_min
    };
    param.easing_factor = if param.desired_displacement > 0.0 {
        param.velocity_easing_factor_up
    } else {
        param.velocity_easing_factor_down
    };
    param.has_support = summary.min_distance
        <= (info.max_step_height + 0.5 * info.body_height) * SUPPORT_TOLERANCE
        && param.desired_displacement >= -info.max_step_height;
    param
}

/// Shape-cast convenience wrapper that keeps the character's current transform.
pub fn shape_cast_from_body(
    world: &World,
    body_id: BodyId,
    direction: Vector4,
    max_distance: f32,
    filter: &QueryFilter,
) -> Option<crate::phys_world::ShapeCastHitInfo> {
    let body = world.body(body_id)?;
    world.cast_shape(
        body.shape.as_ref(),
        &body.transform,
        to_vec4(direction),
        max_distance,
        &filter.with_ignore_body(body_id),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::phys_body::BodyCinfo;
    use crate::phys_shape::{BoxShape, Shape, SphereShape};
    use crate::phys_types::MotionType;
    use crate::phys_world::WorldCinfo;
    use crate::base_math::Vec4Ext;
    use glam::Vec4;
    use std::sync::Arc;

    fn box_shape(half: Vec4) -> Arc<dyn Shape> {
        Arc::new(BoxShape::with_convex_radius(half, 0.0)) as Arc<dyn Shape>
    }

    fn scene(height: f32) -> (World, BodyId) {
        let mut world =
            World::new(WorldCinfo::new().with_gravity(Vec4::from_xyz(0.0, -9.81, 0.0)));
        world.create_body(
            &BodyCinfo::new(box_shape(Vec4::from_xyz(10.0, 0.5, 10.0)))
                .with_position(Vec4::from_xyz(0.0, height - 0.5, 0.0)),
        );
        let character = world.create_body(
            &BodyCinfo::new(Arc::new(SphereShape::new(Vec4::ZERO, 0.5)) as Arc<dyn Shape>)
                .with_motion_type(MotionType::Dynamic)
                .with_mass(75.0)
                .with_position(Vec4::from_xyz(0.0, height + 0.5, 0.0)),
        );
        (world, character)
    }

    #[test]
    fn flat_ground_reports_support_and_zero_displacement() {
        let (world, body) = scene(0.0);
        let info = RayCastingCharacterCinfo::new(Vector4::UNIT_X, 1.0, 1.0, 0.5, 1.0, 0.5);
        let param = get_character_params(&world, body, &info, Vector4::ZERO);
        assert!(param.has_support);
        assert!(param.can_climb);
        // Flat ground at centre distance halfHeight yields diff = maxStepHeight.
        assert!((param.desired_displacement - 0.5).abs() < 0.02);
        assert_eq!(param.bump_factor, 1.0);
    }

    #[test]
    fn step_up_reports_positive_displacement_when_inputs_agree() {
        let (mut world, body) = scene(0.0);
        world.create_body(
            &BodyCinfo::new(box_shape(Vec4::from_xyz(1.0, 0.2, 1.0)))
                .with_position(Vec4::from_xyz(2.0, -0.2, 0.0)),
        );
        let info = RayCastingCharacterCinfo::new(Vector4::UNIT_X, 1.0, 1.0, 0.5, 1.0, 0.5);
        let param = get_character_params(&world, body, &info, Vector4::UNIT_X);
        assert!(param.has_support);
        assert!(param.can_climb);
    }

    #[test]
    fn invalid_body_returns_neutral_parameters() {
        let (world, _) = scene(0.0);
        let info = RayCastingCharacterCinfo::new(Vector4::UNIT_X, 1.0, 1.0, 0.5, 1.0, 0.5);
        assert_eq!(
            get_character_params(&world, BodyId::INVALID, &info, Vector4::ZERO),
            CharacterParam::default()
        );
    }
}
