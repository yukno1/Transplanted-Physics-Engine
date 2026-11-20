//! Climbing state and trigger enter/exit tracking integration tests.

use std::sync::Arc;

use base_math::Vector4 as V4;
use phys_extensions::{
    CharacterContext, CharacterInput, CharacterOutput, CharacterStateManager, CharacterStateType,
    SupportedState, SurfaceInfo, TriggerEvent, TriggerStatus, TriggerTracker,
};
use phys_shape::{BoxShape, Shape};
use phys_types::MotionType;
use phys_world::{QueryFilter, World, WorldCinfo};

const DT: f32 = 1.0 / 60.0;

fn input(surface: SurfaceInfo, at_ladder: bool) -> CharacterInput {
    CharacterInput {
        input_lr: 0.0,
        input_ud: 1.0,
        want_jump: false,
        at_ladder,
        up: V4::UNIT_Y,
        forward: V4::from_xyz(0.0, 0.0, 1.0),
        surface_info: surface,
        delta_time: DT,
        inv_delta_time: 1.0 / DT,
        position: V4::ZERO,
        velocity: V4::ZERO,
        character_gravity: V4::from_xyz(0.0, -20.0, 0.0),
        user_data: 0,
    }
}

fn supported(normal: V4) -> SurfaceInfo {
    SurfaceInfo {
        supported_state: SupportedState::Supported,
        surface_normal: normal,
        surface_velocity: V4::ZERO,
        surface_angular_velocity: V4::ZERO,
    }
}

fn new_context(initial: CharacterStateType) -> CharacterContext {
    let manager = Arc::new(CharacterStateManager::with_default_states());
    CharacterContext::new(manager, initial)
}

#[test]
fn c1_ground_and_air_enter_climbing_when_at_ladder() {
    let mut context = new_context(CharacterStateType::OnGround);
    let mut output = CharacterOutput::default();
    let mut grounded = input(supported(V4::UNIT_X), true);
    grounded.velocity = V4::from_xyz(0.0, 1.0, 0.0);

    context.update(&grounded, &mut output);
    assert_eq!(context.current_state(), CharacterStateType::Climbing);

    // The second update actually runs the Climbing state's movement util.
    output.velocity = V4::ZERO;
    context.update(&grounded, &mut output);
    assert_eq!(context.current_state(), CharacterStateType::Climbing);
    assert!(output.velocity.length3() > 0.0);

    let mut air = new_context(CharacterStateType::InAir);
    let mut air_output = CharacterOutput::default();
    let unsupported = input(SurfaceInfo::unsupported(), true);
    air.update(&unsupported, &mut air_output);
    assert_eq!(air.current_state(), CharacterStateType::Climbing);
}

#[test]
fn c2_climbing_jump_pushes_away_along_ladder_normal() {
    let mut context = new_context(CharacterStateType::Climbing);
    context.set_filter_enable(false);
    let mut jumping = input(supported(V4::UNIT_X), true);
    jumping.want_jump = true;
    jumping.velocity = V4::from_xyz(1.0, 2.0, 3.0);
    let mut output = CharacterOutput::default();

    context.update(&jumping, &mut output);

    assert_eq!(context.current_state(), CharacterStateType::InAir);
    assert!((output.velocity.x - 21.0).abs() < 1e-6);
    assert_eq!(output.velocity.y, 2.0);
    assert_eq!(output.velocity.z, 3.0);
}

#[test]
fn c3_leaving_ladder_returns_to_air() {
    let mut context = new_context(CharacterStateType::Climbing);
    let mut output = CharacterOutput::default();
    let mut off_ladder = input(SurfaceInfo::unsupported(), false);
    off_ladder.velocity = V4::from_xyz(0.0, 3.0, 0.0);
    context.update(&off_ladder, &mut output);
    assert_eq!(context.current_state(), CharacterStateType::InAir);
}

#[test]
fn t1_tracker_reports_enter_exit_and_clear() {
    let mut world = World::new(WorldCinfo::new());
    let shape = || Arc::new(BoxShape::with_convex_radius(V4::splat(0.5), 0.0)) as Arc<dyn Shape>;
    let first = world.create_body(
        &phys_body::BodyCinfo::new(shape())
            .with_motion_type(MotionType::Static)
            .with_position(V4::ZERO),
    );
    let second = world.create_body(
        &phys_body::BodyCinfo::new(shape())
            .with_motion_type(MotionType::Static)
            .with_position(V4::from_xyz(10.0, 0.0, 0.0)),
    );
    let filter = QueryFilter::default();
    let mut tracker = TriggerTracker::new();

    let entered = tracker.update(&world, V4::ZERO, 0.6, &filter);
    assert_eq!(
        entered,
        vec![TriggerEvent {
            body: first,
            status: TriggerStatus::Entered
        }]
    );
    assert!(tracker.update(&world, V4::ZERO, 0.6, &filter).is_empty());

    let mut events = tracker.update(&world, V4::from_xyz(10.0, 0.0, 0.0), 0.6, &filter);
    events.sort_by_key(|event| event.status);
    assert_eq!(events[0].status, TriggerStatus::Entered);
    assert_eq!(events[0].body, second);
    assert_eq!(events[1].status, TriggerStatus::Exited);
    assert_eq!(events[1].body, first);

    assert_eq!(
        tracker.clear(),
        vec![TriggerEvent {
            body: second,
            status: TriggerStatus::Exited
        }]
    );
}
