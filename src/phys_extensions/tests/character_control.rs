//! CharacterControl integration tests: proxy + state machine loops over a
//! `proton::world::World` (Phase 15).
//!
//! These exercise the full per-frame flow from `README` §4 Phase 15:
//! `check_support` → `context.update` → `proxy.integrate`, including the
//! state transitions OnGround → Jumping → InAir → OnGround and the
//! interaction with dynamic bodies.

use std::sync::Arc;

use crate::base_math::Vector4 as V4;
use crate::base_math::Vec4Ext;
use glam::Vec4 as G4;
use crate::phys_extensions::{
    CharacterContext, CharacterInput, CharacterOutput, CharacterProxy, CharacterProxyCinfo,
    CharacterStateManager, CharacterStateType, SupportedState, SurfaceInfo,
};
use crate::phys_shape::{BoxShape, CapsuleShape, Shape};
use crate::phys_types::MotionType;
use crate::phys_world::World;
use crate::phys_world::WorldCinfo;

const DT: f32 = 1.0 / 60.0;
const GRAVITY: V4 = V4::from_xyz(0.0, -20.0, 0.0);
const UP: V4 = V4::from_xyz(0.0, 1.0, 0.0);

fn ground_world() -> World {
    let mut world = World::new(WorldCinfo::new());
    let ground = Arc::new(BoxShape::with_convex_radius(
        G4::from_xyz(20.0, 0.5, 20.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &crate::phys_body::BodyCinfo::new(ground).with_position(V4::from_xyz(0.0, -0.5, 0.0)),
    );
    world
}

fn capsule_proxy(position: V4) -> CharacterProxy {
    let shape = Arc::new(CapsuleShape::new(
        G4::from_xyz(0.0, -0.6, 0.0),
        G4::from_xyz(0.0, 0.6, 0.0),
        0.4,
    )) as Arc<dyn Shape>;
    let mut proxy = CharacterProxy::new(
        &CharacterProxyCinfo::new(shape)
            .with_position(position)
            .with_up(UP)
            .with_keep_distance(0.05),
    );
    proxy.set_position(position);
    proxy
}

/// Drive one frame: check support, run the state machine, integrate.
fn step_frame(
    world: &mut World,
    proxy: &mut CharacterProxy,
    context: &mut CharacterContext,
    input_ud: f32,
    want_jump: bool,
) -> CharacterOutput {
    let surface = proxy.check_support(world, -UP, DT);
    let input = CharacterInput {
        input_lr: 0.0,
        input_ud,
        want_jump,
        at_ladder: false,
        up: UP,
        forward: V4::from_xyz(0.0, 0.0, 1.0),
        surface_info: surface,
        delta_time: DT,
        inv_delta_time: 1.0 / DT,
        position: proxy.position(),
        velocity: proxy.linear_velocity(),
        character_gravity: GRAVITY,
        user_data: 0,
    };
    let mut output = CharacterOutput::default();
    context.update(&input, &mut output);
    proxy.set_linear_velocity(output.velocity);
    proxy.integrate(world, DT, &surface);
    output
}

#[test]
fn walking_into_a_wall_stops_the_character() {
    let mut world = ground_world();
    let wall = Arc::new(BoxShape::with_convex_radius(
        G4::from_xyz(0.5, 2.0, 3.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &crate::phys_body::BodyCinfo::new(wall).with_position(V4::from_xyz(3.0, 1.5, 0.0)),
    );

    let mut proxy = capsule_proxy(V4::from_xyz(0.0, 1.05, 0.0));
    let mut context = CharacterContext::new(
        Arc::new(CharacterStateManager::with_default_states()),
        CharacterStateType::OnGround,
    );

    for _ in 0..300 {
        let surface = proxy.check_support(&world, -UP, DT);
        // The movement util maps the x input onto -forward, so a forward of
        // (-1,0,0) walks along +x.
        let input = CharacterInput {
            input_lr: 0.0,
            input_ud: 1.0,
            want_jump: false,
            at_ladder: false,
            forward: V4::from_xyz(-1.0, 0.0, 0.0),
            up: UP,
            surface_info: surface,
            delta_time: DT,
            inv_delta_time: 1.0 / DT,
            position: proxy.position(),
            velocity: proxy.linear_velocity(),
            character_gravity: GRAVITY,
            user_data: 0,
        };
        let mut output = CharacterOutput::default();
        context.update(&input, &mut output);
        proxy.set_linear_velocity(output.velocity);
        proxy.integrate(&mut world, DT, &surface);
    }

    // The capsule (radius 0.4) stops before the wall face at x = 2.5.
    let x = proxy.position().x;
    assert!(
        x > 1.2 && x < 2.3,
        "character must stop at the wall, got x={}",
        x
    );
    assert_eq!(context.current_state(), CharacterStateType::OnGround);
}

#[test]
fn jump_reaches_in_air_and_lands_back_on_ground() {
    let mut world = ground_world();
    let mut proxy = capsule_proxy(V4::from_xyz(0.0, 1.05, 0.0));
    let mut manager = CharacterStateManager::with_default_states();
    // Register a stronger jump with a snappy context filter (the default
    // gain 1.0 applies only gain*dt of each velocity change per frame).
    manager.register(
        CharacterStateType::Jumping,
        Box::new(crate::phys_extensions::character::states::Jumping { jump_height: 3.0 }),
    );
    let mut context = CharacterContext::new(Arc::new(manager), CharacterStateType::OnGround);
    context.set_filter_parameters(30.0, 20.0, 625.0);

    let mut saw_in_air = false;
    let mut max_height = proxy.position().y;
    for step in 0..360 {
        let output = step_frame(&mut world, &mut proxy, &mut context, 0.0, step == 0);
        if context.current_state() == CharacterStateType::InAir {
            saw_in_air = true;
        }
        max_height = max_height.max(proxy.position().y);
        let _ = output;
    }

    assert!(saw_in_air, "character must enter InAir after jumping");
    assert!(
        max_height > 1.3,
        "character must rise above the resting height, got {}",
        max_height
    );
    assert_eq!(context.current_state(), CharacterStateType::OnGround);
    let y = proxy.position().y;
    assert!(
        (y - 1.05).abs() < 0.25,
        "character must land back near y=1.05, got {}",
        y
    );
}

#[test]
fn walking_character_pushes_dynamic_box() {
    let mut world = ground_world();
    let box_shape = Arc::new(BoxShape::with_convex_radius(
        G4::from_xyz(0.5, 0.5, 0.5),
        0.0,
    )) as Arc<dyn Shape>;
    let box_id = world.create_body(
        &crate::phys_body::BodyCinfo::new(box_shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(2.0)
            .with_position(V4::from_xyz(1.0, 1.05, 0.0)),
    );

    let mut proxy = capsule_proxy(V4::from_xyz(0.0, 1.05, 0.0));
    let mut context = CharacterContext::new(
        Arc::new(CharacterStateManager::with_default_states()),
        CharacterStateType::OnGround,
    );

    for _ in 0..120 {
        step_frame(&mut world, &mut proxy, &mut context, 1.0, false);
    }

    let box_velocity = world.body_linear_velocity(box_id).unwrap_or(V4::ZERO);
    assert!(
        box_velocity.x > 0.0,
        "dynamic box should be pushed in +x, got {:?}",
        box_velocity
    );
    assert_eq!(context.current_state(), CharacterStateType::OnGround);
}

#[test]
fn character_loop_is_deterministic() {
    let run = || {
        let mut world = ground_world();
        let wall = Arc::new(BoxShape::with_convex_radius(
            G4::from_xyz(0.5, 2.0, 3.0),
            0.0,
        )) as Arc<dyn Shape>;
        world.create_body(
            &crate::phys_body::BodyCinfo::new(wall).with_position(V4::from_xyz(3.0, 1.5, 0.0)),
        );
        let mut proxy = capsule_proxy(V4::from_xyz(0.0, 1.05, 0.0));
        let mut context = CharacterContext::new(
            Arc::new(CharacterStateManager::with_default_states()),
            CharacterStateType::OnGround,
        );
        let mut history = Vec::new();
        for step in 0..120 {
            let output = step_frame(&mut world, &mut proxy, &mut context, 1.0, step == 30);
            history.push((
                proxy.position().to_array(),
                output.velocity.to_array(),
                context.current_state() as u8,
            ));
        }
        history
    };
    let a = run();
    let b = run();
    assert_eq!(a, b, "two identical runs must match bit-for-bit");
}

#[test]
fn steep_slope_touches_manifold_and_classifies_support() {
    let mut world = World::new(WorldCinfo::new());
    // A 60° slope: normal has vertical component cos(60°) = 0.5.
    let ramp_angle = 60.0f32.to_radians();
    let ramp = Arc::new(BoxShape::with_convex_radius(
        G4::from_xyz(4.0, 0.25, 4.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &crate::phys_body::BodyCinfo::new(ramp)
            .with_position(V4::from_xyz(0.0, 1.5, 0.0))
            .with_orientation(crate::base_math::Quaternion::from_axis_angle(
                V4::from_xyz(1.0, 0.0, 0.0),
                ramp_angle,
            )),
    );

    let mut proxy = capsule_proxy(V4::from_xyz(0.0, 3.1, 0.0));
    proxy.set_max_slope(45.0f32.to_radians());
    proxy.refresh_manifold(&world);
    assert!(!proxy.manifold().is_empty(), "capsule must touch the ramp");
    let info = proxy.check_support(&world, -UP, DT);
    // Faithful outcome: the projected velocity on the 60° slope keeps an
    // angle of 60° with the (vertical) input direction; cos²(60°) = 0.25 is
    // below max_slope_cosine² = cos²(45°) ≈ 0.5, so the original
    // classification reports SLIDING. (When the character walks into the
    // slope instead, the max-slope blocking plane engages and the solve
    // zeroes the velocity — see the proxy unit tests.)
    assert_eq!(
        info.supported_state,
        SupportedState::Sliding,
        "got {:?}",
        info.supported_state
    );
    let _ = SurfaceInfo::default();
}
