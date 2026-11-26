//! Ragdoll state-machine integration scenarios.
//!
//! Exercises the facade wiring of the Keyframed/Dynamic/Powered states, the
//! powered motor blender and timed transitions through `World::step`.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::{
    Ragdoll, RagdollController, RagdollControllerCinfo, RagdollData, RagdollMotorParams,
    RagdollState,
};
use proton::math::{Quaternion, Transform, Vector4, Vec4, Vec4Ext};
use proton::np_types::{BodyId, MotionType};
use proton::shape::{BoxShape, Shape};
use proton::world::{World, WorldCinfo};

fn body(world: &mut World, position: Vec4) -> BodyId {
    let shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(0.2, 0.2, 0.2),
        0.0,
    )) as Arc<dyn Shape>;
    let id = world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(position),
    );
    let motion_id = world.body(id).unwrap().motion_id;
    world.motion_mut(motion_id).unwrap().inverse_inertia = Vec4::splat(0.5);
    id
}

fn chain(world: &mut World) -> (Ragdoll, [BodyId; 2]) {
    let ids = [
        body(world, Vec4::ZERO),
        body(world, Vec4::from_xyz(0.0, 1.0, 0.0)),
    ];
    let mut data = RagdollData::new();
    data.add_bone("hips", None, Some(ids[0]), Transform::IDENTITY);
    data.add_bone(
        "spine",
        Some(0),
        Some(ids[1]),
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 1.0, 0.0)),
    );
    (Ragdoll::create(world, &data).expect("ragdoll"), ids)
}

fn target_pose() -> [Transform; 2] {
    [
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(2.0, 0.0, 0.0)),
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 1.0, 0.0)),
    ]
}

#[test]
fn s1_keyframed_state_holds_exact_pose_under_gravity() {
    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, ids) = chain(&mut world);
    let target = target_pose();
    let dt = 1.0 / 60.0;
    let mut controller = RagdollController::new(
        &mut world,
        ragdoll,
        RagdollControllerCinfo::default().with_initial_state(RagdollState::Keyframed),
    );

    assert!(world.body(ids[0]).unwrap().is_keyframed());
    for _ in 0..60 {
        controller
            .step(&mut world, &target, Transform::IDENTITY, dt)
            .unwrap();
        world.step(dt);
    }
    assert!(
        world
            .body_position(ids[0])
            .unwrap()
            .all_equal3(Vector4::from_xyz(2.0, 0.0, 0.0), 1e-5)
    );
    assert!(
        world
            .body_position(ids[1])
            .unwrap()
            .all_equal3(Vector4::from_xyz(2.0, 1.0, 0.0), 1e-5)
    );
}

#[test]
fn s2_powered_state_drives_with_blended_motor_parameters() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    let (ragdoll, ids) = chain(&mut world);
    let target = [
        Transform::IDENTITY,
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 2.0, 0.0)),
    ];
    let dt = 1.0 / 60.0;
    let mut controller = RagdollController::new(
        &mut world,
        ragdoll,
        RagdollControllerCinfo::default()
            .with_initial_state(RagdollState::Powered)
            .with_motor(RagdollMotorParams::default()),
    );
    controller.set_strength(0.25);

    let mut driven_velocity_y = 0.0;
    for step in 0..40 {
        controller
            .step(&mut world, &target, Transform::IDENTITY, dt)
            .unwrap();
        if step == 0 {
            driven_velocity_y = world.body_linear_velocity(ids[1]).unwrap().y;
        }
        world.step(dt);
    }
    let motor = controller.last_motor().expect("powered motor blend");
    assert_eq!(motor.max_force, 505.0);
    assert!((motor.tau - 0.45).abs() < 1e-6);
    // The ragdoll joint resists translation away from its pivot, so assert the
    // drive impulse itself rather than the post-constraint resting position.
    assert!(driven_velocity_y > 0.0);
    assert_eq!(controller.state(), RagdollState::Powered);
}

#[test]
fn s3_timed_transition_fades_powered_into_dynamic() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    let (ragdoll, _) = chain(&mut world);
    let target = target_pose();
    let dt = 1.0 / 60.0;
    let mut controller = RagdollController::new(
        &mut world,
        ragdoll,
        RagdollControllerCinfo::default().with_initial_state(RagdollState::Dynamic),
    );
    controller
        .begin_timed_transition(
            &mut world,
            RagdollState::Powered,
            RagdollState::Dynamic,
            0.1,
        )
        .unwrap();

    let mut strengths = Vec::new();
    for _ in 0..6 {
        controller
            .step(&mut world, &target, Transform::IDENTITY, dt)
            .unwrap();
        world.step(dt);
        strengths.push(controller.strength());
    }
    assert_eq!(controller.state(), RagdollState::Dynamic);
    assert!(strengths.windows(2).all(|pair| pair[0] >= pair[1]));
    assert!(strengths[0] > strengths[strengths.len() - 1]);
}

#[test]
fn s4_state_machine_scenario_is_bit_deterministic() {
    let run = |snapshot_steps: usize| {
        let mut world = World::new(WorldCinfo::new());
        world.set_gravity(Vector4::from_xyz(0.0, -9.81, 0.0));
        let (ragdoll, _) = chain(&mut world);
        let target = target_pose();
        let dt = 1.0 / 60.0;
        let mut controller = RagdollController::new(
            &mut world,
            ragdoll,
            RagdollControllerCinfo::default()
                .with_initial_state(RagdollState::Keyframed)
                .with_keyframe_root(true),
        );
        for step in 0..snapshot_steps {
            if step == 30 {
                controller.set_state(&mut world, RagdollState::Powered);
                controller.set_strength(0.6);
            }
            controller
                .step(&mut world, &target, Transform::IDENTITY, dt)
                .unwrap();
            world.step(dt);
        }
        world.snapshot().to_text()
    };
    assert_eq!(run(60), run(60));
}
