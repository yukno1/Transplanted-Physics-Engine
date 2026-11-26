//! Ragdoll constraint-easing and at-rest transition integration scenarios.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::{
    DEFAULT_EASING_DURATION, LimitSnapshot, Ragdoll, RagdollController, RagdollControllerCinfo,
    RagdollData, RagdollError, RagdollState,
};
use proton::math::{Quaternion, Transform, Vector4, Vec4, Vec4Ext};
use proton::np_types::{ConstraintId, MotionType};
use proton::shape::{Shape, SphereShape};
use proton::world::{World, WorldCinfo, WorldJoint};

fn chain(world: &mut World) -> (Ragdoll, proton::np_types::BodyId, ConstraintId) {
    let shape = || Arc::new(SphereShape::new(Vec4::ZERO, 0.25)) as Arc<dyn Shape>;
    let root = world.create_body(
        &BodyCinfo::new(shape())
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0),
    );
    let child = world.create_body(
        &BodyCinfo::new(shape())
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(Vec4::from_xyz(1.0, 0.0, 0.0)),
    );
    let mut data = RagdollData::new();
    data.add_bone("root", None, Some(root), Transform::IDENTITY);
    data.add_bone(
        "child",
        Some(0),
        Some(child),
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0)),
    );
    let ragdoll = Ragdoll::create(world, &data).expect("ragdoll");
    let id = ragdoll.constraints()[1].expect("child joint");
    (ragdoll, child, id)
}

fn limits(world: &World, id: ConstraintId) -> (f32, f32, f32) {
    match world.constraint(id).unwrap() {
        WorldJoint::Ragdoll(info) => (info.cone_angle, info.twist_min, info.twist_max),
        _ => panic!("expected ragdoll joint"),
    }
}

fn rotated_child() -> Transform {
    Transform::from_quaternion(
        Quaternion::from_axis_angle(Vector4::UNIT_Z, core::f32::consts::FRAC_PI_3),
        Vector4::from_xyz(1.0, 0.0, 0.0),
    )
}

#[test]
fn e1_dynamic_state_loosens_then_restores_over_four_seconds() {
    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, child, id) = chain(&mut world);
    let original = limits(&world, id);
    world.set_body_transform(child, rotated_child());

    let mut controller = RagdollController::new(
        &mut world,
        ragdoll,
        RagdollControllerCinfo::default().with_initial_state(RagdollState::Dynamic),
    );
    let action = controller.easing().expect("loosen action");
    assert_eq!(action.duration(), DEFAULT_EASING_DURATION);
    let LimitSnapshot::Ragdoll(saved) = action.saved()[0] else {
        panic!("expected ragdoll limit snapshot");
    };
    assert_eq!(saved.cone_angle, original.0);
    let loose = limits(&world, id);
    assert!(loose.0 > original.0);

    let pose = [Transform::IDENTITY; 2];
    for _ in 0..4 {
        controller
            .step(&mut world, &pose, Transform::IDENTITY, 1.0)
            .unwrap();
    }
    assert!(controller.easing().is_none());
    assert_eq!(limits(&world, id), original);
}

#[test]
fn e2_entering_keyframed_restores_immediately() {
    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, child, id) = chain(&mut world);
    let original = limits(&world, id);
    world.set_body_transform(child, rotated_child());

    let mut controller = RagdollController::new(
        &mut world,
        ragdoll,
        RagdollControllerCinfo::default().with_initial_state(RagdollState::Dynamic),
    );
    assert!(limits(&world, id).0 > original.0);
    controller.set_state(&mut world, RagdollState::Keyframed);
    assert!(controller.easing().is_none());
    assert_eq!(limits(&world, id), original);
}

#[test]
fn e3_at_rest_transition_checks_tagged_body_velocities() {
    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, child, _) = chain(&mut world);
    let mut controller = RagdollController::new(
        &mut world,
        ragdoll,
        RagdollControllerCinfo::default().with_initial_state(RagdollState::Dynamic),
    );
    controller
        .begin_at_rest_transition(
            &mut world,
            RagdollState::Dynamic,
            RagdollState::Keyframed,
            1e-2,
            1e-1,
        )
        .unwrap();

    world.set_body_linear_velocity(child, Vec4::from_xyz(2.0, 0.0, 0.0));
    let pose = [Transform::IDENTITY; 2];
    controller
        .step(&mut world, &pose, Transform::IDENTITY, 1.0 / 60.0)
        .unwrap();
    assert_eq!(controller.state(), RagdollState::Dynamic);

    world.set_body_linear_velocity(child, Vec4::ZERO);
    world.set_body_angular_velocity(child, Vec4::ZERO);
    controller
        .step(&mut world, &pose, Transform::IDENTITY, 1.0 / 60.0)
        .unwrap();
    assert_eq!(controller.state(), RagdollState::Keyframed);

    assert_eq!(
        controller.begin_at_rest_transition(
            &mut world,
            RagdollState::Dynamic,
            RagdollState::Keyframed,
            -1.0,
            0.1,
        ),
        Err(RagdollError::InvalidTolerance)
    );
}
