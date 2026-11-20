//! Ragdoll penetration utility integration scenarios.
//!
//! Exercises root-embedded detection, hierarchy rays with ancestor status,
//! DONT_COLLIDE masking and controller-driven resolution through the facade.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::{
    BodyHierarchy, DetectionOutput, PenetrationInput, PenetrationStatus, Ragdoll,
    RagdollController, RagdollControllerCinfo, RagdollData, RagdollState,
    detect_and_resolve_penetration, detect_penetration,
};
use proton::math::{Transform, Vector4};
use proton::np_types::{BodyId, CollisionFlags, MotionType};
use proton::shape::{BoxShape, Shape, SphereShape};
use proton::world::{World, WorldCinfo};

fn ground(world: &mut World) {
    let shape = Arc::new(BoxShape::with_convex_radius(
        Vector4::from_xyz(2.0, 0.5, 2.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Static)
            .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
    );
}

fn dynamic_sphere(world: &mut World, position: Vector4) -> BodyId {
    let shape = Arc::new(SphereShape::new(Vector4::ZERO, 0.25)) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(position),
    )
}

fn single_bone_ragdoll(world: &mut World, body: BodyId) -> Ragdoll {
    let mut data = RagdollData::new();
    data.add_bone("root", None, Some(body), Transform::IDENTITY);
    Ragdoll::create(world, &data).expect("ragdoll")
}

#[test]
fn p1_detects_root_embedded_in_ground_and_pushes_it_out() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    ground(&mut world);
    let root = dynamic_sphere(&mut world, Vector4::from_xyz(0.0, 0.2, 0.0));
    let child = dynamic_sphere(&mut world, Vector4::from_xyz(0.0, 1.0, 0.0));
    let mut data = RagdollData::new();
    data.add_bone("root", None, Some(root), Transform::IDENTITY);
    data.add_bone(
        "child",
        Some(0),
        Some(child),
        Transform::from_quaternion(
            proton::math::Quaternion::IDENTITY,
            Vector4::from_xyz(0.0, 0.8, 0.0),
        ),
    );
    let ragdoll = Ragdoll::create(&mut world, &data).expect("ragdoll");
    let hierarchy = BodyHierarchy::from_ragdoll(&world, &ragdoll, None);
    let input = PenetrationInput::default().with_resolution_vector(Vector4::UNIT_Y);
    let mut output = DetectionOutput::default();

    assert!(detect_penetration(&world, &hierarchy, &input, &mut output));
    assert!(output.has_root_penetration);
    let penetration = output.root_penetration.expect("root penetration");
    assert!(penetration.normal.y > 0.0);

    assert!(!detect_and_resolve_penetration(
        &mut world,
        &hierarchy,
        &input,
        1.0 / 60.0,
        &mut output,
    ));
    let velocity = world.body_linear_velocity(root).expect("root velocity");
    assert!(
        velocity.y > 0.0,
        "expected upward corrective velocity, got {velocity:?}"
    );
    assert!(
        world
            .body(root)
            .unwrap()
            .flags
            .contains(CollisionFlags::DONT_COLLIDE)
    );
    hierarchy.reenable_collisions(&mut world);
    assert!(
        !world
            .body(root)
            .unwrap()
            .flags
            .contains(CollisionFlags::DONT_COLLIDE)
    );
}

#[test]
fn p2_clean_scene_reports_resolved() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    ground(&mut world);
    let root = dynamic_sphere(&mut world, Vector4::from_xyz(0.0, 2.0, 0.0));
    let ragdoll = single_bone_ragdoll(&mut world, root);
    let hierarchy = BodyHierarchy::from_ragdoll(&world, &ragdoll, None);
    let input = PenetrationInput::default().with_resolution_vector(Vector4::UNIT_Y);
    let mut output = DetectionOutput::default();

    assert!(!detect_penetration(&world, &hierarchy, &input, &mut output));
    assert!(detect_and_resolve_penetration(
        &mut world,
        &hierarchy,
        &input,
        1.0 / 60.0,
        &mut output,
    ));
    assert_eq!(world.body_linear_velocity(root), Some(Vector4::ZERO));
}

#[test]
fn p3_hierarchy_ray_flags_parent_and_ancestor() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    ground(&mut world);
    let parent = dynamic_sphere(&mut world, Vector4::from_xyz(0.0, 1.0, 0.0));
    let child = dynamic_sphere(&mut world, Vector4::from_xyz(0.0, -1.0, 0.0));
    let mut data = RagdollData::new();
    data.add_bone("parent", None, Some(parent), Transform::IDENTITY);
    data.add_bone(
        "child",
        Some(0),
        Some(child),
        Transform::from_quaternion(
            proton::math::Quaternion::IDENTITY,
            Vector4::from_xyz(0.0, -2.0, 0.0),
        ),
    );
    let ragdoll = Ragdoll::create(&mut world, &data).expect("ragdoll");
    let hierarchy = BodyHierarchy::from_ragdoll(&world, &ragdoll, None);
    let input = PenetrationInput::default().with_resolution_vector(Vector4::UNIT_Y);
    let mut output = DetectionOutput::default();

    assert!(detect_penetration(&world, &hierarchy, &input, &mut output));
    assert!(!output.has_root_penetration);
    assert!(!output.penetrations.is_empty());
    assert_eq!(output.penetrations[0].penetrating_node, 0);
    assert!(output.body_status[0].contains(PenetrationStatus::RAY));
    assert!(output.body_status[1].contains(PenetrationStatus::ANCESTOR));

    assert!(!detect_and_resolve_penetration(
        &mut world,
        &hierarchy,
        &input,
        1.0 / 60.0,
        &mut output,
    ));
    assert!(world.body_linear_velocity(parent).unwrap().y > 0.0);
}

#[test]
fn p4_controller_resolves_penetration_before_state_step() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    ground(&mut world);
    let root = dynamic_sphere(&mut world, Vector4::from_xyz(0.0, 0.2, 0.0));
    let ragdoll = single_bone_ragdoll(&mut world, root);
    let mut controller = RagdollController::new(
        &mut world,
        ragdoll,
        RagdollControllerCinfo::default().with_initial_state(RagdollState::Dynamic),
    );
    controller.set_penetration_input(Some(
        PenetrationInput::default().with_resolution_vector(Vector4::UNIT_Y),
    ));

    controller
        .step(
            &mut world,
            &[Transform::IDENTITY],
            Transform::IDENTITY,
            1.0 / 60.0,
        )
        .unwrap();

    assert!(controller.last_penetration().has_root_penetration);
    assert!(world.body_linear_velocity(root).unwrap().y > 0.0);
}
