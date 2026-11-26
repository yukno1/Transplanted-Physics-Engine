//! Ragdoll scheduling-layer integration scenarios.
//!
//! Exercises the facade: bone hierarchy creation from existing bodies,
//! adjacent-body collision masking, drive-to-pose velocity control and hard
//! keyframes, plus the usual deterministic-trace guarantee.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::{Ragdoll, RagdollData, RagdollError};
use proton::math::{Quaternion, Transform, Vector4, Vec4, Vec4Ext};
use proton::np_types::{BodyId, MotionType};
use proton::shape::{BoxShape, Shape};
use proton::world::{JointType, World, WorldCinfo};

fn body(world: &mut World, x: f32, y: f32) -> BodyId {
    let shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(0.2, 0.2, 0.2),
        0.0,
    )) as Arc<dyn Shape>;
    let id = world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(Vec4::from_xyz(x, y, 0.0)),
    );
    let motion_id = world.body(id).unwrap().motion_id;
    world.motion_mut(motion_id).unwrap().inverse_inertia = Vec4::splat(0.5);
    id
}

fn three_bone_chain(world: &mut World) -> (Ragdoll, [BodyId; 3]) {
    let ids = [
        body(world, 0.0, 0.0),
        body(world, 0.0, 1.0),
        body(world, 0.0, 2.0),
    ];
    let mut data = RagdollData::new();
    data.add_bone("hips", None, Some(ids[0]), Transform::IDENTITY);
    data.add_bone(
        "spine",
        Some(0),
        Some(ids[1]),
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 1.0, 0.0)),
    );
    data.add_bone(
        "head",
        Some(1),
        Some(ids[2]),
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 1.0, 0.0)),
    );
    (Ragdoll::create(world, &data).expect("ragdoll"), ids)
}

fn target_pose() -> [Transform; 3] {
    let offset = |x: f32, y: f32| {
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(x, y, 0.0))
    };
    // Parent-relative poses: the head ends up at (2, 2).
    [offset(2.0, 0.0), offset(0.0, 1.0), offset(0.0, 1.0)]
}

#[test]
fn r1_creates_joint_chain_with_adjacent_collisions_disabled() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vec4::ZERO);
    let (ragdoll, ids) = three_bone_chain(&mut world);

    assert_eq!(ragdoll.constraints().len(), 3);
    assert!(ragdoll.constraints()[0].is_none());
    let spine_joint = ragdoll.constraints()[1].expect("spine joint");
    assert_eq!(
        world.constraint(spine_joint).unwrap().joint_type(),
        JointType::Ragdoll
    );
    assert_eq!(ragdoll.bones()[2].parent, Some(1));
    assert_eq!(ragdoll.bones()[2].body, Some(ids[2]));
}

#[test]
fn r2_drive_to_pose_moves_the_chain() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vec4::ZERO);
    let (ragdoll, ids) = three_bone_chain(&mut world);
    let target = target_pose();
    let dt = 1.0 / 60.0;

    for _ in 0..90 {
        ragdoll
            .drive_to_pose(&mut world, &target, Transform::IDENTITY, dt, 1.0, &[])
            .expect("drive");
        world.step(dt);
    }

    let head = world.body_position(ids[2]).expect("head body");
    assert!(
        (head.x - 2.0).abs() < 0.35 && (head.y - 2.0).abs() < 0.35,
        "head should follow the target pose: {head:?}"
    );
    assert!(
        world.body_linear_velocity(ids[2]).unwrap().length3() < 1.0,
        "driven chain must settle near the target"
    );
}

#[test]
fn r3_hard_keyframe_teleports_and_sets_required_velocity() {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vec4::ZERO);
    let (ragdoll, ids) = three_bone_chain(&mut world);
    let target = target_pose();
    let dt = 1.0 / 60.0;

    ragdoll
        .apply_hard_keyframe(&mut world, &target, Transform::IDENTITY, dt)
        .expect("keyframe");
    assert_eq!(
        world.body_position(ids[2]).unwrap(),
        Vec4::from_xyz(2.0, 2.0, 0.0)
    );
    let velocity = world.body_linear_velocity(ids[2]).unwrap();
    assert!((velocity.x - 120.0).abs() < 1e-3, "{velocity:?}");
}

#[test]
fn r4_driving_is_deterministic_and_validates_inputs() {
    let mut worlds = Vec::new();
    for _ in 0..2 {
        let mut world = World::new(WorldCinfo::new());
        world.set_gravity(Vec4::ZERO);
        let (ragdoll, _) = three_bone_chain(&mut world);
        let target = target_pose();
        for _ in 0..30 {
            ragdoll
                .drive_to_pose(
                    &mut world,
                    &target,
                    Transform::IDENTITY,
                    1.0 / 60.0,
                    0.8,
                    &[],
                )
                .expect("drive");
            world.step(1.0 / 60.0);
        }
        worlds.push(world.snapshot().to_text());
    }
    assert_eq!(worlds[0], worlds[1]);

    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, _) = three_bone_chain(&mut world);
    assert_eq!(
        ragdoll.drive_to_pose(
            &mut world,
            &target_pose(),
            Transform::IDENTITY,
            0.0,
            1.0,
            &[],
        ),
        Err(RagdollError::InvalidDeltaTime)
    );
    assert_eq!(
        ragdoll.drive_to_pose(
            &mut world,
            &target_pose(),
            Transform::IDENTITY,
            1.0 / 60.0,
            1.5,
            &[],
        ),
        Err(RagdollError::InvalidStrength)
    );
}
