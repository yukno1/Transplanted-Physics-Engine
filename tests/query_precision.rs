use std::sync::{Arc, Mutex};

use proton::body::BodyCinfo;
use proton::collide::query::leaf_key;
use proton::collide::raycast;
use proton::extensions::{WorldTriggerEvent, hash_world};
use proton::math::{Transform, Vector4, Vec4, Vec4Ext};
use proton::np_types::{MotionType, ShapeKey};
use proton::shape::{
    CapsuleShape, CompoundShape, ConvexHullShape, CylinderShape, HeightFieldShape, MeshShape,
    Shape, ShapeInstance, SphereShape,
};
use proton::world::{World, WorldCinfo};

fn quad_mesh() -> Arc<dyn Shape> {
    Arc::new(MeshShape::new(
        vec![
            Vec4::from_xyz(-2.0, 0.0, -2.0),
            Vec4::from_xyz(2.0, 0.0, -2.0),
            Vec4::from_xyz(2.0, 0.0, 2.0),
            Vec4::from_xyz(-2.0, 0.0, 2.0),
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    )) as Arc<dyn Shape>
}

fn cube_points() -> Vec<Vec4> {
    vec![
        Vec4::from_xyz(-1.0, -1.0, -1.0),
        Vec4::from_xyz(1.0, -1.0, -1.0),
        Vec4::from_xyz(1.0, 1.0, -1.0),
        Vec4::from_xyz(-1.0, 1.0, -1.0),
        Vec4::from_xyz(-1.0, -1.0, 1.0),
        Vec4::from_xyz(1.0, -1.0, 1.0),
        Vec4::from_xyz(1.0, 1.0, 1.0),
        Vec4::from_xyz(-1.0, 1.0, 1.0),
    ]
}

#[test]
fn analytic_normals_are_exposed_through_the_facade() {
    let capsule: Arc<dyn Shape> = Arc::new(CapsuleShape::new(
        Vec4::from_xyz(0.0, 0.0, 0.0),
        Vec4::from_xyz(0.0, 2.0, 0.0),
        0.5,
    ));
    let hit = raycast(
        capsule.as_ref(),
        &Transform::IDENTITY,
        Vector4::from_xyz(5.0, 1.0, 0.0),
        -Vector4::UNIT_X,
        10.0,
    )
    .unwrap();
    assert!(hit.normal.all_equal3(Vec4::X, 1e-2));

    let cylinder: Arc<dyn Shape> = Arc::new(CylinderShape::with_params(
        Vec4::from_xyz(0.0, -1.0, 0.0),
        Vec4::from_xyz(0.0, 1.0, 0.0),
        1.0,
        16,
        0.0,
    ));
    let hit = raycast(
        cylinder.as_ref(),
        &Transform::IDENTITY,
        Vector4::from_xyz(5.0, 0.0, 0.0),
        -Vector4::UNIT_X,
        10.0,
    )
    .unwrap();
    assert!(hit.normal.all_equal3(Vec4::X, 1e-2));

    let hull: Arc<dyn Shape> = Arc::new(ConvexHullShape::from_points(cube_points(), 0.2));
    let hit = raycast(
        hull.as_ref(),
        &Transform::IDENTITY,
        Vector4::from_xyz(5.0, 0.0, 0.0),
        -Vector4::UNIT_X,
        10.0,
    )
    .unwrap();
    assert!(hit.normal.all_equal3(Vec4::X, 1e-2));
}

fn trigger_world_with(
    shape: Arc<dyn Shape>,
    trigger_position: Vec4,
) -> (World, Vec<WorldTriggerEvent>) {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vector4::ZERO);
    let trigger = world.create_body(
        &BodyCinfo::new(Arc::new(SphereShape::new(Vec4::ZERO, 0.3)) as Arc<dyn Shape>)
            .with_motion_type(MotionType::Static)
            .with_position(trigger_position),
    );
    let other = world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Static)
            .with_position(Vec4::ZERO),
    );
    assert!(trigger < other);
    world.set_body_trigger(trigger, true);

    let events = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&events);
    world.subscribe_trigger_events(move |event| sink.lock().unwrap().push(*event));
    world.step(0.0);
    let recorded = events.lock().unwrap().clone();
    (world, recorded)
}

#[test]
fn world_trigger_emits_mesh_triangle_leaf_key() {
    let (_, events) = trigger_world_with(quad_mesh(), Vec4::from_xyz(1.5, 0.0, -1.5));
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].shape_key_b, leaf_key::new(leaf_key::MESH, 0));
}

#[test]
fn world_trigger_emits_height_field_cell_leaf_key() {
    let heightfield: Arc<dyn Shape> =
        Arc::new(HeightFieldShape::new(4, 4, Vec4::ONE, vec![0.0; 16]).unwrap());
    let (_, events) = trigger_world_with(heightfield, Vec4::from_xyz(1.5, 0.0, 1.5));
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].shape_key_b, leaf_key::height_field(1, 1, 0));
}

#[test]
fn world_trigger_emits_compound_child_leaf_key() {
    let compound: Arc<dyn Shape> = Arc::new(CompoundShape::new(vec![
        ShapeInstance::new(
            Transform::IDENTITY,
            Arc::new(SphereShape::new(Vec4::ZERO, 1.0)),
        ),
        ShapeInstance::new(
            Transform::from_quaternion(
                proton::math::Quaternion::IDENTITY,
                Vector4::from_xyz(5.0, 0.0, 0.0),
            ),
            Arc::new(SphereShape::new(Vec4::ZERO, 1.0)),
        ),
    ]));
    let (_, events) = trigger_world_with(compound, Vec4::from_xyz(0.9, 0.0, 0.0));
    assert_eq!(events.len(), 1);
    assert_eq!(
        events[0].shape_key_b,
        leaf_key::compound_child(0, ShapeKey::INVALID)
    );
}

#[test]
fn leaf_key_trigger_scene_is_bitwise_deterministic() {
    let first = trigger_world_with(quad_mesh(), Vec4::from_xyz(1.5, 0.0, -1.5)).0;
    let second = trigger_world_with(quad_mesh(), Vec4::from_xyz(1.5, 0.0, -1.5)).0;
    assert_eq!(hash_world(&first), hash_world(&second));
}
