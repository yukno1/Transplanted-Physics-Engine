//! Shape-key trigger granularity and query leaf key tests.

use std::sync::Arc;

use base_math::{Transform, Vector4 as V4};
use phys_body::BodyCinfo;
use phys_extensions::{TriggerEvent, TriggerObservation, TriggerStatus, TriggerTracker};
use phys_shape::{BoxShape, CompoundShape, Shape, ShapeInstance};
use phys_types::{BodyId, MotionType, ShapeKey};
use phys_world::{QueryFilter, World, WorldCinfo};

#[test]
fn trigger_tracker_distinguishes_shape_keys_on_one_body() {
    let body = BodyId::new(7);
    let key_a = ShapeKey::new(0x0100_0001);
    let key_b = ShapeKey::new(0x0100_0002);
    let mut tracker = TriggerTracker::new();
    let observation = |key, x| TriggerObservation {
        body,
        shape_key: key,
        position: V4::from_xyz(x, 0.0, 0.0),
        normal: V4::UNIT_Y,
    };

    assert_eq!(
        tracker.update_observations(&[observation(key_a, 0.0), observation(key_b, 1.0)]),
        vec![
            TriggerEvent {
                body,
                shape_key: key_a,
                status: TriggerStatus::Entered
            },
            TriggerEvent {
                body,
                shape_key: key_b,
                status: TriggerStatus::Entered
            },
        ]
    );
    assert_eq!(
        tracker.update_observations(&[observation(key_a, 0.5), observation(key_b, 1.0)]),
        vec![TriggerEvent {
            body,
            shape_key: key_a,
            status: TriggerStatus::Updated
        }]
    );
    assert_eq!(
        tracker.update_observations(&[observation(key_b, 1.0)]),
        vec![TriggerEvent {
            body,
            shape_key: key_a,
            status: TriggerStatus::Exited
        }]
    );
    assert_eq!(
        tracker.clear(),
        vec![TriggerEvent {
            body,
            shape_key: key_b,
            status: TriggerStatus::Exited
        }]
    );
}

#[test]
fn compound_ray_hits_carry_distinct_shape_keys() {
    let mut compound = CompoundShape::new(Vec::new());
    for x in [-2.0, 2.0] {
        let shape: Arc<dyn Shape> = Arc::new(BoxShape::with_convex_radius(V4::splat(0.5), 0.0));
        compound.add_child(ShapeInstance::new(
            Transform::from_quaternion(base_math::Quaternion::IDENTITY, V4::from_xyz(x, 0.0, 0.0)),
            shape,
        ));
    }
    let mut world = World::new(WorldCinfo::new());
    let body = world.create_body(
        &BodyCinfo::new(Arc::new(compound) as Arc<dyn Shape>).with_motion_type(MotionType::Static),
    );
    let filter = QueryFilter::default();
    let left = world
        .raycast_filtered(V4::from_xyz(-2.0, -5.0, 0.0), V4::UNIT_Y, 10.0, &filter)
        .expect("left ray hit");
    let right = world
        .raycast_filtered(V4::from_xyz(2.0, -5.0, 0.0), V4::UNIT_Y, 10.0, &filter)
        .expect("right ray hit");
    assert_eq!(left.body, body);
    assert_eq!(right.body, body);
    assert!(left.shape_key.is_valid());
    assert!(right.shape_key.is_valid());
    assert_ne!(left.shape_key, right.shape_key);
}
