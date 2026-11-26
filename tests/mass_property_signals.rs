//! Mass-property change signals and shared-motion invalidation.

use std::sync::{Arc, Mutex};

use proton::body::BodyCinfo;
use proton::math::{Vec4, Vec4Ext};
use proton::np_types::MotionType;
use proton::shape::{MassConfig, Shape, SphereShape};
use proton::world::{MassChangeEvent, MassChangeReason, MotionMassUpdateMode, World, WorldCinfo};

fn body(world: &mut World, radius: f32, position: Vec4) -> proton::np_types::BodyId {
    let shape = Arc::new(SphereShape::new(Vec4::ZERO, radius)) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_mass_config(MassConfig::from_density(1.0))
            .with_position(position),
    )
}

fn recorder(world: &World) -> Arc<Mutex<Vec<MassChangeEvent>>> {
    let events = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&events);
    world.subscribe_mass_changes(move |event| sink.lock().unwrap().push(*event));
    events
}

#[test]
fn s1_shape_replacement_fires_post_rebuild_event() {
    let mut world = World::new(WorldCinfo::new());
    let id = body(&mut world, 0.5, Vec4::ZERO);
    let events = recorder(&world);
    let old_mass = world.body(id).unwrap().mass;

    let replacement = Arc::new(SphereShape::new(Vec4::ZERO, 1.0)) as Arc<dyn Shape>;
    assert!(world.set_body_shape(id, replacement));
    let emitted = events.lock().unwrap().clone();
    assert_eq!(emitted.len(), 1);
    assert_eq!(emitted[0].body, id);
    assert_eq!(emitted[0].reason, MassChangeReason::ShapeReplaced);
    assert!(emitted[0].new.unwrap().mass > old_mass);
    assert_eq!(world.body(id).unwrap().mass, emitted[0].new.unwrap().mass);
}

#[test]
fn s2_mass_config_change_rescales_and_notifies() {
    let mut world = World::new(WorldCinfo::new());
    let id = body(&mut world, 0.5, Vec4::ZERO);
    let events = recorder(&world);
    let old_mass = world.body(id).unwrap().mass;

    assert!(world.set_body_mass_config(id, MassConfig::from_density(2.0)));
    let emitted = events.lock().unwrap().clone();
    assert_eq!(emitted.len(), 1);
    assert_eq!(emitted[0].reason, MassChangeReason::MassConfigChanged);
    assert!((world.body(id).unwrap().mass - 2.0 * old_mass).abs() < 1e-5);
}

#[test]
fn s3_unsubscribe_stops_future_events() {
    let mut world = World::new(WorldCinfo::new());
    let id = body(&mut world, 0.5, Vec4::ZERO);
    let events = recorder(&world);
    let connection = world.subscribe_mass_changes(move |event| {
        let _ = event;
    });
    assert_eq!(world.mass_signal().num_subscriptions(), 2);
    assert!(world.unsubscribe_mass_change(connection));
    assert!(world.set_body_mass_config(id, MassConfig::from_density(2.0)));
    assert_eq!(events.lock().unwrap().len(), 1);
}

#[test]
fn s4_shape_change_rebuilds_shared_motion() {
    let mut world = World::new(WorldCinfo::new());
    let target = body(&mut world, 0.5, Vec4::ZERO);
    let child = body(&mut world, 0.5, Vec4::from_xyz(1.0, 0.0, 0.0));
    let motion = world.body_motion_id(target).unwrap();
    assert!(world.attach_bodies_with_mode(target, &[child], MotionMassUpdateMode::Rebuild));
    let before = world.motion(motion).unwrap().mass;

    let replacement = Arc::new(SphereShape::new(Vec4::ZERO, 1.0)) as Arc<dyn Shape>;
    assert!(world.set_body_shape(child, replacement));
    let after = world.motion(motion).unwrap().mass;
    assert!(after > before);
}
