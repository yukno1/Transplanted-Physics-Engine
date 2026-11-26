//! World trigger event stream and its relation to query-based character triggers.

use std::sync::{Arc, Mutex};

use crate::base_math::Vec4Ext;
use crate::base_math::Vector4 as V4;
use glam::Vec4 as G4;
use crate::phys_body::BodyCinfo;
use crate::phys_collide::query::leaf_key;
use crate::phys_extensions::{TriggerTracker, WorldTriggerEvent, WorldTriggerStatus};
use crate::phys_shape::{BoxShape, Shape};
use crate::phys_types::MotionType;
use crate::phys_world::{QueryFilter, World, WorldCinfo};

use std::sync::Arc as StdArc;

fn box_body(world: &mut World, position: G4) -> crate::phys_types::BodyId {
    let shape = StdArc::new(BoxShape::with_convex_radius(G4::splat(0.5), 0.0)) as StdArc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Static)
            .with_position(position),
    )
}

#[test]
fn world_trigger_events_report_enter_exit_and_updated() {
    let mut world = World::new(WorldCinfo::new().with_raise_trigger_updated_events(true));
    let trigger = box_body(&mut world, G4::ZERO);
    let other = box_body(&mut world, G4::from_xyz(0.4, 0.0, 0.0));
    assert!(world.set_body_trigger(trigger, true));

    let log = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&log);
    world.subscribe_trigger_events(move |event| {
        sink.lock().unwrap().push(event.status);
    });

    world.step(0.0);
    world.set_body_position(other, V4::from_xyz(8.0, 0.0, 0.0));
    world.step(0.0);
    assert_eq!(
        *log.lock().unwrap(),
        vec![WorldTriggerStatus::Entered, WorldTriggerStatus::Exited]
    );

    // Same-step enter + exit is merged into Updated only when enabled.
    world.set_body_position(other, V4::from_xyz(7.0, 0.0, 0.0));
    assert!(world.record_trigger_transition(trigger, other, true));
    assert!(world.record_trigger_transition(other, trigger, false));
    world.step(0.0);
    assert_eq!(
        *log.lock().unwrap(),
        vec![
            WorldTriggerStatus::Entered,
            WorldTriggerStatus::Exited,
            WorldTriggerStatus::Updated,
        ]
    );
}

#[test]
fn world_and_query_trigger_streams_do_not_dispatch_each_other() {
    let mut world = World::new(WorldCinfo::new());
    let trigger = box_body(&mut world, G4::ZERO);
    let _other = box_body(&mut world, G4::from_xyz(0.4, 0.0, 0.0));
    world.set_body_trigger(trigger, true);

    let world_events = Arc::new(Mutex::new(0usize));
    let sink = Arc::clone(&world_events);
    world.subscribe_trigger_events(move |_| *sink.lock().unwrap() += 1);
    world.step(0.0);

    let mut tracker = TriggerTracker::new();
    let query_events = tracker.update(&world, V4::ZERO, 3.0, &QueryFilter::default());
    // The world emits one pair-level event; the query tracker independently
    // reports both overlapping bodies and is never fed by the world signal.
    assert_eq!(*world_events.lock().unwrap(), 1);
    assert_eq!(query_events.len(), 2);
    assert!(world.is_body_trigger(trigger));
}

#[test]
fn explicit_transitions_carry_leaf_shape_keys_and_canonicalise_bodies() {
    let mut world = World::new(WorldCinfo::new());
    let low = box_body(&mut world, G4::from_xyz(0.0, 0.0, 0.0));
    let high = box_body(&mut world, G4::from_xyz(8.0, 0.0, 0.0));
    assert!(low < high);
    let low_key = leaf_key::new(leaf_key::CONVEX_HULL_FACE, 2);
    let high_key = leaf_key::new(leaf_key::CONVEX_HULL_FACE, 5);

    let events: Arc<Mutex<Vec<WorldTriggerEvent>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&events);
    world.subscribe_trigger_events(move |event| sink.lock().unwrap().push(*event));

    // Pass bodies in reverse order: keys must be swapped with them.
    assert!(world.record_trigger_transition_with_keys(high, low, high_key, low_key, true));
    world.step(0.0);
    let recorded = events.lock().unwrap().clone();
    assert_eq!(recorded.len(), 1);
    assert_eq!(recorded[0].body_a, low);
    assert_eq!(recorded[0].body_b, high);
    assert_eq!(recorded[0].shape_key_a, low_key);
    assert_eq!(recorded[0].shape_key_b, high_key);
    assert_eq!(recorded[0].status, WorldTriggerStatus::Entered);
}
