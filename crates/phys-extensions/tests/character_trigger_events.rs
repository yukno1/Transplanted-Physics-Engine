//! World trigger event stream and its relation to query-based character triggers.

use std::sync::{Arc, Mutex};

use base_math::Vector4 as V4;
use phys_body::BodyCinfo;
use phys_extensions::{TriggerTracker, WorldTriggerStatus};
use phys_shape::{BoxShape, Shape};
use phys_types::MotionType;
use phys_world::{QueryFilter, World, WorldCinfo};

use std::sync::Arc as StdArc;

fn box_body(world: &mut World, position: V4) -> phys_types::BodyId {
    let shape = StdArc::new(BoxShape::with_convex_radius(V4::splat(0.5), 0.0)) as StdArc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(shape)
            .with_motion_type(MotionType::Static)
            .with_position(position),
    )
}

#[test]
fn world_trigger_events_report_enter_exit_and_updated() {
    let mut world = World::new(WorldCinfo::new().with_raise_trigger_updated_events(true));
    let trigger = box_body(&mut world, V4::ZERO);
    let other = box_body(&mut world, V4::from_xyz(0.4, 0.0, 0.0));
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
    let trigger = box_body(&mut world, V4::ZERO);
    let _other = box_body(&mut world, V4::from_xyz(0.4, 0.0, 0.0));
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
