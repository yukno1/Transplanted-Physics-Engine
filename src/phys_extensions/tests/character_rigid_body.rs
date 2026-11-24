//! Rigid-body character route and listener entry points.

use std::sync::{Arc, Mutex};

use crate::base_math::Vector4 as V4;
use crate::phys_body::BodyCinfo;
use crate::phys_extensions::{
    CharacterProxyManager, CharacterRigidBody, CharacterRigidBodyCinfo, CharacterRigidBodyListener,
    CharacterRigidBodyState, ContactPointInfo, SurfaceInfo,
};
use crate::phys_shape::{BoxShape, Shape, SphereShape};
use crate::phys_types::MotionType;
use crate::phys_world::{World, WorldCinfo};
use std::sync::Arc as StdArc;

const DT: f32 = 1.0 / 60.0;

#[derive(Default)]
struct Events(Arc<Mutex<Vec<String>>>);

impl CharacterRigidBodyListener for Events {
    fn filter_contact_point(&mut self, point: &mut ContactPointInfo) -> bool {
        self.0
            .lock()
            .unwrap()
            .push(format!("filter:{}", point.body.value()));
        point.distance.is_finite()
    }

    fn on_post_collide(&mut self, points: &[ContactPointInfo]) {
        self.0
            .lock()
            .unwrap()
            .push(format!("collide:{}", points.len()));
    }

    fn on_post_solve(&mut self, state: &CharacterRigidBodyState) {
        self.0
            .lock()
            .unwrap()
            .push(format!("solve:{}:{}", state.body.value(), state.supported));
    }
}

fn world_with_ground() -> World {
    let mut world = World::new(WorldCinfo::new());
    let ground = StdArc::new(BoxShape::with_convex_radius(
        V4::from_xyz(10.0, 0.5, 10.0),
        0.0,
    )) as StdArc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(ground)
            .with_motion_type(MotionType::Static)
            .with_position(V4::from_xyz(0.0, -0.5, 0.0)),
    );
    world
}

fn cinfo() -> CharacterRigidBodyCinfo {
    let shape = StdArc::new(SphereShape::new(V4::ZERO, 0.5)) as StdArc<dyn Shape>;
    CharacterRigidBodyCinfo::new(shape).with_position(V4::from_xyz(0.0, 0.45, 0.0))
}

#[test]
fn rigid_body_listener_receives_all_three_entries() {
    let mut world = world_with_ground();
    let manager = CharacterProxyManager::new();
    let character = CharacterRigidBody::new(&mut world, &cinfo());
    let id = manager.add_rigid_body(character);
    let events = Events::default();
    let log = Arc::clone(&events.0);
    manager.with_rigid_body(id, |character| character.set_listener(Box::new(events)));
    manager.attach(&world);

    let surface = manager
        .with_rigid_body(id, |character| {
            character.check_support(&world, -V4::UNIT_Y, DT)
        })
        .expect("rigid body");
    assert!(surface.supported_state == crate::phys_extensions::SupportedState::Supported);
    manager.step_rigid_bodies(&mut world, DT, &[(id, surface)]);
    world.step(DT);

    let events = log.lock().unwrap();
    assert!(events.iter().any(|event| event.starts_with("filter:")));
    assert!(events.iter().any(|event| event.starts_with("collide:")));
    assert!(events.iter().any(|event| event.starts_with("solve:")));
}

#[test]
#[should_panic(expected = "invalid character mass")]
fn rigid_body_rejects_non_positive_mass() {
    let mut world = world_with_ground();
    let _ = CharacterRigidBody::new(&mut world, &cinfo().with_mass(0.0));
}

#[test]
fn rigid_body_step_is_deterministic() {
    let mut first = world_with_ground();
    let mut second = world_with_ground();
    let mut first_body = CharacterRigidBody::new(&mut first, &cinfo());
    let mut second_body = CharacterRigidBody::new(&mut second, &cinfo());
    let first_surface = first_body.check_support(&first, -V4::UNIT_Y, DT);
    let second_surface = second_body.check_support(&second, -V4::UNIT_Y, DT);
    first_body.step(&mut first, DT, &first_surface);
    second_body.step(&mut second, DT, &second_surface);
    first.step(DT);
    second.step(DT);
    assert_eq!(first.snapshot().to_text(), second.snapshot().to_text());
    let _ = SurfaceInfo::UNSUPPORTED;
}
