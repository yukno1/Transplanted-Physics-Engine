//! World contact filtering wired through `CharacterRigidBodyListener`.

use std::sync::{Arc, Mutex};

use base_math::Vector4 as V4;
use phys_body::BodyCinfo;
use phys_extensions::{
    CharacterProxyManager, CharacterRigidBody, CharacterRigidBodyCinfo, CharacterRigidBodyListener,
    CharacterRigidBodyState, ContactPointInfo,
};
use phys_shape::{BoxShape, Shape, SphereShape};
use phys_types::MotionType;
use phys_world::{World, WorldCinfo};

const DT: f32 = 1.0 / 60.0;

#[derive(Clone)]
struct DropAndLog {
    log: Arc<Mutex<Vec<String>>>,
    drop_points: bool,
}

impl CharacterRigidBodyListener for DropAndLog {
    fn filter_contact_point(&mut self, _point: &mut ContactPointInfo) -> bool {
        self.log.lock().unwrap().push("filter".to_string());
        !self.drop_points
    }

    fn on_post_collide(&mut self, points: &[ContactPointInfo]) {
        self.log
            .lock()
            .unwrap()
            .push(format!("collide:{}", points.len()));
    }

    fn on_post_solve(&mut self, state: &CharacterRigidBodyState) {
        self.log
            .lock()
            .unwrap()
            .push(format!("solve:{}", state.body.value()));
    }
}

fn world_with_ground() -> (World, phys_types::BodyId) {
    let mut world = World::new(WorldCinfo::new());
    let ground = Arc::new(BoxShape::with_convex_radius(
        V4::from_xyz(10.0, 0.5, 10.0),
        0.0,
    )) as Arc<dyn Shape>;
    let ground = world.create_body(
        &BodyCinfo::new(ground)
            .with_motion_type(MotionType::Static)
            .with_position(V4::from_xyz(0.0, -0.5, 0.0)),
    );
    (world, ground)
}

fn character_cinfo() -> CharacterRigidBodyCinfo {
    let shape = Arc::new(SphereShape::new(V4::ZERO, 0.5)) as Arc<dyn Shape>;
    CharacterRigidBodyCinfo::new(shape).with_position(V4::from_xyz(0.0, 0.45, 0.0))
}

#[test]
fn managed_listener_drops_solver_contacts_and_sees_post_collide() {
    let (mut world, _) = world_with_ground();
    let dropped = Arc::new(Mutex::new(0usize));
    let sink = Arc::clone(&dropped);
    world.subscribe_post_collide(move |event| {
        *sink.lock().unwrap() += event.dropped_point_count;
    });

    let manager = CharacterProxyManager::new();
    let character = CharacterRigidBody::new(&mut world, &character_cinfo());
    let id = manager.add_rigid_body(character);
    let log = Arc::new(Mutex::new(Vec::new()));
    manager.with_rigid_body(id, |body| {
        body.set_listener(Box::new(DropAndLog {
            log: Arc::clone(&log),
            drop_points: true,
        }));
    });
    manager.attach(&world);
    assert_eq!(world.contact_modifier_count(), 1);
    assert_eq!(manager.contact_connection_count(), 3);

    for _ in 0..20 {
        manager.step_rigid_bodies(&mut world, DT, &[]);
        world.step(DT);
    }
    assert_eq!(world.stats().last_contact_count, 0);
    assert!(*dropped.lock().unwrap() > 0);
    assert!(
        log.lock()
            .unwrap()
            .iter()
            .any(|entry| entry.starts_with("collide:"))
    );

    manager.detach(&world);
    assert_eq!(world.contact_modifier_count(), 0);
    assert_eq!(manager.contact_connection_count(), 0);
    assert!(manager.remove_rigid_body(id).is_some());
}

#[test]
fn removing_rigid_body_unregisters_its_contact_bridge_entry() {
    let (mut world, _) = world_with_ground();
    let manager = CharacterProxyManager::new();
    let character = CharacterRigidBody::new(&mut world, &character_cinfo());
    let id = manager.add_rigid_body(character);
    manager.attach(&world);
    assert_eq!(world.contact_modifier_count(), 1);
    assert!(manager.remove_rigid_body(id).is_some());
    assert!(manager.remove_rigid_body(id).is_none());
    manager.detach(&world);
    assert_eq!(world.contact_modifier_count(), 0);
}

#[test]
fn contact_editing_is_bitwise_identical_across_thread_counts() {
    fn simulate(threads: usize) -> String {
        let (mut world, _) = world_with_ground();
        let manager = CharacterProxyManager::new();
        let character = CharacterRigidBody::new(&mut world, &character_cinfo());
        let id = manager.add_rigid_body(character);
        let log = Arc::new(Mutex::new(Vec::new()));
        manager.with_rigid_body(id, |body| {
            body.set_listener(Box::new(DropAndLog {
                log: Arc::clone(&log),
                drop_points: false,
            }));
        });
        manager.attach(&world);
        for _ in 0..60 {
            manager.step_rigid_bodies(&mut world, DT, &[]);
            world.step_with_threads(DT, threads);
        }
        manager.detach(&world);
        world.snapshot().to_text()
    }

    assert_eq!(simulate(1), simulate(4));
}
