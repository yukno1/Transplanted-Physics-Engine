//! Pre-solve Jacobian-like row views wired through the character bridge.

use std::sync::{Arc, Mutex};

use base_math::Vector4 as V4;
use phys_body::BodyCinfo;
use phys_extensions::{
    CharacterProxyManager, CharacterRigidBody, CharacterRigidBodyCinfo, CharacterRigidBodyListener,
    WorldSolverDataView,
};
use phys_shape::{BoxShape, Shape, SphereShape};
use phys_types::MotionType;
use phys_world::{World, WorldCinfo};

use std::sync::Arc as StdArc;

const DT: f32 = 1.0 / 60.0;

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

fn character_cinfo() -> CharacterRigidBodyCinfo {
    let shape = StdArc::new(SphereShape::new(V4::ZERO, 0.5)) as StdArc<dyn Shape>;
    CharacterRigidBodyCinfo::new(shape).with_position(V4::from_xyz(0.0, 0.45, 0.0))
}

struct SetFriction(f32);

impl CharacterRigidBodyListener for SetFriction {
    fn modify_solver_data(&mut self, view: &mut WorldSolverDataView<'_>) {
        *view.friction = self.0;
        *view.normal_impulse = 1.0;
        view.clamp_impulses();
    }
}

#[test]
fn manager_hook_edits_friction_and_unregisters_on_detach() {
    let mut world = world_with_ground();
    let frictions = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&frictions);
    world.subscribe_post_collide(move |event| {
        for point in &event.points {
            sink.lock().unwrap().push(point.friction);
        }
    });

    let manager = CharacterProxyManager::new();
    let character = CharacterRigidBody::new(&mut world, &character_cinfo());
    let id = manager.add_rigid_body(character);
    manager.with_rigid_body(id, |body| {
        body.set_listener(Box::new(SetFriction(0.9)));
    });
    manager.attach(&world);
    assert_eq!(world.solver_data_hook_count(), 1);
    assert_eq!(manager.contact_connection_count(), 3);

    for _ in 0..10 {
        manager.step_rigid_bodies(&mut world, DT, &[]);
        world.step(DT);
    }
    let observed = frictions.lock().unwrap().clone();
    assert!(!observed.is_empty());
    assert!(observed.iter().all(|friction| *friction == 0.9));

    manager.detach(&world);
    assert_eq!(world.solver_data_hook_count(), 0);
    assert_eq!(manager.contact_connection_count(), 0);
}

#[test]
fn solver_data_editing_is_bitwise_identical_across_thread_counts() {
    fn simulate(threads: usize) -> String {
        let mut world = world_with_ground();
        let manager = CharacterProxyManager::new();
        let character = CharacterRigidBody::new(&mut world, &character_cinfo());
        let id = manager.add_rigid_body(character);
        manager.with_rigid_body(id, |body| {
            body.set_listener(Box::new(SetFriction(0.75)));
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
