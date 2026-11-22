//! Determinism of batched character stepping across thread counts.

use std::sync::{Arc, Mutex};

use base_math::Vector4 as V4;
use phys_body::BodyCinfo;
use phys_extensions::{
    CharacterProxy, CharacterProxyCinfo, CharacterProxyListener, CharacterProxyManager,
    CharacterProxyState,
};
use phys_shape::{CapsuleShape, Shape};
use phys_world::{World, WorldCinfo};
use std::sync::Arc as StdArc;

const DT: f32 = 1.0 / 60.0;

struct Recorder {
    log: Arc<Mutex<Vec<String>>>,
}

impl CharacterProxyListener for Recorder {
    fn character_callback(&mut self, state: &CharacterProxyState) {
        self.log
            .lock()
            .unwrap()
            .push(format!("step:{}:{}", state.id, state.contact_count));
    }
}

fn scene() -> (World, CharacterProxyManager, Arc<Mutex<Vec<String>>>) {
    let mut world = World::new(WorldCinfo::new());
    let dynamic = StdArc::new(phys_shape::SphereShape::new(V4::ZERO, 0.4)) as StdArc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(dynamic)
            .with_mass(1.0)
            .with_position(V4::from_xyz(0.5, 0.0, 0.0)),
    );

    let manager = CharacterProxyManager::new();
    let log = Arc::new(Mutex::new(Vec::new()));
    for x in [0.0, 0.3, 0.6] {
        let shape = StdArc::new(CapsuleShape::new(
            V4::from_xyz(0.0, -0.6, 0.0),
            V4::from_xyz(0.0, 0.6, 0.0),
            0.4,
        )) as StdArc<dyn Shape>;
        let id = manager.add_proxy(CharacterProxy::new(
            &CharacterProxyCinfo::new(shape).with_position(V4::from_xyz(x, 0.0, 0.0)),
        ));
        manager.with_proxy(id, |proxy| {
            proxy.add_listener(Box::new(Recorder {
                log: Arc::clone(&log),
            }));
        });
    }
    manager.attach(&world);
    (world, manager, log)
}

#[test]
fn character_steps_are_bit_identical_for_one_and_four_threads() {
    let (mut single, single_manager, single_log) = scene();
    let (mut parallel, parallel_manager, parallel_log) = scene();

    single_manager.step_characters(&mut single, DT, &[], 1);
    parallel_manager.step_characters(&mut parallel, DT, &[], 4);
    single.step(DT);
    parallel.step(DT);

    assert_eq!(single.snapshot().to_text(), parallel.snapshot().to_text());
    assert_eq!(*single_log.lock().unwrap(), *parallel_log.lock().unwrap());
    assert_eq!(single_log.lock().unwrap().len(), 3);
}
