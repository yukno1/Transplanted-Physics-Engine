//! World post-solve signal and manager callback ownership tests.

use std::sync::{Arc, Mutex};

use crate::base_math::Vector4 as V4;
use crate::base_math::Vec4Ext;
use glam::Vec4 as G4;
use crate::phys_extensions::{
    CharacterProxy, CharacterProxyCinfo, CharacterProxyListener, CharacterProxyManager,
    CharacterProxyState,
};
use crate::phys_shape::{CapsuleShape, Shape};
use crate::phys_world::{World, WorldCinfo};
use std::sync::Arc as StdArc;

const DT: f32 = 1.0 / 60.0;

fn proxy() -> CharacterProxy {
    let shape = StdArc::new(CapsuleShape::new(
        G4::from_xyz(0.0, -0.6, 0.0),
        G4::from_xyz(0.0, 0.6, 0.0),
        0.4,
    )) as StdArc<dyn Shape>;
    CharacterProxy::new(&CharacterProxyCinfo::new(shape).with_position(V4::ZERO))
}

struct StepRecorder {
    name: &'static str,
    log: Arc<Mutex<Vec<String>>>,
}

impl CharacterProxyListener for StepRecorder {
    fn character_callback(&mut self, state: &CharacterProxyState) {
        self.log.lock().unwrap().push(format!(
            "{}:{}:{}",
            self.name, state.id, state.contact_count
        ));
    }
}

#[test]
fn post_solve_signal_fires_in_subscription_order_and_unsubscribes() {
    let world = World::new(WorldCinfo::new());
    let log = Arc::new(Mutex::new(Vec::new()));
    let first = world.subscribe_post_solve({
        let log = Arc::clone(&log);
        move |event| log.lock().unwrap().push(format!("a:{}", event.step_count))
    });
    world.subscribe_post_solve({
        let log = Arc::clone(&log);
        move |event| log.lock().unwrap().push(format!("b:{}", event.step_count))
    });

    let mut world = world;
    world.step(DT);
    assert_eq!(*log.lock().unwrap(), vec!["a:1", "b:1"]);
    assert!(world.unsubscribe_post_solve(first));
    world.step(DT);
    assert_eq!(*log.lock().unwrap(), vec!["a:1", "b:1", "b:2"]);
    assert_eq!(world.post_solve_signal().num_subscriptions(), 1);
}

#[test]
fn manager_owns_post_solve_callback_without_duplicate_dispatch() {
    let mut world = World::new(WorldCinfo::new());
    let manager = CharacterProxyManager::new();
    let id = manager.add_proxy(proxy());
    let log = Arc::new(Mutex::new(Vec::new()));
    manager.with_proxy(id, |proxy| {
        proxy.add_listener(Box::new(StepRecorder {
            name: "step",
            log: Arc::clone(&log),
        }));
    });
    manager.attach(&world);
    assert_eq!(manager.connection_count(), 1);

    manager.step_characters(&mut world, DT, &[], 1);
    assert!(log.lock().unwrap().is_empty());
    world.step(DT);
    assert_eq!(*log.lock().unwrap(), vec!["step:1:0"]);

    manager.detach(&world);
    world.step(DT);
    assert_eq!(manager.connection_count(), 0);
    assert_eq!(*log.lock().unwrap(), vec!["step:1:0"]);
}
