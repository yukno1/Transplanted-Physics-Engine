//! Character object/character interaction callback tests.

use std::sync::{Arc, Mutex};

use crate::base_math::Vector4 as V4;
use crate::base_math::Vec4Ext;
use glam::Vec4 as G4;
use crate::phys_body::BodyCinfo;
use crate::phys_extensions::{
    CharacterContact as Contact, CharacterInteractionInfo, CharacterObjectInteractionEvent,
    CharacterObjectInteractionResult, CharacterProxy, CharacterProxyCinfo, CharacterProxyListener,
    CharacterProxyManager, SurfaceInfo,
};
use crate::phys_shape::{BoxShape, CapsuleShape, Shape};
use crate::phys_types::MotionType;
use crate::phys_world::{World, WorldCinfo};
use std::sync::Arc as StdArc;

const DT: f32 = 1.0 / 60.0;

fn capsule(x: f32) -> CharacterProxy {
    let shape = StdArc::new(CapsuleShape::new(
        G4::from_xyz(0.0, -0.6, 0.0),
        G4::from_xyz(0.0, 0.6, 0.0),
        0.4,
    )) as StdArc<dyn Shape>;
    CharacterProxy::new(&CharacterProxyCinfo::new(shape).with_position(V4::from_xyz(x, 0.0, 0.0)))
}

struct InteractionRecorder {
    name: &'static str,
    log: Arc<Mutex<Vec<String>>>,
    cancel_push: bool,
}

impl CharacterProxyListener for InteractionRecorder {
    fn object_interaction(
        &mut self,
        event: &CharacterObjectInteractionEvent,
        result: &mut CharacterObjectInteractionResult,
    ) {
        self.log.lock().unwrap().push(format!(
            "{}:object:{}:{}",
            self.name,
            event.body.value(),
            event.object_impulse
        ));
        if self.cancel_push {
            result.object_impulse = V4::ZERO;
        }
    }

    fn character_interaction(&mut self, info: &CharacterInteractionInfo) {
        self.log
            .lock()
            .unwrap()
            .push(format!("{}:character:{}", self.name, info.other_id));
    }

    fn contact_added(&mut self, contact: &Contact) {
        self.log
            .lock()
            .unwrap()
            .push(format!("{}:add:{}", self.name, contact.body.value()));
    }
}

#[test]
fn object_interaction_result_can_cancel_push() {
    let mut world = World::new(WorldCinfo::new());
    let dynamic =
        StdArc::new(BoxShape::with_convex_radius(G4::splat(0.3), 0.0)) as StdArc<dyn Shape>;
    let body = world.create_body(
        &BodyCinfo::new(dynamic)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(G4::from_xyz(0.45, 0.0, 0.0)),
    );

    let mut proxy = capsule(0.0);
    let log = Arc::new(Mutex::new(Vec::new()));
    proxy.add_listener(Box::new(InteractionRecorder {
        name: "cancel",
        log: Arc::clone(&log),
        cancel_push: true,
    }));
    let surface = proxy.check_support(&world, -V4::UNIT_Y, DT);
    proxy.refresh_manifold(&world);
    assert!(
        proxy.manifold().iter().any(|contact| contact.body == body),
        "dynamic body was not collected into the proxy manifold: {:?}",
        proxy.manifold()
    );
    proxy.integrate(&mut world, DT, &surface);

    assert!(
        log.lock()
            .unwrap()
            .iter()
            .any(|event| event.starts_with("cancel:object:")),
        "object interaction callback did not fire"
    );
    assert_eq!(
        world.body_linear_velocity(body),
        Some(G4::ZERO),
        "cancelled push must not change body velocity"
    );
    assert!(proxy.deferred_interactions().is_empty());
}

#[test]
fn manager_dispatches_character_interactions_to_both_proxies() {
    let mut world = World::new(WorldCinfo::new());
    let manager = CharacterProxyManager::new();
    let first = manager.add_proxy(capsule(0.0));
    let second = manager.add_proxy(capsule(0.2));
    let log = Arc::new(Mutex::new(Vec::new()));
    manager.with_proxy(first, |proxy| {
        proxy.add_listener(Box::new(InteractionRecorder {
            name: "first",
            log: Arc::clone(&log),
            cancel_push: false,
        }));
    });
    manager.with_proxy(second, |proxy| {
        proxy.add_listener(Box::new(InteractionRecorder {
            name: "second",
            log: Arc::clone(&log),
            cancel_push: false,
        }));
    });

    manager.step_characters(&mut world, DT, &[(first, SurfaceInfo::UNSUPPORTED)], 2);
    let events = log.lock().unwrap();
    assert!(
        events.iter().any(|event| event == "first:character:2"),
        "missing first→second interaction: {events:?}"
    );
    assert!(
        events.iter().any(|event| event == "second:character:1"),
        "missing second→first interaction: {events:?}"
    );
}
