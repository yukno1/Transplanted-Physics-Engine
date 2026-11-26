//! Character proxy listener and phantom presence integration tests.

use std::sync::{Arc, Mutex};

use crate::base_math::Vector4 as V4;
use crate::base_math::Vec4Ext;
use glam::Vec4 as G4;
use crate::phys_extensions::{
    CharacterContact as Contact, CharacterProxy, CharacterProxyCinfo, CharacterProxyListener,
    CharacterProxyState, SurfaceInfo, TriggerEvent, TriggerStatus,
};
use crate::phys_shape::{BoxShape, CapsuleShape, Shape};
use crate::phys_types::{BodyId, CollisionFlags, MotionType, ShapeKey};
use crate::phys_world::{World, WorldCinfo};

const DT: f32 = 1.0 / 60.0;
const UP: V4 = V4::UNIT_Y;

type Log = Arc<Mutex<Vec<String>>>;

struct Recorder {
    name: &'static str,
    log: Log,
}

impl CharacterProxyListener for Recorder {
    fn shape_changed(&mut self) {
        self.log
            .lock()
            .unwrap()
            .push(format!("{}:shape", self.name));
    }

    fn contact_added(&mut self, contact: &Contact) {
        self.log
            .lock()
            .unwrap()
            .push(format!("{}:add:{}", self.name, contact.body.value()));
    }

    fn contact_removed(&mut self, contact: &Contact) {
        self.log
            .lock()
            .unwrap()
            .push(format!("{}:remove:{}", self.name, contact.body.value()));
    }

    fn character_callback(&mut self, state: &CharacterProxyState) {
        self.log.lock().unwrap().push(format!(
            "{}:step:{}:{}",
            self.name, state.contact_count, state.position.x
        ));
    }

    fn trigger_volume_interaction(&mut self, event: TriggerEvent) {
        let status = match event.status {
            TriggerStatus::Entered => "enter",
            TriggerStatus::Updated => "update",
            TriggerStatus::Exited => "exit",
        };
        self.log.lock().unwrap().push(format!(
            "{}:trigger:{}:{}",
            self.name,
            status,
            event.body.value()
        ));
    }
}

fn recorder(name: &'static str, log: &Log) -> Box<dyn CharacterProxyListener> {
    Box::new(Recorder {
        name,
        log: log.clone(),
    })
}

fn proxy_at(position: V4) -> CharacterProxy {
    let shape = Arc::new(CapsuleShape::new(
        G4::from_xyz(0.0, -0.6, 0.0),
        G4::from_xyz(0.0, 0.6, 0.0),
        0.4,
    )) as Arc<dyn Shape>;
    CharacterProxy::new(
        &CharacterProxyCinfo::new(shape)
            .with_position(position)
            .with_up(UP)
            .with_keep_distance(0.05),
    )
}

fn world_with_ground() -> (World, BodyId) {
    let mut world = World::new(WorldCinfo::new());
    let ground = Arc::new(BoxShape::with_convex_radius(
        G4::from_xyz(20.0, 0.5, 20.0),
        0.0,
    )) as Arc<dyn Shape>;
    let id = world.create_body(
        &crate::phys_body::BodyCinfo::new(ground)
            .with_motion_type(MotionType::Static)
            .with_position(G4::from_xyz(0.0, -0.5, 0.0)),
    );
    (world, id)
}

#[test]
fn l1_add_remove_and_reverse_order_dispatch() {
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let mut proxy = proxy_at(V4::ZERO);
    let first = proxy.add_listener(recorder("A", &log));
    proxy.add_listener(recorder("B", &log));
    assert_eq!(
        *log.lock().unwrap(),
        vec!["A:shape".to_string(), "B:shape".to_string()]
    );

    let trigger = TriggerEvent {
        body: BodyId::new(7),
        shape_key: ShapeKey::INVALID,
        status: TriggerStatus::Entered,
    };
    proxy.dispatch_trigger_events(&[trigger]);
    assert_eq!(
        *log.lock().unwrap(),
        vec![
            "A:shape".to_string(),
            "B:shape".to_string(),
            "B:trigger:enter:7".to_string(),
            "A:trigger:enter:7".to_string(),
        ]
    );

    assert!(proxy.remove_listener(first));
    assert!(!proxy.remove_listener(first));
    log.lock().unwrap().clear();
    proxy.dispatch_trigger_events(&[trigger]);
    assert_eq!(*log.lock().unwrap(), vec!["B:trigger:enter:7".to_string()]);
}

#[test]
fn l2_manifold_changes_fire_contact_events() {
    let (mut world, ground) = world_with_ground();
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let mut proxy = proxy_at(V4::from_xyz(0.0, 0.05, 0.0));
    proxy.add_listener(recorder("A", &log));

    proxy.refresh_manifold(&world);
    assert!(
        log.lock()
            .unwrap()
            .iter()
            .any(|event| event == &format!("A:add:{}", ground.value()))
    );

    log.lock().unwrap().clear();
    assert!(world.destroy_body(ground));
    proxy.refresh_manifold(&world);
    assert!(
        log.lock()
            .unwrap()
            .iter()
            .any(|event| event == &format!("A:remove:{}", ground.value()))
    );
}

#[test]
fn l3_character_callback_runs_once_after_integrate() {
    let (mut world, _) = world_with_ground();
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let mut proxy = proxy_at(V4::from_xyz(0.0, 0.05, 0.0));
    proxy.add_listener(recorder("A", &log));
    let surface = proxy.check_support(&world, -UP, DT);

    proxy.integrate(&mut world, DT, &surface);

    let binding = log.lock().unwrap();
    let steps: Vec<_> = binding.iter().filter(|e| e.contains(":step:")).collect();
    assert_eq!(steps.len(), 1);
    assert!(steps[0].starts_with("A:step:"));
}

#[test]
fn l4_phantom_body_matches_proxy_and_can_be_destroyed() {
    let (mut world, _) = world_with_ground();
    let mut proxy = proxy_at(V4::ZERO);
    let phantom = proxy.create_phantom(&mut world).expect("phantom");
    assert_eq!(proxy.phantom(), Some(phantom));
    let motion_id = world.body(phantom).unwrap().motion_id;
    assert_eq!(
        world.motion(motion_id).unwrap().motion_type,
        MotionType::Keyframed
    );
    assert!(
        world
            .body(phantom)
            .unwrap()
            .flags
            .contains(CollisionFlags::DONT_COLLIDE)
    );
    assert_eq!(
        world.body(phantom).unwrap().name.as_deref(),
        Some("hknpCharacterProxy")
    );

    proxy.set_position(V4::from_xyz(3.0, 2.0, 0.0));
    proxy.integrate(&mut world, DT, &SurfaceInfo::unsupported());
    assert!(
        world
            .body_transform(phantom)
            .unwrap()
            .translation
            .all_equal3(V4::from_xyz(3.0, 2.0, 0.0), 1e-6)
    );

    assert!(proxy.destroy_phantom(&mut world));
    assert!(world.body(phantom).is_none());
    assert_eq!(proxy.phantom(), None);
}
