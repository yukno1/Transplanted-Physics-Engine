//! Local monitor-style diagnostics for proxy and rigid-body characters.

use std::sync::{Arc, Mutex};

use crate::base_math::Vector4 as V4;
use crate::phys_body::BodyCinfo;
use crate::phys_extensions::{
    CharacterMonitorEvent, CharacterMonitorPhase, CharacterProxy, CharacterProxyCinfo,
    CharacterProxyListener, CharacterProxyManager, CharacterRigidBody, CharacterRigidBodyCinfo,
    CharacterRigidBodyListener, SurfaceInfo,
};
use crate::phys_shape::{BoxShape, CapsuleShape, Shape, SphereShape};
use crate::phys_types::MotionType;
use crate::phys_world::{World, WorldCinfo};

use std::sync::Arc as StdArc;

const DT: f32 = 1.0 / 60.0;

fn proxy() -> CharacterProxy {
    let shape = StdArc::new(CapsuleShape::new(
        V4::from_xyz(0.0, -0.6, 0.0),
        V4::from_xyz(0.0, 0.6, 0.0),
        0.4,
    )) as StdArc<dyn Shape>;
    CharacterProxy::new(&CharacterProxyCinfo::new(shape).with_position(V4::ZERO))
}

#[derive(Clone)]
struct ProxyMonitor {
    log: Arc<Mutex<Vec<(CharacterMonitorPhase, u64)>>>,
}

impl CharacterProxyListener for ProxyMonitor {
    fn monitor(&mut self, event: &CharacterMonitorEvent) {
        self.log
            .lock()
            .unwrap()
            .push((event.phase, event.step_count));
    }
}

#[derive(Clone)]
struct RigidMonitor {
    log: Arc<Mutex<Vec<CharacterMonitorPhase>>>,
}

impl CharacterRigidBodyListener for RigidMonitor {
    fn monitor(&mut self, event: &CharacterMonitorEvent) {
        self.log.lock().unwrap().push(event.phase);
    }
}

#[test]
fn standalone_proxy_emits_begin_end_spans_in_step_order() {
    let mut world = World::new(WorldCinfo::new());
    let mut character = proxy();
    let log = Arc::new(Mutex::new(Vec::new()));
    character.add_listener(Box::new(ProxyMonitor {
        log: Arc::clone(&log),
    }));
    for _ in 0..2 {
        character.integrate(&mut world, DT, &SurfaceInfo::UNSUPPORTED);
    }
    assert_eq!(
        *log.lock().unwrap(),
        vec![
            (CharacterMonitorPhase::IntegrateBegin, 1),
            (CharacterMonitorPhase::IntegrateEnd, 1),
            (CharacterMonitorPhase::IntegrateBegin, 2),
            (CharacterMonitorPhase::IntegrateEnd, 2),
        ]
    );
}

#[test]
fn managed_proxy_emits_post_solve_without_duplicating_spans() {
    let mut world = World::new(WorldCinfo::new());
    let manager = CharacterProxyManager::new();
    let id = manager.add_proxy(proxy());
    let log = Arc::new(Mutex::new(Vec::new()));
    manager.with_proxy(id, |character| {
        character.add_listener(Box::new(ProxyMonitor {
            log: Arc::clone(&log),
        }));
    });
    manager.attach(&world);
    manager.step_characters(&mut world, DT, &[], 1);
    world.step(DT);
    assert_eq!(
        *log.lock().unwrap(),
        vec![
            (CharacterMonitorPhase::IntegrateBegin, 1),
            (CharacterMonitorPhase::IntegrateEnd, 1),
            (CharacterMonitorPhase::PostSolve, 1),
        ]
    );
    manager.detach(&world);
}

#[test]
fn rigid_body_reports_all_monitor_phases() {
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
    let manager = CharacterProxyManager::new();
    let shape = StdArc::new(SphereShape::new(V4::ZERO, 0.5)) as StdArc<dyn Shape>;
    let cinfo = CharacterRigidBodyCinfo::new(shape).with_position(V4::from_xyz(0.0, 0.45, 0.0));
    let character = CharacterRigidBody::new(&mut world, &cinfo);
    let id = manager.add_rigid_body(character);
    let log = Arc::new(Mutex::new(Vec::new()));
    manager.with_rigid_body(id, |body| {
        body.set_listener(Box::new(RigidMonitor {
            log: Arc::clone(&log),
        }));
    });
    manager.attach(&world);
    manager.step_rigid_bodies(&mut world, DT, &[]);
    world.step(DT);
    let phases = log.lock().unwrap().clone();
    assert!(phases.contains(&CharacterMonitorPhase::IntegrateBegin));
    assert!(phases.contains(&CharacterMonitorPhase::IntegrateEnd));
    assert!(phases.contains(&CharacterMonitorPhase::PostCollide));
    assert!(phases.contains(&CharacterMonitorPhase::PostSolve));
    manager.detach(&world);
}
