//! User-defined ragdoll state registration and scheduling.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::{
    ConstraintEasingMode, DEFAULT_EASING_DURATION, Ragdoll, RagdollController,
    RagdollControllerCinfo, RagdollData, RagdollError, RagdollState, RagdollStateBehavior,
    RagdollStateContext,
};
use proton::math::{Quaternion, Transform, Vector4, Vec4, Vec4Ext};
use proton::np_types::{BodyId, MotionType};
use proton::shape::{Shape, SphereShape};
use proton::world::{World, WorldCinfo};

#[derive(Debug, Clone)]
struct CountingState {
    enters: usize,
    leaves: usize,
    steps: usize,
    last_strength: f32,
    next: Option<RagdollState>,
    easing: (ConstraintEasingMode, f32),
}

impl CountingState {
    fn new() -> Self {
        Self {
            enters: 0,
            leaves: 0,
            steps: 0,
            last_strength: -1.0,
            next: None,
            easing: (ConstraintEasingMode::Enforce, DEFAULT_EASING_DURATION),
        }
    }
}

impl RagdollStateBehavior for CountingState {
    fn set_strength(&mut self, strength: f32) {
        self.last_strength = strength;
    }

    fn enter(&mut self, _world: &mut World, _previous: RagdollState, _tag_mask: u32) {
        self.enters += 1;
    }

    fn leave(&mut self, _world: &mut World, _next: RagdollState) {
        self.leaves += 1;
    }

    fn step(
        &mut self,
        _context: &mut RagdollStateContext<'_>,
    ) -> Result<Option<RagdollState>, RagdollError> {
        self.steps += 1;
        Ok(self.next)
    }

    fn easing(&self) -> (ConstraintEasingMode, f32) {
        self.easing
    }

    fn clone_state(&self) -> Box<dyn RagdollStateBehavior> {
        Box::new(self.clone())
    }
}

fn chain(world: &mut World) -> (Ragdoll, [BodyId; 2]) {
    let shape = || Arc::new(SphereShape::new(Vec4::ZERO, 0.25)) as Arc<dyn Shape>;
    let ids = [
        world.create_body(
            &BodyCinfo::new(shape())
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        ),
        world.create_body(
            &BodyCinfo::new(shape())
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vec4::from_xyz(1.0, 0.0, 0.0)),
        ),
    ];
    let mut data = RagdollData::new();
    data.add_bone("a", None, Some(ids[0]), Transform::IDENTITY);
    data.add_bone(
        "b",
        Some(0),
        Some(ids[1]),
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0)),
    );
    (Ragdoll::create(world, &data).expect("ragdoll"), ids)
}

#[test]
fn c1_register_and_enter_leave_user_state() {
    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, _) = chain(&mut world);
    let mut controller =
        RagdollController::new(&mut world, ragdoll, RagdollControllerCinfo::default());
    assert!(
        controller
            .register_state(&mut world, 0, CountingState::new())
            .is_ok()
    );
    assert!(controller.has_custom_state(0));
    assert_eq!(
        controller.register_state(&mut world, 0, CountingState::new()),
        Err(RagdollError::InvalidState(0))
    );
    assert_eq!(
        controller.register_state(&mut world, 10, CountingState::new()),
        Err(RagdollError::InvalidState(10))
    );

    controller.set_state(&mut world, RagdollState::Custom(0));
    assert_eq!(controller.state(), RagdollState::Custom(0));
    controller.set_state(&mut world, RagdollState::Dynamic);
    assert_eq!(controller.state(), RagdollState::Dynamic);
    assert!(controller.unregister_state(0));
    assert!(!controller.has_custom_state(0));
    controller.set_state(&mut world, RagdollState::Custom(0));
    assert_eq!(controller.state(), RagdollState::Dynamic);
}

#[test]
fn c2_custom_state_selects_easing_mode() {
    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, _) = chain(&mut world);
    let mut state = CountingState::new();
    state.easing = (ConstraintEasingMode::LoosenNoRestore, 2.0);
    let mut controller =
        RagdollController::new(&mut world, ragdoll, RagdollControllerCinfo::default());
    controller.register_state(&mut world, 3, state).unwrap();
    controller.set_state(&mut world, RagdollState::Custom(3));
    let action = controller.easing().expect("custom easing action");
    assert_eq!(action.duration(), -1.0);
}

#[test]
fn c3_custom_state_can_queue_a_transition() {
    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, _) = chain(&mut world);
    let mut state = CountingState::new();
    state.next = Some(RagdollState::Dynamic);
    let mut controller =
        RagdollController::new(&mut world, ragdoll, RagdollControllerCinfo::default());
    controller.register_state(&mut world, 1, state).unwrap();
    controller.set_state(&mut world, RagdollState::Custom(1));
    let pose = [Transform::IDENTITY; 2];
    controller
        .step(&mut world, &pose, Transform::IDENTITY, 1.0 / 60.0)
        .unwrap();
    assert_eq!(controller.state(), RagdollState::Dynamic);
}

#[test]
fn c4_clone_keeps_distinct_state_schedules() {
    let mut world = World::new(WorldCinfo::new());
    let (ragdoll, _) = chain(&mut world);
    let mut controller =
        RagdollController::new(&mut world, ragdoll, RagdollControllerCinfo::default());
    controller
        .register_state(&mut world, 2, CountingState::new())
        .unwrap();
    controller.set_state(&mut world, RagdollState::Custom(2));
    let mut clone = controller.clone();
    clone.set_strength(0.5);
    assert_eq!(controller.strength(), 1.0);
    assert_eq!(clone.strength(), 0.5);
}

#[test]
fn c5_custom_state_scenario_is_bit_deterministic() {
    fn run() -> String {
        let mut world = World::new(WorldCinfo::new());
        world.set_gravity(Vec4::ZERO);
        let (ragdoll, _) = chain(&mut world);
        let mut state = CountingState::new();
        state.next = None;
        let mut controller =
            RagdollController::new(&mut world, ragdoll, RagdollControllerCinfo::default());
        controller.register_state(&mut world, 4, state).unwrap();
        controller.set_state(&mut world, RagdollState::Custom(4));
        let pose = [Transform::IDENTITY; 2];
        for _ in 0..60 {
            controller
                .step(&mut world, &pose, Transform::IDENTITY, 1.0 / 60.0)
                .unwrap();
            world.step(1.0 / 60.0);
        }
        world.snapshot().to_text()
    }
    assert_eq!(run(), run());
}
