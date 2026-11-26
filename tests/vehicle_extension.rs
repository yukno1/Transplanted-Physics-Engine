//! Vehicle extension integration scenarios (Phase 13, `hknpVehicleInstance`
//! subset: ray-cast suspension + default engine/transmission + simplified
//! friction solver).
//!
//! These tests exercise the public facade (`proton`): a ray-cast vehicle on a
//! static ground plane, driven by the engine/transmission/friction pipeline,
//! plus the usual determinism guarantees.

use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::vehicle::{DriverInput, VehicleInstance, VehicleSetup, WheelSetup};
use proton::extensions::world_hash::hash_world;
use proton::math::{Vector4, Vec4, Vec4Ext};
use proton::np_types::{BodyId, MotionType};
use proton::shape::{BoxShape, Shape};
use proton::world::{World, WorldCinfo};

fn ground_plane(world: &mut World) {
    // Static ground with its top surface at y = 0.
    world.create_body(
        &BodyCinfo::new(Arc::new(BoxShape::with_convex_radius(
            Vec4::from_xyz(50.0, 0.5, 50.0),
            0.0,
        )) as Arc<dyn Shape>)
        .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
    );
}

/// A 400 kg chassis box with four ray-cast wheels. The forward driving
/// direction is `suspension_direction × axle` = `(-Y) × (+X)` = `+Z`.
fn raycast_vehicle(world: &mut World) -> (BodyId, VehicleInstance) {
    ground_plane(world);
    let chassis = world.create_body(
        &BodyCinfo::new(Arc::new(BoxShape::with_convex_radius(
            Vec4::from_xyz(0.5, 0.25, 1.2),
            0.0,
        )) as Arc<dyn Shape>)
        .with_motion_type(MotionType::Dynamic)
        .with_mass(400.0)
        .with_position(Vector4::from_xyz(0.0, 0.9, 0.0)),
    );
    // Box inertia (1 × 0.5 × 2.4 m, 400 kg) so suspension and tyre friction
    // can pitch/roll/yaw the chassis (a zero inverse inertia chassis cannot
    // be steered). The COM sits low (≈ 0.82 m) and the wheelbase is long
    // (± 1.1 m) so friction-limited drive/brake forces (≈ 1 g) transfer less
    // than the static axle load — a higher COM or shorter wheelbase lifts the
    // unloaded axle and the contact-only suspension starts bouncing.
    let motion_id = world.body(chassis).unwrap().motion_id;
    let motion = world.motion_mut(motion_id).unwrap();
    motion.inverse_inertia = Vector4::from_xyz(0.006, 0.005, 0.02);
    let mut setup = VehicleSetup::new(chassis).with_brake_torques(1200.0, 1200.0);
    for (x, z) in [(-0.5, -1.1), (0.5, -1.1), (-0.5, 1.1), (0.5, 1.1)] {
        let mut wheel = WheelSetup::new(Vector4::from_xyz(x, -0.25, z), 0.3)
            .with_suspension_rest_length(0.35)
            .with_spring_strength(30.0)
            .with_damping(4.0, 5.0)
            .with_tyre_friction(1.0)
            .with_torque_ratio(0.25);
        // Front wheels (negative z) are steered.
        if z < 0.0 {
            wheel.steering_angle_fraction = 1.0;
        }
        // Rear wheels carry the handbrake.
        wheel.handbrake = z > 0.0;
        setup.wheels.push(wheel);
    }
    let vehicle = VehicleInstance::new(setup).expect("valid setup");
    (chassis, vehicle)
}

fn run_steps(world: &mut World, vehicle: &mut VehicleInstance, steps: usize, input: &DriverInput) {
    for _ in 0..steps {
        vehicle.step(world, 1.0 / 60.0, input);
        world.step(1.0 / 60.0);
    }
}

#[test]
fn v1_suspension_supports_the_chassis_and_settles() {
    let mut world = World::new(WorldCinfo::new());
    let (chassis, mut vehicle) = raycast_vehicle(&mut world);

    run_steps(&mut world, &mut vehicle, 300, &DriverInput::new());

    let position = world.body_position(chassis).unwrap();
    // Equilibrium: each wheel carries 1/4 of the weight, spring strength 30
    // per unit mass → Δlength ≈ g/120 ≈ 0.082 → chassis y ≈ 0.82.
    assert!(
        position.y > 0.65 && position.y < 1.0,
        "chassis must settle on its suspension: {position:?}"
    );
    let velocity = world.body_linear_velocity(chassis).unwrap();
    assert!(
        velocity.length3() < 0.3,
        "suspension damping must settle the chassis: {velocity:?}"
    );
    // All four wheels rest on the ground.
    for wheel in vehicle.wheels() {
        assert!(wheel.contact_body.is_some(), "wheel must touch the ground");
        assert!(wheel.suspension_force > 0.0, "suspension must carry load");
    }
    // A settled vehicle idles at the clutch-slipped RPM without throttle.
    assert!(
        vehicle.rpm() > 900.0 && vehicle.rpm() < 1500.0,
        "idle RPM with clutch slip: {}",
        vehicle.rpm()
    );
}

#[test]
fn v2_throttle_drives_the_vehicle_forward() {
    let mut world = World::new(WorldCinfo::new());
    let (chassis, mut vehicle) = raycast_vehicle(&mut world);

    run_steps(
        &mut world,
        &mut vehicle,
        120,
        &DriverInput::new().with_throttle(1.0),
    );

    // Forward is +Z (suspension down × axle +X).
    let position = world.body_position(chassis).unwrap();
    assert!(
        position.z > 5.0,
        "the vehicle must drive forward along +Z: {position:?}"
    );
    let velocity = world.body_linear_velocity(chassis).unwrap();
    assert!(
        velocity.z > 2.0,
        "the vehicle must accumulate forward speed: {velocity:?}"
    );
    assert!(
        position.y > 0.8 && position.y < 1.6,
        "the vehicle must stay on its suspension (pitching up under thrust): {position:?}"
    );
}

#[test]
fn v3_reverse_drives_backwards() {
    let mut world = World::new(WorldCinfo::new());
    let (chassis, mut vehicle) = raycast_vehicle(&mut world);

    run_steps(
        &mut world,
        &mut vehicle,
        120,
        &DriverInput::new().with_throttle(1.0).with_reverse(true),
    );

    let position = world.body_position(chassis).unwrap();
    assert!(
        position.z < -5.0,
        "reverse gear must drive along -Z: {position:?}"
    );
    assert!(vehicle.is_reversing());
}

#[test]
fn v4_steering_turns_the_vehicle() {
    let run = |steer: f32| {
        let mut world = World::new(WorldCinfo::new());
        let (chassis, mut vehicle) = raycast_vehicle(&mut world);
        run_steps(
            &mut world,
            &mut vehicle,
            120,
            &DriverInput::new()
                .with_throttle(1.0)
                .with_steering_angle(steer),
        );
        world.body_position(chassis).unwrap()
    };
    let straight = run(0.0);
    let steered = run(0.6);
    assert!(
        steered.x.abs() > 0.3,
        "steering must deviate the heading: straight {straight:?} vs steered {steered:?}"
    );
    assert!(
        (straight.x.abs() < 0.05),
        "the straight run must not drift sideways: {straight:?}"
    );
}

#[test]
fn v5_brake_stops_the_vehicle() {
    let drive_steps = 60;
    let run = |brake: f32| {
        let mut world = World::new(WorldCinfo::new());
        let (chassis, mut vehicle) = raycast_vehicle(&mut world);
        run_steps(
            &mut world,
            &mut vehicle,
            drive_steps,
            &DriverInput::new().with_throttle(1.0),
        );
        let coasted_speed = world.body_linear_velocity(chassis).unwrap().length3();
        // Friction-limited braking decelerates at ≈ μ·g, so it takes about
        // 1.5 s to stop from highway speed.
        run_steps(
            &mut world,
            &mut vehicle,
            180,
            &DriverInput::new().with_brake(brake),
        );
        let final_speed = world.body_linear_velocity(chassis).unwrap().length3();
        (coasted_speed, final_speed)
    };
    let (coasted_speed, braked_speed) = run(1.0);
    assert!(
        braked_speed < 0.5,
        "full brake must stop the vehicle: {braked_speed}"
    );
    assert!(
        coasted_speed > 2.0,
        "the vehicle must have built up speed first: {coasted_speed}"
    );
}

#[test]
fn v6_handbrake_locks_the_rear_wheels() {
    let mut world = World::new(WorldCinfo::new());
    let (chassis, mut vehicle) = raycast_vehicle(&mut world);
    run_steps(
        &mut world,
        &mut vehicle,
        90,
        &DriverInput::new().with_throttle(1.0),
    );
    let before = world.body_linear_velocity(chassis).unwrap().length3();
    // The handbrake only locks the rear wheels, so the deceleration is
    // limited by the rear axle load (≈ μ·g/2); give it 5 s to stop.
    run_steps(
        &mut world,
        &mut vehicle,
        300,
        &DriverInput::new().with_handbrake(true),
    );
    let after = world.body_linear_velocity(chassis).unwrap().length3();
    assert!(
        before > 2.0,
        "the vehicle must have built up speed first: {before}"
    );
    assert!(after < 0.3, "the handbrake must stop the vehicle: {after}");
}

#[test]
fn v7_steered_wheels_report_the_steer_angle() {
    let mut world = World::new(WorldCinfo::new());
    let (_chassis, mut vehicle) = raycast_vehicle(&mut world);
    let input = DriverInput::new().with_steering_angle(0.3);
    run_steps(&mut world, &mut vehicle, 1, &input);
    for (index, wheel) in vehicle.wheels().iter().enumerate() {
        let expected = if index < 2 { 0.3 } else { 0.0 };
        assert!((wheel.steer_angle - expected).abs() < 1e-5);
    }
}

#[test]
fn v8_same_scene_is_bit_exact_across_runs() {
    let run = || {
        let mut world = World::new(WorldCinfo::new());
        let (_chassis, mut vehicle) = raycast_vehicle(&mut world);
        let mut input = DriverInput::new().with_throttle(1.0);
        for step in 0..90 {
            if step >= 60 {
                input = DriverInput::new().with_steering_angle(0.3).with_brake(0.5);
            }
            vehicle.step(&mut world, 1.0 / 60.0, &input);
            world.step(1.0 / 60.0);
        }
        hash_world(&world)
    };
    let first = run();
    let second = run();
    assert_eq!(first, second, "identical vehicle runs must be bit-exact");
}
