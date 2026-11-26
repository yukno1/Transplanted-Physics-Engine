//! `hknpVehicleInstance` — vehicle extension (`Extensions/Vehicle` subset).
//!
//! Rust port of `original/.../Extensions/Vehicle/` (migration Phase 13,
//! README §4). The SDK drives the vehicle through an `hknpUnaryAction` hooked
//! into the world's action manager; this port exposes the same per-step
//! pipeline as an explicit [`VehicleInstance::step`] call that the caller runs
//! between world steps:
//!
//! ```text
//! update_before_collision_detection   (hardpoints / ray end points, world space)
//!   → ray-cast wheel collide          (`World::raycast_filtered`, chassis ignored)
//!   → steering / transmission / engine / brake
//!   → suspension forces               (spring + damper at the hardpoint)
//!   → friction solver                 (forward/side impulses, friction circle)
//!   → write chassis & ground velocities
//! ```
//!
//! # Scope and deviations (documented subset)
//!
//! * **Ray-cast suspension only** — `hknpVehicleRayCastWheelCollide` subset;
//!   `LinearCastWheelCollide` is not ported. Ground friction comes from the
//!   tyre setup ([`WheelSetup::tyre_friction`]) instead of ground material
//!   (materials are not wired into the port yet).
//! * **Default engine / transmission** — the parabolic torque curve of
//!   `hknpVehicleDefaultEngine` is translated formula-for-formula. Gear shifts
//!   happen immediately at the upshift/downshift RPM thresholds (the SDK's
//!   clutch delay timer is not ported).
//! * **Simplified friction solver** — per-wheel impulses with a circular
//!   friction clamp instead of the SDK's per-axle 2×2 jacobian system with
//!   wheel-spin state, friction-table curve and delayed forward impulse. The
//!   transmitted drive/brake force is applied as a forward impulse at the
//!   contact point (braking never overshoots past the rolling condition);
//!   lateral friction cancels the side slip; both are clamped together by the
//!   friction circle `μ·(suspension force)·dt` (bullet-style raycast vehicle
//!   model).
//! * **Suspension spring/damper parameters are per unit chassis mass**
//!   (`calcSuspension` multiplies by the chassis mass at the end); forces are
//!   clamped to be non-negative.
//! * Suspension and friction impulses include the angular response through
//!   the diagonal inverse inertia in motion space (like `Motion`); a chassis
//!   with `inverse_inertia == 0` will not rotate from vehicle forces, so set
//!   the inertia explicitly for full behaviour (same limitation as joints).
//! * Aerodynamics, velocity damper, tyremarks, cameras and the driver-input
//!   filtering pipeline are not ported; [`DriverInput`] is the already
//!   filtered input.
//!
//! # Usage
//!
//! ```no_run
//! use std::sync::Arc;
//!
//! use proton::body::BodyCinfo;
//! use proton::extensions::vehicle::{DriverInput, VehicleInstance, VehicleSetup, WheelSetup};
//! use proton::math::Vector4;
//! use proton::math::{Vec4, Vec4Ext};
//! use proton::np_types::MotionType;
//! use proton::shape::{BoxShape, Shape};
//! use proton::world::{World, WorldCinfo};
//!
//! let mut world = World::new(WorldCinfo::new());
//! let ground = BoxShape::with_convex_radius(Vec4::from_xyz(20.0, 0.5, 20.0), 0.0);
//! world.create_body(
//!     &BodyCinfo::new(Arc::new(ground) as Arc<dyn Shape>)
//!         .with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
//! );
//!
//! let chassis_shape = BoxShape::new(Vec4::from_xyz(0.5, 0.25, 1.0));
//! let chassis = world.create_body(
//!     &BodyCinfo::new(Arc::new(chassis_shape) as Arc<dyn Shape>)
//!         .with_motion_type(MotionType::Dynamic)
//!         .with_mass(800.0)
//!         .with_position(Vector4::from_xyz(0.0, 1.0, 0.0)),
//! );
//!
//! let mut setup = VehicleSetup::new(chassis);
//! for (x, z) in [(-0.5, -0.8), (0.5, -0.8), (-0.5, 0.8), (0.5, 0.8)] {
//!     setup.wheels.push(WheelSetup::new(Vector4::from_xyz(x, -0.25, z), 0.3));
//! }
//! let mut vehicle = VehicleInstance::new(setup).unwrap();
//!
//! // Called between world steps (the SDK runs it as a world action):
//! for _ in 0..60 {
//!     vehicle.step(&mut world, 1.0 / 60.0, &DriverInput::new().with_throttle(0.8));
//!     world.step(1.0 / 60.0);
//! }
//! ```

use crate::base_math::{Quaternion, Vector4};
use crate::phys_body::Motion;
use crate::phys_types::BodyId;
use crate::phys_world::{QueryFilter, World};

/// Already filtered driver input (`hknpVehicleDriverInput::FilteredDriverInputOutput`
/// subset).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriverInput {
    /// Accelerator pedal in `[0, 1]`.
    pub throttle: f32,
    /// Brake pedal in `[0, 1]` (torque opposing the current forward motion).
    pub brake: f32,
    /// Main steering angle in radians (positive steers wheels with a positive
    /// [`WheelSetup::steering_angle_fraction`] around their suspension axis).
    pub steering_angle: f32,
    /// Handbrake engaged (locks the handbrake wheels).
    pub handbrake: bool,
    /// Drive in reverse (uses [`TransmissionSetup::reverse_ratio`]).
    pub reverse: bool,
}

impl DriverInput {
    /// Neutral input: no throttle, no brake, centred steering.
    pub fn new() -> Self {
        Self {
            throttle: 0.0,
            brake: 0.0,
            steering_angle: 0.0,
            handbrake: false,
            reverse: false,
        }
    }

    /// Set the throttle pedal (`[0, 1]`).
    #[must_use]
    pub fn with_throttle(mut self, throttle: f32) -> Self {
        self.throttle = throttle.clamp(0.0, 1.0);
        self
    }

    /// Set the brake pedal (`[0, 1]`).
    #[must_use]
    pub fn with_brake(mut self, brake: f32) -> Self {
        self.brake = brake.clamp(0.0, 1.0);
        self
    }

    /// Set the main steering angle (radians).
    #[must_use]
    pub fn with_steering_angle(mut self, angle: f32) -> Self {
        self.steering_angle = angle;
        self
    }

    /// Engage the handbrake.
    #[must_use]
    pub fn with_handbrake(mut self, handbrake: bool) -> Self {
        self.handbrake = handbrake;
        self
    }

    /// Drive in reverse.
    #[must_use]
    pub fn with_reverse(mut self, reverse: bool) -> Self {
        self.reverse = reverse;
        self
    }
}

impl Default for DriverInput {
    fn default() -> Self {
        Self::new()
    }
}

/// Static setup of one wheel (`hknpVehicleData::WheelSpec` subset).
#[derive(Clone, Debug, PartialEq)]
pub struct WheelSetup {
    /// Suspension hardpoint on the chassis (chassis local space).
    pub hardpoint: Vector4,
    /// Suspension travel direction pointing *down* from the hardpoint
    /// (chassis local space; normalized on construction). Default `(0, -1, 0)`.
    pub suspension_direction: Vector4,
    /// Wheel axle direction (chassis local space; normalized on
    /// construction). Default `(1, 0, 0)`. The forward driving direction is
    /// `suspension_direction × axle`.
    pub axle: Vector4,
    /// Fraction of the main steering angle applied to this wheel
    /// (`0` = no steering).
    pub steering_angle_fraction: f32,
    /// Wheel radius (also drives the ray length).
    pub wheel_radius: f32,
    /// Suspension rest length (`hknpVehicleDefaultSuspension::m_length`).
    pub suspension_rest_length: f32,
    /// Spring strength per unit chassis mass
    /// (`hknpVehicleDefaultSpringSettings::m_strength`).
    pub spring_strength: f32,
    /// Damping during compression per unit chassis mass.
    pub damping_compression: f32,
    /// Damping during relaxation (extension) per unit chassis mass.
    pub damping_relaxation: f32,
    /// Tyre dynamic friction coefficient (see the module deviations).
    pub tyre_friction: f32,
    /// Share of the transmitted torque driving this wheel
    /// (`m_wheelsTorqueRatio`). The sum does not have to be `1`.
    pub torque_ratio: f32,
    /// This wheel is locked by the handbrake.
    pub handbrake: bool,
}

impl WheelSetup {
    /// Wheel at `hardpoint` with `wheel_radius`, pointing straight down
    /// (`suspension_direction = -Y`, `axle = +X`), un-steered, with default
    /// friction and no torque share.
    pub fn new(hardpoint: Vector4, wheel_radius: f32) -> Self {
        Self {
            hardpoint,
            suspension_direction: Vector4::from_xyz(0.0, -1.0, 0.0),
            axle: Vector4::from_xyz(1.0, 0.0, 0.0),
            steering_angle_fraction: 0.0,
            wheel_radius,
            suspension_rest_length: 0.4,
            spring_strength: 30.0,
            damping_compression: 2.5,
            damping_relaxation: 3.5,
            tyre_friction: 1.0,
            torque_ratio: 0.0,
            handbrake: false,
        }
    }

    /// Set the suspension direction (chassis local, non-zero).
    #[must_use]
    pub fn with_suspension_direction(mut self, direction: Vector4) -> Self {
        self.suspension_direction = direction;
        self
    }

    /// Set the axle direction (chassis local, non-zero).
    #[must_use]
    pub fn with_axle(mut self, axle: Vector4) -> Self {
        self.axle = axle;
        self
    }

    /// Set the steering angle fraction.
    #[must_use]
    pub fn with_steering_angle_fraction(mut self, fraction: f32) -> Self {
        self.steering_angle_fraction = fraction;
        self
    }

    /// Set the suspension rest length.
    #[must_use]
    pub fn with_suspension_rest_length(mut self, length: f32) -> Self {
        self.suspension_rest_length = length;
        self
    }

    /// Set the spring strength (per unit chassis mass).
    #[must_use]
    pub fn with_spring_strength(mut self, strength: f32) -> Self {
        self.spring_strength = strength;
        self
    }

    /// Set the compression/relaxation damping (per unit chassis mass).
    #[must_use]
    pub fn with_damping(mut self, compression: f32, relaxation: f32) -> Self {
        self.damping_compression = compression;
        self.damping_relaxation = relaxation;
        self
    }

    /// Set the tyre friction coefficient.
    #[must_use]
    pub fn with_tyre_friction(mut self, friction: f32) -> Self {
        self.tyre_friction = friction;
        self
    }

    /// Set the torque share of this wheel.
    #[must_use]
    pub fn with_torque_ratio(mut self, ratio: f32) -> Self {
        self.torque_ratio = ratio;
        self
    }

    /// Mark the wheel as handbrake-locked.
    #[must_use]
    pub fn with_handbrake(mut self, handbrake: bool) -> Self {
        self.handbrake = handbrake;
        self
    }

    /// Normalize `suspension_direction`/`axle` (called by
    /// [`VehicleInstance::new`]).
    fn normalized(&self) -> Self {
        let mut copy = self.clone();
        let length = copy.suspension_direction.length3();
        if length > 1e-10 {
            copy.suspension_direction *= 1.0 / length;
        } else {
            copy.suspension_direction = Vector4::from_xyz(0.0, -1.0, 0.0);
        }
        let length = copy.axle.length3();
        if length > 1e-10 {
            copy.axle *= 1.0 / length;
        } else {
            copy.axle = Vector4::from_xyz(1.0, 0.0, 0.0);
        }
        copy
    }
}

/// Default engine setup (`hknpVehicleDefaultEngine`), translated
/// formula-for-formula. Defaults match the C++ constructor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineSetup {
    /// Idle RPM (`m_minRPM`).
    pub min_rpm: f32,
    /// RPM of peak torque (`m_optRPM`).
    pub opt_rpm: f32,
    /// Rev limit (`m_maxRPM`).
    pub max_rpm: f32,
    /// Peak torque (`m_maxTorque`).
    pub max_torque: f32,
    /// Torque factor at `min_rpm` relative to the peak
    /// (`m_torqueFactorAtMinRPM`).
    pub torque_factor_at_min_rpm: f32,
    /// Torque factor at `max_rpm` relative to the peak
    /// (`m_torqueFactorAtMaxRPM`).
    pub torque_factor_at_max_rpm: f32,
    /// Engine resistance factor at `min_rpm` (`m_resistanceFactorAtMinRPM`).
    pub resistance_factor_at_min_rpm: f32,
    /// Engine resistance factor at `opt_rpm` (`m_resistanceFactorAtOptRPM`).
    pub resistance_factor_at_opt_rpm: f32,
    /// Engine resistance factor at `max_rpm` (`m_resistanceFactorAtMaxRPM`).
    pub resistance_factor_at_max_rpm: f32,
    /// Clutch slip RPM added at low transmission RPM (`m_clutchSlipRPM`).
    pub clutch_slip_rpm: f32,
}

impl Default for EngineSetup {
    fn default() -> Self {
        Self {
            min_rpm: 1000.0,
            opt_rpm: 4000.0,
            max_rpm: 6000.0,
            max_torque: 800.0,
            torque_factor_at_min_rpm: 1.0,
            torque_factor_at_max_rpm: 1.0,
            resistance_factor_at_min_rpm: 0.0,
            resistance_factor_at_opt_rpm: 0.0,
            resistance_factor_at_max_rpm: 0.0,
            clutch_slip_rpm: 2000.0,
        }
    }
}

/// Default transmission setup (`hknpVehicleDefaultTransmission` subset).
#[derive(Clone, Debug, PartialEq)]
pub struct TransmissionSetup {
    /// Forward gear ratios (at least one entry).
    pub gears: Vec<f32>,
    /// Reverse gear ratio (positive number; applied with a negative sign).
    pub reverse_ratio: f32,
    /// Primary (final drive) ratio multiplying every gear.
    pub primary_ratio: f32,
    /// RPM above which the transmission upshifts (`m_upShiftRPM`).
    pub upshift_rpm: f32,
    /// RPM below which the transmission downshifts (`m_downShiftRPM`).
    pub downshift_rpm: f32,
}

impl Default for TransmissionSetup {
    fn default() -> Self {
        Self {
            gears: vec![3.0, 2.0, 1.4, 1.0],
            reverse_ratio: 3.0,
            primary_ratio: 1.0,
            upshift_rpm: 5500.0,
            downshift_rpm: 2000.0,
        }
    }
}

/// Per-wheel runtime state (subset of `hknpVehicleInstance::WheelInfo` +
/// `CollisionDetectionWheelOutput`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelState {
    /// Ground body the ray hit (`None` = no contact this step).
    pub contact_body: Option<BodyId>,
    /// Contact point (world space, `ZERO` without contact).
    pub contact_point: Vector4,
    /// Ground normal at the contact (world space, points towards the chassis).
    pub contact_normal: Vector4,
    /// Current suspension length (from the ray-cast fraction).
    pub suspension_length: f32,
    /// Suspension closing speed in the SDK convention: negative while the
    /// suspension is compressing, positive while separating.
    pub closing_speed: f32,
    /// Suspension force computed this step (positive = pushing the chassis up).
    pub suspension_force: f32,
    /// Wheel forward driving direction (world space).
    pub forward_dir: Vector4,
    /// Steer angle applied this step (radians).
    pub steer_angle: f32,
    /// Ideal no-slip wheel spin (rad/s) used for the transmission RPM.
    pub spin_velocity: f32,
    /// Friction impulse applied along [`WheelState::forward_dir`].
    pub forward_impulse: f32,
    /// Friction impulse applied along the axle direction.
    pub side_impulse: f32,
}

impl WheelState {
    fn empty() -> Self {
        Self {
            contact_body: None,
            contact_point: Vector4::ZERO,
            contact_normal: Vector4::ZERO,
            suspension_length: 0.0,
            closing_speed: 0.0,
            suspension_force: 0.0,
            forward_dir: Vector4::ZERO,
            steer_angle: 0.0,
            spin_velocity: 0.0,
            forward_impulse: 0.0,
            side_impulse: 0.0,
        }
    }
}

/// Vehicle assembly (chassis + wheels + engine/transmission/brake).
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleSetup {
    /// Chassis body id.
    pub chassis: BodyId,
    /// Wheel setups (at least one).
    pub wheels: Vec<WheelSetup>,
    /// Engine parameters.
    pub engine: EngineSetup,
    /// Transmission parameters.
    pub transmission: TransmissionSetup,
    /// Maximum brake torque per wheel at full pedal (`m_maxBrakingTorque`).
    pub max_brake_torque: f32,
    /// Handbrake torque applied to handbrake wheels (`m_maxHandbrakeTorque`).
    pub handbrake_torque: f32,
}

impl VehicleSetup {
    /// Setup for `chassis` with default engine/transmission and no wheels.
    pub fn new(chassis: BodyId) -> Self {
        Self {
            chassis,
            wheels: Vec::new(),
            engine: EngineSetup::default(),
            transmission: TransmissionSetup::default(),
            max_brake_torque: 1500.0,
            handbrake_torque: 1500.0,
        }
    }

    /// Set the engine parameters.
    #[must_use]
    pub fn with_engine(mut self, engine: EngineSetup) -> Self {
        self.engine = engine;
        self
    }

    /// Set the transmission parameters.
    #[must_use]
    pub fn with_transmission(mut self, transmission: TransmissionSetup) -> Self {
        self.transmission = transmission;
        self
    }

    /// Set the brake torques.
    #[must_use]
    pub fn with_brake_torques(mut self, brake: f32, handbrake: f32) -> Self {
        self.max_brake_torque = brake;
        self.handbrake_torque = handbrake;
        self
    }
}

/// Transmission outputs (`hknpVehicleTransmission::TransmissionOutput` subset).
#[derive(Clone, Copy, Debug, PartialEq)]
struct TransmissionOutput {
    /// Engine RPM implied by the wheel spins and the current gear.
    transmission_rpm: f32,
    /// Total ratio (gear × primary, signed; negative in reverse).
    ratio: f32,
}

/// Instantiated vehicle (`hknpVehicleInstance` subset). Holds the runtime
/// state and steps the pipeline against a [`World`].
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleInstance {
    setup: VehicleSetup,
    /// Normalized wheel setups (parallel to [`VehicleInstance::state`]).
    wheels: Vec<WheelSetup>,
    /// Per-wheel runtime state.
    state: Vec<WheelState>,
    /// Engine RPM of the last step.
    rpm: f32,
    /// Engine output torque of the last step.
    torque: f32,
    /// Current forward gear index (0-based into `gears`).
    gear: usize,
    /// Whether the transmission is in reverse.
    is_reversing: bool,
}

impl VehicleInstance {
    /// Validate the setup and build the instance.
    pub fn new(setup: VehicleSetup) -> Result<Self, String> {
        if !setup.chassis.is_valid() {
            return Err("chassis body id is invalid".to_string());
        }
        if setup.wheels.is_empty() {
            return Err("vehicle needs at least one wheel".to_string());
        }
        if setup.transmission.gears.is_empty() {
            return Err("transmission needs at least one gear".to_string());
        }
        let wheels: Vec<WheelSetup> = setup.wheels.iter().map(WheelSetup::normalized).collect();
        let state = wheels.iter().map(|_| WheelState::empty()).collect();
        Ok(Self {
            setup,
            wheels,
            state,
            rpm: 0.0,
            torque: 0.0,
            gear: 0,
            is_reversing: false,
        })
    }

    /// The vehicle setup.
    pub fn setup(&self) -> &VehicleSetup {
        &self.setup
    }

    /// Per-wheel runtime state of the last step.
    pub fn wheels(&self) -> &[WheelState] {
        &self.state
    }

    /// Engine RPM of the last step.
    pub fn rpm(&self) -> f32 {
        self.rpm
    }

    /// Engine torque of the last step.
    pub fn torque(&self) -> f32 {
        self.torque
    }

    /// Current forward gear index (0-based).
    pub fn gear(&self) -> usize {
        self.gear
    }

    /// Whether the transmission is in reverse.
    pub fn is_reversing(&self) -> bool {
        self.is_reversing
    }

    /// Run one vehicle step (`hknpVehicleInstance::applyAction` subset).
    ///
    /// Reads the chassis motion, ray-casts every wheel, computes the
    /// suspension/friction impulses and writes back the chassis and ground
    /// velocities. Call this *before* [`World::step`]; the world keeps
    /// handling gravity and contacts.
    pub fn step(&mut self, world: &mut World, delta_time: f32, input: &DriverInput) {
        if delta_time <= 0.0 || !delta_time.is_finite() {
            return;
        }
        let Some(body) = world.body(self.setup.chassis) else {
            return;
        };
        let Some(chassis_motion) = world.motion(body.motion_id).copied() else {
            return;
        };
        let chassis_transform = body.transform;
        let chassis_mass = chassis_motion.mass;
        self.is_reversing = input.reverse;

        // -- updateBeforeCollisionDetection + ray-cast wheel collide --------
        let filter = QueryFilter::default().with_ignore_body(self.setup.chassis);
        let mut contacts: Vec<Option<(BodyId, Vector4, Vector4, f32)>> =
            Vec::with_capacity(self.wheels.len());
        for wheel in &self.wheels {
            let hardpoint_ws = chassis_transform.transform_point(wheel.hardpoint);
            let dir_ws = chassis_transform.transform_vector(wheel.suspension_direction);
            let ray_length = wheel.suspension_rest_length + wheel.wheel_radius;
            let hit = world.raycast_filtered(hardpoint_ws, dir_ws, ray_length, &filter);
            contacts.push(hit.map(|hit| (hit.body, hit.position, hit.normal, hit.hit_fraction)));
        }

        // -- updateWheels: contact outputs, steering, closing speed, spin ---
        for ((wheel, state), contact) in self
            .wheels
            .iter()
            .zip(self.state.iter_mut())
            .zip(contacts.iter())
        {
            *state = WheelState::empty();
            if let Some((hit_body, hit_point, hit_normal, hit_distance)) = contact {
                state.contact_body = Some(*hit_body);
                state.contact_point = *hit_point;
                state.contact_normal = *hit_normal;
                // `getCollisionOutputFromCastResult`:
                // currentSuspensionLength = hitDistance - wheelRadius
                // (the C++ hit fraction is normalized; `World::RayCastHit`
                // already reports the distance in world units).
                state.suspension_length = hit_distance - wheel.wheel_radius;
            } else {
                state.suspension_length = wheel.suspension_rest_length;
            }

            // Steering rotates the axle around the suspension axis
            // (`setSteeringAngle` semantics).
            let steer_angle = input.steering_angle * wheel.steering_angle_fraction;
            state.steer_angle = steer_angle;
            let dir_ws = chassis_transform.transform_vector(wheel.suspension_direction);
            let axle_ws = chassis_transform.transform_vector(wheel.axle);
            let steered_axle = if steer_angle != 0.0 {
                Quaternion::from_axis_angle(dir_ws, steer_angle).rotate(axle_ws)
            } else {
                axle_ws
            };
            let forward = dir_ws.cross(steered_axle);
            let forward_length = forward.length3();
            state.forward_dir = if forward_length > 1e-10 {
                forward * (1.0 / forward_length)
            } else {
                Vector4::ZERO
            };

            if state.contact_body.is_some() {
                // Closing speed in the SDK convention: relative velocity along
                // the suspension direction, negative while compressing.
                let velocity = velocity_at(&chassis_motion, state.contact_point);
                state.closing_speed = -velocity.dot3(dir_ws);
                // Ideal rolling spin used by the transmission RPM
                // (`m_noSlipIdealSpinVelocity`): the chassis speed along the
                // forward direction divided by the wheel radius.
                let ground_velocity = match state.contact_body {
                    Some(hit_body) => ground_velocity_at(world, hit_body, state.contact_point),
                    None => Vector4::ZERO,
                };
                let rolling = velocity - ground_velocity;
                state.spin_velocity = if wheel.wheel_radius > 0.0 {
                    rolling.dot3(state.forward_dir) / wheel.wheel_radius
                } else {
                    0.0
                };
            }
        }

        // -- updateTransmission → updateEngine → updateBrake -----------------
        let transmission = self.update_transmission();
        let engine = calc_engine(
            &self.setup.engine,
            transmission.transmission_rpm,
            input.throttle,
        );
        self.rpm = engine.rpm;
        self.torque = engine.torque;
        let transmitted_torque = engine.torque * transmission.ratio;

        // -- updateSuspension (translated `calcSuspension`) ------------------
        for (wheel, state) in self.wheels.iter().zip(self.state.iter_mut()) {
            if state.contact_body.is_none() {
                state.suspension_force = 0.0;
                continue;
            }
            let length_diff = wheel.suspension_rest_length - state.suspension_length;
            let mut force = wheel.spring_strength * length_diff;
            // Negative closing speed = compressing (SDK convention).
            let damping = if state.closing_speed < 0.0 {
                wheel.damping_compression
            } else {
                wheel.damping_relaxation
            };
            force -= damping * state.closing_speed;
            state.suspension_force = (force * chassis_mass).max(0.0);
        }

        // -- simulateVehicle: suspension impulses at the hardpoints ----------
        for (wheel, state) in self.wheels.iter().zip(self.state.iter_mut()) {
            if state.suspension_force <= 0.0 {
                continue;
            }
            let dir_ws = chassis_transform.transform_vector(wheel.suspension_direction);
            let hardpoint_ws = chassis_transform.transform_point(wheel.hardpoint);
            let impulse = dir_ws * (-state.suspension_force * delta_time);
            apply_impulse_to_body(world, self.setup.chassis, hardpoint_ws, impulse);
            if let Some(hit_body) = state.contact_body {
                apply_impulse_to_body(world, hit_body, state.contact_point, -impulse);
            }
        }

        // -- simulateVehicle: friction solver --------------------------------
        for (wheel, state) in self.wheels.iter().zip(self.state.iter_mut()) {
            let Some(hit_body) = state.contact_body else {
                state.forward_impulse = 0.0;
                state.side_impulse = 0.0;
                continue;
            };
            if state.forward_dir == Vector4::ZERO {
                continue;
            }
            let ground_velocity = ground_velocity_at(world, hit_body, state.contact_point);
            let chassis_velocity = velocity_at(&chassis_motion, state.contact_point);
            let slip = ground_velocity - chassis_velocity;
            let forward_slip = slip.dot3(state.forward_dir);
            let side_dir = chassis_transform.transform_vector(wheel.axle);
            let side_slip = slip.dot3(side_dir);

            let forward_force =
                drive_force(wheel, &self.setup, input, transmitted_torque, forward_slip);
            let forward_mass =
                effective_mass(&chassis_motion, state.contact_point, state.forward_dir);
            let side_mass = effective_mass(&chassis_motion, state.contact_point, side_dir);

            // Drive/brake force as a forward impulse; braking (same direction
            // as the slip) must never overshoot past the rolling condition.
            let mut forward_impulse = delta_time * forward_force;
            let stop_impulse = forward_slip * forward_mass;
            if forward_impulse * forward_slip > 0.0 && forward_impulse.abs() > stop_impulse.abs() {
                forward_impulse = stop_impulse;
            }
            // Lateral friction cancels the side slip (`J = slip·m_eff`).
            let side_impulse = side_slip * side_mass;

            // Friction circle clamp (`hkVehicleFrictionSolverClipImpulse`
            // simplified to a circular clamp).
            let max_impulse = state.suspension_force.abs() * wheel.tyre_friction * delta_time;
            let (forward_impulse, side_impulse) =
                clip_friction_circle(forward_impulse, side_impulse, max_impulse);
            state.forward_impulse = forward_impulse;
            state.side_impulse = side_impulse;

            let impulse = state.forward_dir * forward_impulse + side_dir * side_impulse;
            apply_impulse_to_body(world, self.setup.chassis, state.contact_point, impulse);
            apply_impulse_to_body(world, hit_body, state.contact_point, -impulse);
        }
    }

    /// Update the transmission state and return the outputs
    /// (`hknpVehicleDefaultTransmission::calcTransmission` subset).
    fn update_transmission(&mut self) -> TransmissionOutput {
        // Wheel-spin weighted average (`m_noSlipIdealSpinVelocity`).
        let mut avg_spin = 0.0;
        for (wheel, state) in self.wheels.iter().zip(self.state.iter()) {
            avg_spin += wheel.torque_ratio * state.spin_velocity;
        }
        let current_ratio = if self.is_reversing {
            -self.setup.transmission.reverse_ratio * self.setup.transmission.primary_ratio
        } else {
            self.setup.transmission.gears[self.gear] * self.setup.transmission.primary_ratio
        };
        let transmission_rpm = (avg_spin * current_ratio * 60.0 / core::f32::consts::TAU).abs();

        // Automatic shifts at the RPM thresholds (immediate; see the module
        // deviations).
        if !self.is_reversing {
            if transmission_rpm > self.setup.transmission.upshift_rpm
                && self.gear + 1 < self.setup.transmission.gears.len()
            {
                self.gear += 1;
            } else if transmission_rpm < self.setup.transmission.downshift_rpm && self.gear > 0 {
                self.gear -= 1;
            }
        }
        let ratio = if self.is_reversing {
            current_ratio
        } else {
            self.setup.transmission.gears[self.gear] * self.setup.transmission.primary_ratio
        };
        TransmissionOutput {
            transmission_rpm,
            ratio,
        }
    }
}

/// Drive force at the contact point along the forward direction
/// (`totalLinearForceAtWheel = (brakeTorque + transmissionTorque) / radius`).
///
/// `forward_slip` is the (ground − chassis) velocity along the forward
/// direction; braking opposes the chassis motion, i.e. it shares the sign of
/// `forward_slip`. The handbrake contributes its torque on handbrake wheels.
fn drive_force(
    wheel: &WheelSetup,
    setup: &VehicleSetup,
    input: &DriverInput,
    transmitted_torque: f32,
    forward_slip: f32,
) -> f32 {
    let drive = if wheel.torque_ratio != 0.0 && wheel.wheel_radius > 0.0 {
        transmitted_torque * wheel.torque_ratio / wheel.wheel_radius
    } else {
        0.0
    };
    let brake_torque = input.brake * setup.max_brake_torque
        + if input.handbrake && wheel.handbrake {
            setup.handbrake_torque
        } else {
            0.0
        };
    let brake = if wheel.wheel_radius > 0.0 {
        brake_torque * forward_slip.signum() / wheel.wheel_radius
    } else {
        0.0
    };
    drive + brake
}

/// Engine output (`hknpVehicleDefaultEngine::EngineOutput`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineOutput {
    /// Engine RPM (after clutch slip).
    pub rpm: f32,
    /// Output torque (throttle-scaled minus resistance).
    pub torque: f32,
}

/// Translate `hknpVehicleDefaultEngine::calcEngineInfo` formula-for-formula.
pub fn calc_engine(engine: &EngineSetup, transmission_rpm: f32, throttle: f32) -> EngineOutput {
    let mut rpm = transmission_rpm;

    // Clutch slip at low transmission RPM.
    if rpm < engine.min_rpm {
        let slip = throttle * engine.clutch_slip_rpm;
        let slope_change_point_x = 0.5 * engine.min_rpm;
        if rpm < slope_change_point_x {
            rpm = engine.min_rpm + slip;
        } else {
            rpm = engine.min_rpm
                + slip * (rpm - slope_change_point_x) / (engine.min_rpm - slope_change_point_x);
        }
    }

    let dx = rpm - engine.opt_rpm;
    let (torque, mut resistance);
    if dx < 0.0 {
        let inv_dmin = 1.0 / (engine.min_rpm - engine.opt_rpm);
        let resistance_slope =
            (engine.resistance_factor_at_min_rpm - engine.resistance_factor_at_opt_rpm) * inv_dmin;
        let torque_par_factor = (engine.torque_factor_at_min_rpm - 1.0) * inv_dmin * inv_dmin;
        resistance =
            engine.max_torque * (dx * resistance_slope + engine.resistance_factor_at_opt_rpm);
        torque = engine.max_torque * (dx * dx * torque_par_factor + 1.0);
        if transmission_rpm < engine.min_rpm {
            // Very slow speeds scale the resistance down.
            resistance *= transmission_rpm / engine.min_rpm;
        }
    } else if rpm < engine.max_rpm {
        let inv_dmax = 1.0 / (engine.max_rpm - engine.opt_rpm);
        let resistance_slope =
            (engine.resistance_factor_at_max_rpm - engine.resistance_factor_at_opt_rpm) * inv_dmax;
        let torque_par_factor = (engine.torque_factor_at_max_rpm - 1.0) * inv_dmax * inv_dmax;
        resistance =
            engine.max_torque * (dx * resistance_slope + engine.resistance_factor_at_opt_rpm);
        torque = engine.max_torque * (dx * dx * torque_par_factor + 1.0);
    } else {
        // Max RPM exceeded.
        rpm = engine.max_rpm;
        torque = 0.0;
        resistance = engine.resistance_factor_at_max_rpm * engine.max_torque;
    }

    EngineOutput {
        rpm,
        torque: torque * throttle - resistance,
    }
}

/// Velocity of a body at a world-space point (`v + ω × r`).
fn velocity_at(motion: &Motion, point: Vector4) -> Vector4 {
    let r = point - motion.position;
    motion.linear_velocity + motion.angular_velocity.cross(r)
}

/// Ground body velocity at `point` (`ZERO` for static bodies).
fn ground_velocity_at(world: &World, body: BodyId, point: Vector4) -> Vector4 {
    let Some(body) = world.body(body) else {
        return Vector4::ZERO;
    };
    if body.is_static() {
        return Vector4::ZERO;
    }
    match world.motion(body.motion_id) {
        Some(motion) => velocity_at(motion, point),
        None => Vector4::ZERO,
    }
}

/// Effective mass of a body along `dir` at `point`
/// (`1 / (invMass + (I⁻¹(r×d) × r)·d)` using the diagonal inverse inertia in
/// motion space).
fn effective_mass(motion: &Motion, point: Vector4, dir: Vector4) -> f32 {
    let inverse_mass = motion.inverse_mass();
    if inverse_mass == 0.0 {
        return 0.0;
    }
    let r = point - motion.position;
    let rxn = r.cross(dir);
    let tau_motion = motion.orientation.rotate_inverse(rxn);
    let omega_motion = Vector4::from_xyz(
        tau_motion.x * motion.inverse_inertia.x,
        tau_motion.y * motion.inverse_inertia.y,
        tau_motion.z * motion.inverse_inertia.z,
    );
    let omega_world = motion.orientation.rotate(omega_motion);
    let angular_term = omega_world.cross(r).dot3(dir);
    let total = inverse_mass + angular_term;
    if total > 1e-12 { 1.0 / total } else { 0.0 }
}

/// Impulse applied at `point` to the body's motion, then written back through
/// the world (linear + angular velocity, auto wake).
fn apply_impulse_to_body(world: &mut World, body: BodyId, point: Vector4, impulse: Vector4) {
    // Copy the motion and id first so the immutable borrow is released before
    // the velocity write-back below.
    let Some(body_ref) = world.body(body) else {
        return;
    };
    let body_id = body_ref.id;
    let Some(motion) = world.motion(body_ref.motion_id).copied() else {
        return;
    };
    if motion.inverse_mass() == 0.0 {
        return;
    }
    let r = point - motion.position;
    let torque = r.cross(impulse);
    let tau_motion = motion.orientation.rotate_inverse(torque);
    let delta_omega_motion = Vector4::from_xyz(
        tau_motion.x * motion.inverse_inertia.x,
        tau_motion.y * motion.inverse_inertia.y,
        tau_motion.z * motion.inverse_inertia.z,
    );
    let delta_omega = motion.orientation.rotate(delta_omega_motion);
    let new_linear = motion.linear_velocity + impulse * motion.inverse_mass();
    let new_angular = motion.angular_velocity + delta_omega;
    world.set_body_linear_velocity(body_id, new_linear);
    world.set_body_angular_velocity(body_id, new_angular);
}

/// Circular friction clamp (`hkVehicleFrictionSolverClipImpulse` subset):
/// scales `(forward, side)` back into the friction circle when their length
/// exceeds `max_impulse`.
fn clip_friction_circle(forward: f32, side: f32, max_impulse: f32) -> (f32, f32) {
    let length_squared = forward * forward + side * side;
    if max_impulse <= 0.0 || length_squared <= max_impulse * max_impulse {
        return (forward, side);
    }
    let scale = max_impulse / length_squared.sqrt();
    (forward * scale, side * scale)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::phys_types::MotionType;

    fn default_engine() -> EngineSetup {
        EngineSetup::default()
    }

    #[test]
    fn engine_torque_peaks_at_opt_rpm() {
        let engine = default_engine();
        let out = calc_engine(&engine, 4000.0, 1.0);
        assert!((out.rpm - 4000.0).abs() < 1e-4);
        assert!(
            (out.torque - 800.0).abs() < 1e-3,
            "peak torque {}",
            out.torque
        );
    }

    #[test]
    fn engine_torque_zero_without_throttle() {
        let engine = default_engine();
        let out = calc_engine(&engine, 4000.0, 0.0);
        assert!(out.torque.abs() < 1e-4, "{}", out.torque);
    }

    #[test]
    fn engine_clutch_slip_lifts_low_rpm() {
        let engine = default_engine();
        let out = calc_engine(&engine, 0.0, 1.0);
        // Transmission RPM 0 < min/2 → rpm = minRPM + clutchSlipRPM.
        assert!((out.rpm - 3000.0).abs() < 1e-3, "{}", out.rpm);
    }

    #[test]
    fn engine_clutch_slip_scales_between_half_and_min_rpm() {
        let engine = default_engine();
        // transmission rpm = 750 = min/2 + 250 → interpolation factor 250/500.
        let out = calc_engine(&engine, 750.0, 1.0);
        assert!((out.rpm - 2000.0).abs() < 1e-3, "{}", out.rpm);
    }

    #[test]
    fn engine_revs_capped_at_max_rpm() {
        let engine = default_engine();
        let out = calc_engine(&engine, 8000.0, 1.0);
        assert!((out.rpm - 6000.0).abs() < 1e-4);
        assert!(out.torque.abs() < 1e-3, "{}", out.torque);
    }

    #[test]
    fn engine_resistance_reduces_torque() {
        let engine = EngineSetup {
            resistance_factor_at_opt_rpm: 0.2,
            ..EngineSetup::default()
        };
        let out = calc_engine(&engine, 4000.0, 1.0);
        assert!(
            (out.torque - (800.0 - 160.0)).abs() < 1e-2,
            "{}",
            out.torque
        );
    }

    #[test]
    fn friction_circle_clamps_only_over_max() {
        let (f, s) = clip_friction_circle(3.0, 4.0, 10.0);
        assert_eq!((f, s), (3.0, 4.0));
        let (f, s) = clip_friction_circle(3.0, 4.0, 5.0);
        let length = (f * f + s * s).sqrt();
        assert!((length - 5.0).abs() < 1e-5);
        assert!((f - 3.0).abs() < 1e-5);
        assert!((s - 4.0).abs() < 1e-5);
    }

    #[test]
    fn effective_mass_matches_inverse_mass_without_inertia() {
        let mut motion = Motion::new(
            MotionType::Dynamic,
            Vector4::ZERO,
            Quaternion::IDENTITY,
            Vector4::ZERO,
            Vector4::ZERO,
            2.0,
        );
        motion.inverse_inertia = Vector4::ZERO;
        let mass = effective_mass(&motion, Vector4::UNIT_X, Vector4::UNIT_Y);
        assert!((mass - 2.0).abs() < 1e-5);
    }

    #[test]
    fn effective_mass_is_zero_for_static_motion() {
        let motion = Motion::static_motion();
        let mass = effective_mass(&motion, Vector4::UNIT_X, Vector4::UNIT_Y);
        assert_eq!(mass, 0.0);
    }

    #[test]
    fn vehicle_setup_validates_input() {
        let setup = VehicleSetup::new(BodyId::INVALID);
        assert!(VehicleInstance::new(setup).is_err());
        let setup = VehicleSetup::new(BodyId::new(0));
        assert!(VehicleInstance::new(setup).is_err());
    }

    #[test]
    fn transmission_upshifts_and_downshifts_at_thresholds() {
        let chassis = BodyId::new(1);
        let mut setup = VehicleSetup::new(chassis);
        setup
            .wheels
            .push(WheelSetup::new(Vector4::ZERO, 0.3).with_torque_ratio(1.0));
        let mut vehicle = VehicleInstance::new(setup).unwrap();

        // Spin 200 rad/s × gear0 ratio 3 → 5729 rpm > upshift 5500 → gear 1.
        vehicle.state[0].spin_velocity = 200.0;
        let out = vehicle.update_transmission();
        assert_eq!(vehicle.gear(), 1);
        assert!((out.ratio - 2.0).abs() < 1e-4);

        // Lower spin in gear 1 (3800 rpm) stays; below 2000 rpm downshifts.
        vehicle.state[0].spin_velocity = 150.0;
        let out = vehicle.update_transmission();
        assert_eq!(vehicle.gear(), 1);
        assert!((out.ratio - 2.0).abs() < 1e-4);
        vehicle.state[0].spin_velocity = 50.0;
        let out = vehicle.update_transmission();
        assert_eq!(vehicle.gear(), 0);
        assert!((out.ratio - 3.0).abs() < 1e-4);
    }
}
