//! Ragdoll state machine and powered-state motor blender subset.
//!
//! Rust port of `Extensions/Ragdoll/StateMachine/hknpRagdollStateManager`,
//! the `Keyframed` / `Dynamic` / `PoweredDynamicAnimation` states and the
//! timed transition described by `hknpRagdollStateAutoTransitionTimed`.
//!
//! The SDK's powered state drives `hknpRagdollConstraintData` position motors.
//! This port's `RagdollJoint` is a limits-only row set, so the blender computes
//! the SDK motor parameters as data and the pose is driven with the
//! already-ported `Ragdoll::drive_to_pose` velocity
//! blend. Body tags are represented by per-bone strengths (zero disables the
//! bone); `keyframeRoot` locks the root body with an exact keyframe.

use base_math::Transform;
use phys_types::MotionType;
use phys_world::World;

use crate::ragdoll::{Ragdoll, RagdollError};
use crate::ragdoll_penetration::{
    BodyHierarchy, DetectionOutput, PenetrationInput, detect_and_resolve_penetration,
};

/// Ragdoll states mirroring the SDK's basic `hknpRagdollState::StateId`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RagdollState {
    /// Follow the animation exactly: bodies are keyframed and teleported.
    Keyframed,
    /// Let the simulation own the bodies; no pose driving is performed.
    Dynamic,
    /// Dynamic bodies driven toward the animation with blended motor strength.
    Powered,
}

/// `hknpRagdollStatePoweredDynamicAnimation::Cinfo` motor-parameter limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RagdollMotorParams {
    /// Lower `maxForce` limit (SDK default `10`).
    pub max_force_lower_limit: f32,
    /// Upper `maxForce` limit (SDK default `1000`).
    pub max_force_upper_limit: f32,
    /// Lower `tau` limit (SDK default `0.1`).
    pub tau_lower_limit: f32,
    /// Upper `tau` limit (SDK default `0.8`).
    pub tau_upper_limit: f32,
    /// Lower `damping` limit (SDK default `0.1`).
    pub damping_lower_limit: f32,
    /// Upper `damping` limit (SDK default `1.0`).
    pub damping_upper_limit: f32,
    /// `hknpPositionConstraintMotor::m_proportionalRecoveryVelocity` (default `1`).
    pub proportional_recovery_velocity: f32,
    /// `hknpPositionConstraintMotor::m_constantRecoveryVelocity` (default `2`).
    pub constant_recovery_velocity: f32,
}

impl Default for RagdollMotorParams {
    fn default() -> Self {
        Self {
            max_force_lower_limit: 10.0,
            max_force_upper_limit: 1000.0,
            tau_lower_limit: 0.1,
            tau_upper_limit: 0.8,
            damping_lower_limit: 0.1,
            damping_upper_limit: 1.0,
            proportional_recovery_velocity: 1.0,
            constant_recovery_velocity: 2.0,
        }
    }
}

impl RagdollMotorParams {
    /// `updatePositionMotorProperties`: `lerp(lower, upper, sqrt(strength))`.
    ///
    /// Recovery velocities pass through unscaled, exactly as in the SDK.
    pub fn blend(&self, strength: f32) -> BlendedRagdollMotor {
        let t = strength.clamp(0.0, 1.0).sqrt();
        let lerp = |lower: f32, upper: f32| lower * (1.0 - t) + upper * t;
        BlendedRagdollMotor {
            max_force: lerp(self.max_force_lower_limit, self.max_force_upper_limit),
            tau: lerp(self.tau_lower_limit, self.tau_upper_limit),
            damping: lerp(self.damping_lower_limit, self.damping_upper_limit),
            proportional_recovery_velocity: self.proportional_recovery_velocity,
            constant_recovery_velocity: self.constant_recovery_velocity,
        }
    }
}

/// Motor parameters produced by [`RagdollMotorParams::blend`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BlendedRagdollMotor {
    /// Blended position-motor maximum force.
    pub max_force: f32,
    /// Blended position-motor time constant.
    pub tau: f32,
    /// Blended position-motor damping.
    pub damping: f32,
    /// Proportional recovery velocity (unscaled).
    pub proportional_recovery_velocity: f32,
    /// Constant recovery velocity (unscaled).
    pub constant_recovery_velocity: f32,
}

/// Construction info for [`RagdollController`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RagdollControllerCinfo {
    /// State entered by [`RagdollController::new`] (default `Dynamic`).
    pub initial_state: RagdollState,
    /// Whether the powered state locks its root body (SDK default `true`).
    pub keyframe_root: bool,
    /// Powered motor-parameter limits.
    pub motor: RagdollMotorParams,
}

impl Default for RagdollControllerCinfo {
    fn default() -> Self {
        Self {
            initial_state: RagdollState::Dynamic,
            keyframe_root: true,
            motor: RagdollMotorParams::default(),
        }
    }
}

impl RagdollControllerCinfo {
    /// Set the initial state.
    #[must_use]
    pub fn with_initial_state(mut self, state: RagdollState) -> Self {
        self.initial_state = state;
        self
    }

    /// Enable or disable the powered root keyframe.
    #[must_use]
    pub fn with_keyframe_root(mut self, keyframe_root: bool) -> Self {
        self.keyframe_root = keyframe_root;
        self
    }

    /// Override the motor-parameter limits.
    #[must_use]
    pub fn with_motor(mut self, motor: RagdollMotorParams) -> Self {
        self.motor = motor;
        self
    }
}

#[derive(Clone, Copy, Debug)]
struct TimedTransition {
    final_state: RagdollState,
    elapsed: f32,
    duration: f32,
}

/// Stateful driver for a [`Ragdoll`] (`hknpRagdollStateManager` subset).
///
/// Call [`RagdollController::step`] before `World::step`, as with the vehicle
/// extension. Bone strengths stand in for the SDK's body tag mask; a bone with
/// strength `0` is excluded from state changes and pose driving.
#[derive(Clone, Debug)]
pub struct RagdollController {
    ragdoll: Ragdoll,
    state: RagdollState,
    strength: f32,
    bone_strengths: Vec<f32>,
    motor: RagdollMotorParams,
    keyframe_root: bool,
    transition: Option<TimedTransition>,
    last_motor: Option<BlendedRagdollMotor>,
    penetration: Option<PenetrationInput>,
    penetration_resolved: bool,
    last_penetration: DetectionOutput,
}

impl RagdollController {
    /// Bind a controller to a ragdoll and enter `cinfo.initial_state`.
    pub fn new(world: &mut World, ragdoll: Ragdoll, cinfo: RagdollControllerCinfo) -> Self {
        let bone_strengths = vec![1.0; ragdoll.bones().len()];
        let initial_state = cinfo.initial_state;
        let mut controller = Self {
            ragdoll,
            state: initial_state,
            strength: 1.0,
            bone_strengths,
            motor: cinfo.motor,
            keyframe_root: cinfo.keyframe_root,
            transition: None,
            last_motor: None,
            penetration: None,
            penetration_resolved: false,
            last_penetration: DetectionOutput::default(),
        };
        controller.set_state(world, initial_state);
        controller
    }

    /// Current state.
    pub fn state(&self) -> RagdollState {
        self.state
    }

    /// Current drive strength in `[0, 1]`.
    pub fn strength(&self) -> f32 {
        self.strength
    }

    /// Motor parameters used by the most recent powered step, if any.
    pub fn last_motor(&self) -> Option<BlendedRagdollMotor> {
        self.last_motor
    }

    /// Bone strengths, in bone order (missing entries in driving default to `1`).
    pub fn bone_strengths(&self) -> &[f32] {
        &self.bone_strengths
    }

    /// Most recent penetration-detection result.
    pub fn last_penetration(&self) -> &DetectionOutput {
        &self.last_penetration
    }

    /// Enable or disable penetration resolution before each state step.
    ///
    /// Once a clean detection snapshot is obtained, resolution is skipped
    /// until [`RagdollController::set_state`] or this method is called again.
    pub fn set_penetration_input(&mut self, input: Option<PenetrationInput>) {
        self.penetration = input;
        self.penetration_resolved = false;
    }

    /// Clamp and set the drive strength (`hknpRagdollController::setStrength`).
    pub fn set_strength(&mut self, strength: f32) {
        self.strength = if strength.is_finite() {
            strength.clamp(0.0, 1.0)
        } else {
            0.0
        };
    }

    /// Set one bone's strength. Like the SDK's `setBoneStrength`, the value is
    /// not clamped; `0` disables the bone for state changes and pose driving.
    pub fn set_bone_strength(&mut self, bone_index: usize, strength: f32) -> bool {
        let Some(entry) = self.bone_strengths.get_mut(bone_index) else {
            return false;
        };
        *entry = strength;
        true
    }

    /// Enter a state immediately, cancelling any pending timed transition.
    ///
    /// Tagged (strength > 0) bodies switch motion type: keyframed for
    /// `Keyframed`, dynamic for `Dynamic` and `Powered` (except the powered
    /// root when `keyframe_root` is set).
    pub fn set_state(&mut self, world: &mut World, state: RagdollState) {
        self.transition = None;
        self.state = state;
        self.penetration_resolved = false;
        let keyframe_root = self.keyframe_root;
        for (index, bone) in self.ragdoll.bones().iter().enumerate() {
            let Some(body) = bone.body else {
                continue;
            };
            if self.bone_strengths.get(index).copied().unwrap_or(1.0) <= 0.0 {
                continue;
            }
            let motion_type = match state {
                RagdollState::Keyframed => MotionType::Keyframed,
                RagdollState::Dynamic => MotionType::Dynamic,
                RagdollState::Powered if keyframe_root && index == 0 => MotionType::Keyframed,
                RagdollState::Powered => MotionType::Dynamic,
            };
            world.set_body_motion_type(body, motion_type);
        }
    }

    /// Queue a `hknpRagdollStateAutoTransitionTimed`-style blend.
    ///
    /// The `via` state is entered with strength 1 and faded to 0 over
    /// `blend_seconds`, then `final_state` is entered. Call this before
    /// `step`; each step advances the fade by its delta time.
    pub fn begin_timed_transition(
        &mut self,
        world: &mut World,
        via: RagdollState,
        final_state: RagdollState,
        blend_seconds: f32,
    ) -> Result<(), RagdollError> {
        if blend_seconds <= 0.0 || !blend_seconds.is_finite() {
            return Err(RagdollError::InvalidDuration);
        }
        self.set_state(world, via);
        self.transition = Some(TimedTransition {
            final_state,
            elapsed: 0.0,
            duration: blend_seconds,
        });
        self.strength = 1.0;
        Ok(())
    }

    /// Advance the active state by `delta_time`.
    ///
    /// `desired_local` holds parent-relative poses in bone order and
    /// `model_to_world` is the root's parent transform.
    pub fn step(
        &mut self,
        world: &mut World,
        desired_local: &[Transform],
        model_to_world: Transform,
        delta_time: f32,
    ) -> Result<(), RagdollError> {
        if delta_time <= 0.0 || !delta_time.is_finite() {
            return Err(RagdollError::InvalidDeltaTime);
        }

        if !self.penetration_resolved {
            if let Some(input) = self.penetration {
                let hierarchy =
                    BodyHierarchy::from_ragdoll(world, &self.ragdoll, Some(&self.bone_strengths));
                if detect_and_resolve_penetration(
                    world,
                    &hierarchy,
                    &input,
                    delta_time,
                    &mut self.last_penetration,
                ) {
                    self.penetration_resolved = true;
                }
            }
        }

        if let Some(transition) = self.transition {
            let remaining = transition.duration - transition.elapsed;
            if delta_time >= remaining {
                self.strength = 0.0;
                self.step_state(world, desired_local, model_to_world, delta_time)?;
                self.set_state(world, transition.final_state);
            } else {
                self.strength = (1.0 - transition.elapsed / transition.duration).clamp(0.0, 1.0);
                self.step_state(world, desired_local, model_to_world, delta_time)?;
                self.transition = Some(TimedTransition {
                    elapsed: transition.elapsed + delta_time,
                    ..transition
                });
            }
            return Ok(());
        }

        self.step_state(world, desired_local, model_to_world, delta_time)
    }

    fn step_state(
        &mut self,
        world: &mut World,
        desired_local: &[Transform],
        model_to_world: Transform,
        delta_time: f32,
    ) -> Result<(), RagdollError> {
        match self.state {
            RagdollState::Keyframed => {
                let desired = self
                    .ragdoll
                    .calc_desired_pose_world_space(desired_local, model_to_world)?;
                for (index, bone) in self.ragdoll.bones().iter().enumerate() {
                    if bone.body.is_some()
                        && self.bone_strengths.get(index).copied().unwrap_or(1.0) > 0.0
                    {
                        self.ragdoll.apply_hard_keyframe_bone(
                            world,
                            index,
                            desired[index],
                            delta_time,
                        )?;
                    }
                }
                Ok(())
            }
            RagdollState::Dynamic => {
                if desired_local.len() != self.ragdoll.bones().len() {
                    return Err(RagdollError::PoseLength {
                        expected: self.ragdoll.bones().len(),
                        actual: desired_local.len(),
                    });
                }
                Ok(())
            }
            RagdollState::Powered => {
                self.last_motor = Some(self.motor.blend(self.strength));
                self.ragdoll.drive_to_pose(
                    world,
                    desired_local,
                    model_to_world,
                    delta_time,
                    self.strength,
                    &self.bone_strengths,
                )?;
                if self.keyframe_root
                    && self.bone_strengths.first().copied().unwrap_or(1.0) > 0.0
                    && self
                        .ragdoll
                        .bones()
                        .first()
                        .and_then(|bone| bone.body)
                        .is_some()
                {
                    let desired = self
                        .ragdoll
                        .calc_desired_pose_world_space(desired_local, model_to_world)?;
                    self.ragdoll
                        .apply_hard_keyframe_bone(world, 0, desired[0], delta_time)?;
                }
                Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ragdoll::{RagdollData, RagdollLimits};
    use base_math::{Quaternion, Transform, Vector4};
    use phys_body::BodyCinfo;
    use phys_shape::{Shape, SphereShape};
    use phys_types::BodyId;
    use phys_world::WorldCinfo;
    use std::sync::Arc;

    fn sphere() -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(Vector4::ZERO, 0.5)) as Arc<dyn Shape>
    }

    fn two_bone_world() -> (World, Ragdoll, BodyId, BodyId) {
        let mut world = World::new(WorldCinfo::new());
        let root = world.create_body(
            &BodyCinfo::new(sphere())
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0),
        );
        let child = world.create_body(
            &BodyCinfo::new(sphere())
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(1.0, 0.0, 0.0)),
        );
        let mut data = RagdollData::new();
        let identity = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::ZERO);
        let offset =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.0, 0.0, 0.0));
        data.add_bone("root", None, Some(root), identity);
        data.bones[0].limits = RagdollLimits::default();
        data.add_bone("child", Some(0), Some(child), offset);
        let ragdoll = Ragdoll::create(&mut world, &data).unwrap();
        (world, ragdoll, root, child)
    }

    #[test]
    fn motor_blender_uses_sqrt_strength() {
        let motor = RagdollMotorParams::default();
        let blended = motor.blend(0.25);
        assert_eq!(blended.max_force, 505.0);
        assert!((blended.tau - 0.45).abs() < 1e-6);
        assert!((blended.damping - 0.55).abs() < 1e-6);
        assert_eq!(blended.proportional_recovery_velocity, 1.0);
        assert_eq!(blended.constant_recovery_velocity, 2.0);
        assert_eq!(motor.blend(0.0).max_force, 10.0);
        assert_eq!(motor.blend(1.0).max_force, 1000.0);
        assert_eq!(motor.blend(4.0), motor.blend(1.0));
    }

    #[test]
    fn set_state_switches_motion_types_and_honours_bone_strength() {
        let (mut world, ragdoll, root, child) = two_bone_world();
        let mut controller = RagdollController::new(
            &mut world,
            ragdoll,
            RagdollControllerCinfo::default().with_initial_state(RagdollState::Keyframed),
        );
        assert_eq!(controller.state(), RagdollState::Keyframed);
        assert!(world.body(root).unwrap().is_keyframed());
        assert!(world.body(child).unwrap().is_keyframed());

        controller.set_state(&mut world, RagdollState::Powered);
        assert!(world.body(root).unwrap().is_keyframed(), "keyframeRoot");
        assert!(world.body(child).unwrap().is_dynamic());

        assert!(controller.set_bone_strength(1, 0.0));
        controller.set_state(&mut world, RagdollState::Keyframed);
        assert!(
            world.body(child).unwrap().is_dynamic(),
            "disabled bone keeps motion type"
        );
        assert!(!controller.set_bone_strength(99, 1.0));
    }

    #[test]
    fn timed_transition_fades_strength_then_enters_final_state() {
        let (mut world, ragdoll, _, _) = two_bone_world();
        let identity = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::ZERO);
        let pose = vec![identity; 2];
        let mut controller = RagdollController::new(
            &mut world,
            ragdoll,
            RagdollControllerCinfo::default().with_initial_state(RagdollState::Dynamic),
        );
        controller
            .begin_timed_transition(
                &mut world,
                RagdollState::Powered,
                RagdollState::Dynamic,
                1.0,
            )
            .unwrap();

        // The SDK fades using elapsed time before incrementing it.
        controller.step(&mut world, &pose, identity, 0.25).unwrap();
        assert_eq!(controller.strength(), 1.0);
        assert_eq!(controller.state(), RagdollState::Powered);
        let powered_motor = controller.last_motor().unwrap();

        controller.step(&mut world, &pose, identity, 0.25).unwrap();
        assert!((controller.strength() - 0.75).abs() < 1e-6);
        assert!(controller.last_motor().unwrap().max_force < powered_motor.max_force);

        controller.step(&mut world, &pose, identity, 0.25).unwrap();
        controller.step(&mut world, &pose, identity, 0.25).unwrap();
        assert_eq!(controller.state(), RagdollState::Dynamic);
        assert_eq!(controller.strength(), 0.0);

        assert_eq!(
            controller.begin_timed_transition(
                &mut world,
                RagdollState::Powered,
                RagdollState::Dynamic,
                0.0,
            ),
            Err(RagdollError::InvalidDuration)
        );
    }

    #[test]
    fn keyframed_holds_exact_pose_and_powered_drives_below_full_strength() {
        let (mut world, ragdoll, root, child) = two_bone_world();
        let identity = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::ZERO);
        let target =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 2.0, 0.0));
        let child_offset =
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 1.0, 0.0));
        let pose = vec![target, child_offset];
        let mut controller = RagdollController::new(
            &mut world,
            ragdoll,
            RagdollControllerCinfo::default().with_initial_state(RagdollState::Keyframed),
        );
        for _ in 0..5 {
            controller
                .step(&mut world, &pose, identity, 1.0 / 60.0)
                .unwrap();
            world.step(1.0 / 60.0);
        }
        assert!(
            world
                .body_position(root)
                .unwrap()
                .all_equal3(Vector4::from_xyz(0.0, 2.0, 0.0), 1e-6)
        );

        controller.set_state(&mut world, RagdollState::Powered);
        controller.set_strength(0.5);
        let powered_pose = vec![
            target,
            Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(0.0, 2.0, 0.0)),
        ];
        controller
            .step(&mut world, &powered_pose, identity, 1.0 / 60.0)
            .unwrap();
        let velocity = world.body_linear_velocity(child).unwrap();
        assert!(velocity.y > 0.0 && velocity.y < 120.0);
        assert_eq!(
            controller.last_motor().unwrap().max_force,
            10.0 + 990.0 * 0.5f32.sqrt()
        );
    }
}
