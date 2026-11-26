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

use std::fmt;

use crate::base_math::{Transform, Vector4};
use crate::phys_types::MotionType;
use crate::phys_world::World;

use crate::phys_extensions::ragdoll::{Ragdoll, RagdollError};
use crate::phys_extensions::ragdoll_ease::{
    ConstraintEasingMode, DEFAULT_EASING_DURATION, EaseConstraintsAction,
};
use crate::phys_extensions::ragdoll_penetration::{
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
    /// A user-registered state (`hknpRagdollState::STATE_USER_0..9`).
    Custom(u16),
}

/// Number of user state slots, mirroring the SDK's ten `STATE_USER_*` ids.
pub const USER_STATE_COUNT: usize = 10;

/// Per-step data handed to a user-defined ragdoll state.
pub struct RagdollStateContext<'a> {
    /// World to drive.
    pub world: &'a mut World,
    /// Ragdoll whose tagged bones the state may drive.
    pub ragdoll: &'a Ragdoll,
    /// Per-bone strengths (the SDK's tag mask subset).
    pub bone_strengths: &'a [f32],
    /// Current blend strength of the state manager.
    pub strength: f32,
    /// Parent-relative desired poses, in bone order.
    pub desired_local: &'a [Transform],
    /// Root's parent transform.
    pub model_to_world: Transform,
    /// Step size.
    pub delta_time: f32,
}

impl RagdollStateContext<'_> {
    /// Drive the tagged bones toward the desired pose with `strength`
    /// (`Ragdoll::drive_to_pose`).
    pub fn drive_pose(&mut self, strength: f32) -> Result<(), RagdollError> {
        self.ragdoll.drive_to_pose(
            self.world,
            self.desired_local,
            self.model_to_world,
            self.delta_time,
            strength,
            self.bone_strengths,
        )
    }
}

/// Behavior hooks of a user-defined ragdoll state (`hknpRagdollState` subset).
///
/// The 2021.2 SDK has no virtual `getNextState` or `createAction`: transitions
/// are queued via `queueStateChange` and the state manager owns the easing
/// action. A custom state therefore only reports a requested successor from
/// [`Self::step`] and declares its easing mode through [`Self::easing`].
pub trait RagdollStateBehavior: fmt::Debug {
    /// Called once when the state is registered.
    fn initialize(&mut self, _world: &mut World) {}
    /// Called whenever the state manager's strength changes.
    fn set_strength(&mut self, _strength: f32) {}
    /// Called when this state is entered.
    fn enter(&mut self, _world: &mut World, _previous: RagdollState, _tag_mask: u32) {}
    /// Called when this state is left.
    fn leave(&mut self, _world: &mut World, _next: RagdollState) {}
    /// Advance the state. Returning `Some(state)` queues a successor for the
    /// end of this step.
    fn step(
        &mut self,
        _context: &mut RagdollStateContext<'_>,
    ) -> Result<Option<RagdollState>, RagdollError> {
        Ok(None)
    }
    /// Constraint easing mode and duration (`hknpRagdollState::Cinfo` subset).
    fn easing(&self) -> (ConstraintEasingMode, f32) {
        (ConstraintEasingMode::Enforce, DEFAULT_EASING_DURATION)
    }
    /// Explicit clone used when a controller is cloned.
    fn clone_state(&self) -> Box<dyn RagdollStateBehavior>;
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

#[derive(Clone, Copy, Debug)]
struct AtRestTransition {
    final_state: RagdollState,
    linear_tolerance: f32,
    angular_tolerance: f32,
}

/// Stateful driver for a [`Ragdoll`] (`hknpRagdollStateManager` subset).
///
/// Call [`RagdollController::step`] before `World::step`, as with the vehicle
/// extension. Bone strengths stand in for the SDK's body tag mask; a bone with
/// strength `0` is excluded from state changes and pose driving.
#[derive(Debug)]
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
    easing: Option<EaseConstraintsAction>,
    at_rest: Option<AtRestTransition>,
    /// User states keyed by `STATE_USER_0..9`; slots are never reordered.
    states: Vec<Option<Box<dyn RagdollStateBehavior>>>,
    /// State requested by a custom state's `step`, applied at step end.
    queued_state: Option<RagdollState>,
}

impl Clone for RagdollController {
    fn clone(&self) -> Self {
        Self {
            ragdoll: self.ragdoll.clone(),
            state: self.state,
            strength: self.strength,
            bone_strengths: self.bone_strengths.clone(),
            motor: self.motor,
            keyframe_root: self.keyframe_root,
            transition: self.transition,
            last_motor: self.last_motor,
            penetration: self.penetration,
            penetration_resolved: self.penetration_resolved,
            last_penetration: self.last_penetration.clone(),
            easing: self.easing.clone(),
            at_rest: self.at_rest,
            states: self
                .states
                .iter()
                .map(|state| state.as_ref().map(|state| state.clone_state()))
                .collect(),
            queued_state: self.queued_state,
        }
    }
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
            easing: None,
            at_rest: None,
            states: (0..USER_STATE_COUNT).map(|_| None).collect(),
            queued_state: None,
        };
        controller.set_state(world, initial_state);
        controller
    }

    /// Current state.
    pub fn state(&self) -> RagdollState {
        self.state
    }

    /// Register a user state in one of the ten SDK user-state slots.
    pub fn register_state(
        &mut self,
        world: &mut World,
        id: u16,
        mut state: impl RagdollStateBehavior + 'static,
    ) -> Result<(), RagdollError> {
        let index = usize::from(id);
        if index >= USER_STATE_COUNT || self.states[index].is_some() {
            return Err(RagdollError::InvalidState(id));
        }
        state.initialize(world);
        self.states[index] = Some(Box::new(state));
        Ok(())
    }

    /// Remove a user state. If it is active, the controller falls back to
    /// [`RagdollState::Dynamic`] on the next state step.
    pub fn unregister_state(&mut self, id: u16) -> bool {
        let index = usize::from(id);
        if index >= USER_STATE_COUNT || self.states[index].is_none() {
            return false;
        }
        self.states[index] = None;
        true
    }

    /// Whether a user state is registered in this slot.
    pub fn has_custom_state(&self, id: u16) -> bool {
        usize::from(id) < USER_STATE_COUNT && self.states[usize::from(id)].is_some()
    }

    /// Queue a state change for the end of the next [`Self::step`], mirroring
    /// `hknpRagdollStateManager::queueStateChange`.
    pub fn queue_state_change(&mut self, state: RagdollState) {
        self.queued_state = Some(state);
    }

    fn easing_for(&self, state: RagdollState) -> (ConstraintEasingMode, f32) {
        match state {
            RagdollState::Keyframed | RagdollState::Powered => {
                (ConstraintEasingMode::Enforce, DEFAULT_EASING_DURATION)
            }
            RagdollState::Dynamic => (
                ConstraintEasingMode::LoosenThenRestore,
                DEFAULT_EASING_DURATION,
            ),
            RagdollState::Custom(id) => usize::from(id)
                .lt(&USER_STATE_COUNT)
                .then(|| self.states[usize::from(id)].as_ref())
                .flatten()
                .map(|state| state.easing())
                .unwrap_or((ConstraintEasingMode::Enforce, DEFAULT_EASING_DURATION)),
        }
    }

    fn custom_state_mut(&mut self, id: u16) -> Option<&mut Box<dyn RagdollStateBehavior>> {
        let index = usize::from(id);
        if index >= USER_STATE_COUNT {
            return None;
        }
        self.states[index].as_mut()
    }

    fn tag_mask(&self) -> u32 {
        self.bone_strengths
            .iter()
            .enumerate()
            .fold(0u32, |mask, (index, &strength)| {
                if strength > 0.0 && index < 32 {
                    mask | (1u32 << index)
                } else {
                    mask
                }
            })
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

    /// Active constraint-easing action, if the current state has one.
    pub fn easing(&self) -> Option<&EaseConstraintsAction> {
        self.easing.as_ref()
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
        let strength = if strength.is_finite() {
            strength.clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.strength = strength;
        if let RagdollState::Custom(id) = self.state {
            if let Some(state) = self.custom_state_mut(id) {
                state.set_strength(strength);
            }
        }
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
        let tag_mask = self.tag_mask();
        let previous = self.state;
        if let RagdollState::Custom(id) = previous {
            if let Some(behavior) = self.custom_state_mut(id) {
                behavior.leave(world, state);
            }
        }
        if let RagdollState::Custom(id) = state {
            if !self.has_custom_state(id) {
                return;
            }
        }
        if let Some(mut action) = self.easing.take() {
            // The SDK checks the *incoming* state's easing mode: entering an
            // enforcing state restores immediately, entering another loosened
            // state drops the old action without restoring.
            if self.easing_for(state).0 == ConstraintEasingMode::Enforce {
                action.restore_now(world);
            }
        }
        self.transition = None;
        self.at_rest = None;
        self.queued_state = None;
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
                RagdollState::Powered | RagdollState::Custom(_) => MotionType::Dynamic,
            };
            world.set_body_motion_type(body, motion_type);
        }

        if let RagdollState::Custom(id) = state {
            if let Some(behavior) = self.custom_state_mut(id) {
                behavior.enter(world, previous, tag_mask);
            }
        }

        let (mode, duration) = self.easing_for(state);
        if mode != ConstraintEasingMode::Enforce {
            let ids: Vec<_> = self
                .ragdoll
                .constraints()
                .iter()
                .flatten()
                .copied()
                .collect();
            let mut action = EaseConstraintsAction::new(world, &ids);
            action.loosen(world);
            if mode == ConstraintEasingMode::LoosenThenRestore {
                action.restore(world, duration);
            }
            self.easing = Some(action);
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

    /// Queue an `hknpRagdollStateAutoTransitionAtRest`-style transition.
    ///
    /// The `via` state is entered immediately; after each step, the final
    /// state is entered once every tagged body's linear/angular velocity is
    /// within the given component tolerances (SDK defaults `1e-2` / `1e-1`).
    pub fn begin_at_rest_transition(
        &mut self,
        world: &mut World,
        via: RagdollState,
        final_state: RagdollState,
        linear_tolerance: f32,
        angular_tolerance: f32,
    ) -> Result<(), RagdollError> {
        if linear_tolerance < 0.0
            || angular_tolerance < 0.0
            || !linear_tolerance.is_finite()
            || !angular_tolerance.is_finite()
        {
            return Err(RagdollError::InvalidTolerance);
        }
        self.set_state(world, via);
        self.at_rest = Some(AtRestTransition {
            final_state,
            linear_tolerance,
            angular_tolerance,
        });
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
        } else {
            self.step_state(world, desired_local, model_to_world, delta_time)?;
        }

        if let Some(next) = self.queued_state.take() {
            self.set_state(world, next);
        }

        let easing_done = match self.easing.as_mut() {
            Some(action) => action.step(world, delta_time),
            None => false,
        };
        if easing_done {
            self.easing = None;
        }

        if let Some(at_rest) = self.at_rest {
            if self.all_bodies_at_rest(world, at_rest.linear_tolerance, at_rest.angular_tolerance) {
                self.at_rest = None;
                self.set_state(world, at_rest.final_state);
            }
        }
        Ok(())
    }

    fn all_bodies_at_rest(
        &self,
        world: &World,
        linear_tolerance: f32,
        angular_tolerance: f32,
    ) -> bool {
        for (index, bone) in self.ragdoll.bones().iter().enumerate() {
            let Some(body) = bone.body else {
                continue;
            };
            if self.bone_strengths.get(index).copied().unwrap_or(1.0) <= 0.0 {
                continue;
            }
            let linear = world.body_linear_velocity(body).unwrap_or(Vector4::ZERO);
            let angular = world.body_angular_velocity(body).unwrap_or(Vector4::ZERO);
            if !linear.all_equal3(Vector4::ZERO, linear_tolerance)
                || !angular.all_equal3(Vector4::ZERO, angular_tolerance)
            {
                return false;
            }
        }
        true
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
            RagdollState::Custom(id) => {
                let index = usize::from(id);
                if index >= USER_STATE_COUNT || self.states[index].is_none() {
                    return Ok(());
                }
                let Some(mut behavior) = self.states[index].take() else {
                    return Ok(());
                };
                let result = {
                    let mut context = RagdollStateContext {
                        world,
                        ragdoll: &self.ragdoll,
                        bone_strengths: &self.bone_strengths,
                        strength: self.strength,
                        desired_local,
                        model_to_world,
                        delta_time,
                    };
                    behavior.step(&mut context)
                };
                self.states[index] = Some(behavior);
                if let Some(next) = result? {
                    self.queued_state = Some(next);
                }
                Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_math::{Quaternion, Transform, Vector4};
    use glam::Vec4;
    use crate::phys_body::BodyCinfo;
    use crate::phys_extensions::ragdoll::{RagdollData, RagdollLimits};
    use crate::phys_shape::{Shape, SphereShape};
    use crate::phys_types::BodyId;
    use crate::phys_world::WorldCinfo;
    use std::sync::Arc;

    fn sphere() -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(Vec4::ZERO, 0.5)) as Arc<dyn Shape>
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
