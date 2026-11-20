//! Character state machine (`hknpCharacterStateManager` / `Context` / default
//! states subset).
//!
//! Port of `original/Physics/Source/Physics/Physics/Extensions/
//! CharacterControl/StateMachine/`:
//!
//! * [`movement_util::calculate_movement`] — direct port of
//!   `Utils/hknpCharacterMovementUtil.cpp`.
//! * [`CharacterState`] trait — `hknpCharacterState` (update / get_next_state /
//!   enter_state / leave_state).
//! * Default states [`states::OnGround`], [`states::Jumping`],
//!   [`states::InAir`], [`states::Climbing`] — direct ports of
//!   `States/hknpCharacterState{OnGround,Jumping,InAir,Climbing}.*` with the
//!   original default parameters.
//! * [`CharacterStateManager`] — the state registry (`hknpCharacterStateManager`).
//! * [`CharacterContext`] — `hknpCharacterContext` including the output
//!   velocity filter (`updateInternal`, the active `#else` branch) and the
//!   one-frame update skip semantics.
//!
//! # Subset decisions
//!
//! * `Flying` exists for API parity but no default implementation is
//!   registered (the original state is never entered either).
//! * User states (`HK_CHARACTER_USER_STATE_0..5`), cloning, the
//!   `CharacterType` discriminator and the deprecated first filter branch
//!   (`#if 0`) are not ported.
//! * As in the SDK, `at_ladder` is application-supplied: the SDK has no
//!   ladder-query utility, so detecting a ladder remains the caller's job.
//!   `m_surfaceDistanceExcess` stays implicit zero (the proxy always reports
//!   zero, see `hknpCharacterProxy.cpp:202`).

use std::sync::Arc;

use base_math::{Matrix3, Vector4};

use super::surface::SurfaceInfo;

/// Character state types (`hknpCharacterState::hknpCharacterStateType`
/// subset; the `Climbing`/`Flying` variants exist for parity but no default
/// implementation is registered).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CharacterStateType {
    /// Walking on a supporting surface.
    OnGround,
    /// One-frame jump impulse state.
    Jumping,
    /// Airborne, integrating gravity.
    InAir,
    /// Climbing a ladder (registered by the default manager).
    Climbing,
    /// Free flying (not registered by the default manager).
    Flying,
}

/// Input to the state machine (`hknpCharacterInput`).
#[derive(Clone, Debug)]
pub struct CharacterInput {
    /// Input X range -1 to 1 (left / right).
    pub input_lr: f32,
    /// Input Y range -1 to 1 (forward / back).
    pub input_ud: f32,
    /// Set to try to jump.
    pub want_jump: bool,
    /// Whether the character is at a ladder (`m_atLadder`, application supplied).
    pub at_ladder: bool,
    /// Up vector in world space (opposite of gravity).
    pub up: Vector4,
    /// Forward vector in world space (facing direction).
    pub forward: Vector4,
    /// The surface information from the character controller.
    pub surface_info: SurfaceInfo,
    /// Time step between calls.
    pub delta_time: f32,
    /// Reciprocal of `delta_time`.
    pub inv_delta_time: f32,
    /// Current position.
    pub position: Vector4,
    /// Current velocity.
    pub velocity: Vector4,
    /// Gravity applied to the character (e.g. in air).
    pub character_gravity: Vector4,
    /// User data, not used by the engine.
    pub user_data: u64,
}

impl Default for CharacterInput {
    fn default() -> Self {
        Self {
            input_lr: 0.0,
            input_ud: 0.0,
            want_jump: false,
            at_ladder: false,
            up: Vector4::UNIT_Y,
            forward: Vector4::from_xyz(0.0, 0.0, 1.0),
            surface_info: SurfaceInfo::unsupported(),
            delta_time: 1.0 / 60.0,
            inv_delta_time: 60.0,
            position: Vector4::ZERO,
            velocity: Vector4::ZERO,
            character_gravity: Vector4::ZERO,
            user_data: 0,
        }
    }
}

/// Output from the state machine (`hknpCharacterOutput`).
#[derive(Clone, Copy, Debug, Default)]
pub struct CharacterOutput {
    /// The output velocity of the character.
    pub velocity: Vector4,
    /// Modified input velocity (feeds the context velocity filtering).
    pub input_velocity: Vector4,
}

/// Character state interface (`hknpCharacterState`).
pub trait CharacterState: Send + Sync {
    /// Process the user input — causes state actions.
    fn update(
        &self,
        context: &CharacterContext,
        input: &CharacterInput,
        output: &mut CharacterOutput,
    );

    /// Process the user input — causes state transitions. `None` means "no
    /// change" (`HK_CHARACTER_INVALID_STATE`).
    fn get_next_state(
        &self,
        context: &CharacterContext,
        input: &CharacterInput,
        output: &mut CharacterOutput,
    ) -> Option<CharacterStateType>;

    /// Called when the character enters this state (no-op by default).
    fn enter_state(
        &self,
        _context: &CharacterContext,
        _previous: CharacterStateType,
        _input: &CharacterInput,
        _output: &mut CharacterOutput,
    ) {
    }

    /// Called when the character leaves this state (no-op by default).
    fn leave_state(
        &self,
        _context: &CharacterContext,
        _next: CharacterStateType,
        _input: &CharacterInput,
        _output: &mut CharacterOutput,
    ) {
    }
}

/// Shared movement math (`hknpCharacterMovementUtil`).
pub mod movement_util {
    use super::{Matrix3, Vector4};

    /// The space the desired velocity is defined in
    /// (`hknpMovementUtilInput::DesiredVelocitySpace`).
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum DesiredVelocitySpace {
        /// Surface-relative frame (default).
        SurfaceSpace,
        /// World space.
        WorldSpace,
    }

    /// Input of [`calculate_movement`] (`hknpMovementUtilInput`).
    pub struct MovementUtilInput {
        /// Forward direction in world space.
        pub forward: Vector4,
        /// Up direction in world space.
        pub up: Vector4,
        /// Normal of the surface the character stands on (world space).
        pub surface_normal: Vector4,
        /// Current velocity (world space).
        pub current_velocity: Vector4,
        /// Desired velocity in the frame selected by `desired_velocity_space`.
        pub desired_velocity: Vector4,
        /// Velocity of the supporting surface (world space).
        pub surface_velocity: Vector4,
        /// Feedback gain (acceleration control; scale by the time step).
        pub gain: f32,
        /// Maximum velocity change per call.
        pub max_velocity_delta: f32,
        /// The space `desired_velocity` is defined in.
        pub desired_velocity_space: DesiredVelocitySpace,
    }

    /// Calculate a new output velocity (`hknpCharacterMovementUtil::
    /// calculateMovement`). Leaves `velocity_out` untouched on a degenerate
    /// frame configuration (forward parallel to up).
    pub fn calculate_movement(input: &MovementUtilInput, velocity_out: &mut Vector4) {
        // Construct a frame on the supporting surface.
        let binorm0 = input.forward.cross(input.up);
        if binorm0.length_squared3() < 1e-5 {
            // Bad configuration space.
            return;
        }
        let binorm0 = binorm0.normalized3();
        let tangent = binorm0.cross(input.surface_normal).normalized3();
        let binorm = tangent.cross(input.surface_normal).normalized3();
        let surface_frame = Matrix3::from_cols(tangent, binorm, input.surface_normal);

        // Relative velocity in the surface frame.
        let mut relative = input.current_velocity - input.surface_velocity;
        relative = surface_frame.rotate_inverse(relative);

        // Difference between desired and relative velocity, gain-limited and
        // clamped by the maximum velocity delta.
        let mut diff = match input.desired_velocity_space {
            DesiredVelocitySpace::SurfaceSpace => input.desired_velocity - relative,
            DesiredVelocitySpace::WorldSpace => {
                // Note: intentionally reversed cross to avoid a negation.
                let side_vec = input.up.cross(input.forward);
                let fwd = input.desired_velocity.dot3(input.forward);
                let side = input.desired_velocity.dot3(side_vec);
                let len = input.desired_velocity.length3();
                let desired_velocity_sf = Vector4::new(-fwd, side, 0.0, 0.0).normalized3() * len;
                desired_velocity_sf - relative
            }
        };
        {
            let len = diff.length_squared3();
            let factor = if len * input.gain * input.gain
                > input.max_velocity_delta * input.max_velocity_delta
            {
                input.max_velocity_delta / len.sqrt()
            } else {
                input.gain
            };
            diff *= factor;
        }

        relative += diff;

        // Transform back to world space and add the surface velocity.
        *velocity_out = surface_frame.rotate(relative) + input.surface_velocity;
    }
}

/// Default states (`States/`).
pub mod states {
    use super::movement_util::{DesiredVelocitySpace, MovementUtilInput, calculate_movement};
    use super::{
        CharacterContext, CharacterInput, CharacterOutput, CharacterState, CharacterStateType,
        Vector4,
    };
    use crate::character::surface::SupportedState;

    /// Walking on the ground (`hknpCharacterStateOnGround`).
    pub struct OnGround {
        /// Feedback controller gain.
        pub gain: f32,
        /// Walking speed.
        pub walk_speed: f32,
        /// Maximum linear acceleration.
        pub max_linear_acceleration: f32,
        /// Kill the vertical velocity when leaving the ground
        /// ("ground hugging").
        pub kill_velocity_on_launch: bool,
        /// Clamp the downward velocity against gravity.
        pub limit_vertical_velocity: bool,
        /// Disable the horizontal projection of slope velocities.
        pub disable_horizontal_projection: bool,
    }

    impl Default for OnGround {
        fn default() -> Self {
            Self {
                gain: 0.95,
                walk_speed: 10.0,
                max_linear_acceleration: 200.0,
                kill_velocity_on_launch: true,
                limit_vertical_velocity: false,
                disable_horizontal_projection: false,
            }
        }
    }

    impl CharacterState for OnGround {
        fn get_next_state(
            &self,
            _context: &CharacterContext,
            input: &CharacterInput,
            output: &mut CharacterOutput,
        ) -> Option<CharacterStateType> {
            if input.want_jump {
                return Some(CharacterStateType::Jumping);
            }
            if input.at_ladder {
                return Some(CharacterStateType::Climbing);
            }
            if input.surface_info.supported_state != SupportedState::Supported {
                if self.kill_velocity_on_launch {
                    // Remove the velocity in the vertical component.
                    let vertical = input.velocity.dot3(input.up);
                    output.velocity -= input.up * vertical;
                }
                return Some(CharacterStateType::InAir);
            }
            Some(CharacterStateType::OnGround)
        }

        fn update(
            &self,
            context: &CharacterContext,
            input: &CharacterInput,
            output: &mut CharacterOutput,
        ) {
            // Correct the input velocity for the change in surface velocity
            // (avoids filtering artifacts on moving surfaces).
            let input_velocity = (input.velocity - context.previous_surface_velocity())
                + input.surface_info.surface_velocity;

            let mu_input = MovementUtilInput {
                gain: self.gain,
                forward: input.forward,
                up: input.up,
                surface_normal: input.surface_info.surface_normal,
                current_velocity: input_velocity,
                desired_velocity: Vector4::new(
                    input.input_ud * self.walk_speed,
                    input.input_lr * self.walk_speed,
                    0.0,
                    0.0,
                ),
                max_velocity_delta: self.max_linear_acceleration * input.delta_time,
                surface_velocity: input.surface_info.surface_velocity,
                desired_velocity_space: DesiredVelocitySpace::SurfaceSpace,
            };
            calculate_movement(&mu_input, &mut output.velocity);

            // Do not let the character fall faster than gravity.
            if self.limit_vertical_velocity {
                let dt = input.delta_time;
                let max_gravity_velocity_inc = input.character_gravity.dot3(input.up) * dt;
                let actual_velocity_inc =
                    output.velocity.dot3(input.up) - input.velocity.dot3(input.up);
                if output.velocity.dot3(input.up) < 0.0
                    && actual_velocity_inc < max_gravity_velocity_inc
                {
                    output.velocity += input.up * (max_gravity_velocity_inc - actual_velocity_inc);
                }
            }

            // Project slope-climbing velocity onto the horizontal plane to
            // avoid wall jitter (enabled by default).
            if !self.disable_horizontal_projection {
                output.velocity -= input.surface_info.surface_velocity;
                if output.velocity.dot3(input.up) > 0.001 {
                    let vel_len = output.velocity.length3();
                    let normal_dot_up = input.surface_info.surface_normal.dot3(input.up);
                    let horiz_len = vel_len / normal_dot_up;
                    let c = input.surface_info.surface_normal.cross(output.velocity);
                    output.velocity = c.cross(input.up) * horiz_len;
                }
                output.velocity += input.surface_info.surface_velocity;
            }
        }
    }

    /// One-frame jump impulse state (`hknpCharacterStateJumping`).
    pub struct Jumping {
        /// Target jump height.
        pub jump_height: f32,
    }

    impl Default for Jumping {
        fn default() -> Self {
            Self { jump_height: 1.5 }
        }
    }

    impl CharacterState for Jumping {
        fn get_next_state(
            &self,
            context: &CharacterContext,
            _input: &CharacterInput,
            _output: &mut CharacterOutput,
        ) -> Option<CharacterStateType> {
            if context.num_updates_in_current_state() == 2 {
                return Some(CharacterStateType::InAir);
            }
            Some(CharacterStateType::Jumping)
        }

        fn update(
            &self,
            context: &CharacterContext,
            input: &CharacterInput,
            output: &mut CharacterOutput,
        ) {
            // Jump: v² = u² + 2as with v = 0 at the apex.
            if context.num_updates_in_current_state() == 1 {
                let gravity_length = input.character_gravity.length3();
                if gravity_length <= f32::EPSILON {
                    return;
                }
                let u = (2.0 * gravity_length * self.jump_height.abs()).sqrt();
                let rel_velocity = input.velocity - input.surface_info.surface_velocity;
                let cur_rel_vel = rel_velocity.dot3(input.up);
                if cur_rel_vel < u {
                    output.velocity = input.velocity + input.up * (u - cur_rel_vel);
                }
            }
        }
    }

    /// Airborne state (`hknpCharacterStateInAir`).
    pub struct InAir {
        /// Feedback controller gain.
        pub gain: f32,
        /// Air control speed.
        pub air_speed: f32,
        /// Maximum linear acceleration (25% of the on-ground default).
        pub max_linear_acceleration: f32,
    }

    impl Default for InAir {
        fn default() -> Self {
            Self {
                gain: 0.05,
                air_speed: 10.0,
                max_linear_acceleration: 50.0,
            }
        }
    }

    impl CharacterState for InAir {
        fn get_next_state(
            &self,
            _context: &CharacterContext,
            input: &CharacterInput,
            _output: &mut CharacterOutput,
        ) -> Option<CharacterStateType> {
            if input.surface_info.supported_state == SupportedState::Supported {
                return Some(CharacterStateType::OnGround);
            }
            if input.at_ladder {
                return Some(CharacterStateType::Climbing);
            }
            Some(CharacterStateType::InAir)
        }

        fn update(
            &self,
            _context: &CharacterContext,
            input: &CharacterInput,
            output: &mut CharacterOutput,
        ) {
            let mu_input = MovementUtilInput {
                gain: self.gain,
                forward: input.forward,
                up: input.up,
                surface_normal: input.up,
                current_velocity: input.velocity,
                desired_velocity: Vector4::new(
                    input.input_ud * self.air_speed,
                    input.input_lr * self.air_speed,
                    0.0,
                    0.0,
                ),
                max_velocity_delta: self.max_linear_acceleration * input.delta_time,
                surface_velocity: Vector4::ZERO,
                desired_velocity_space: DesiredVelocitySpace::SurfaceSpace,
            };
            calculate_movement(&mu_input, &mut output.velocity);

            // Restore the original vertical component, then apply gravity.
            let vertical_out = output.velocity.dot3(input.up);
            let vertical_in = input.velocity.dot3(input.up);
            output.velocity -= input.up * vertical_out;
            output.velocity += input.up * vertical_in;
            output.velocity += input.character_gravity * input.delta_time;
        }
    }

    /// Ladder climbing (`hknpCharacterStateClimbing`).
    ///
    /// The ladder surface normal doubles as the movement "up" direction.
    /// Jumping pushes the character away along that normal.
    pub struct Climbing {
        /// Feedback gain; the SDK hard-codes `1.0`.
        pub gain: f32,
        /// Input scale on the ladder plane.
        pub climb_speed: f32,
        /// Constant velocity attracting the character toward the ladder.
        pub ladder_attraction: f32,
        /// Maximum velocity change per update.
        pub max_velocity_delta: f32,
    }

    impl Default for Climbing {
        fn default() -> Self {
            Self {
                gain: 1.0,
                climb_speed: 10.0,
                ladder_attraction: -0.1,
                max_velocity_delta: 100.0,
            }
        }
    }

    impl CharacterState for Climbing {
        fn get_next_state(
            &self,
            _context: &CharacterContext,
            input: &CharacterInput,
            output: &mut CharacterOutput,
        ) -> Option<CharacterStateType> {
            if !input.at_ladder {
                return Some(CharacterStateType::InAir);
            }
            if input.want_jump {
                output.velocity +=
                    input.surface_info.surface_normal * input.character_gravity.length3();
                return Some(CharacterStateType::InAir);
            }
            Some(CharacterStateType::Climbing)
        }

        fn update(
            &self,
            _context: &CharacterContext,
            input: &CharacterInput,
            output: &mut CharacterOutput,
        ) {
            let normal = input.surface_info.surface_normal;
            let mu_input = MovementUtilInput {
                gain: self.gain,
                forward: input.forward,
                up: normal,
                surface_normal: normal,
                current_velocity: input.velocity,
                desired_velocity: Vector4::new(
                    input.input_ud * self.climb_speed,
                    input.input_lr * self.climb_speed,
                    self.ladder_attraction,
                    0.0,
                ),
                max_velocity_delta: self.max_velocity_delta,
                surface_velocity: input.surface_info.surface_velocity,
                desired_velocity_space: DesiredVelocitySpace::SurfaceSpace,
            };
            calculate_movement(&mu_input, &mut output.velocity);
        }
    }
}

/// State registry shared between contexts (`hknpCharacterStateManager`).
///
/// States are stateless singletons; parameters live on the state objects and
/// per-character data flows through the input/output.
pub struct CharacterStateManager {
    states: Vec<Option<Box<dyn CharacterState>>>,
}

impl CharacterStateManager {
    const NUM_STATES: usize = 5;

    /// Empty registry.
    pub fn new() -> Self {
        Self {
            states: (0..Self::NUM_STATES).map(|_| None).collect(),
        }
    }

    /// Registry with the four wired default states (OnGround, Jumping,
    /// InAir, Climbing) using the original default parameters.
    pub fn with_default_states() -> Self {
        let mut manager = Self::new();
        manager.register(
            CharacterStateType::OnGround,
            Box::new(states::OnGround::default()),
        );
        manager.register(
            CharacterStateType::Jumping,
            Box::new(states::Jumping::default()),
        );
        manager.register(
            CharacterStateType::InAir,
            Box::new(states::InAir::default()),
        );
        manager.register(
            CharacterStateType::Climbing,
            Box::new(states::Climbing::default()),
        );
        manager
    }

    fn slot(state_type: CharacterStateType) -> usize {
        match state_type {
            CharacterStateType::OnGround => 0,
            CharacterStateType::Jumping => 1,
            CharacterStateType::InAir => 2,
            CharacterStateType::Climbing => 3,
            CharacterStateType::Flying => 4,
        }
    }

    /// Register (or replace) a state implementation.
    pub fn register(&mut self, state_type: CharacterStateType, state: Box<dyn CharacterState>) {
        self.states[Self::slot(state_type)] = Some(state);
    }

    /// The state implementation.
    ///
    /// # Panics
    /// Panics when the state was never registered (mirrors the original
    /// assert "Bad state transition ... Has this state been registered?").
    pub fn get(&self, state_type: CharacterStateType) -> &dyn CharacterState {
        self.states[Self::slot(state_type)]
            .as_deref()
            .expect("Bad state transition: state not registered")
    }
}

impl Default for CharacterStateManager {
    fn default() -> Self {
        Self::with_default_states()
    }
}

/// Character context (`hknpCharacterContext`): current state plus the output
/// velocity filter.
pub struct CharacterContext {
    manager: Arc<CharacterStateManager>,
    current_state: CharacterStateType,
    previous_state: CharacterStateType,
    filter_enable: bool,
    max_linear_acceleration: f32,
    max_linear_speed: f32,
    gain: f32,
    previous_surface_velocity: Vector4,
    num_updates_in_current_state: usize,
    forced_next_state: Option<CharacterStateType>,
}

impl CharacterContext {
    /// Context with the original defaults (filter enabled, gain 1.0,
    /// max speed 20, max acceleration 625).
    pub fn new(manager: Arc<CharacterStateManager>, initial_state: CharacterStateType) -> Self {
        // Panic early on an unregistered initial state (the original asserts
        // lazily at the first update).
        let _ = manager.get(initial_state);
        Self {
            manager,
            current_state: initial_state,
            previous_state: initial_state,
            filter_enable: true,
            max_linear_acceleration: 625.0,
            max_linear_speed: 20.0,
            gain: 1.0,
            previous_surface_velocity: Vector4::ZERO,
            num_updates_in_current_state: 0,
            forced_next_state: None,
        }
    }

    /// The current state.
    pub fn current_state(&self) -> CharacterStateType {
        self.current_state
    }

    /// The number of updates in the current state (reset on
    /// [`CharacterContext::set_state`]).
    pub fn num_updates_in_current_state(&self) -> usize {
        self.num_updates_in_current_state
    }

    /// The supporting surface velocity of the previous update.
    pub fn previous_surface_velocity(&self) -> Vector4 {
        self.previous_surface_velocity
    }

    /// Enable/disable the output velocity filter.
    pub fn set_filter_enable(&mut self, enable: bool) {
        self.filter_enable = enable;
    }

    /// Configure the output velocity filter.
    pub fn set_filter_parameters(&mut self, gain: f32, max_speed: f32, max_acceleration: f32) {
        self.gain = gain;
        self.max_linear_speed = max_speed;
        self.max_linear_acceleration = max_acceleration;
    }

    /// Force the next transition (overriding `get_next_state`); consumed by
    /// the next non-predict update.
    pub fn force_next_state(&mut self, state: Option<CharacterStateType>) {
        self.forced_next_state = state;
    }

    /// Cause a state transition (`setState`): leave the old state, switch,
    /// reset the counter, enter the new state.
    pub fn set_state(
        &mut self,
        state: CharacterStateType,
        input: &CharacterInput,
        output: &mut CharacterOutput,
    ) {
        let manager = self.manager.clone();
        let previous = self.current_state;
        manager
            .get(previous)
            .leave_state(self, state, input, output);
        self.current_state = state;
        self.num_updates_in_current_state = 0;
        manager
            .get(state)
            .enter_state(self, previous, input, output);
    }

    /// Update the state machine (`update`).
    pub fn update(&mut self, input: &CharacterInput, output: &mut CharacterOutput) {
        self.update_internal(input, output, false);
    }

    /// Predict the state machine output without mutating the context
    /// (`predict`).
    pub fn predict(&mut self, input: &CharacterInput, output: &mut CharacterOutput) {
        self.update_internal(input, output, true);
    }

    fn update_internal(
        &mut self,
        input: &CharacterInput,
        output: &mut CharacterOutput,
        predict: bool,
    ) {
        // Sensible initialized output values.
        output.velocity = input.velocity;
        output.input_velocity = input.velocity;

        let manager = self.manager.clone();

        // State transition logic.
        let mut new_state = manager
            .get(self.current_state)
            .get_next_state(self, input, output);
        if let Some(forced) = self.forced_next_state {
            new_state = Some(forced);
            if !predict {
                self.forced_next_state = None;
            }
        }
        if !predict {
            if let Some(new_state) = new_state {
                if new_state != self.current_state {
                    self.set_state(new_state, input, output);
                }
            }
        }

        // Update with the potential new state (skip one frame — mirrors the
        // original `newState == m_previousState` check).
        if new_state == Some(self.previous_state) {
            manager.get(self.current_state).update(self, input, output);
        }

        // Output velocity filtering.
        if self.filter_enable {
            let current_speed = output.velocity.length3();
            if current_speed > self.max_linear_speed {
                output.velocity *= self.max_linear_speed / current_speed;
            }
            let current_acceleration = output.velocity - output.input_velocity;
            let ca = current_acceleration.length3() * input.inv_delta_time
                / self.max_linear_acceleration;
            output.velocity = output.input_velocity;
            let factor = if ca > 1.0 && self.current_state != CharacterStateType::Jumping {
                self.gain * input.delta_time / ca
            } else {
                self.gain * input.delta_time
            };
            output.velocity += current_acceleration * factor;
        }

        if !predict {
            self.previous_state = self.current_state;
            self.previous_surface_velocity = input.surface_info.surface_velocity;
            self.num_updates_in_current_state += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::surface::SupportedState;
    use base_math::Vector4 as V4;

    fn supported_surface() -> SurfaceInfo {
        SurfaceInfo {
            supported_state: SupportedState::Supported,
            surface_normal: V4::UNIT_Y,
            surface_velocity: V4::ZERO,
            surface_angular_velocity: V4::ZERO,
        }
    }

    fn input(surface: SurfaceInfo, want_jump: bool) -> CharacterInput {
        CharacterInput {
            want_jump,
            surface_info: surface,
            ..CharacterInput::default()
        }
    }

    fn make_context() -> CharacterContext {
        CharacterContext::new(
            Arc::new(CharacterStateManager::with_default_states()),
            CharacterStateType::OnGround,
        )
    }

    #[test]
    fn movement_util_reaches_desired_velocity() {
        let mut velocity = V4::ZERO;
        let mu_input = movement_util::MovementUtilInput {
            forward: V4::from_xyz(0.0, 0.0, 1.0),
            up: V4::UNIT_Y,
            surface_normal: V4::UNIT_Y,
            current_velocity: V4::ZERO,
            desired_velocity: V4::new(5.0, 0.0, 0.0, 0.0),
            surface_velocity: V4::ZERO,
            gain: 0.95,
            max_velocity_delta: 200.0 * (1.0 / 60.0),
            desired_velocity_space: movement_util::DesiredVelocitySpace::SurfaceSpace,
        };
        movement_util::calculate_movement(&mu_input, &mut velocity);
        // The surface frame maps the x component onto -forward (the
        // original's `binorm0 = forward x up` construction), so a positive
        // desired-x moves along -z.
        assert!(
            velocity.z < 0.0 && velocity.z > -5.0,
            "gain-limited step towards the desired velocity: {:?}",
            velocity
        );
        assert!(velocity.x.abs() < 1e-5 && velocity.y.abs() < 1e-5);
    }

    #[test]
    fn movement_util_handles_degenerate_frame() {
        let mut velocity = V4::ZERO;
        let mu_input = movement_util::MovementUtilInput {
            forward: V4::UNIT_Y,
            up: V4::UNIT_Y,
            surface_normal: V4::UNIT_Y,
            current_velocity: V4::ZERO,
            desired_velocity: V4::new(5.0, 0.0, 0.0, 0.0),
            surface_velocity: V4::ZERO,
            gain: 1.0,
            max_velocity_delta: 10.0,
            desired_velocity_space: movement_util::DesiredVelocitySpace::SurfaceSpace,
        };
        movement_util::calculate_movement(&mu_input, &mut velocity);
        assert!(velocity.all_equal3(V4::ZERO, 0.0));
    }

    #[test]
    fn movement_util_clamps_by_max_velocity_delta() {
        let mut velocity = V4::ZERO;
        let mu_input = movement_util::MovementUtilInput {
            forward: V4::from_xyz(0.0, 0.0, 1.0),
            up: V4::UNIT_Y,
            surface_normal: V4::UNIT_Y,
            current_velocity: V4::ZERO,
            desired_velocity: V4::new(5.0, 0.0, 0.0, 0.0),
            surface_velocity: V4::ZERO,
            gain: 0.95,
            max_velocity_delta: 1.0,
            desired_velocity_space: movement_util::DesiredVelocitySpace::SurfaceSpace,
        };
        movement_util::calculate_movement(&mu_input, &mut velocity);
        assert!(
            velocity.length3() < 1.05,
            "velocity delta must be clamped: {:?}",
            velocity
        );
    }

    #[test]
    fn walk_reaches_walk_speed_over_time() {
        let mut context = make_context();
        let surface = supported_surface();
        let mut velocity = V4::ZERO;
        // The on-ground util is clamped by max_linear_acceleration (200 m/s²
        // => 3.33 m/s per frame) and the context filter takes gain*dt of the
        // remaining change, so walk speed is approached over ~200 frames.
        for _ in 0..240 {
            let input = CharacterInput {
                input_ud: 1.0,
                surface_info: surface,
                velocity,
                ..CharacterInput::default()
            };
            let mut output = CharacterOutput::default();
            context.update(&input, &mut output);
            velocity = output.velocity;
        }
        // The movement util maps the x input onto -forward (see the
        // surface-frame note in movement_util_reaches_desired_velocity).
        assert!(
            (velocity.z + 10.0).abs() < 0.6,
            "velocity should approach walk speed 10 along -forward, got {:?}",
            velocity
        );
    }

    #[test]
    fn walk_reaches_walk_speed_over_time_with_state_history() {
        let mut context = make_context();
        let surface = supported_surface();
        let mut velocity = V4::ZERO;
        let mut state_history = Vec::new();
        for _ in 0..30 {
            let input = CharacterInput {
                input_ud: 1.0,
                surface_info: surface,
                velocity,
                ..CharacterInput::default()
            };
            let mut output = CharacterOutput::default();
            context.update(&input, &mut output);
            velocity = output.velocity;
            if state_history.last() != Some(&context.current_state()) {
                state_history.push(context.current_state());
            }
        }
        assert_eq!(state_history, vec![CharacterStateType::OnGround]);
        assert!(context.num_updates_in_current_state() == 30);
    }

    #[test]
    fn jump_transitions_through_states() {
        let mut context = make_context();
        let surface = supported_surface();
        let mut velocity = V4::ZERO;
        let mut state_history = vec![context.current_state()];

        let mut input = input(surface, false);
        input.character_gravity = V4::from_xyz(0.0, -20.0, 0.0);
        for step in 0..30 {
            input.want_jump = step == 0;
            input.velocity = velocity;
            let mut output = CharacterOutput::default();
            context.update(&input, &mut output);
            velocity = output.velocity;
            if state_history.last() != Some(&context.current_state()) {
                state_history.push(context.current_state());
            }
        }
        assert_eq!(
            state_history,
            vec![
                CharacterStateType::OnGround,
                CharacterStateType::Jumping,
                CharacterStateType::InAir,
                CharacterStateType::OnGround
            ],
            "state history {:?}",
            state_history
        );
    }

    #[test]
    fn jumping_state_applies_impulse_once() {
        let mut context = make_context();
        context.force_next_state(Some(CharacterStateType::Jumping));
        let surface = supported_surface();
        let input = CharacterInput {
            surface_info: surface,
            character_gravity: V4::from_xyz(0.0, -20.0, 0.0),
            ..CharacterInput::default()
        };
        let mut output = CharacterOutput::default();
        context.update(&input, &mut output);
        let first = output.velocity.y;
        let mut output = CharacterOutput::default();
        context.update(&input, &mut output);
        let second = output.velocity.y;
        // The impulse is applied on the first state update (second frame);
        // the context filter takes gain*dt of the change, so the velocity
        // ramps towards the impulse target sqrt(2*g*h) ≈ 7.75.
        assert!(
            second > first && second > 0.1,
            "jump impulse should lift the character: {} -> {}",
            first,
            second
        );
    }

    #[test]
    fn in_air_state_applies_gravity() {
        let manager = Arc::new(CharacterStateManager::with_default_states());
        let mut context = CharacterContext::new(manager, CharacterStateType::InAir);
        let surface = SurfaceInfo::unsupported();
        // The context filter applies gain*dt of the velocity change per
        // frame, so gravity builds up over frames (original behavior).
        let mut velocity = V4::ZERO;
        for _ in 0..60 {
            let input = CharacterInput {
                surface_info: surface,
                character_gravity: V4::from_xyz(0.0, -20.0, 0.0),
                velocity,
                ..CharacterInput::default()
            };
            let mut output = CharacterOutput::default();
            context.update(&input, &mut output);
            velocity = output.velocity;
        }
        assert!(
            velocity.y < -0.2,
            "gravity should accelerate downwards over frames: {:?}",
            velocity
        );
        // No horizontal drift without input.
        assert!(velocity.x.abs() < 1e-5);
        assert!(velocity.z.abs() < 1e-5);
    }

    #[test]
    fn in_air_returns_to_on_ground_when_supported() {
        let manager = Arc::new(CharacterStateManager::with_default_states());
        let mut context = CharacterContext::new(manager, CharacterStateType::InAir);
        let input = CharacterInput {
            surface_info: supported_surface(),
            ..CharacterInput::default()
        };
        let mut output = CharacterOutput::default();
        context.update(&input, &mut output);
        assert_eq!(context.current_state(), CharacterStateType::OnGround);
    }

    #[test]
    fn unsupported_leaving_ground_transitions_to_in_air() {
        let mut context = make_context();
        let surface = SurfaceInfo::unsupported();
        let input = CharacterInput {
            surface_info: surface,
            velocity: V4::from_xyz(1.0, 2.0, 0.0),
            ..CharacterInput::default()
        };
        let mut output = CharacterOutput::default();
        context.update(&input, &mut output);
        assert_eq!(context.current_state(), CharacterStateType::InAir);
        // The "ground hugging" kill happens inside get_next_state; with the
        // context filter enabled its effect is applied gradually (the
        // InAir state restores the input's vertical component on update).
        assert!(
            output.velocity.y < input.velocity.y,
            "vertical velocity must not increase: {:?}",
            output.velocity
        );
    }

    #[test]
    fn predict_does_not_change_state_or_counter() {
        let mut context = make_context();
        let before_state = context.current_state();
        let before_updates = context.num_updates_in_current_state();
        let surface = SurfaceInfo::unsupported();
        let input = input(surface, true);
        let mut output = CharacterOutput::default();
        context.predict(&input, &mut output);
        assert_eq!(context.current_state(), before_state);
        assert_eq!(context.num_updates_in_current_state(), before_updates);
    }

    #[test]
    fn filter_limits_acceleration() {
        let manager = Arc::new(CharacterStateManager::with_default_states());
        let mut context = CharacterContext::new(manager, CharacterStateType::InAir);
        context.set_filter_parameters(0.5, 20.0, 10.0);
        let surface = SurfaceInfo::unsupported();
        let input = CharacterInput {
            surface_info: surface,
            ..CharacterInput::default()
        };
        let mut output = CharacterOutput::default();
        context.update(&input, &mut output);
        assert!(
            output.velocity.length_squared3() < 1e-4,
            "no input => no velocity: {:?}",
            output.velocity
        );
    }

    #[test]
    fn unregistered_state_panics_on_get() {
        let manager = CharacterStateManager::new();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            manager.get(CharacterStateType::Climbing)
        }));
        assert!(result.is_err(), "unregistered state must panic");
    }
}
