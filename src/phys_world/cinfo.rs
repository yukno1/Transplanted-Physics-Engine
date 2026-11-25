//! `hknpWorldCinfo` — world construction info.

use crate::base_math::Vector4;

/// Construction info for a [`crate::phys_world::World`] (`hknpWorldCinfo`).
///
/// ```
/// use proton::math::Vector4;
/// use proton::world::WorldCinfo;
///
/// let cinfo = WorldCinfo::new()
///     .with_gravity(Vector4::from_xyz(0.0, -9.81, 0.0))
///     .with_body_capacity(1024);
/// assert_eq!(cinfo.body_buffer_capacity, 1024);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct WorldCinfo {
    /// Optional world name.
    pub name: Option<String>,
    /// Gravity acceleration in world space (`m_gravity`).
    pub gravity: Vector4,
    /// Initial body slot capacity (`m_bodyBufferCapacity`, default 4096).
    pub body_buffer_capacity: usize,
    /// Initial motion slot capacity (`m_motionBufferCapacity`, default 4096).
    pub motion_buffer_capacity: usize,
    /// Collision tolerance (`m_collisionTolerance`, default 0.05).
    pub collision_tolerance: f32,
    /// Solver `tau` (default 0.6).
    pub solver_tau: f32,
    /// Solver damping (default 1.0).
    pub solver_damp: f32,
    /// Solver iteration count (default 4).
    pub solver_iterations: u32,
    /// Whether deactivation is enabled (default true).
    pub enable_deactivation: bool,
    /// Whether island sleeping is enabled (default false).
    ///
    /// This is the deactivation subset actually implemented: constraint islands
    /// whose dynamic bodies stay below the sleeping velocity thresholds for
    /// `time_to_sleep` seconds fall asleep (zero velocity, no narrow-phase
    /// contacts) until touched by an active body or woken explicitly.
    pub enable_sleeping: bool,
    /// Seconds a body must stay below the sleeping thresholds before its
    /// island falls asleep (default 0.5).
    pub time_to_sleep: f32,
    /// Linear velocity below which a body counts as low-energy (default 0.1).
    pub sleeping_linear_threshold: f32,
    /// Angular velocity below which a body counts as low-energy (default 0.1).
    pub sleeping_angular_threshold: f32,
    /// Whether contact impulses are warm started across steps (default true).
    pub enable_warm_starting: bool,
    /// Whether a trigger pair that exits and re-enters in the same step emits
    /// `TriggerStatus::Updated` (`m_raiseTriggerUpdatedEvents`, default false).
    pub raise_trigger_updated_events: bool,
    /// Expected step delta time (default 1/30 s).
    pub expected_delta_time: f32,
}

/// Backend used by [`crate::phys_world::World::step_with_scheduler`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ExecutorKind {
    /// Existing FIFO [`crate::thread::ThreadPool`] (default).
    #[default]
    Pool,
    /// Optional work-stealing pool for uneven island sizes.
    WorkStealing,
}

/// Opt-in scheduling behavior for physics steps (`hknpTaskGraph` subset).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SchedulerConfig {
    /// Executor backend used by the deterministic task graph.
    pub executor: ExecutorKind,
    /// Submit same-level tasks by descending priority, then ascending task id.
    pub respect_priority: bool,
    /// Build solver contact rows in parallel chunks (default off).
    pub parallel_contact_rows: bool,
    /// Compute solver write-back and sleep timers in parallel chunks (default off).
    pub parallel_writeback: bool,
}

impl SchedulerConfig {
    /// Construct a scheduler configuration with stage parallelism disabled.
    pub fn new(executor: ExecutorKind, respect_priority: bool) -> Self {
        Self {
            executor,
            respect_priority,
            parallel_contact_rows: false,
            parallel_writeback: false,
        }
    }

    /// Select the executor backend.
    #[must_use]
    pub fn with_executor(mut self, executor: ExecutorKind) -> Self {
        self.executor = executor;
        self
    }

    /// Enable or disable priority-aware submission inside one task level.
    #[must_use]
    pub fn with_priority(mut self, respect_priority: bool) -> Self {
        self.respect_priority = respect_priority;
        self
    }

    /// Enable or disable parallel contact-row construction.
    #[must_use]
    pub fn with_parallel_contact_rows(mut self, enabled: bool) -> Self {
        self.parallel_contact_rows = enabled;
        self
    }

    /// Enable or disable parallel write-back candidate computation.
    #[must_use]
    pub fn with_parallel_writeback(mut self, enabled: bool) -> Self {
        self.parallel_writeback = enabled;
        self
    }

    /// Enable or disable both newly parallelizable stages at once.
    #[must_use]
    pub fn with_stage_parallelism(mut self, rows: bool, writeback: bool) -> Self {
        self.parallel_contact_rows = rows;
        self.parallel_writeback = writeback;
        self
    }
}

impl Default for WorldCinfo {
    fn default() -> Self {
        Self {
            name: None,
            gravity: Vector4::from_xyz(0.0, -9.81, 0.0),
            body_buffer_capacity: 4096,
            motion_buffer_capacity: 4096,
            collision_tolerance: 0.05,
            solver_tau: 0.6,
            solver_damp: 1.0,
            solver_iterations: 4,
            enable_deactivation: true,
            enable_sleeping: false,
            time_to_sleep: 0.5,
            sleeping_linear_threshold: 0.1,
            sleeping_angular_threshold: 0.1,
            enable_warm_starting: true,
            raise_trigger_updated_events: false,
            expected_delta_time: 1.0 / 30.0,
        }
    }
}

impl WorldCinfo {
    /// Construct with the SDK defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the world gravity.
    #[must_use]
    pub fn with_gravity(mut self, gravity: Vector4) -> Self {
        self.gravity = gravity;
        self
    }

    /// Set the world name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set the initial body slot capacity.
    #[must_use]
    pub fn with_body_capacity(mut self, capacity: usize) -> Self {
        self.body_buffer_capacity = capacity;
        self
    }

    /// Set the initial motion slot capacity.
    #[must_use]
    pub fn with_motion_capacity(mut self, capacity: usize) -> Self {
        self.motion_buffer_capacity = capacity;
        self
    }

    /// Set the collision tolerance.
    #[must_use]
    pub fn with_collision_tolerance(mut self, tolerance: f32) -> Self {
        self.collision_tolerance = tolerance;
        self
    }

    /// Set the solver settings (`m_solverTau`, `m_solverDamp`, `m_solverIterations`).
    #[must_use]
    pub fn with_solver_settings(mut self, tau: f32, damp: f32, iterations: u32) -> Self {
        self.solver_tau = tau;
        self.solver_damp = damp;
        self.solver_iterations = iterations;
        self
    }

    /// Enable or disable deactivation.
    #[must_use]
    pub fn with_deactivation(mut self, enabled: bool) -> Self {
        self.enable_deactivation = enabled;
        self
    }

    /// Enable or disable island sleeping (the implemented deactivation subset).
    #[must_use]
    pub fn with_sleeping(mut self, enabled: bool) -> Self {
        self.enable_sleeping = enabled;
        self
    }

    /// Set the island sleeping parameters.
    #[must_use]
    pub fn with_sleeping_settings(
        mut self,
        time_to_sleep: f32,
        linear_threshold: f32,
        angular_threshold: f32,
    ) -> Self {
        self.time_to_sleep = time_to_sleep;
        self.sleeping_linear_threshold = linear_threshold;
        self.sleeping_angular_threshold = angular_threshold;
        self
    }

    /// Enable or disable contact warm starting.
    #[must_use]
    pub fn with_warm_starting(mut self, enabled: bool) -> Self {
        self.enable_warm_starting = enabled;
        self
    }

    /// Enable or disable same-step trigger `Updated` events.
    #[must_use]
    pub fn with_raise_trigger_updated_events(mut self, enabled: bool) -> Self {
        self.raise_trigger_updated_events = enabled;
        self
    }

    /// Set the expected step delta time used for solver scaling.
    #[must_use]
    pub fn with_expected_delta_time(mut self, delta_time: f32) -> Self {
        self.expected_delta_time = delta_time;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_sdk() {
        let cinfo = WorldCinfo::default();
        assert_eq!(cinfo.gravity, Vector4::from_xyz(0.0, -9.81, 0.0));
        assert_eq!(cinfo.body_buffer_capacity, 4096);
        assert_eq!(cinfo.motion_buffer_capacity, 4096);
        assert_eq!(cinfo.collision_tolerance, 0.05);
        assert_eq!(cinfo.solver_tau, 0.6);
        assert_eq!(cinfo.solver_damp, 1.0);
        assert_eq!(cinfo.solver_iterations, 4);
        assert!(cinfo.enable_deactivation);
        assert!(!cinfo.enable_sleeping);
        assert!((cinfo.time_to_sleep - 0.5).abs() < 1e-6);
        assert!((cinfo.sleeping_linear_threshold - 0.1).abs() < 1e-6);
        assert!((cinfo.sleeping_angular_threshold - 0.1).abs() < 1e-6);
        assert!(cinfo.enable_warm_starting);
        assert!(!cinfo.raise_trigger_updated_events);
        assert!((cinfo.expected_delta_time - 1.0 / 30.0).abs() < 1e-6);
    }

    #[test]
    fn builder_overrides() {
        let cinfo = WorldCinfo::new()
            .with_gravity(Vector4::from_xyz(0.0, -1.0, 0.0))
            .with_name("test")
            .with_body_capacity(10)
            .with_motion_capacity(5)
            .with_collision_tolerance(0.1)
            .with_solver_settings(0.5, 0.9, 8)
            .with_deactivation(false)
            .with_sleeping(true)
            .with_sleeping_settings(1.0, 0.2, 0.3)
            .with_warm_starting(false)
            .with_raise_trigger_updated_events(true);
        assert_eq!(cinfo.name.as_deref(), Some("test"));
        assert_eq!(cinfo.body_buffer_capacity, 10);
        assert_eq!(cinfo.solver_tau, 0.5);
        assert_eq!(cinfo.solver_iterations, 8);
        assert!(!cinfo.enable_deactivation);
        assert!(cinfo.enable_sleeping);
        assert!((cinfo.time_to_sleep - 1.0).abs() < 1e-6);
        assert!((cinfo.sleeping_linear_threshold - 0.2).abs() < 1e-6);
        assert!((cinfo.sleeping_angular_threshold - 0.3).abs() < 1e-6);
        assert!(!cinfo.enable_warm_starting);
        assert!(cinfo.raise_trigger_updated_events);
    }

    #[test]
    fn scheduler_config_defaults_and_builder() {
        let default = SchedulerConfig::default();
        assert_eq!(default.executor, ExecutorKind::Pool);
        assert!(!default.respect_priority);
        assert!(!default.parallel_contact_rows);
        assert!(!default.parallel_writeback);

        let work_stealing =
            SchedulerConfig::new(ExecutorKind::WorkStealing, false).with_priority(true);
        assert_eq!(work_stealing.executor, ExecutorKind::WorkStealing);
        assert!(work_stealing.respect_priority);
        assert!(!work_stealing.parallel_contact_rows);
        assert!(!work_stealing.parallel_writeback);
        let back_to_pool = work_stealing.with_executor(ExecutorKind::Pool);
        assert_eq!(back_to_pool.executor, ExecutorKind::Pool);
        assert!(back_to_pool.respect_priority);

        let staged = SchedulerConfig::new(ExecutorKind::Pool, false)
            .with_stage_parallelism(true, true)
            .with_parallel_contact_rows(false)
            .with_parallel_writeback(true);
        assert!(!staged.parallel_contact_rows);
        assert!(staged.parallel_writeback);
    }
}
