//! `hknpWorldCinfo` — world construction info.

use hk_math::Vector4;

/// Construction info for a [`crate::World`] (`hknpWorldCinfo`).
///
/// ```
/// use hk_math::Vector4;
/// use hknp_world::WorldCinfo;
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
    /// Expected step delta time (default 1/30 s).
    pub expected_delta_time: f32,
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
            .with_deactivation(false);
        assert_eq!(cinfo.name.as_deref(), Some("test"));
        assert_eq!(cinfo.body_buffer_capacity, 10);
        assert_eq!(cinfo.solver_tau, 0.5);
        assert_eq!(cinfo.solver_iterations, 8);
        assert!(!cinfo.enable_deactivation);
    }
}
