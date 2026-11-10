//! `hknp-solver` — iterative contact solver.
//!
//! Rust port of the contact branch of
//! `original/Physics/Source/Physics/Physics/Dynamics/Solver/` (migration
//! Phase 8, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`SolverBody`] | solver velocity buffer entry | inverse mass/inertia, position, linear/angular velocity |
//! | [`ContactConstraint`] | `hknpContactJacobian` (single point) | body indices, B→A normal, contact point, penetration, friction |
//! | [`SolverSettings`] | `hknpSolverInfo` subset | iterations, Baumgarte factor, recovery speed cap, friction |
//! | [`Solver`] | `hknpSolver` | sequential-impulse normal + tangential solve |
//!
//! # Algorithm
//!
//! The solver is a sequential-impulse (Gauss-Seidel) velocity solver, matching
//! the iteration order of the SDK: contacts are solved in insertion order for
//! `iterations` sweeps, with accumulated normal impulses clamped to be non-
//! negative. Penetration is corrected with a Baumgarte velocity bias capped by
//! `max_recovery_speed` (the `hknpBody` depenetration speed limit analog).
//! Tangential impulses use one tangent direction derived from the relative
//! velocity and are clamped by `friction * accumulated_normal`.
//!
//! # Not ported in this phase
//!
//! The full constraint-atom system (`Solver/ConstraintAtom`, 49 files), friction
//! direction caching, restitution/mass modifiers, solver bodies' inertia in
//! inertia space, island/solver-batch partitioning, warm starting, deactivation,
//! and the SDK's determinism-check instrumentation. Material-driven friction/
//! restitution and multi-point manifolds arrive with the material library and
//! the extended manifold work.

use hk_math::Vector4;

/// A body as seen by the solver (a velocity-buffer entry).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolverBody {
    /// Inverse mass (`0` for static/keyframed/infinite mass).
    pub inverse_mass: f32,
    /// Diagonal inverse inertia (world-space approximation).
    pub inverse_inertia: Vector4,
    /// Center-of-mass position in world space.
    pub position: Vector4,
    /// Linear velocity in world space.
    pub linear_velocity: Vector4,
    /// Angular velocity in world space.
    pub angular_velocity: Vector4,
}

impl SolverBody {
    /// Construct a solver body.
    pub fn new(
        inverse_mass: f32,
        inverse_inertia: Vector4,
        position: Vector4,
        linear_velocity: Vector4,
        angular_velocity: Vector4,
    ) -> Self {
        Self {
            inverse_mass,
            inverse_inertia,
            position,
            linear_velocity,
            angular_velocity,
        }
    }

    /// A non-responsive body (static or keyframed).
    pub fn static_body(position: Vector4) -> Self {
        Self::new(0.0, Vector4::ZERO, position, Vector4::ZERO, Vector4::ZERO)
    }

    /// Whether this body can respond to impulses.
    pub fn is_dynamic(&self) -> bool {
        self.inverse_mass > 0.0
    }

    /// Velocity at a world point: `v + ω × r`.
    pub fn velocity_at(&self, point: Vector4) -> Vector4 {
        self.linear_velocity + self.angular_velocity.cross(point - self.position)
    }

    /// Apply a world-space impulse at a world point.
    pub fn apply_impulse_at(&mut self, point: Vector4, impulse: Vector4) {
        if self.inverse_mass > 0.0 {
            self.linear_velocity += impulse * self.inverse_mass;
        }
        let r = point - self.position;
        let torque = r.cross(impulse);
        self.angular_velocity += Vector4::from_xyz(
            torque.x * self.inverse_inertia.x,
            torque.y * self.inverse_inertia.y,
            torque.z * self.inverse_inertia.z,
        );
    }
}

/// Single-point contact constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactConstraint {
    /// Solver index of shape A's body.
    pub body_a: usize,
    /// Solver index of shape B's body.
    pub body_b: usize,
    /// Unit normal from B to A.
    pub normal: Vector4,
    /// Contact point in world space (midway between the surfaces).
    pub point: Vector4,
    /// Penetration depth (positive).
    pub penetration: f32,
    /// Coulomb friction coefficient.
    pub friction: f32,
}

impl ContactConstraint {
    /// Construct a constraint; `normal` is normalized defensively.
    pub fn new(
        body_a: usize,
        body_b: usize,
        normal: Vector4,
        point: Vector4,
        penetration: f32,
        friction: f32,
    ) -> Self {
        let length = normal.length3();
        let normal = if length > 0.0 {
            Vector4::from_xyz(normal.x / length, normal.y / length, normal.z / length)
        } else {
            Vector4::UNIT_Y
        };
        Self {
            body_a,
            body_b,
            normal,
            point,
            penetration: penetration.max(0.0),
            friction: friction.max(0.0),
        }
    }
}

/// Solver configuration (`hknpSolverInfo` subset).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolverSettings {
    /// Number of Gauss-Seidel sweeps.
    pub iterations: u32,
    /// Baumgarte penetration-recovery factor (`m_tau` analog).
    pub baumgarte: f32,
    /// Maximum penetration-recovery velocity.
    pub max_recovery_speed: f32,
    /// Default Coulomb friction coefficient.
    pub friction: f32,
}

impl Default for SolverSettings {
    fn default() -> Self {
        Self {
            iterations: 4,
            baumgarte: 0.2,
            max_recovery_speed: 1.0,
            friction: 0.5,
        }
    }
}

/// Statistics returned by a solve.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SolverStats {
    /// Number of contact constraints processed.
    pub contacts: usize,
    /// Number of sweeps executed.
    pub iterations: u32,
}

/// Sequential-impulse contact solver.
#[derive(Clone, Copy, Debug, Default)]
pub struct Solver {
    settings: SolverSettings,
}

impl Solver {
    /// Construct with the given settings.
    pub fn new(settings: SolverSettings) -> Self {
        Self { settings }
    }

    /// Current settings.
    pub fn settings(&self) -> SolverSettings {
        self.settings
    }

    /// Solve `contacts` in place on `bodies`.
    ///
    /// Constraints must reference valid body indices; invalid ones are skipped.
    /// `delta_time` is used for the Baumgarte bias (0 disables recovery).
    pub fn solve(
        &self,
        bodies: &mut [SolverBody],
        contacts: &[ContactConstraint],
        delta_time: f32,
    ) -> SolverStats {
        debug_assert!(delta_time >= 0.0 && delta_time.is_finite());
        let bias_scale = if delta_time > 0.0 {
            self.settings.baumgarte / delta_time
        } else {
            0.0
        };
        let mut accumulated_normal = vec![0.0f32; contacts.len()];

        for _ in 0..self.settings.iterations {
            for (index, contact) in contacts.iter().enumerate() {
                let (Ok(body_a), Ok(body_b)) = (
                    body_index(bodies.len(), contact.body_a),
                    body_index(bodies.len(), contact.body_b),
                ) else {
                    continue;
                };
                if body_a == body_b {
                    continue;
                }
                let normal = contact.normal;
                let point = contact.point;

                let relative =
                    velocity_at(bodies, body_a, point) - velocity_at(bodies, body_b, point);
                let normal_velocity = normal.dot3(relative);
                let bias = (contact.penetration * bias_scale).min(self.settings.max_recovery_speed);
                let normal_mass = effective_mass(bodies, body_a, body_b, normal, point);
                if normal_mass <= 1e-12 {
                    continue;
                }

                let lambda = (bias - normal_velocity) / normal_mass;
                let previous = accumulated_normal[index];
                let next = (previous + lambda).max(0.0);
                let delta = next - previous;
                accumulated_normal[index] = next;
                if delta != 0.0 {
                    apply_impulses(bodies, body_a, body_b, point, normal * delta);
                }

                // Tangential friction, clamped by the accumulated normal impulse.
                if contact.friction > 0.0 && next > 0.0 {
                    let relative =
                        velocity_at(bodies, body_a, point) - velocity_at(bodies, body_b, point);
                    let tangent = relative - normal * relative.dot3(normal);
                    let tangent_length = tangent.length3();
                    if tangent_length > 1e-6 {
                        let direction = tangent / tangent_length;
                        let tangent_mass = effective_mass(bodies, body_a, body_b, direction, point);
                        if tangent_mass > 1e-12 {
                            let max_friction = contact.friction * next;
                            let tangent_impulse = (-relative.dot3(direction) / tangent_mass)
                                .clamp(-max_friction, max_friction);
                            if tangent_impulse != 0.0 {
                                apply_impulses(
                                    bodies,
                                    body_a,
                                    body_b,
                                    point,
                                    direction * tangent_impulse,
                                );
                            }
                        }
                    }
                }
            }
        }

        SolverStats {
            contacts: contacts.len(),
            iterations: self.settings.iterations,
        }
    }
}

fn body_index(length: usize, index: usize) -> Result<usize, ()> {
    if index < length { Ok(index) } else { Err(()) }
}

fn velocity_at(bodies: &[SolverBody], index: usize, point: Vector4) -> Vector4 {
    bodies[index].velocity_at(point)
}

fn apply_impulses(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    point: Vector4,
    impulse: Vector4,
) {
    bodies[body_a].apply_impulse_at(point, impulse);
    bodies[body_b].apply_impulse_at(point, -impulse);
}

/// Effective inverse mass along `direction` at `point`:
/// `1/mA + 1/mB + n·((IA^-1 (rA×n))×rA) + n·((IB^-1 (rB×n))×rB)`.
fn effective_mass(
    bodies: &[SolverBody],
    body_a: usize,
    body_b: usize,
    direction: Vector4,
    point: Vector4,
) -> f32 {
    let body_a = &bodies[body_a];
    let body_b = &bodies[body_b];
    body_a.inverse_mass
        + body_b.inverse_mass
        + angular_mass(body_a, direction, point)
        + angular_mass(body_b, direction, point)
}

fn angular_mass(body: &SolverBody, direction: Vector4, point: Vector4) -> f32 {
    if body.inverse_inertia == Vector4::ZERO {
        return 0.0;
    }
    let r = point - body.position;
    let torque = r.cross(direction);
    let angular = Vector4::from_xyz(
        torque.x * body.inverse_inertia.x,
        torque.y * body.inverse_inertia.y,
        torque.z * body.inverse_inertia.z,
    );
    angular.cross(r).dot3(direction)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ground_and_falling() -> (Vec<SolverBody>, ContactConstraint) {
        let bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 1.0, 0.0),
                Vector4::from_xyz(0.0, -1.0, 0.0),
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let contact = ContactConstraint::new(
            0,
            1,
            Vector4::UNIT_Y,
            Vector4::from_xyz(0.0, 0.5, 0.0),
            0.1,
            0.5,
        );
        (bodies, contact)
    }

    #[test]
    fn normal_impulse_stops_falling_body() {
        let (mut bodies, contact) = ground_and_falling();
        let stats = Solver::default().solve(&mut bodies, &[contact], 1.0 / 60.0);
        assert_eq!(stats.contacts, 1);
        assert_eq!(stats.iterations, 4);
        assert!(
            bodies[0].linear_velocity.y >= -1e-3,
            "vy {}",
            bodies[0].linear_velocity.y
        );
        assert_eq!(bodies[1].linear_velocity, Vector4::ZERO);
    }

    #[test]
    fn dynamic_dynamic_relative_velocity_is_separated() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 0.0, 0.0),
                Vector4::from_xyz(0.0, -1.0, 0.0),
                Vector4::ZERO,
            ),
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 1.0, 0.0),
                Vector4::from_xyz(0.0, 1.0, 0.0),
                Vector4::ZERO,
            ),
        ];
        let contact = ContactConstraint::new(
            0,
            1,
            Vector4::UNIT_Y,
            Vector4::from_xyz(0.0, 0.5, 0.0),
            0.02,
            0.0,
        );
        Solver::default().solve(&mut bodies, &[contact], 1.0 / 60.0);
        let relative =
            (bodies[0].linear_velocity - bodies[1].linear_velocity).dot3(Vector4::UNIT_Y);
        assert!(relative >= -1e-3, "relative normal velocity {relative}");
    }

    #[test]
    fn friction_reduces_tangential_velocity() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 1.0, 0.0),
                Vector4::from_xyz(3.0, -1.0, 0.0),
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let contact = ContactConstraint::new(
            0,
            1,
            Vector4::UNIT_Y,
            Vector4::from_xyz(0.0, 0.5, 0.0),
            0.1,
            0.5,
        );
        Solver::default().solve(&mut bodies, &[contact], 1.0 / 60.0);
        assert!(bodies[0].linear_velocity.x.abs() < 3.0);
        assert!(bodies[0].linear_velocity.y >= -1e-3);
    }

    #[test]
    fn non_dynamic_pair_is_unchanged() {
        let mut bodies = vec![
            SolverBody::static_body(Vector4::from_xyz(0.0, 1.0, 0.0)),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let contact = ContactConstraint::new(
            0,
            1,
            Vector4::UNIT_Y,
            Vector4::from_xyz(0.0, 0.5, 0.0),
            0.1,
            0.5,
        );
        Solver::default().solve(&mut bodies, &[contact], 1.0 / 60.0);
        assert_eq!(bodies[0].linear_velocity, Vector4::ZERO);
        assert_eq!(bodies[1].linear_velocity, Vector4::ZERO);
    }

    #[test]
    fn invalid_indices_are_ignored() {
        let mut bodies = vec![SolverBody::new(
            1.0,
            Vector4::ZERO,
            Vector4::ZERO,
            Vector4::UNIT_X,
            Vector4::ZERO,
        )];
        let contact = ContactConstraint::new(0, 99, Vector4::UNIT_Y, Vector4::ZERO, 0.1, 0.5);
        Solver::default().solve(&mut bodies, &[contact], 1.0 / 60.0);
        assert_eq!(bodies[0].linear_velocity, Vector4::UNIT_X);
    }
}
