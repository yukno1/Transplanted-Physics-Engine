//! `phys-solver` — iterative contact solver.
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
//! Tangential impulses use a deterministic orthonormal basis of the contact
//! normal and are clamped by `friction * accumulated_normal`.
//!
//! # Warm starting
//!
//! [`ContactImpulse`] holds the accumulated normal/tangent impulses of one
//! contact row. Callers may carry these across steps (the world matches rows by
//! body pair and local contact point); [`Solver::solve_with_impulses`] applies
//! the previous impulses before the first sweep and writes the new ones back,
//! which is the SDK's warm starting. [`Solver::solve`] is the cold-start
//! shortcut (zeroed impulses).
//!
//! # Subsets (islands)
//!
//! [`Solver::solve_contacts_subset`] / [`Solver::solve_joints_subset`] solve only
//! the listed constraint indices. The world passes one island at a time; since
//! islands share no dynamic body the impulses are identical to a global sweep,
//! but solver state stays local to an island.
//!
//! # Not ported in this phase
//!
//! The full constraint-atom system (`Solver/ConstraintAtom`, 49 files),
//! restitution/mass modifiers, solver bodies' inertia in inertia space,
//! deactivation, and the SDK's determinism-check instrumentation. Material-
//! driven friction/restitution and multi-point manifolds arrive with the
//! material library and the extended manifold work.

use base_math::Vector4;

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

/// Accumulated impulse of one contact row (`hknpContactJacobian` accumulator).
///
/// Persisting these values between steps and feeding them back through
/// [`Solver::solve_with_impulses`] implements warm starting: the solver starts
/// from last step's solution instead of zero, which removes most of the
/// convergence error in stacks.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactImpulse {
    /// Accumulated impulse along the contact normal (never negative).
    pub normal: f32,
    /// Accumulated friction impulse along the first tangent.
    pub tangent_1: f32,
    /// Accumulated friction impulse along the second tangent.
    pub tangent_2: f32,
}

impl ContactImpulse {
    /// Whether no impulse has been accumulated yet.
    pub fn is_zero(&self) -> bool {
        self.normal == 0.0 && self.tangent_1 == 0.0 && self.tangent_2 == 0.0
    }

    /// Total accumulated impulse vector given the contact normal.
    pub fn to_vector(&self, normal: Vector4) -> Vector4 {
        let (tangent_1, tangent_2) = contact_tangents(normal);
        normal * self.normal + tangent_1 * self.tangent_1 + tangent_2 * self.tangent_2
    }
}

/// Deterministic orthonormal tangent basis of a (unit) contact normal.
///
/// The reference axis is `+X` unless the normal is nearly parallel to it, in
/// which case `+Y` is used, so the basis only depends on the normal and is
/// therefore stable across steps (required for warm starting).
pub fn contact_tangents(normal: Vector4) -> (Vector4, Vector4) {
    let reference = if normal.x.abs() < 0.9 {
        Vector4::UNIT_X
    } else {
        Vector4::UNIT_Y
    };
    let tangent_1 = normal.cross(reference).normalized3();
    let tangent_2 = normal.cross(tangent_1);
    (tangent_1, tangent_2)
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
    /// Apply the supplied accumulated impulses before the first sweep and keep
    /// them updated afterwards (`hknpSolverInfo::m_warmStarting`).
    pub warm_starting: bool,
}

impl Default for SolverSettings {
    fn default() -> Self {
        Self {
            iterations: 4,
            baumgarte: 0.2,
            max_recovery_speed: 1.0,
            friction: 0.5,
            warm_starting: true,
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

    /// Solve `contacts` in place on `bodies` without warm starting.
    ///
    /// Constraints must reference valid body indices; invalid ones are skipped.
    /// `delta_time` is used for the Baumgarte bias (0 disables recovery).
    pub fn solve(
        &self,
        bodies: &mut [SolverBody],
        contacts: &[ContactConstraint],
        delta_time: f32,
    ) -> SolverStats {
        let mut impulses = vec![ContactImpulse::default(); contacts.len()];
        let indices: Vec<usize> = (0..contacts.len()).collect();
        self.solve_contacts_subset(bodies, contacts, &indices, &mut impulses, delta_time)
    }

    /// Solve all `contacts`, warm starting from (and updating) `impulses`.
    ///
    /// `impulses` is indexed like `contacts`; entries left over from a previous
    /// step are applied before the first sweep. When
    /// [`SolverSettings::warm_starting`] is `false` the incoming impulses are
    /// discarded and the call behaves like [`Solver::solve`].
    pub fn solve_with_impulses(
        &self,
        bodies: &mut [SolverBody],
        contacts: &[ContactConstraint],
        impulses: &mut [ContactImpulse],
        delta_time: f32,
    ) -> SolverStats {
        let indices: Vec<usize> = (0..contacts.len()).collect();
        self.solve_contacts_subset(bodies, contacts, &indices, impulses, delta_time)
    }

    /// Solve only the contacts listed in `subset`, in ascending index order.
    ///
    /// `subset` indices address `contacts`; `impulses` must be at least as long
    /// as `contacts` so warm-start state can be updated in place. The world uses
    /// this to solve one constraint island at a time.
    pub fn solve_contacts_subset(
        &self,
        bodies: &mut [SolverBody],
        contacts: &[ContactConstraint],
        subset: &[usize],
        impulses: &mut [ContactImpulse],
        delta_time: f32,
    ) -> SolverStats {
        debug_assert!(delta_time >= 0.0 && delta_time.is_finite());
        debug_assert!(impulses.len() >= contacts.len());

        let bias_scale = if delta_time > 0.0 {
            self.settings.baumgarte / delta_time
        } else {
            0.0
        };

        // Warm start: replay last step's impulses, or reset them when disabled.
        for &index in subset {
            let contact = &contacts[index];
            let (Ok(body_a), Ok(body_b)) = (
                body_index(bodies.len(), contact.body_a),
                body_index(bodies.len(), contact.body_b),
            ) else {
                impulses[index] = ContactImpulse::default();
                continue;
            };
            if body_a == body_b {
                impulses[index] = ContactImpulse::default();
                continue;
            }
            if !self.settings.warm_starting {
                impulses[index] = ContactImpulse::default();
                continue;
            }
            let accumulated = impulses[index];
            if !accumulated.is_zero() {
                let impulse = accumulated.to_vector(contact.normal);
                apply_impulses(bodies, body_a, body_b, contact.point, impulse);
            }
        }

        for _ in 0..self.settings.iterations {
            for &index in subset {
                let contact = &contacts[index];
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
                let mut accumulated = impulses[index];

                let relative =
                    velocity_at(bodies, body_a, point) - velocity_at(bodies, body_b, point);
                let normal_velocity = normal.dot3(relative);
                let bias = (contact.penetration * bias_scale).min(self.settings.max_recovery_speed);
                let normal_mass = effective_mass(bodies, body_a, body_b, normal, point);
                if normal_mass <= 1e-12 {
                    continue;
                }

                let lambda = (bias - normal_velocity) / normal_mass;
                let next = (accumulated.normal + lambda).max(0.0);
                let delta = next - accumulated.normal;
                accumulated.normal = next;
                if delta != 0.0 {
                    apply_impulses(bodies, body_a, body_b, point, normal * delta);
                }

                // Tangential friction along both basis vectors, clamped by the
                // accumulated normal impulse (box friction cone).
                if contact.friction > 0.0 && next > 0.0 {
                    let (tangent_1, tangent_2) = contact_tangents(normal);
                    let max_friction = contact.friction * next;
                    for (slot, tangent) in [
                        (&mut accumulated.tangent_1, tangent_1),
                        (&mut accumulated.tangent_2, tangent_2),
                    ] {
                        let tangent_mass = effective_mass(bodies, body_a, body_b, tangent, point);
                        if tangent_mass <= 1e-12 {
                            continue;
                        }
                        let relative =
                            velocity_at(bodies, body_a, point) - velocity_at(bodies, body_b, point);
                        let previous = *slot;
                        let next = (previous - relative.dot3(tangent) / tangent_mass)
                            .clamp(-max_friction, max_friction);
                        let delta = next - previous;
                        *slot = next;
                        if delta != 0.0 {
                            apply_impulses(bodies, body_a, body_b, point, tangent * delta);
                        }
                    }
                }

                impulses[index] = accumulated;
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

pub(crate) fn angular_mass(body: &SolverBody, direction: Vector4, point: Vector4) -> f32 {
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
    fn contact_tangents_are_orthonormal() {
        for normal in [
            Vector4::UNIT_Y,
            Vector4::UNIT_X,
            Vector4::from_xyz(0.3, -0.8, 0.5).normalized3(),
        ] {
            let (tangent_1, tangent_2) = contact_tangents(normal);
            assert!((tangent_1.length3() - 1.0).abs() < 1e-5);
            assert!((tangent_2.length3() - 1.0).abs() < 1e-5);
            assert!(tangent_1.dot3(normal).abs() < 1e-5);
            assert!(tangent_2.dot3(normal).abs() < 1e-5);
            assert!(tangent_1.dot3(tangent_2).abs() < 1e-5);
            // Deterministic: the same normal always maps to the same basis.
            assert_eq!(contact_tangents(normal), (tangent_1, tangent_2));
        }
    }

    #[test]
    fn warm_start_impulses_are_reused() {
        // Resting contact: no penetration, approaching at the per-step gravity
        // delta so the required impulse is 0.1 N·s.
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 1.0, 0.0),
                Vector4::from_xyz(0.0, -0.1, 0.0),
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let contact = ContactConstraint::new(
            0,
            1,
            Vector4::UNIT_Y,
            Vector4::from_xyz(0.0, 0.5, 0.0),
            0.0,
            0.0,
        );
        let solver = Solver::default();
        let mut impulses = vec![ContactImpulse::default(); 1];
        solver.solve_with_impulses(&mut bodies, &[contact], &mut impulses, 1.0 / 60.0);
        assert!((impulses[0].normal - 0.1).abs() < 1e-4);

        // Next step: gravity adds the same velocity again. Warm starting must
        // reuse the cached impulse and need no extra correction.
        bodies[0].linear_velocity = Vector4::from_xyz(0.0, -0.1, 0.0);
        solver.solve_with_impulses(&mut bodies, &[contact], &mut impulses, 1.0 / 60.0);
        assert!((impulses[0].normal - 0.1).abs() < 1e-4);
        assert!(
            bodies[0].linear_velocity.y.abs() < 1e-4,
            "vy {}",
            bodies[0].linear_velocity.y
        );
    }

    #[test]
    fn warm_starting_disabled_discards_impulses() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 1.0, 0.0),
                Vector4::from_xyz(0.0, -0.1, 0.0),
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
        ];
        let contact = ContactConstraint::new(
            0,
            1,
            Vector4::UNIT_Y,
            Vector4::from_xyz(0.0, 0.5, 0.0),
            0.0,
            0.0,
        );
        let mut solver = Solver::default();
        solver.settings.warm_starting = false;
        let mut impulses = vec![ContactImpulse {
            normal: 100.0,
            tangent_1: 0.0,
            tangent_2: 0.0,
        }];
        solver.solve_with_impulses(&mut bodies, &[contact], &mut impulses, 1.0 / 60.0);
        // The bogus 100 N·s warm-start impulse must not have been applied.
        assert!(
            (impulses[0].normal - 0.1).abs() < 1e-4,
            "impulse {}",
            impulses[0].normal
        );
        assert!(bodies[0].linear_velocity.y.abs() < 1e-4);
    }

    #[test]
    fn subset_solve_leaves_other_contacts_untouched() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 1.0, 0.0),
                Vector4::from_xyz(0.0, -1.0, 0.0),
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::ZERO),
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(5.0, 1.0, 0.0),
                Vector4::from_xyz(0.0, -1.0, 0.0),
                Vector4::ZERO,
            ),
            SolverBody::static_body(Vector4::from_xyz(5.0, 0.0, 0.0)),
        ];
        let contacts = [
            ContactConstraint::new(
                0,
                1,
                Vector4::UNIT_Y,
                Vector4::from_xyz(0.0, 0.5, 0.0),
                0.1,
                0.5,
            ),
            ContactConstraint::new(
                2,
                3,
                Vector4::UNIT_Y,
                Vector4::from_xyz(5.0, 0.5, 0.0),
                0.1,
                0.5,
            ),
        ];
        let mut impulses = vec![ContactImpulse::default(); 2];
        let stats = Solver::default().solve_contacts_subset(
            &mut bodies,
            &contacts,
            &[1],
            &mut impulses,
            1.0 / 60.0,
        );
        assert_eq!(stats.contacts, 2);
        assert_eq!(bodies[0].linear_velocity.y, -1.0);
        assert!(bodies[2].linear_velocity.y >= -1e-3);
        assert_eq!(impulses[0], ContactImpulse::default());
        assert!(impulses[1].normal > 0.0);
    }

    #[test]
    fn friction_impulses_stay_inside_the_cone() {
        let mut bodies = vec![
            SolverBody::new(
                1.0,
                Vector4::ZERO,
                Vector4::from_xyz(0.0, 1.0, 0.0),
                Vector4::from_xyz(10.0, -5.0, 3.0),
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
        let mut impulses = vec![ContactImpulse::default(); 1];
        Solver::default().solve_with_impulses(&mut bodies, &[contact], &mut impulses, 1.0 / 60.0);
        let impulse = impulses[0];
        let limit = 0.5 * impulse.normal;
        assert!(impulse.tangent_1.abs() <= limit + 1e-5);
        assert!(impulse.tangent_2.abs() <= limit + 1e-5);
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
