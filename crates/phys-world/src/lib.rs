//! `phys-world` — world assembly and full single-threaded step pipeline.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Dynamics/World/`
//! (migration Phase 6, extended in Phase 8, README §4).
//!
//! | Rust item | Original C++ | Notes |
//! |---|---|---|
//! | [`WorldCinfo`] | `hknpWorldCinfo` | builder with gravity/capacities/solver settings |
//! | [`World`] | `hknpWorld` | owns body/motion managers and runs collide → solve → integrate |
//! | [`ContactManifold`] | manifold stream item | body pair plus [`phys_collide::Manifold`] |
//! | [`WorldStats`] | counters | body/motion/step/contact counters |
//! | [`BodySnapshot`]/[`WorldSnapshot`] | determinism trace | `hvk-trace v1` text |
//! | [`BallAndSocketCinfo`]/[`HingeCinfo`]/[`FixedCinfo`]/[`RagdollCinfo`]/[`PrismaticCinfo`]/[`SpringCinfo`]/[`PointToPlaneCinfo`] | `Dynamics/Constraint/Data` subset | local-space joint construction info |
//! | [`WorldJoint`]/[`JointType`] | `hknpConstraintData::ConstraintType` subset | stored joint families and body↔constraint lookup |
//! | [`QueryFilter`] | `hknpCollisionQuery` filter subset | body/motion-type filtering for world queries |
//! | [`RayCastHit`]/[`ShapeCastHitInfo`]/[`ClosestPointHit`]/[`ClosestPointsHit`] | `Collide/Query` subset | world ray cast, shape cast, closest point/shape and AABB queries |
//!
//! # Step pipeline
//!
//! [`World::step`] runs the single-threaded simulation pipeline:
//!
//! 1. **Collide**: build world AABBs, run the deterministic grid broad phase,
//!    then narrow-phase convex-convex contact generation (`phys-collide`).
//!    [`World::collect_contacts`] exposes this stage for tests/tools.
//! 2. **Islands**: partition dynamic bodies into constraint islands
//!    (`union-find` over contacts and joints; static bodies never merge
//!    islands). [`World::islands`] exposes the partition.
//! 3. **Solve**: apply gravity to dynamic motions, build solver bodies and
//!    single-point contact constraints, warm start each contact row from the
//!    previous step's impulses, then solve every island (contacts followed by
//!    the enabled joints: ragdoll/prismatic/spring/point-to-plane/…) and write
//!    solved velocities back to motions.
//! 4. **Integrate**: semi-implicit Euler integration of dynamic motions and
//!    refresh of body transforms/AABBs for the next step.
//!
//! Queries ([`World::raycast`], [`World::cast_shape`],
//! [`World::closest_point`], [`World::closest_points_to_shape`],
//! [`World::query_aabb`]) run against the current cached poses and never
//! modify the world.
//!
//! The pipeline is deliberately single-threaded and deterministic: bodies are
//! visited in slot order, contacts in canonical broad-phase pair order, and the
//! solver sweeps contacts in insertion order.
//!
//! # Scope limits
//!
//! Only convex shapes participate in narrow-phase contacts; mesh/height-field/
//! compound bodies generate no contacts yet (queries do traverse them). The
//! solver uses one contact point per pair (multi-point clipping is deferred),
//! together with a velocity-level Baumgarte recovery bias and Coulomb friction.
//! Contact impulses are warm started across steps ([`World::clear_contact_cache`]
//! drops the cache), and so are joint rows: each constraint slot keeps its
//! accumulated solver-row impulses and replays them before the next step's
//! sweeps (unilateral rows only while violated; disabling a constraint clears
//! them). Motors (hinge velocity/position/spring, prismatic position/spring)
//! are applied once per solve as clamped impulses.
//!
//! # Island sleeping and parallel solving
//!
//! With [`WorldCinfo::enable_sleeping`] (default off) an island whose dynamic
//! bodies stay under the sleeping velocity thresholds for
//! [`WorldCinfo::time_to_sleep`] seconds falls asleep: its motions freeze, its
//! narrow-phase contacts are skipped and gravity no longer applies, until an
//! active or keyframed body touches it (wake propagation through the island)
//! or [`World::wake_body`] is called. Islands are disjoint in dynamic solver
//! bodies, so [`World::step_with_threads`] can solve island groups in parallel
//! and merge the per-island results with bit-identical velocities.
//!
//! Material libraries, restitution, broad-phase layers, command queues and
//! events/modifiers remain future work.
//!
//! # Not ported in this phase
//!
//! Body addition queues, command queue/determinism checking, deactivation
//! manager, collision filters, events/modifiers, user body buffers,
//! `hknpWorldReader`/`hknpWorldWriter` traits, body attachment and motion
//! sharing, the SDK constraint-atom/constraint-group system, the vehicle
//! extension layer (suspension raycast / engine / friction solver), and `hkx`
//! asset loading.

use base_math::{Quaternion, Vector4};
use phys_collide::Manifold;
use phys_types::BodyId;

mod cache;
pub mod cinfo;
pub mod constraint;
mod island;
pub mod query;
pub mod world;

pub use cinfo::WorldCinfo;
pub use constraint::{
    BallAndSocketCinfo, FixedCinfo, HingeCinfo, HingeLimitsCinfo, HingeMotorCinfo,
    HingePositionMotorCinfo, HingeSpringMotorCinfo, HingeVelocityMotorCinfo, JointType,
    PointToPlaneCinfo, PrismaticCinfo, PrismaticMotorCinfo, PrismaticPositionMotorCinfo,
    PrismaticSpringMotorCinfo, RagdollCinfo, SpringCinfo, WheelCinfo, WheelDriveCinfo,
    WheelSuspensionCinfo, WorldJoint,
};
pub use island::IslandInfo;
pub use query::{ClosestPointHit, ClosestPointsHit, QueryFilter, RayCastHit, ShapeCastHitInfo};
pub use world::World;

/// Deterministic per-body state captured by [`World::snapshot`].
///
/// Bodies are listed in body-manager slot order. The orientation is taken from
/// the motion when available, otherwise from the cached body transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodySnapshot {
    /// Body identifier.
    pub body: BodyId,
    /// World position (body transform translation).
    pub position: Vector4,
    /// World orientation.
    pub orientation: Quaternion,
    /// World linear velocity.
    pub linear_velocity: Vector4,
    /// World angular velocity.
    pub angular_velocity: Vector4,
}

/// Debug snapshot of a world, used for determinism checks and golden traces.
#[derive(Clone, Debug, PartialEq)]
pub struct WorldSnapshot {
    /// Number of completed steps.
    pub step_count: u64,
    /// Body states in slot order.
    pub bodies: Vec<BodySnapshot>,
}

impl WorldSnapshot {
    /// Render the snapshot in `hvk-trace v1` format.
    ///
    /// Floats are written as raw `f32::to_bits()` hex to make text traces
    /// bit-exact and independent of `Display` formatting.
    pub fn to_text(&self) -> String {
        use std::fmt::Write as _;

        let mut output = String::from("# hvk-trace v1\n");
        writeln!(output, "step {}", self.step_count).expect("writing to a String cannot fail");
        for body in &self.bodies {
            writeln!(
                output,
                "body {:016x} p {:08x} {:08x} {:08x} q {:08x} {:08x} {:08x} {:08x} v {:08x} {:08x} {:08x} w {:08x} {:08x} {:08x}",
                body.body.value(),
                body.position.x.to_bits(),
                body.position.y.to_bits(),
                body.position.z.to_bits(),
                body.orientation.x.to_bits(),
                body.orientation.y.to_bits(),
                body.orientation.z.to_bits(),
                body.orientation.w.to_bits(),
                body.linear_velocity.x.to_bits(),
                body.linear_velocity.y.to_bits(),
                body.linear_velocity.z.to_bits(),
                body.angular_velocity.x.to_bits(),
                body.angular_velocity.y.to_bits(),
                body.angular_velocity.z.to_bits(),
            )
            .expect("writing to a String cannot fail");
        }
        output
    }
}

/// One body pair with a generated contact manifold.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifold {
    /// Body A (lower broad-phase pair id; manifold normal points from B to A).
    pub body_a: BodyId,
    /// Body B.
    pub body_b: BodyId,
    /// Narrow-phase manifold (normal B → A, positions on B).
    pub manifold: Manifold,
}

/// Snapshot of world counters (`WorldStats`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorldStats {
    /// Number of live bodies.
    pub body_count: usize,
    /// Number of live non-static motions.
    pub motion_count: usize,
    /// Number of completed steps.
    pub step_count: u64,
    /// Manifolds generated during the most recent step.
    pub last_manifold_count: usize,
    /// Contact points generated during the most recent step.
    pub last_contact_count: usize,
    /// Constraint islands solved during the most recent step.
    pub last_island_count: usize,
    /// Dynamic bodies currently asleep (deactivation subset).
    pub sleeping_body_count: usize,
}
