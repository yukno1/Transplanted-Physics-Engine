//! Local monitor-style diagnostics for character controllers.
//!
//! The SDK routes `monitorStreamCallback` through `hkMonitorStream` and draws
//! debug markers through `hkDebugDisplayHandler`. This zero-dependency port
//! intentionally stays local: listeners receive small, copyable snapshots at
//! fixed phases, in deterministic reverse-registration order. No stream,
//! socket or rendering backend is involved, and no private controller object
//! is passed to callbacks.

use crate::base_math::Vector4;
use crate::phys_types::BodyId;

/// Phase in which a monitor snapshot is produced.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CharacterMonitorPhase {
    /// Before a proxy/rigid-body movement step.
    IntegrateBegin,
    /// After a proxy/rigid-body movement step.
    IntegrateEnd,
    /// While filtered post-collide contacts are dispatched.
    PostCollide,
    /// After the world step (from the cached state).
    PostSolve,
}

/// Copyable diagnostic snapshot (`hkMonitorStream`-style, local subset).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterMonitorEvent {
    /// Per-character step counter (starts at zero, increments once per step).
    pub step_count: u64,
    /// Phase that produced the snapshot.
    pub phase: CharacterMonitorPhase,
    /// Character body id (zero for a standalone proxy without a world body).
    pub body: BodyId,
    /// World position at the snapshot point.
    pub position: Vector4,
    /// Linear velocity at the snapshot point.
    pub linear_velocity: Vector4,
    /// Number of visible contacts at the snapshot point.
    pub contact_count: usize,
}

/// A begin/end pair for the same character step.
///
/// Provided for callers that want to record spans instead of individual
/// events; the controller dispatches the two events independently.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterMonitorSpan {
    /// Snapshot before the step.
    pub begin: CharacterMonitorEvent,
    /// Snapshot after the step.
    pub end: CharacterMonitorEvent,
}

impl CharacterMonitorSpan {
    /// Pair two phase snapshots into one span.
    pub fn new(begin: CharacterMonitorEvent, end: CharacterMonitorEvent) -> Self {
        Self { begin, end }
    }
}
