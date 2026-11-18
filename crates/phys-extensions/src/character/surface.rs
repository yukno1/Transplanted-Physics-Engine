//! Character surface information (`hknpCharacterSurfaceInfo` subset).
//!
//! The result of a character controller's `checkSupport` query: whether the
//! character stands on a supporting surface, slides down a too-steep slope or
//! is airborne, plus the averaged supporting surface normal and velocity.
//!
//! Original: `original/Physics/Source/Physics/Physics/Extensions/
//! CharacterControl/hknpCharacterSurfaceInfo.h` (the `m_surfaceDistanceExcess`
//! field is not ported; the proxy controller always leaves it at zero, see
//! `hknpCharacterProxy.cpp:202`).

use base_math::Vector4;

/// Support state reported by [`crate::character::CharacterProxy::check_support`]
/// (`hknpCharacterSurfaceInfo::SupportedState`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupportedState {
    /// The character does not touch any supporting surface.
    Unsupported,
    /// The character touches a surface steeper than its maximum slope.
    Sliding,
    /// The character is supported by a walkable surface.
    Supported,
}

/// Supporting surface information (`hknpCharacterSurfaceInfo`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceInfo {
    /// Whether the character is supported / sliding / unsupported.
    pub supported_state: SupportedState,
    /// Average normal of all touched supporting planes (world space, unit
    /// length; zero when [`SupportedState::Unsupported`]).
    pub surface_normal: Vector4,
    /// Average velocity of the supporting planes (world space).
    pub surface_velocity: Vector4,
    /// Average angular velocity of the supporting bodies (world space).
    pub surface_angular_velocity: Vector4,
}

impl SurfaceInfo {
    /// Unsupported surface with zeroed normal/velocity
    /// (`hknpCharacterSurfaceInfo` default state).
    pub const UNSUPPORTED: Self = Self {
        supported_state: SupportedState::Unsupported,
        surface_normal: Vector4::ZERO,
        surface_velocity: Vector4::ZERO,
        surface_angular_velocity: Vector4::ZERO,
    };

    /// Unsupported result with all fields zeroed.
    pub fn unsupported() -> Self {
        Self::UNSUPPORTED
    }
}

impl Default for SurfaceInfo {
    fn default() -> Self {
        Self::UNSUPPORTED
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_unsupported_with_zeroed_fields() {
        let info = SurfaceInfo::default();
        assert_eq!(info.supported_state, SupportedState::Unsupported);
        assert!(info.surface_normal.all_equal3(Vector4::ZERO, 0.0));
        assert!(info.surface_velocity.all_equal3(Vector4::ZERO, 0.0));
        assert!(info.surface_angular_velocity.all_equal3(Vector4::ZERO, 0.0));
    }
}
