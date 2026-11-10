//! `hknpMotion` — rigid motion state.

use hk_math::{Quaternion, Transform, Vector4};
use hknp_types::MotionType;

/// Motion state of a body (`hknpMotion`).
///
/// The SDK stores the orientation plus a double-precision center of mass and
/// packed half-precision inverse inertia. This phase stores the center of mass
/// as an f32 [`Vector4`], the mass as `0 == infinite`, and inverse inertia as a
/// diagonal `Vector4` in motion space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion {
    /// Motion type (`STATIC`, `KEYFRAMED`, `DYNAMIC`).
    pub motion_type: MotionType,
    /// Center of mass in world space (`m_centerOfMass`).
    pub position: Vector4,
    /// Orientation from world to inertia space (`m_orientation`).
    pub orientation: Quaternion,
    /// Current linear velocity in world space.
    pub linear_velocity: Vector4,
    /// Current angular velocity in world space.
    ///
    /// Note: the SDK stores angular velocity in motion/local space; this port
    /// exposes world-space values directly (see the crate docs).
    pub angular_velocity: Vector4,
    /// Mass; `0` means infinite mass (`m_inverseInertia[3] == 0` in the SDK).
    pub mass: f32,
    /// Diagonal inverse inertia in motion space (`m_inverseInertia[0..3]`).
    pub inverse_inertia: Vector4,
}

impl Motion {
    /// The shared static motion (`hknpMotionId::PRESET_STATIC`).
    pub fn static_motion() -> Self {
        Self {
            motion_type: MotionType::Static,
            position: Vector4::ZERO,
            orientation: Quaternion::IDENTITY,
            linear_velocity: Vector4::ZERO,
            angular_velocity: Vector4::ZERO,
            mass: 0.0,
            inverse_inertia: Vector4::ZERO,
        }
    }

    /// Construct a motion with the given type and state.
    pub fn new(
        motion_type: MotionType,
        position: Vector4,
        orientation: Quaternion,
        linear_velocity: Vector4,
        angular_velocity: Vector4,
        mass: f32,
    ) -> Self {
        Self {
            motion_type,
            position,
            orientation,
            linear_velocity,
            angular_velocity,
            mass,
            inverse_inertia: Vector4::ZERO,
        }
    }

    /// The motion transform (`orientation`, `position`).
    pub fn transform(&self) -> Transform {
        Transform::from_quaternion(self.orientation, self.position)
    }

    /// Update position/orientation from a transform.
    pub fn set_transform(&mut self, transform: Transform) {
        self.position = transform.translation;
        self.orientation = transform.rotation_quaternion();
    }

    /// Inverse mass (`0` for infinite mass).
    pub fn inverse_mass(&self) -> f32 {
        if self.mass > 0.0 {
            1.0 / self.mass
        } else {
            0.0
        }
    }

    /// Whether this motion is static.
    pub fn is_static(&self) -> bool {
        self.motion_type.is_static()
    }

    /// Whether this motion is dynamic.
    pub fn is_dynamic(&self) -> bool {
        self.motion_type.is_dynamic()
    }

    /// Integrate the motion for `delta_time` (semi-implicit Euler, no forces).
    ///
    /// Dynamic motions advance `position += linear_velocity * dt` and apply the
    /// world angular velocity as `q' = normalize(Δq * q)`. Static and keyframed
    /// motions are left untouched; collision/contact impulses are Phase 7/8 work,
    /// so this is only the kinematic half of the integrator.
    pub fn integrate(&mut self, delta_time: f32) {
        debug_assert!(delta_time >= 0.0 && delta_time.is_finite());
        if !self.is_dynamic() || delta_time == 0.0 {
            return;
        }

        self.position += self.linear_velocity * delta_time;

        let angular_speed = self.angular_velocity.length3();
        if angular_speed > 0.0 {
            let axis = self.angular_velocity / angular_speed;
            let delta_rotation = Quaternion::from_axis_angle(axis, angular_speed * delta_time);
            self.orientation = delta_rotation.mul(self.orientation).normalized();
        }
    }

    /// Apply a world-space linear impulse.
    pub fn apply_linear_impulse(&mut self, impulse: Vector4) {
        self.linear_velocity += impulse * self.inverse_mass();
    }

    /// Apply a world-space angular impulse using the diagonal inverse inertia.
    pub fn apply_angular_impulse(&mut self, impulse: Vector4) {
        self.angular_velocity += Vector4::from_xyz(
            impulse.x * self.inverse_inertia.x,
            impulse.y * self.inverse_inertia.y,
            impulse.z * self.inverse_inertia.z,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;
    use hknp_types::MotionType;

    #[test]
    fn static_motion_is_at_rest() {
        let motion = Motion::static_motion();
        assert!(motion.is_static());
        assert_eq!(motion.position, Vector4::ZERO);
        assert_eq!(motion.orientation, Quaternion::IDENTITY);
        assert_eq!(motion.inverse_mass(), 0.0);
    }

    #[test]
    fn dynamic_motion_integrates_translation() {
        let mut motion = Motion::new(
            MotionType::Dynamic,
            Vector4::ZERO,
            Quaternion::IDENTITY,
            Vector4::from_xyz(2.0, 0.0, 0.0),
            Vector4::ZERO,
            1.0,
        );
        motion.integrate(0.5);
        assert!(
            motion
                .position
                .all_equal3(Vector4::from_xyz(1.0, 0.0, 0.0), 1e-6)
        );
        motion.integrate(0.0);
        assert!(
            motion
                .position
                .all_equal3(Vector4::from_xyz(1.0, 0.0, 0.0), 1e-6)
        );
    }

    #[test]
    fn dynamic_motion_integrates_rotation() {
        let mut motion = Motion::new(
            MotionType::Dynamic,
            Vector4::ZERO,
            Quaternion::IDENTITY,
            Vector4::ZERO,
            Vector4::from_xyz(0.0, 0.0, FRAC_PI_2),
            1.0,
        );
        motion.integrate(1.0);
        let transform = motion.transform();
        // 90° about Z maps +X to +Y.
        let mapped = transform.transform_point(Vector4::UNIT_X);
        assert!(
            mapped.all_equal3(Vector4::UNIT_Y, 1e-5),
            "mapped {mapped:?}"
        );
        assert!((motion.orientation.length() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn static_and_keyframed_do_not_integrate() {
        let mut motion = Motion::new(
            MotionType::Static,
            Vector4::ZERO,
            Quaternion::IDENTITY,
            Vector4::UNIT_X,
            Vector4::ZERO,
            1.0,
        );
        motion.integrate(1.0);
        assert_eq!(motion.position, Vector4::ZERO);

        let mut motion = Motion::new(
            MotionType::Keyframed,
            Vector4::ZERO,
            Quaternion::IDENTITY,
            Vector4::UNIT_X,
            Vector4::ZERO,
            1.0,
        );
        motion.integrate(1.0);
        assert_eq!(motion.position, Vector4::ZERO);
    }

    #[test]
    fn impulses_use_mass_and_diagonal_inertia() {
        let mut motion = Motion::new(
            MotionType::Dynamic,
            Vector4::ZERO,
            Quaternion::IDENTITY,
            Vector4::ZERO,
            Vector4::ZERO,
            2.0,
        );
        motion.inverse_inertia = Vector4::from_xyz(1.0, 2.0, 3.0);
        motion.apply_linear_impulse(Vector4::from_xyz(2.0, 0.0, 0.0));
        assert_eq!(motion.linear_velocity, Vector4::from_xyz(1.0, 0.0, 0.0));
        motion.apply_angular_impulse(Vector4::from_xyz(1.0, 1.0, 1.0));
        assert_eq!(motion.angular_velocity, Vector4::from_xyz(1.0, 2.0, 3.0));

        // Infinite mass ignores linear impulses.
        let mut infinite = Motion::new(
            MotionType::Dynamic,
            Vector4::ZERO,
            Quaternion::IDENTITY,
            Vector4::ZERO,
            Vector4::ZERO,
            0.0,
        );
        infinite.apply_linear_impulse(Vector4::UNIT_X);
        assert_eq!(infinite.linear_velocity, Vector4::ZERO);
    }
}
