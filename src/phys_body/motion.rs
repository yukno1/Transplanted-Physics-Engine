//! `hknpMotion` — rigid motion state.

use crate::base_math::{
    Mat3, Mat3Ext, Quat, QuatExt, Transform, Vec4, Vec4Ext, ext_quat::{quat_mul, quat_normalize},
};
use crate::phys_shape::{MassProperties, to_mat3, to_quaternion, to_vec4, to_vector4};
use crate::phys_types::MotionType;
use glam::Vec3;

/// Whether the off-diagonal terms are small enough to use the legacy fast path.
#[inline]
pub fn inverse_inertia_is_diagonal(matrix: Mat3, epsilon: f32) -> bool {
    matrix.element_sdk(0, 1).abs() <= epsilon
        && matrix.element_sdk(0, 2).abs() <= epsilon
        && matrix.element_sdk(1, 0).abs() <= epsilon
        && matrix.element_sdk(1, 2).abs() <= epsilon
        && matrix.element_sdk(2, 0).abs() <= epsilon
        && matrix.element_sdk(2, 1).abs() <= epsilon
}

/// Compute the world-space inverse inertia matrix from shape mass properties.
///
/// Singular (zero-volume/static) inertia produces a zero matrix, matching the
/// SDK's `HK_INVERSION_FAILURE_SET_ZERO` policy.
///
/// The `Transform` argument stays on the legacy type while the `Transform`
/// signature chain awaits its own migration step (plan §Phase 3 boundary ①).
pub fn world_inverse_inertia_from_mass_properties(
    properties: &MassProperties,
    transform: &Transform,
) -> Mat3 {
    if properties.mass <= 0.0 {
        return Mat3::ZERO;
    }
    let (inverse_local, _) = properties.inertia_tensor.inverse_sdk();
    let rotation = to_mat3(&transform.rotation);
    rotation
        .mul_mat_sdk(inverse_local)
        .mul_mat_sdk(rotation.transpose())
}

/// Motion state of a body (`hknpMotion`).
///
/// The SDK stores the orientation plus a double-precision center of mass and
/// packed half-precision inverse inertia. This phase stores the center of mass
/// as an f32 [`Vec4`], the mass as `0 == infinite`, and inverse inertia as a
/// diagonal [`Vec4`] in motion space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion {
    /// Motion type (`STATIC`, `KEYFRAMED`, `DYNAMIC`).
    pub motion_type: MotionType,
    /// Center of mass in world space (`m_centerOfMass`).
    pub position: Vec4,
    /// Orientation from world to inertia space (`m_orientation`).
    pub orientation: Quat,
    /// Current linear velocity in world space.
    pub linear_velocity: Vec4,
    /// Current angular velocity in world space.
    ///
    /// Note: the SDK stores angular velocity in motion/local space; this port
    /// exposes world-space values directly (see the crate docs).
    pub angular_velocity: Vec4,
    /// Mass; `0` means infinite mass (`m_inverseInertia[3] == 0` in the SDK).
    pub mass: f32,
    /// Diagonal inverse inertia in motion space (`m_inverseInertia[0..3]`).
    ///
    /// Kept as the compatibility view of [`Motion::inverse_inertia_world`].
    pub inverse_inertia: Vec4,
    /// Full world-space inverse inertia matrix used by the matrix solver path.
    pub inverse_inertia_world: Mat3,
}

impl Motion {
    /// The shared static motion (`hknpMotionId::PRESET_STATIC`).
    pub fn static_motion() -> Self {
        Self {
            motion_type: MotionType::Static,
            position: Vec4::ZERO,
            orientation: Quat::IDENTITY,
            linear_velocity: Vec4::ZERO,
            angular_velocity: Vec4::ZERO,
            mass: 0.0,
            inverse_inertia: Vec4::ZERO,
            inverse_inertia_world: Mat3::ZERO,
        }
    }

    /// Construct a motion with the given type and state.
    pub fn new(
        motion_type: MotionType,
        position: Vec4,
        orientation: Quat,
        linear_velocity: Vec4,
        angular_velocity: Vec4,
        mass: f32,
    ) -> Self {
        Self {
            motion_type,
            position,
            orientation,
            linear_velocity,
            angular_velocity,
            mass,
            inverse_inertia: Vec4::ZERO,
            inverse_inertia_world: Mat3::ZERO,
        }
    }

    /// The motion transform (`orientation`, `position`).
    pub fn transform(&self) -> Transform {
        Transform::from_quaternion(to_quaternion(self.orientation), to_vector4(self.position))
    }

    /// Update position/orientation from a transform.
    pub fn set_transform(&mut self, transform: Transform) {
        self.position = to_vec4(transform.translation);
        self.orientation = Quat::from_matrix_sdk(to_mat3(&transform.rotation));
    }

    /// Inverse mass (`0` for infinite mass).
    pub fn inverse_mass(&self) -> f32 {
        if self.mass > 0.0 {
            1.0 / self.mass
        } else {
            0.0
        }
    }

    /// Set the diagonal inverse inertia view and keep the matrix in sync.
    pub fn set_inverse_inertia_diagonal(&mut self, value: Vec4) {
        self.inverse_inertia = value;
        self.inverse_inertia_world = Mat3::from_diagonal(Vec3::new(value.x, value.y, value.z));
    }

    /// Set the full world-space inverse inertia matrix and refresh the diagonal view.
    pub fn set_inverse_inertia_world(&mut self, matrix: Mat3) {
        self.inverse_inertia_world = matrix;
        self.inverse_inertia = Vec4::from_xyz(
            matrix.element_sdk(0, 0),
            matrix.element_sdk(1, 1),
            matrix.element_sdk(2, 2),
        );
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

        let angular_speed = self.angular_velocity.length_squared3().sqrt();
        if angular_speed > 0.0 {
            let axis = self.angular_velocity / angular_speed;
            let delta_rotation = Quat::from_axis_angle(
                Vec3::new(axis.x, axis.y, axis.z),
                angular_speed * delta_time,
            );
            self.orientation = quat_normalize(quat_mul(delta_rotation, self.orientation));
        }
    }

    /// Apply a world-space linear impulse.
    pub fn apply_linear_impulse(&mut self, impulse: Vec4) {
        self.linear_velocity += impulse * self.inverse_mass();
    }

    /// Apply a world-space angular impulse.
    ///
    /// Diagonal matrices keep the original component-wise operation order; full
    /// tensors use a single matrix-vector product.
    pub fn apply_angular_impulse(&mut self, impulse: Vec4) {
        if inverse_inertia_is_diagonal(self.inverse_inertia_world, 1e-8) {
            self.angular_velocity += Vec4::from_xyz(
                impulse.x * self.inverse_inertia.x,
                impulse.y * self.inverse_inertia.y,
                impulse.z * self.inverse_inertia.z,
            );
        } else {
            let delta = self
                .inverse_inertia_world
                .mul_vec_sdk(Vec3::new(impulse.x, impulse.y, impulse.z));
            self.angular_velocity += Vec4::from_xyz(delta.x, delta.y, delta.z);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::phys_types::MotionType;
    use core::f32::consts::FRAC_PI_2;

    #[test]
    fn static_motion_is_at_rest() {
        let motion = Motion::static_motion();
        assert!(motion.is_static());
        assert_eq!(motion.position, Vec4::ZERO);
        assert_eq!(motion.orientation, Quat::IDENTITY);
        assert_eq!(motion.inverse_mass(), 0.0);
    }

    #[test]
    fn dynamic_motion_integrates_translation() {
        let mut motion = Motion::new(
            MotionType::Dynamic,
            Vec4::ZERO,
            Quat::IDENTITY,
            Vec4::from_xyz(2.0, 0.0, 0.0),
            Vec4::ZERO,
            1.0,
        );
        motion.integrate(0.5);
        assert!(
            motion
                .position
                .all_equal3(Vec4::from_xyz(1.0, 0.0, 0.0), 1e-6)
        );
        motion.integrate(0.0);
        assert!(
            motion
                .position
                .all_equal3(Vec4::from_xyz(1.0, 0.0, 0.0), 1e-6)
        );
    }

    #[test]
    fn dynamic_motion_integrates_rotation() {
        let mut motion = Motion::new(
            MotionType::Dynamic,
            Vec4::ZERO,
            Quat::IDENTITY,
            Vec4::ZERO,
            Vec4::from_xyz(0.0, 0.0, FRAC_PI_2),
            1.0,
        );
        motion.integrate(1.0);
        let transform = motion.transform();
        // 90° about Z maps +X to +Y.
        let mapped = transform.transform_point(crate::base_math::Vector4::UNIT_X);
        assert!(
            mapped.all_equal3(crate::base_math::Vector4::UNIT_Y, 1e-5),
            "mapped {mapped:?}"
        );
        let length_squared = motion.orientation.x * motion.orientation.x
            + motion.orientation.y * motion.orientation.y
            + motion.orientation.z * motion.orientation.z
            + motion.orientation.w * motion.orientation.w;
        assert!((length_squared.sqrt() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn static_and_keyframed_do_not_integrate() {
        let mut motion = Motion::new(
            MotionType::Static,
            Vec4::ZERO,
            Quat::IDENTITY,
            Vec4::from_xyz(1.0, 0.0, 0.0),
            Vec4::ZERO,
            1.0,
        );
        motion.integrate(1.0);
        assert_eq!(motion.position, Vec4::ZERO);

        let mut motion = Motion::new(
            MotionType::Keyframed,
            Vec4::ZERO,
            Quat::IDENTITY,
            Vec4::from_xyz(1.0, 0.0, 0.0),
            Vec4::ZERO,
            1.0,
        );
        motion.integrate(1.0);
        assert_eq!(motion.position, Vec4::ZERO);
    }

    #[test]
    fn impulses_use_mass_and_diagonal_inertia() {
        let mut motion = Motion::new(
            MotionType::Dynamic,
            Vec4::ZERO,
            Quat::IDENTITY,
            Vec4::ZERO,
            Vec4::ZERO,
            2.0,
        );
        motion.inverse_inertia = Vec4::from_xyz(1.0, 2.0, 3.0);
        motion.apply_linear_impulse(Vec4::from_xyz(2.0, 0.0, 0.0));
        assert_eq!(motion.linear_velocity, Vec4::from_xyz(1.0, 0.0, 0.0));
        motion.apply_angular_impulse(Vec4::from_xyz(1.0, 1.0, 1.0));
        assert_eq!(motion.angular_velocity, Vec4::from_xyz(1.0, 2.0, 3.0));

        // Infinite mass ignores linear impulses.
        let mut infinite = Motion::new(
            MotionType::Dynamic,
            Vec4::ZERO,
            Quat::IDENTITY,
            Vec4::ZERO,
            Vec4::ZERO,
            0.0,
        );
        infinite.apply_linear_impulse(Vec4::from_xyz(1.0, 0.0, 0.0));
        assert_eq!(infinite.linear_velocity, Vec4::ZERO);
    }
}
