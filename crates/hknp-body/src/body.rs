//! `hknpBody` and `hknpBodyCinfo`.

use std::sync::Arc;

use hk_math::{Aabb, Quaternion, Transform, Vector4};
use hknp_shape::Shape;
use hknp_types::{
    BodyId, BodyIndex, BodyQualityId, CollisionFlags, MaterialId, MotionId, MotionPropertiesId,
    MotionType,
};

/// Default `hknpBodyCinfo::s_DefaultMaxDepenetrationSpeed` (1.0 m/s).
pub const DEFAULT_MAX_DEPENETRATION_SPEED: f32 = 1.0;

/// Construction information for a [`Body`] (`hknpBodyCinfo`).
///
/// Build with [`BodyCinfo::new`] (a shape is required) and chain the `with_*`
/// methods:
///
/// ```
/// use std::sync::Arc;
/// use hk_math::Vector4;
/// use hknp_body::BodyCinfo;
/// use hknp_shape::SphereShape;
/// use hknp_types::MotionType;
///
/// let shape = Arc::new(SphereShape::new(Vector4::ZERO, 1.0));
/// let cinfo = BodyCinfo::new(shape)
///     .with_motion_type(MotionType::Dynamic)
///     .with_position(Vector4::from_xyz(0.0, 10.0, 0.0))
///     .with_mass(2.0);
/// assert_eq!(cinfo.motion_type, MotionType::Dynamic);
/// ```
#[derive(Clone, Debug)]
pub struct BodyCinfo {
    /// Collision shape (required by the SDK, cannot be null).
    pub shape: Arc<dyn Shape>,
    /// Body flags (events/modifiers).
    pub flags: CollisionFlags,
    /// Collision filtering info (`hknpCollisionFilter`).
    pub collision_filter_info: u32,
    /// Root material id.
    pub material_id: MaterialId,
    /// Body quality id (`INVALID` selects the motion-type preset).
    pub quality_id: BodyQualityId,
    /// Optional body name.
    pub name: Option<String>,
    /// User data copied into [`Body::user_data`].
    pub user_data: u64,
    /// Initial motion type.
    pub motion_type: MotionType,
    /// Initial world position (center of mass).
    pub position: Vector4,
    /// Initial world orientation.
    pub orientation: Quaternion,
    /// Initial world linear velocity (ignored for static bodies).
    pub linear_velocity: Vector4,
    /// Initial world angular velocity (ignored for static bodies).
    pub angular_velocity: Vector4,
    /// Mass: `< 0` use the shape mass, `0` infinite mass, `> 0` explicit mass.
    pub mass: f32,
    /// Motion properties template.
    pub motion_properties_id: MotionPropertiesId,
    /// Maximum penetration recovery speed.
    pub max_depenetration_speed: f32,
    /// Extra collision distance (continuous collision look-ahead).
    pub collision_look_ahead_distance: f32,
}

impl BodyCinfo {
    /// Construct from the required shape; every other field gets the SDK default
    /// (static body at the origin with identity orientation).
    pub fn new(shape: Arc<dyn Shape>) -> Self {
        Self {
            shape,
            flags: CollisionFlags::NONE,
            collision_filter_info: 0,
            material_id: MaterialId::PRESET_DEFAULT,
            quality_id: BodyQualityId::INVALID,
            name: None,
            user_data: 0,
            motion_type: MotionType::Static,
            position: Vector4::ZERO,
            orientation: Quaternion::IDENTITY,
            linear_velocity: Vector4::ZERO,
            angular_velocity: Vector4::ZERO,
            mass: -1.0,
            motion_properties_id: MotionPropertiesId::INVALID,
            max_depenetration_speed: DEFAULT_MAX_DEPENETRATION_SPEED,
            collision_look_ahead_distance: 0.0,
        }
    }

    /// Set the motion type.
    #[must_use]
    pub fn with_motion_type(mut self, motion_type: MotionType) -> Self {
        self.motion_type = motion_type;
        self
    }

    /// Set the world position.
    #[must_use]
    pub fn with_position(mut self, position: Vector4) -> Self {
        self.position = position;
        self
    }

    /// Set the world orientation.
    #[must_use]
    pub fn with_orientation(mut self, orientation: Quaternion) -> Self {
        self.orientation = orientation;
        self
    }

    /// Set both position and orientation from a transform.
    #[must_use]
    pub fn with_transform(mut self, transform: Transform) -> Self {
        self.position = transform.translation;
        self.orientation = transform.rotation_quaternion();
        self
    }

    /// Set the initial linear velocity.
    #[must_use]
    pub fn with_linear_velocity(mut self, velocity: Vector4) -> Self {
        self.linear_velocity = velocity;
        self
    }

    /// Set the initial angular velocity (world space).
    #[must_use]
    pub fn with_angular_velocity(mut self, velocity: Vector4) -> Self {
        self.angular_velocity = velocity;
        self
    }

    /// Set the mass (`0` means infinite mass).
    #[must_use]
    pub fn with_mass(mut self, mass: f32) -> Self {
        self.mass = mass;
        self
    }

    /// Set the body flags.
    #[must_use]
    pub fn with_flags(mut self, flags: CollisionFlags) -> Self {
        self.flags = flags;
        self
    }

    /// Set the root material id.
    #[must_use]
    pub fn with_material(mut self, material_id: MaterialId) -> Self {
        self.material_id = material_id;
        self
    }

    /// Set the body quality id.
    #[must_use]
    pub fn with_quality(mut self, quality_id: BodyQualityId) -> Self {
        self.quality_id = quality_id;
        self
    }

    /// Set the collision filter info.
    #[must_use]
    pub fn with_collision_filter_info(mut self, info: u32) -> Self {
        self.collision_filter_info = info;
        self
    }

    /// Set the user data.
    #[must_use]
    pub fn with_user_data(mut self, user_data: u64) -> Self {
        self.user_data = user_data;
        self
    }

    /// Set the body name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// The initial transform implied by `position`/`orientation`.
    pub fn transform(&self) -> Transform {
        Transform::from_quaternion(self.orientation, self.position)
    }
}

/// Position-only transform helper. `Transform` already carries translation,
/// so this keeps `BodyCinfo` code explicit at call sites.
impl From<&BodyCinfo> for Transform {
    fn from(cinfo: &BodyCinfo) -> Self {
        cinfo.transform()
    }
}

/// A rigid body (`hknpBody`).
///
/// Bodies are allocated/owned by the body manager; the `id`/`index` fields are
/// assigned at allocation and remain stable for the body's lifetime.
#[derive(Clone, Debug)]
pub struct Body {
    /// Public body id assigned by [`crate::BodyManager`].
    pub id: BodyId,
    /// Internal slot index assigned by [`crate::BodyManager`].
    pub index: BodyIndex,
    /// Cached body transform.
    pub transform: Transform,
    /// Collision shape (shared).
    pub shape: Arc<dyn Shape>,
    /// Motion identifier (`PRESET_STATIC` for static bodies).
    pub motion_id: MotionId,
    /// Body state flags (including `IS_STATIC`/`IS_DYNAMIC`/`IS_KEYFRAMED`/`IS_ACTIVE`).
    pub flags: CollisionFlags,
    /// Root material id.
    pub material_id: MaterialId,
    /// Body quality id.
    pub quality_id: BodyQualityId,
    /// Collision filter info.
    pub collision_filter_info: u32,
    /// User data.
    pub user_data: u64,
    /// Maximum penetration recovery speed.
    pub max_depenetration_speed: f32,
    /// Extra collision look-ahead distance.
    pub collision_look_ahead_distance: f32,
    /// Optional name.
    pub name: Option<String>,
    /// Cached world-space AABB (`hknpBody::m_aabb`).
    pub aabb: Aabb,
}

impl Body {
    /// Construct a body without ids (assigned by the manager on allocation).
    pub fn new(shape: Arc<dyn Shape>, transform: Transform, motion_id: MotionId) -> Self {
        let aabb = shape.calc_aabb(&transform);
        Self {
            id: BodyId::INVALID,
            index: BodyIndex::INVALID,
            transform,
            shape,
            motion_id,
            flags: CollisionFlags::NONE,
            material_id: MaterialId::PRESET_DEFAULT,
            quality_id: BodyQualityId::INVALID,
            collision_filter_info: 0,
            user_data: 0,
            max_depenetration_speed: DEFAULT_MAX_DEPENETRATION_SPEED,
            collision_look_ahead_distance: 0.0,
            name: None,
            aabb,
        }
    }

    /// Compute the world-space AABB of the shape at the current transform
    /// (`hknpBody::calcShapeAabb`).
    pub fn calc_shape_aabb(&self) -> Aabb {
        self.shape.calc_aabb(&self.transform)
    }

    /// Recompute and cache [`Body::aabb`].
    pub fn refresh_aabb(&mut self) {
        self.aabb = self.calc_shape_aabb();
    }

    /// Set the transform and refresh the cached AABB.
    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
        self.refresh_aabb();
    }

    /// Replace the shape and refresh the cached AABB.
    pub fn set_shape(&mut self, shape: Arc<dyn Shape>) {
        self.shape = shape;
        self.refresh_aabb();
    }

    /// Whether this body has the `IS_STATIC` flag.
    pub fn is_static(&self) -> bool {
        self.flags.contains(CollisionFlags::IS_STATIC)
    }

    /// Whether this body has the `IS_DYNAMIC` flag.
    pub fn is_dynamic(&self) -> bool {
        self.flags.contains(CollisionFlags::IS_DYNAMIC)
    }

    /// Whether this body has the `IS_KEYFRAMED` flag.
    pub fn is_keyframed(&self) -> bool {
        self.flags.contains(CollisionFlags::IS_KEYFRAMED)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;
    use hknp_shape::SphereShape;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn cinfo_defaults_match_sdk() {
        let shape = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        let cinfo = BodyCinfo::new(shape);
        assert_eq!(cinfo.motion_type, MotionType::Static);
        assert_eq!(cinfo.material_id, MaterialId::PRESET_DEFAULT);
        assert_eq!(cinfo.quality_id, BodyQualityId::INVALID);
        assert_eq!(cinfo.mass, -1.0);
        assert_eq!(
            cinfo.max_depenetration_speed,
            DEFAULT_MAX_DEPENETRATION_SPEED
        );
        assert_eq!(cinfo.user_data, 0);
        assert_eq!(cinfo.collision_filter_info, 0);
    }

    #[test]
    fn cinfo_builder_sets_transform() {
        let shape = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        let cinfo = BodyCinfo::new(shape)
            .with_motion_type(MotionType::Dynamic)
            .with_position(Vector4::from_xyz(1.0, 2.0, 3.0))
            .with_orientation(Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2))
            .with_mass(2.0)
            .with_user_data(7)
            .with_name("body");
        let transform = cinfo.transform();
        assert!(
            transform
                .translation
                .all_equal3(Vector4::from_xyz(1.0, 2.0, 3.0), 1e-6)
        );
        assert_eq!(cinfo.name.as_deref(), Some("body"));
        assert_eq!(cinfo.user_data, 7);
        assert_eq!(cinfo.mass, 2.0);
    }

    #[test]
    fn body_transform_updates_aabb() {
        let shape = Arc::new(SphereShape::new(Vector4::ZERO, 2.0)) as Arc<dyn Shape>;
        let mut body = Body::new(
            Arc::clone(&shape),
            Transform::IDENTITY,
            MotionId::PRESET_STATIC,
        );
        assert!(body.aabb.min.all_equal3(Vector4::splat(-2.0), 1e-6));
        body.set_transform(Transform::from_quaternion(
            Quaternion::IDENTITY,
            Vector4::from_xyz(10.0, 0.0, 0.0),
        ));
        assert!(
            body.aabb
                .min
                .all_equal3(Vector4::from_xyz(8.0, -2.0, -2.0), 1e-6)
        );
        assert!(
            body.aabb
                .max
                .all_equal3(Vector4::from_xyz(12.0, 2.0, 2.0), 1e-6)
        );
    }

    #[test]
    fn flags_and_motion_helpers() {
        let shape = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        let mut body = Body::new(shape, Transform::IDENTITY, MotionId::PRESET_STATIC);
        body.flags = CollisionFlags::IS_DYNAMIC | CollisionFlags::IS_ACTIVE;
        assert!(body.is_dynamic());
        assert!(!body.is_static());
        assert!(!body.is_keyframed());
        body.flags = CollisionFlags::IS_KEYFRAMED;
        assert!(body.is_keyframed());
    }
}
