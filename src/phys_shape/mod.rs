//! `phys-shape` — shape hierarchy and shape interfaces.
//!
//! Rust port of `original/Physics/Source/Physics/Physics/Collide/Shape/`
//! (migration Phase 5, README §4).
//!
//! The C++ hierarchy has a base `hknpShape` with a `hknpShapeFunctions` virtual
//! table for collision dispatch. This port uses a single [`Shape`]
//! trait (`Send + Sync`) and concrete structs per shape:
//!
//! | Rust type | Original C++ | Kind |
//! |---|---|---|
//! | [`SphereShape`] | `hknpSphereShape` | convex |
//! | [`BoxShape`] | `hknpBoxShape` | convex (axis-aligned local frame; OBB transform is carried by the body) |
//! | [`CapsuleShape`] | `hknpCapsuleShape` | convex |
//! | [`CylinderShape`] | `hknpCylinderShape` | convex |
//! | [`TriangleShape`] | `hknpTriangleShape` | convex (triangle; quad form deferred) |
//! | [`ConvexHullShape`] | `hknpConvexHull` / `hknpConvexPolytopeShape` | convex |
//! | [`CompoundShape`] | `hknpCompoundShape` | composite |
//! | [`MeshShape`] | `hknpMeshShape` | composite, with a simple BVH |
//! | [`HeightFieldShape`] | `hknpHeightFieldShape` | composite height grid |
//!
//! # Shape interface
//!
//! [`Shape::aabb`] is the local-space AABB (including the convex radius), [`Shape::calc_aabb`]
//! maps it through a [`proton::math::Transform`], [`Shape::support`] is the local support
//! point (convex shapes only) and [`Shape::volume`] is the analytic volume where
//! available (`None` for composites/meshes). [`Shape::mass_properties`] builds full
//! volume/COM/inertia data from a [`MassConfig`], using analytic formulas for
//! solid primitives and deterministic aggregation/approximation for composites.
//!
//! The default [`Shape::dispatch_type`]/[`Shape::flags`] implementations mirror
//! the C++ `hknpShape::getDispatchType()`/`FlagsEnum` mapping: convex shapes
//! dispatch as `CONVEX`, meshes/compounds/height fields as `COMPOSITE`.
//!
//! # Convex radius
//!
//! `HKNP_SHAPE_DEFAULT_CONVEX_RADIUS` is `0.015f`. Boxes/capsules/cylinders/
//! triangles/convex hulls carry a convex radius that inflates their AABB and
//! support point, exactly like the SDK's "surface plus rounding" model. Volumes
//! are reported for the nominal solid (without the rounding), matching how the
//! SDK separates geometry from convex radius.
//!
//! # Not ported in this phase
//!
//! `hknpCompressedMeshShape`, `hknpExternMeshShape`, `hknpBreakableCompoundShape`,
//! `hknpLodShape`, decorator/masked shapes, shape tag codecs, shape key masks/
//! iterators, attached `hknpShapeMassProperties` and the full `hknpShapeFunctions`
//! surface (queries/GSK/face info). Mass-property building is available through
//! [`Shape::mass_properties`], with a portable compressed representation and
//! deterministic principal-axis solver in [`mass`]; exact SDK packed-byte
//! compatibility and attached-property caches remain unverified.

use core::any::Any;

use glam::{Mat3, Quat, Vec4};

use crate::base_math::{Aabb, Matrix3, Quaternion, Transform, Vector4, Vec4Ext};
use crate::phys_types::{CollisionDispatchType, ShapeFlags, ShapeType};

pub mod box_shape;
pub mod capsule;
pub mod compound;
pub mod convex_hull;
pub mod cylinder;
pub mod height_field;
pub mod mass;
pub mod mesh;
pub mod sphere;
pub mod triangle;

pub use box_shape::BoxShape;
pub use capsule::CapsuleShape;
pub use compound::{CompoundShape, ShapeInstance};
pub use convex_hull::{ConvexHullBuildError, ConvexHullShape};
pub use cylinder::CylinderShape;
pub use height_field::HeightFieldShape;
pub use mass::{
    CompressedMassProperties, MassConfig, MassDistribution, MassError, MassProperties,
    MassPropertiesTree, MassQuality, PackedUnitVector4, PackedVector3, ShapeCacheKey,
    ShapeMassCache, diagonalize_symmetric,
};
pub use mesh::{BvhNode, MeshShape};
pub use sphere::SphereShape;
pub use triangle::TriangleShape;

/// `HKNP_SHAPE_DEFAULT_CONVEX_RADIUS` (`hknpBoxShape.h`).
pub const DEFAULT_CONVEX_RADIUS: f32 = 0.015;

/// Common interface of all shapes (`hknpShape`).
pub trait Shape: Send + Sync + core::fmt::Debug {
    /// Concrete shape type (`hknpShape::getType()`).
    fn shape_type(&self) -> ShapeType;

    /// Narrow-phase dispatch type (`hknpShape::getDispatchType()`).
    fn dispatch_type(&self) -> CollisionDispatchType {
        match self.shape_type() {
            ShapeType::Convex
            | ShapeType::Sphere
            | ShapeType::Capsule
            | ShapeType::Cylinder
            | ShapeType::Triangle
            | ShapeType::Box => CollisionDispatchType::Convex,
            ShapeType::Composite
            | ShapeType::Mesh
            | ShapeType::Compound
            | ShapeType::HeightField => CollisionDispatchType::Composite,
            _ => CollisionDispatchType::None,
        }
    }

    /// Internal shape flags (`hknpShape::getFlags()`).
    fn flags(&self) -> ShapeFlags {
        match self.shape_type() {
            ShapeType::Convex
            | ShapeType::Sphere
            | ShapeType::Capsule
            | ShapeType::Cylinder
            | ShapeType::Triangle
            | ShapeType::Box => ShapeFlags::IS_CONVEX_SHAPE,
            ShapeType::Mesh | ShapeType::Compound => ShapeFlags::IS_COMPOSITE_SHAPE,
            ShapeType::HeightField => {
                ShapeFlags::IS_COMPOSITE_SHAPE | ShapeFlags::IS_HEIGHT_FIELD_SHAPE
            }
            _ => ShapeFlags::NONE,
        }
    }

    /// Local-space AABB, including the convex radius.
    fn aabb(&self) -> Aabb;

    /// Bounding radius around the shape origin (`hknpShape::calcBoundingRadius`).
    fn bounding_radius(&self) -> f32;

    /// World-space AABB for a shape placed by `transform`
    /// (`hknpShape::calcAabb(transform, aabbOut)`).
    fn calc_aabb(&self, transform: &Transform) -> Aabb {
        transform_aabb(transform, &self.aabb())
    }

    /// Analytic volume, when the shape has one (`None` for meshes/composites).
    fn volume(&self) -> Option<f32>;

    /// Build full mass properties from `config` (`hknpShape::buildMassProperties`).
    ///
    /// The default implementation uses the local AABB, matching the SDK's
    /// null-function fallback. Primitives override this with analytic tensors.
    fn mass_properties(&self, config: MassConfig) -> Result<MassProperties, MassError> {
        mass::mass_from_aabb(&self.aabb(), config)
    }

    /// Local support point in `direction` (convex shapes only).
    fn support(&self, direction: Vec4) -> Option<Vec4> {
        let _ = direction;
        None
    }

    /// Whether this shape is convex (`hknpShape::isConvexShape`).
    fn is_convex(&self) -> bool {
        self.flags().contains(ShapeFlags::IS_CONVEX_SHAPE)
    }

    /// Downcast hook (`hknpShape::asConvexShape` style casts).
    fn as_any(&self) -> &dyn Any;
}

/// Transform a local AABB by a rigid transform, returning the world AABB
/// (`hknpShape::calcAabb` for an axis-aligned box).
///
/// The result is exact for a rotated box: the extents are the absolute values of
/// the rotation rows dotted with the half extents.
pub fn transform_aabb(transform: &Transform, local: &Aabb) -> Aabb {
    let half = local.half_extents();
    let center = local.center();
    let center = transform.transform_point(Vector4::from_xyz(center.x, center.y, center.z));

    let rotation = &transform.rotation;
    let extent_x = rotation.element(0, 0).abs() * half.x
        + rotation.element(0, 1).abs() * half.y
        + rotation.element(0, 2).abs() * half.z;
    let extent_y = rotation.element(1, 0).abs() * half.x
        + rotation.element(1, 1).abs() * half.y
        + rotation.element(1, 2).abs() * half.z;
    let extent_z = rotation.element(2, 0).abs() * half.x
        + rotation.element(2, 1).abs() * half.y
        + rotation.element(2, 2).abs() * half.z;

    Aabb::new(
        Vector4::from_xyz(
            center.x - extent_x,
            center.y - extent_y,
            center.z - extent_z,
        ),
        Vector4::from_xyz(
            center.x + extent_x,
            center.y + extent_y,
            center.z + extent_z,
        ),
    )
}

/// Bounding radius of an AABB around the origin.
pub(crate) fn aabb_bounding_radius(aabb: &Aabb) -> f32 {
    let min = aabb.min;
    let max = aabb.max;
    let x = min.x.abs().max(max.x.abs());
    let y = min.y.abs().max(max.y.abs());
    let z = min.z.abs().max(max.z.abs());
    (x * x + y * y + z * z).sqrt()
}

/// Unit direction, or `None` for a zero direction (`hkVector4f::normalize<3>`
/// with an explicit zero-length check; same scalar order as legacy
/// `unit_direction`, `mod.rs:210-221`).
pub(crate) fn unit_direction(direction: Vec4) -> Option<Vec4> {
    let length = direction.dot3(direction).sqrt();
    if length > 0.0 {
        Some(Vec4::new(
            direction.x / length,
            direction.y / length,
            direction.z / length,
            0.0,
        ))
    } else {
        None
    }
}

/// Union of two AABBs (empty boxes are handled by `Aabb::include_aabb`).
pub(crate) fn aabb_union(a: &Aabb, b: &Aabb) -> Aabb {
    let mut result = *a;
    result.include_aabb(b);
    result
}

// 过渡期 legacy↔glam 逐位复制转换（`Shape` 构造签名在提交 ② 换 glam，
// phys_body/phys_world 的对应迁移在步骤 3/5；全部转换随后删除）。

/// legacy `Vector4` → glam `Vec4`.
#[inline]
pub(crate) fn to_vec4(value: Vector4) -> Vec4 {
    Vec4::new(value.x, value.y, value.z, value.w)
}

/// glam `Vec4` → legacy `Vector4`.
#[inline]
pub(crate) fn to_vector4(value: Vec4) -> Vector4 {
    Vector4::new(value.x, value.y, value.z, value.w)
}

/// legacy `Matrix3` → glam `Mat3`（列的 `w` 为填充位，恒为零）。
#[inline]
pub(crate) fn to_mat3(value: &Matrix3) -> Mat3 {
    Mat3::from_cols(
        glam::Vec3::new(value.col0.x, value.col0.y, value.col0.z),
        glam::Vec3::new(value.col1.x, value.col1.y, value.col1.z),
        glam::Vec3::new(value.col2.x, value.col2.y, value.col2.z),
    )
}

/// glam `Mat3` → legacy `Matrix3`。
// 过渡期保留：phys_extensions（步骤 6）迁移后整删。
#[inline]
#[allow(dead_code)]
pub(crate) fn to_matrix3(value: Mat3) -> Matrix3 {
    Matrix3::from_cols(
        Vector4::from_xyz(value.x_axis.x, value.x_axis.y, value.x_axis.z),
        Vector4::from_xyz(value.y_axis.x, value.y_axis.y, value.y_axis.z),
        Vector4::from_xyz(value.z_axis.x, value.z_axis.y, value.z_axis.z),
    )
}

/// glam `Quat` → legacy `Quaternion`。
#[inline]
pub(crate) fn to_quaternion(value: Quat) -> Quaternion {
    Quaternion::new(value.x, value.y, value.z, value.w)
}

/// legacy `Quaternion` → glam `Quat`。
#[inline]
pub(crate) fn to_quat(value: Quaternion) -> Quat {
    Quat::from_xyzw(value.x, value.y, value.z, value.w)
}

/// 外积矩阵（legacy `mass.rs::matrix_outer(a, b)`：列 `i` = `a * b[i]`，
/// 分量逐位一致；`a` 的 `w` 为填充位恒零）。
#[inline]
pub(crate) fn outer_product(a: Vec4, b: Vec4) -> Mat3 {
    Mat3::from_cols(
        glam::Vec3::new(a.x * b.x, a.y * b.x, a.z * b.x),
        glam::Vec3::new(a.x * b.y, a.y * b.y, a.z * b.y),
        glam::Vec3::new(a.x * b.z, a.y * b.z, a.z * b.z),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// glam↔legacy 四元数转换辅助的位级往返（glam_parity 门模式）。
    #[test]
    fn quat_conversion_helpers_are_bit_exact() {
        let glam_value = Quat::from_xyzw(0.5, -0.5, 1.0e-8, 2.0);
        let legacy_value = Quaternion::new(0.5, -0.5, 1.0e-8, 2.0);
        let converted = to_quat(legacy_value);
        assert_eq!(converted.x.to_bits(), glam_value.x.to_bits());
        assert_eq!(converted.y.to_bits(), glam_value.y.to_bits());
        assert_eq!(converted.z.to_bits(), glam_value.z.to_bits());
        assert_eq!(converted.w.to_bits(), glam_value.w.to_bits());
        let round = to_quaternion(glam_value);
        for (a, b) in [
            (round.x, legacy_value.x),
            (round.y, legacy_value.y),
            (round.z, legacy_value.z),
            (round.w, legacy_value.w),
        ] {
            assert_eq!(a.to_bits(), b.to_bits());
        }

        let neg_zero = Quat::from_xyzw(-0.0, f32::NAN, f32::INFINITY, f32::from_bits(1));
        let round_trip = to_quat(to_quaternion(neg_zero));
        for (a, b) in [
            (round_trip.x, neg_zero.x),
            (round_trip.y, neg_zero.y),
            (round_trip.z, neg_zero.z),
            (round_trip.w, neg_zero.w),
        ] {
            assert_eq!(a.to_bits(), b.to_bits());
        }
    }

    #[test]
    fn dispatch_and_flags_defaults() {
        let sphere = SphereShape::new(Vec4::ZERO, 1.0);
        assert_eq!(sphere.dispatch_type(), CollisionDispatchType::Convex);
        assert!(sphere.is_convex());
        assert!(sphere.flags().contains(ShapeFlags::IS_CONVEX_SHAPE));

        let compound = CompoundShape::new(Vec::new());
        assert_eq!(compound.dispatch_type(), CollisionDispatchType::Composite);
        assert!(!compound.is_convex());
        assert!(compound.flags().contains(ShapeFlags::IS_COMPOSITE_SHAPE));

        let heightfield = HeightFieldShape::new(2, 2, Vec4::ONE, vec![0.0; 4]).unwrap();
        assert!(
            heightfield
                .flags()
                .contains(ShapeFlags::IS_HEIGHT_FIELD_SHAPE)
        );
    }

    #[test]
    fn transform_aabb_matches_rotated_box_extents() {
        use crate::base_math::Quaternion;
        use core::f32::consts::FRAC_PI_2;

        let local = Aabb::new(
            Vector4::from_xyz(-1.0, -2.0, -3.0),
            Vector4::from_xyz(1.0, 2.0, 3.0),
        );
        let rotation = Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2);
        let transform = Transform::from_quaternion(rotation, Vector4::from_xyz(5.0, 0.0, 0.0));
        let world = transform_aabb(&transform, &local);

        // 90° about Z swaps the x/y extents (2, 1) and translates by +5.
        assert!((world.min.x - 3.0).abs() < 1e-5);
        assert!((world.max.x - 7.0).abs() < 1e-5);
        assert!((world.min.y - -1.0).abs() < 1e-5);
        assert!((world.max.y - 1.0).abs() < 1e-5);
        assert!((world.min.z - -3.0).abs() < 1e-5);
        assert!((world.max.z - 3.0).abs() < 1e-5);
    }

    #[test]
    fn aabb_bounding_radius_uses_far_corner() {
        let aabb = Aabb::new(
            Vector4::from_xyz(-1.0, -1.0, -1.0),
            Vector4::from_xyz(2.0, 2.0, 2.0),
        );
        let radius = aabb_bounding_radius(&aabb);
        assert!((radius - (12.0f32).sqrt()).abs() < 1e-6);
    }
}
