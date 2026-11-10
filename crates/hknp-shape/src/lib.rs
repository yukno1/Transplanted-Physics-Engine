//! `hknp-shape` — shape hierarchy and shape interfaces.
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
//! maps it through a [`hk_math::Transform`], [`Shape::support`] is the local support
//! point (convex shapes only) and [`Shape::volume`] is the analytic volume where
//! available (`None` for composites/meshes).
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
//! iterators, `hknpShapeMassProperties` and the full `hknpShapeFunctions` surface
//! (queries/GSK/face info). Those follow with `hknp-collide` (Phase 7).

use core::any::Any;

use hk_math::{Aabb, Transform, Vector4};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

pub mod box_shape;
pub mod capsule;
pub mod compound;
pub mod convex_hull;
pub mod cylinder;
pub mod height_field;
pub mod mesh;
pub mod sphere;
pub mod triangle;

pub use box_shape::BoxShape;
pub use capsule::CapsuleShape;
pub use compound::{CompoundShape, ShapeInstance};
pub use convex_hull::ConvexHullShape;
pub use cylinder::CylinderShape;
pub use height_field::HeightFieldShape;
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

    /// Local support point in `direction` (convex shapes only).
    fn support(&self, direction: Vector4) -> Option<Vector4> {
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

/// Unit direction, or `None` for a zero direction.
pub(crate) fn unit_direction(direction: Vector4) -> Option<Vector4> {
    let length = direction.length3();
    if length > 0.0 {
        Some(Vector4::from_xyz(
            direction.x / length,
            direction.y / length,
            direction.z / length,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dispatch_and_flags_defaults() {
        let sphere = SphereShape::new(Vector4::ZERO, 1.0);
        assert_eq!(sphere.dispatch_type(), CollisionDispatchType::Convex);
        assert!(sphere.is_convex());
        assert!(sphere.flags().contains(ShapeFlags::IS_CONVEX_SHAPE));

        let compound = CompoundShape::new(Vec::new());
        assert_eq!(compound.dispatch_type(), CollisionDispatchType::Composite);
        assert!(!compound.is_convex());
        assert!(compound.flags().contains(ShapeFlags::IS_COMPOSITE_SHAPE));

        let heightfield = HeightFieldShape::new(2, 2, Vector4::ONE, vec![0.0; 4]).unwrap();
        assert!(
            heightfield
                .flags()
                .contains(ShapeFlags::IS_HEIGHT_FIELD_SHAPE)
        );
    }

    #[test]
    fn transform_aabb_matches_rotated_box_extents() {
        use core::f32::consts::FRAC_PI_2;
        use hk_math::Quaternion;

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
