//! Compound shape (`hknpCompoundShape`).
//!
//! Simplified port of `Collide/Shape/Composite/Compound/hknpCompoundShape.h`.
//! The C++ compound stores child instances with shapes and transforms plus a
//! bounding-volume hierarchy; this phase ports the scene-graph part (children,
//! transforms, AABB/radius aggregation). The BVH that accelerates leaf queries
//! is part of `hknp-collide` (Phase 7).
//!
//! Children are `Arc<dyn Shape>`, so one immutable shape can be shared between
//! many compound instances/bodies (the Rust equivalent of the SDK's `hkRefPtr`
//! sharing).

use core::any::Any;
use std::sync::Arc;

use hk_math::{Aabb, Transform};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::{Shape, aabb_union};

/// One child of a [`CompoundShape`] (`hknpShapeInstance`).
#[derive(Clone)]
pub struct ShapeInstance {
    /// Child placement in the compound's local space.
    pub transform: Transform,
    /// Shared child shape.
    pub shape: Arc<dyn Shape>,
}

impl ShapeInstance {
    /// Construct a child instance.
    #[inline]
    pub fn new(transform: Transform, shape: Arc<dyn Shape>) -> Self {
        Self { transform, shape }
    }

    /// World (compound-local) AABB of the child.
    #[inline]
    pub fn calc_aabb(&self) -> Aabb {
        self.shape.calc_aabb(&self.transform)
    }
}

impl core::fmt::Debug for ShapeInstance {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ShapeInstance")
            .field("transform", &self.transform)
            .field("shape_type", &self.shape.shape_type())
            .field("shape", &self.shape)
            .finish()
    }
}

/// Compound shape (`hknpCompoundShape`).
#[derive(Clone, Debug, Default)]
pub struct CompoundShape {
    children: Vec<ShapeInstance>,
}

impl CompoundShape {
    /// Construct from children.
    #[inline]
    pub fn new(children: Vec<ShapeInstance>) -> Self {
        Self { children }
    }

    /// Empty compound with reserved capacity.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            children: Vec::with_capacity(capacity),
        }
    }

    /// Append a child instance.
    #[inline]
    pub fn add_child(&mut self, child: ShapeInstance) {
        self.children.push(child);
    }

    /// Child instances.
    #[inline]
    pub fn children(&self) -> &[ShapeInstance] {
        &self.children
    }

    /// Number of children.
    #[inline]
    pub fn len(&self) -> usize {
        self.children.len()
    }

    /// Whether the compound has no children.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.children.is_empty()
    }
}

impl Shape for CompoundShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Compound
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Composite
    }

    fn flags(&self) -> ShapeFlags {
        ShapeFlags::IS_COMPOSITE_SHAPE
    }

    fn aabb(&self) -> Aabb {
        if self.children.is_empty() {
            return Aabb::EMPTY;
        }
        let mut result = Aabb::EMPTY;
        for child in &self.children {
            result = aabb_union(&result, &child.calc_aabb());
        }
        result
    }

    fn bounding_radius(&self) -> f32 {
        self.children.iter().fold(0.0f32, |radius, child| {
            radius.max(child.shape.bounding_radius() + child.transform.translation.length3())
        })
    }

    fn volume(&self) -> Option<f32> {
        if self.children.is_empty() {
            return Some(0.0);
        }
        let mut total = 0.0f32;
        for child in &self.children {
            total += child.shape.volume()?;
        }
        Some(total)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BoxShape, SphereShape};
    use core::f32::consts::FRAC_PI_2;
    use hk_math::{Quaternion, Vector4};

    #[test]
    fn empty_compound_is_empty() {
        let compound = CompoundShape::new(Vec::new());
        assert!(compound.is_empty());
        assert!(compound.aabb().is_empty());
        assert_eq!(compound.volume(), Some(0.0));
        assert_eq!(compound.bounding_radius(), 0.0);
    }

    #[test]
    fn aabb_is_union_of_transformed_children() {
        let sphere = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        let box_shape =
            Arc::new(BoxShape::with_convex_radius(Vector4::splat(1.0), 0.0)) as Arc<dyn Shape>;

        let compound = CompoundShape::new(vec![
            ShapeInstance::new(Transform::IDENTITY, Arc::clone(&sphere)),
            ShapeInstance::new(
                Transform::new(hk_math::Matrix3::IDENTITY, Vector4::from_xyz(5.0, 0.0, 0.0)),
                Arc::clone(&box_shape),
            ),
        ]);

        let aabb = compound.aabb();
        assert!(
            aabb.min
                .all_equal3(Vector4::from_xyz(-1.0, -1.0, -1.0), 1e-6)
        );
        assert!(aabb.max.all_equal3(Vector4::from_xyz(6.0, 1.0, 1.0), 1e-6));
        assert_eq!(compound.len(), 2);
    }

    #[test]
    fn volume_sums_children() {
        let sphere = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        let box_shape =
            Arc::new(BoxShape::with_convex_radius(Vector4::splat(1.0), 0.0)) as Arc<dyn Shape>;
        let compound = CompoundShape::new(vec![
            ShapeInstance::new(Transform::IDENTITY, sphere),
            ShapeInstance::new(Transform::IDENTITY, box_shape),
        ]);
        let expected = 4.0 / 3.0 * std::f32::consts::PI + 8.0;
        assert!((compound.volume().unwrap() - expected).abs() < 1e-5);
    }

    #[test]
    fn volume_is_none_when_a_child_has_no_volume() {
        let mesh = Arc::new(crate::MeshShape::new(
            vec![
                Vector4::ZERO,
                Vector4::from_xyz(1.0, 0.0, 0.0),
                Vector4::from_xyz(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        )) as Arc<dyn Shape>;
        let compound = CompoundShape::new(vec![ShapeInstance::new(Transform::IDENTITY, mesh)]);
        assert_eq!(compound.volume(), None);
    }

    #[test]
    fn rotated_child_uses_world_aabb() {
        let box_shape = Arc::new(BoxShape::with_convex_radius(
            Vector4::from_xyz(2.0, 1.0, 1.0),
            0.0,
        )) as Arc<dyn Shape>;
        let transform = Transform::from_quaternion(
            Quaternion::from_axis_angle(Vector4::UNIT_Z, FRAC_PI_2),
            Vector4::from_xyz(10.0, 0.0, 0.0),
        );
        let compound = CompoundShape::new(vec![ShapeInstance::new(transform, box_shape)]);
        let aabb = compound.aabb();
        // 90° rotation swaps the 2 and 1 extents: x in [9,11], y in [-2,2].
        assert!((aabb.min.x - 9.0).abs() < 1e-5);
        assert!((aabb.max.x - 11.0).abs() < 1e-5);
        assert!((aabb.min.y - -2.0).abs() < 1e-5);
        assert!((aabb.max.y - 2.0).abs() < 1e-5);
    }
}
