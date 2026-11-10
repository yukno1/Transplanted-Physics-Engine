//! Height field (`hknpHeightFieldShape`).
//!
//! Simplified port of `Collide/Shape/Composite/HeightField/hknpHeightFieldShape.h`.
//! The SDK height field is a grid of heights on the XZ plane (scaled by
//! `m_floatToIntScale`/`m_intToFloatScale`) with a bounding-volume structure per
//! cell; this phase keeps the grid samples, the float scales and the derived
//! AABB. The per-cell BVH and SDF queries live in `hknp-collide` (Phase 7).

use core::any::Any;

use hk_math::{Aabb, Vector4};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::{Shape, aabb_bounding_radius};

/// Errors from [`HeightFieldShape::new`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeightFieldError {
    /// A grid dimension is zero.
    InvalidDimensions,
    /// `heights.len() != size_x * size_z`.
    LengthMismatch {
        /// Expected sample count.
        expected: usize,
        /// Provided sample count.
        actual: usize,
    },
    /// The scale has a non-positive or non-finite component.
    InvalidScale,
}

impl core::fmt::Display for HeightFieldError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidDimensions => f.write_str("height field dimensions must be non-zero"),
            Self::LengthMismatch { expected, actual } => {
                write!(f, "height field needs {expected} samples, got {actual}")
            }
            Self::InvalidScale => f.write_str("height field scale must be positive and finite"),
        }
    }
}

impl std::error::Error for HeightFieldError {}

/// Height field grid (`hknpHeightFieldShape`).
#[derive(Clone, Debug, PartialEq)]
pub struct HeightFieldShape {
    int_size_x: usize,
    int_size_z: usize,
    scale: Vector4,
    heights: Vec<f32>,
    aabb: Aabb,
}

impl HeightFieldShape {
    /// Build a height field from integer dimensions, a scale and the samples.
    ///
    /// `heights` are stored row-major along X (index = `z * size_x + x`). The
    /// shape's XZ plane occupies `[0, (size_x-1)*scale.x] × [0, (size_z-1)*scale.z]`;
    /// sample heights are multiplied by `scale.y`.
    pub fn new(
        int_size_x: usize,
        int_size_z: usize,
        scale: Vector4,
        heights: Vec<f32>,
    ) -> Result<Self, HeightFieldError> {
        if int_size_x == 0 || int_size_z == 0 {
            return Err(HeightFieldError::InvalidDimensions);
        }
        if !(scale.x > 0.0 && scale.y > 0.0 && scale.z > 0.0)
            || !scale.x.is_finite()
            || !scale.y.is_finite()
            || !scale.z.is_finite()
        {
            return Err(HeightFieldError::InvalidScale);
        }
        let expected = int_size_x * int_size_z;
        if heights.len() != expected {
            return Err(HeightFieldError::LengthMismatch {
                expected,
                actual: heights.len(),
            });
        }

        let min_height = heights.iter().copied().fold(f32::INFINITY, f32::min) * scale.y;
        let max_height = heights.iter().copied().fold(f32::NEG_INFINITY, f32::max) * scale.y;
        let aabb = Aabb::new(
            Vector4::from_xyz(0.0, min_height, 0.0),
            Vector4::from_xyz(
                (int_size_x - 1) as f32 * scale.x,
                max_height,
                (int_size_z - 1) as f32 * scale.z,
            ),
        );

        Ok(Self {
            int_size_x,
            int_size_z,
            scale,
            heights,
            aabb,
        })
    }

    /// Integer size along X (`m_intSizeX`).
    #[inline]
    pub fn int_size_x(&self) -> usize {
        self.int_size_x
    }

    /// Integer size along Z (`m_intSizeZ`).
    #[inline]
    pub fn int_size_z(&self) -> usize {
        self.int_size_z
    }

    /// XZ cell scale and Y height scale (`getShapeFromGeometryScale`).
    #[inline]
    pub fn scale(&self) -> Vector4 {
        self.scale
    }

    /// Height samples, row-major along X.
    #[inline]
    pub fn heights(&self) -> &[f32] {
        &self.heights
    }

    /// Sample at grid coordinates `(x, z)`.
    #[inline]
    pub fn height(&self, x: usize, z: usize) -> f32 {
        assert!(
            x < self.int_size_x && z < self.int_size_z,
            "height field sample out of range"
        );
        self.heights[z * self.int_size_x + x]
    }

    /// World-space vertex at grid coordinates `(x, z)`.
    #[inline]
    pub fn vertex(&self, x: usize, z: usize) -> Vector4 {
        Vector4::from_xyz(
            x as f32 * self.scale.x,
            self.height(x, z) * self.scale.y,
            z as f32 * self.scale.z,
        )
    }
}

impl Shape for HeightFieldShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::HeightField
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Composite
    }

    fn flags(&self) -> ShapeFlags {
        ShapeFlags::IS_COMPOSITE_SHAPE | ShapeFlags::IS_HEIGHT_FIELD_SHAPE
    }

    fn aabb(&self) -> Aabb {
        self.aabb
    }

    fn bounding_radius(&self) -> f32 {
        aabb_bounding_radius(&self.aabb)
    }

    fn volume(&self) -> Option<f32> {
        None
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_grid() -> Result<HeightFieldShape, HeightFieldError> {
        HeightFieldShape::new(
            3,
            3,
            Vector4::new(1.0, 1.0, 1.0, 1.0),
            vec![0.0, 1.0, 2.0, 3.0, 8.0, 5.0, 6.0, 7.0, 4.0],
        )
    }

    #[test]
    fn dimensions_and_samples() {
        let heightfield = sample_grid().unwrap();
        assert_eq!(heightfield.int_size_x(), 3);
        assert_eq!(heightfield.int_size_z(), 3);
        assert_eq!(heightfield.height(0, 0), 0.0);
        assert_eq!(heightfield.height(1, 1), 8.0);
        assert_eq!(heightfield.vertex(1, 1), Vector4::from_xyz(1.0, 8.0, 1.0));
        assert_eq!(heightfield.vertex(2, 0), Vector4::from_xyz(2.0, 2.0, 0.0));
    }

    #[test]
    fn aabb_reflects_extent_and_height_range() {
        let heightfield = sample_grid().unwrap();
        let aabb = heightfield.aabb();
        assert!(aabb.min.all_equal3(Vector4::from_xyz(0.0, 0.0, 0.0), 0.0));
        assert!(aabb.max.all_equal3(Vector4::from_xyz(2.0, 8.0, 2.0), 0.0));
    }

    #[test]
    fn scale_applies_to_spacing_and_heights() {
        let heightfield = HeightFieldShape::new(
            2,
            2,
            Vector4::new(2.0, 0.5, 3.0, 1.0),
            vec![0.0, 4.0, 8.0, -2.0],
        )
        .unwrap();
        assert_eq!(heightfield.vertex(1, 0), Vector4::from_xyz(2.0, 2.0, 0.0));
        assert_eq!(heightfield.vertex(0, 1), Vector4::from_xyz(0.0, 4.0, 3.0));
        let aabb = heightfield.aabb();
        assert_eq!(aabb.min.y, -1.0);
        assert_eq!(aabb.max.y, 4.0);
    }

    #[test]
    fn validation_errors() {
        assert_eq!(
            HeightFieldShape::new(0, 2, Vector4::ONE, Vec::new()),
            Err(HeightFieldError::InvalidDimensions)
        );
        assert_eq!(
            HeightFieldShape::new(2, 2, Vector4::ONE, vec![0.0; 3]),
            Err(HeightFieldError::LengthMismatch {
                expected: 4,
                actual: 3
            })
        );
        assert_eq!(
            HeightFieldShape::new(2, 2, Vector4::new(0.0, 1.0, 1.0, 1.0), vec![0.0; 4]),
            Err(HeightFieldError::InvalidScale)
        );
    }

    #[test]
    fn is_a_composite_height_field() {
        let heightfield = sample_grid().unwrap();
        assert_eq!(
            heightfield.dispatch_type(),
            CollisionDispatchType::Composite
        );
        assert!(
            heightfield
                .flags()
                .contains(ShapeFlags::IS_HEIGHT_FIELD_SHAPE)
        );
        assert_eq!(heightfield.volume(), None);
    }
}
