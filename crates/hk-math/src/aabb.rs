//! Axis-aligned bounding boxes.
//!
//! Port of `hkAabb` from
//! `original/Common/Source/Common/Base/Types/Geometry/Aabb/`.
//!
//! The C++ box stores `m_min` / `m_max` as `hkVector4` and only the first three
//! components are meaningful (`hkVector4ComparisonMask::MASK_XYZ`); `w` is carried
//! along by the vector operations. The empty box is `min = +MAX, max = -MAX`, the
//! full box is `min = -MAX, max = +MAX`.
//!
//! ```
//! use hk_math::{Aabb, Vector4};
//! let mut aabb = Aabb::empty();
//! aabb.include_point(Vector4::UNIT_X);
//! aabb.include_point(Vector4::UNIT_Y);
//! assert!(aabb.contains_point(Vector4::new(0.5, 0.5, 0.0, 0.0)));
//! assert!(!aabb.contains_point(Vector4::new(0.5, 1.5, 0.0, 0.0)));
//! ```

use crate::vector::Vector4;

/// Axis-aligned bounding box (`hkAabb`).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    /// Minimum corner.
    pub min: Vector4,
    /// Maximum corner.
    pub max: Vector4,
}

impl Default for Aabb {
    /// The C++ default constructor leaves storage uninitialized; an empty box is
    /// the only sensible Rust default.
    fn default() -> Self {
        Self::EMPTY
    }
}

impl Aabb {
    /// Empty (invalid) box: `min = +MAX`, `max = -MAX` (`hkAabb::setEmpty`).
    pub const EMPTY: Self = Self {
        min: Vector4::splat(f32::MAX),
        max: Vector4::splat(-f32::MAX),
    };

    /// Full box: `min = -MAX`, `max = +MAX` (`hkAabb::setFull`).
    pub const FULL: Self = Self {
        min: Vector4::splat(-f32::MAX),
        max: Vector4::splat(f32::MAX),
    };

    /// Build from min/max corners.
    #[inline]
    pub const fn new(min: Vector4, max: Vector4) -> Self {
        Self { min, max }
    }

    /// Empty box constructor.
    #[inline]
    pub const fn empty() -> Self {
        Self::EMPTY
    }

    /// Full box constructor.
    #[inline]
    pub const fn full() -> Self {
        Self::FULL
    }

    /// `hkAabb::setEmpty`.
    #[inline]
    pub fn set_empty(&mut self) {
        *self = Self::EMPTY;
    }

    /// `hkAabb::setFull`.
    #[inline]
    pub fn set_full(&mut self) {
        *self = Self::FULL;
    }

    /// `hkAabb::setFromPoint`.
    #[inline]
    pub fn from_point(point: Vector4) -> Self {
        Self::new(point, point)
    }

    /// `hkAabb::setFromLine`.
    #[inline]
    pub fn from_line(start: Vector4, end: Vector4) -> Self {
        Self::new(start.min(end), start.max(end))
    }

    /// `hkAabb::setFromCenterRadius(center, halfExtents)`.
    #[inline]
    pub fn from_center_half_extents(center: Vector4, half_extents: Vector4) -> Self {
        Self::new(center - half_extents, center + half_extents)
    }

    /// `hkAabb::setFromCenterRadius(center, radius)` (uniform radius).
    #[inline]
    pub fn from_center_radius(center: Vector4, radius: f32) -> Self {
        let r = Vector4::splat(radius);
        Self::new(center - r, center + r)
    }

    /// `hkAabb::includePoint`.
    #[inline]
    pub fn include_point(&mut self, point: Vector4) {
        self.min = self.min.min(point);
        self.max = self.max.max(point);
    }

    /// `hkAabb::includeAabb`.
    #[inline]
    pub fn include_aabb(&mut self, other: &Self) {
        self.min = self.min.min(other.min);
        self.max = self.max.max(other.max);
    }

    /// `hkAabb::expandBy` — grow the box by `amount` in all directions.
    #[inline]
    pub fn expand_by(&mut self, amount: f32) {
        let delta = Vector4::splat(amount);
        self.max += delta;
        self.min -= delta;
    }

    /// `hkAabb::translate`.
    #[inline]
    pub fn translate(&mut self, offset: Vector4) {
        self.min += offset;
        self.max += offset;
    }

    /// `hkAabb::getExtents` — `max - min`.
    #[inline]
    pub fn extents(&self) -> Vector4 {
        self.max - self.min
    }

    /// `hkAabb::getHalfExtents` — `(max - min) / 2`.
    #[inline]
    pub fn half_extents(&self) -> Vector4 {
        self.extents() / 2.0
    }

    /// `hkAabb::getCenter` — `(min + max) / 2`.
    #[inline]
    pub fn center(&self) -> Vector4 {
        (self.min + self.max) / 2.0
    }

    /// `hkAabb::getCenter2` — `min + max` (twice the center, avoids division).
    #[inline]
    pub fn center2(&self) -> Vector4 {
        self.min + self.max
    }

    /// `hkAabb::getVolume` — product of the xyz extents.
    #[inline]
    pub fn volume(&self) -> f32 {
        let e = self.extents();
        e.x * e.y * e.z
    }

    /// `hkAabb::overlaps` — zero-volume overlaps count as overlapping.
    #[inline]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min
            .less_equal(other.max)
            .and(other.min.less_equal(self.max))
            .all_xyz()
    }

    /// `hkAabb::disjoint`.
    #[inline]
    pub fn disjoint(&self, other: &Self) -> bool {
        !self.overlaps(other)
    }

    /// `hkAabb::contains` — `other` lies inside `self`.
    #[inline]
    pub fn contains(&self, other: &Self) -> bool {
        self.min
            .less_equal(other.min)
            .and(other.max.less_equal(self.max))
            .all_xyz()
    }

    /// `hkAabb::notContains`.
    #[inline]
    pub fn not_contains(&self, other: &Self) -> bool {
        other
            .min
            .less(self.min)
            .or(other.max.greater(self.max))
            .any_xyz()
    }

    /// `hkAabb::containsPoint`.
    #[inline]
    pub fn contains_point(&self, point: Vector4) -> bool {
        self.min
            .less_equal(point)
            .and(point.less_equal(self.max))
            .all_xyz()
    }

    /// `hkAabb::isEmpty` — `min > max` on any xyz component.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.min.greater(self.max).any_xyz()
    }

    /// `hkAabb::isValid` — `min <= max` on all xyz components.
    #[inline]
    pub fn is_valid(&self) -> bool {
        !self.is_empty()
    }

    /// `hkAabb::isEqual` — component-wise exact equality on xyz.
    #[inline]
    pub fn is_equal(&self, other: &Self) -> bool {
        self.min
            .equal(other.min)
            .and(self.max.equal(other.max))
            .all_xyz()
    }

    /// `hkAabb::getVertex` — corner `index` (bit 0 = x, bit 1 = y, bit 2 = z;
    /// a set bit selects the max corner).
    #[inline]
    pub fn vertex(&self, index: usize) -> Vector4 {
        assert!(index <= 7, "AABB vertex index out of range: {index}");
        let mut mask = crate::Vector4Mask::NONE;
        for (component, bit) in [(0usize, 1usize), (1, 2), (2, 4)] {
            if index & bit != 0 {
                mask = mask.or(crate::Vector4Mask::from_bits(1 << component));
            }
        }
        // The C++ implementation selects m_max for set bits and m_min otherwise.
        Vector4::select(mask, self.max, self.min)
    }

    /// `hkAabb::getSupportingVertex` — corner furthest along `direction`.
    #[inline]
    pub fn supporting_vertex(&self, direction: Vector4) -> Vector4 {
        Vector4::select(direction.less_zero(), self.min, self.max)
    }

    /// `hkAabb::signedDistanceToPoint` (gradient and value), xyz only.
    ///
    /// Returns `(gradient, distance)` where the gradient points from the box
    /// toward the point; inside the box the distance is the (negative) distance to
    /// the nearest face.
    pub fn signed_distance_to_point(&self, point: Vector4) -> (Vector4, f32) {
        let center = self.center();
        let half = self.half_extents();
        let delta = point - center;

        // Outside/inside test per component, like hkAabbUtil.
        let outside = |d: f32, h: f32| d.abs() - h;
        let x = outside(delta.x, half.x);
        let y = outside(delta.y, half.y);
        let z = outside(delta.z, half.z);
        let max = x.max(y).max(z);
        let gradient = Vector4::new(
            if x > 0.0 { delta.x.signum() } else { 0.0 },
            if y > 0.0 { delta.y.signum() } else { 0.0 },
            if z > 0.0 { delta.z.signum() } else { 0.0 },
            0.0,
        );
        (gradient, max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector4;

    #[test]
    fn empty_and_full() {
        let empty = Aabb::EMPTY;
        assert!(empty.is_empty());
        assert!(!empty.is_valid());
        assert!(!empty.overlaps(&Aabb::EMPTY));

        let full = Aabb::FULL;
        assert!(full.is_valid());
        assert!(!full.is_empty());
        assert!(full.contains_point(Vector4::from_xyz(100.0, -50.0, 7.0)));
        assert!(full.contains(&Aabb::from_point(Vector4::from_xyz(1.0, 2.0, 3.0))));
    }

    #[test]
    fn inclusion_and_overlap() {
        let mut a = Aabb::from_point(Vector4::from_xyz(1.0, 2.0, 3.0));
        a.include_point(Vector4::from_xyz(-1.0, 5.0, 0.0));
        assert_eq!(a.min, Vector4::from_xyz(-1.0, 2.0, 0.0));
        assert_eq!(a.max, Vector4::from_xyz(1.0, 5.0, 3.0));

        let b = Aabb::new(
            Vector4::from_xyz(0.0, 3.0, 0.0),
            Vector4::from_xyz(2.0, 4.0, 2.0),
        );
        assert!(a.overlaps(&b));
        assert!(!a.disjoint(&b));

        let c = Aabb::new(
            Vector4::from_xyz(5.0, 0.0, 0.0),
            Vector4::from_xyz(6.0, 1.0, 1.0),
        );
        assert!(!a.overlaps(&c));
        assert!(a.disjoint(&c));

        let inner = Aabb::new(
            Vector4::from_xyz(0.0, 3.0, 1.0),
            Vector4::from_xyz(0.5, 4.0, 2.0),
        );
        assert!(a.contains(&inner));
        assert!(!inner.contains(&a));
        assert!(inner.not_contains(&a));
    }

    #[test]
    fn point_containment_boundaries_are_inclusive() {
        let box_ = Aabb::new(
            Vector4::from_xyz(0.0, 0.0, 0.0),
            Vector4::from_xyz(1.0, 1.0, 1.0),
        );
        assert!(box_.contains_point(Vector4::from_xyz(0.0, 0.0, 0.0)));
        assert!(box_.contains_point(Vector4::from_xyz(1.0, 1.0, 1.0)));
        assert!(box_.contains_point(Vector4::from_xyz(0.5, 0.5, 0.5)));
        assert!(!box_.contains_point(Vector4::from_xyz(1.5, 0.5, 0.5)));
    }

    #[test]
    fn center_extents_volume_and_expand() {
        let mut a = Aabb::new(
            Vector4::from_xyz(-1.0, -2.0, -3.0),
            Vector4::from_xyz(1.0, 2.0, 3.0),
        );
        assert_eq!(a.center(), Vector4::ZERO);
        assert_eq!(a.half_extents(), Vector4::from_xyz(1.0, 2.0, 3.0));
        assert_eq!(a.extents(), Vector4::from_xyz(2.0, 4.0, 6.0));
        assert_eq!(a.volume(), 48.0);

        a.expand_by(1.0);
        // `w` is carried along by the vector operations (it is meaningless for an
        // AABB); compare the xyz components only.
        assert!(a.min.all_equal3(Vector4::from_xyz(-2.0, -3.0, -4.0), 0.0));
        assert!(a.max.all_equal3(Vector4::from_xyz(2.0, 3.0, 4.0), 0.0));

        a.translate(Vector4::from_xyz(5.0, 0.0, 0.0));
        assert!(a.center().all_equal3(Vector4::from_xyz(5.0, 0.0, 0.0), 0.0));
    }

    #[test]
    fn vertices_and_support() {
        let a = Aabb::new(
            Vector4::from_xyz(0.0, 0.0, 0.0),
            Vector4::from_xyz(1.0, 1.0, 1.0),
        );
        assert_eq!(a.vertex(0), Vector4::from_xyz(0.0, 0.0, 0.0));
        assert_eq!(a.vertex(7), Vector4::from_xyz(1.0, 1.0, 1.0));
        assert_eq!(a.vertex(3), Vector4::from_xyz(1.0, 1.0, 0.0));

        let up = Vector4::from_xyz(0.0, 1.0, 0.0);
        assert_eq!(a.supporting_vertex(up), Vector4::from_xyz(1.0, 1.0, 1.0));
        // Only y is negative; x and z pick the max corners (matching the SDK's
        // `getMask<MASK_XYZ>` + `setSelect` behavior).
        let down = Vector4::from_xyz(0.0, -1.0, 0.0);
        assert_eq!(a.supporting_vertex(down), Vector4::from_xyz(1.0, 0.0, 1.0));
    }

    #[test]
    fn signed_distance() {
        let a = Aabb::new(
            Vector4::from_xyz(0.0, 0.0, 0.0),
            Vector4::from_xyz(1.0, 1.0, 1.0),
        );
        // Outside along +x by 1: distance 1, gradient (1,0,0).
        let (gradient, distance) = a.signed_distance_to_point(Vector4::from_xyz(2.0, 0.5, 0.5));
        assert_eq!(distance, 1.0);
        assert_eq!(gradient, Vector4::from_xyz(1.0, 0.0, 0.0));

        // Center: negative distance to nearest face.
        let (_, distance) = a.signed_distance_to_point(Vector4::from_xyz(0.5, 0.5, 0.5));
        assert_eq!(distance, -0.5);
    }

    #[test]
    fn default_is_empty() {
        assert_eq!(Aabb::default(), Aabb::EMPTY);
    }
}
