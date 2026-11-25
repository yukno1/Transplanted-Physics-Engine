//! Axis-aligned bounding boxes.
//!
//! Port of `hkAabb` from
//! `original/Common/Source/Common/Base/Types/Geometry/Aabb/`.
//!
//! Storage is glam-backed (`Vec3A`) since the glam migration
//! (`docs/refactor-glam-plan.md`): the legacy `hkVector4` `w` component was
//! pure padding and every legacy comparison was xyz-only
//! (`hkVector4ComparisonMask::MASK_XYZ`), so dropping `w` does not change any
//! observable result. The empty box is `min = +MAX, max = -MAX`, the full box
//! is `min = -MAX, max = +MAX`.
//!
//! Legacy-boundary constructors still take the legacy `Vector4` (their `w` is
//! ignored) so call sites keep compiling during the Phase 3 migration; new
//! code can pass glam vectors through [`Aabb::from_corners`].
//!
//! ```
//! use proton::math::{Aabb, Vector4};
//! let mut aabb = Aabb::empty();
//! aabb.include_point(Vector4::UNIT_X);
//! aabb.include_point(Vector4::UNIT_Y);
//! assert!(aabb.contains_point(Vector4::new(0.5, 0.5, 0.0, 0.0)));
//! assert!(!aabb.contains_point(Vector4::new(0.5, 1.5, 0.0, 0.0)));
//! ```

use glam::Vec3A;

use crate::base_math::vector::Vector4;

/// Axis-aligned bounding box (`hkAabb`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    /// Minimum corner.
    pub min: Vec3A,
    /// Maximum corner.
    pub max: Vec3A,
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
        min: Vec3A::splat(f32::MAX),
        max: Vec3A::splat(-f32::MAX),
    };

    /// Full box: `min = -MAX`, `max = +MAX` (`hkAabb::setFull`).
    pub const FULL: Self = Self {
        min: Vec3A::splat(-f32::MAX),
        max: Vec3A::splat(f32::MAX),
    };

    /// Build from min/max corners (legacy boundary; `w` components ignored).
    #[inline]
    pub fn new(min: Vector4, max: Vector4) -> Self {
        Self::from_corners(
            Vec3A::new(min.x, min.y, min.z),
            Vec3A::new(max.x, max.y, max.z),
        )
    }

    /// Build from glam min/max corners.
    #[inline]
    pub const fn from_corners(min: Vec3A, max: Vec3A) -> Self {
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

    /// `hkAabb::setFromPoint` (legacy boundary; `w` ignored).
    #[inline]
    pub fn from_point(point: Vector4) -> Self {
        let p = Vec3A::new(point.x, point.y, point.z);
        Self::from_corners(p, p)
    }

    /// `hkAabb::setFromCenterRadius(center, halfExtents)` (legacy boundary).
    #[inline]
    pub fn from_center_half_extents(center: Vector4, half_extents: Vector4) -> Self {
        let c = Vec3A::new(center.x, center.y, center.z);
        let h = Vec3A::new(half_extents.x, half_extents.y, half_extents.z);
        Self::from_corners(c - h, c + h)
    }

    /// `hkAabb::setFromCenterRadius(center, radius)` (uniform radius).
    #[inline]
    pub fn from_center_radius(center: Vector4, radius: f32) -> Self {
        let c = Vec3A::new(center.x, center.y, center.z);
        let r = Vec3A::splat(radius);
        Self::from_corners(c - r, c + r)
    }

    /// `hkAabb::includePoint` (legacy boundary; `w` ignored).
    ///
    /// `glam`'s `min`/`max` follow `f32::min`/`f32::max` NaN semantics, matching
    /// the legacy scalar implementation.
    #[inline]
    pub fn include_point(&mut self, point: Vector4) {
        self.include_point3(Vec3A::new(point.x, point.y, point.z));
    }

    /// glam-native `includePoint`.
    ///
    /// Per-component `f32::min`/`f32::max`: glam's SIMD `min`/`max` follow
    /// `minps` operand order and would propagate NaN here, while the legacy
    /// scalar path (and `hkAabb::includePoint`) keeps the non-NaN operand.
    #[inline]
    pub fn include_point3(&mut self, point: Vec3A) {
        self.min = Vec3A::new(
            self.min.x.min(point.x),
            self.min.y.min(point.y),
            self.min.z.min(point.z),
        );
        self.max = Vec3A::new(
            self.max.x.max(point.x),
            self.max.y.max(point.y),
            self.max.z.max(point.z),
        );
    }

    /// `hkAabb::includeAabb`.
    #[inline]
    pub fn include_aabb(&mut self, other: &Self) {
        self.min = Vec3A::new(
            self.min.x.min(other.min.x),
            self.min.y.min(other.min.y),
            self.min.z.min(other.min.z),
        );
        self.max = Vec3A::new(
            self.max.x.max(other.max.x),
            self.max.y.max(other.max.y),
            self.max.z.max(other.max.z),
        );
    }

    /// `hkAabb::expandBy` — grow the box by `amount` in all directions.
    #[inline]
    pub fn expand_by(&mut self, amount: f32) {
        let delta = Vec3A::splat(amount);
        self.max += delta;
        self.min -= delta;
    }

    /// `hkAabb::translate`.
    #[inline]
    pub fn translate(&mut self, offset: Vec3A) {
        self.min += offset;
        self.max += offset;
    }

    /// `hkAabb::getExtents` — `max - min`.
    #[inline]
    pub fn extents(&self) -> Vec3A {
        self.max - self.min
    }

    /// `hkAabb::getHalfExtents` — `(max - min) / 2`.
    #[inline]
    pub fn half_extents(&self) -> Vec3A {
        self.extents() / 2.0
    }

    /// `hkAabb::getCenter` — `(min + max) / 2`.
    #[inline]
    pub fn center(&self) -> Vec3A {
        (self.min + self.max) / 2.0
    }

    /// `hkAabb::getVolume` — product of the xyz extents.
    #[inline]
    pub fn volume(&self) -> f32 {
        let e = self.extents();
        e.x * e.y * e.z
    }

    /// `hkAabb::overlaps` — zero-volume overlaps count as overlapping
    /// (legacy: xyz-only `lessEqual` masks).
    #[inline]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.min.y <= other.max.y
            && self.min.z <= other.max.z
            && other.min.x <= self.max.x
            && other.min.y <= self.max.y
            && other.min.z <= self.max.z
    }

    /// `hkAabb::disjoint`.
    #[inline]
    pub fn disjoint(&self, other: &Self) -> bool {
        !self.overlaps(other)
    }

    /// `hkAabb::contains` — `other` lies inside `self` (xyz-only, inclusive).
    #[inline]
    pub fn contains(&self, other: &Self) -> bool {
        self.min.x <= other.min.x
            && self.min.y <= other.min.y
            && self.min.z <= other.min.z
            && other.max.x <= self.max.x
            && other.max.y <= self.max.y
            && other.max.z <= self.max.z
    }

    /// `hkAabb::notContains` (xyz-only).
    #[inline]
    pub fn not_contains(&self, other: &Self) -> bool {
        other.min.x < self.min.x
            || other.min.y < self.min.y
            || other.min.z < self.min.z
            || other.max.x > self.max.x
            || other.max.y > self.max.y
            || other.max.z > self.max.z
    }

    /// `hkAabb::containsPoint` (legacy boundary; `w` ignored).
    #[inline]
    pub fn contains_point(&self, point: Vector4) -> bool {
        self.contains_point3(Vec3A::new(point.x, point.y, point.z))
    }

    /// glam-native `containsPoint` (xyz-only, inclusive).
    #[inline]
    pub fn contains_point3(&self, point: Vec3A) -> bool {
        self.min.x <= point.x
            && self.min.y <= point.y
            && self.min.z <= point.z
            && point.x <= self.max.x
            && point.y <= self.max.y
            && point.z <= self.max.z
    }

    /// `hkAabb::isEmpty` — `min > max` on any xyz component.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// `hkAabb::isValid` — `min <= max` on all xyz components.
    #[inline]
    pub fn is_valid(&self) -> bool {
        !self.is_empty()
    }

    /// `hkAabb::isEqual` — component-wise exact equality on xyz.
    #[inline]
    pub fn is_equal(&self, other: &Self) -> bool {
        self.min == other.min && self.max == other.max
    }

    /// `hkAabb::getVertex` — corner `index` (bit 0 = x, bit 1 = y, bit 2 = z;
    /// a set bit selects the max corner).
    #[inline]
    pub fn vertex(&self, index: usize) -> Vec3A {
        assert!(index <= 7, "AABB vertex index out of range: {index}");
        Vec3A::new(
            if index & 1 != 0 {
                self.max.x
            } else {
                self.min.x
            },
            if index & 2 != 0 {
                self.max.y
            } else {
                self.min.y
            },
            if index & 4 != 0 {
                self.max.z
            } else {
                self.min.z
            },
        )
    }

    /// `hkAabb::signedDistanceToPoint` (gradient and value), xyz only
    /// (legacy boundary; `w` ignored).
    ///
    /// Returns `(gradient, distance)` where the gradient points from the box
    /// toward the point; inside the box the distance is the (negative) distance
    /// to the nearest face.
    pub fn signed_distance_to_point(&self, point: Vector4) -> (Vec3A, f32) {
        self.signed_distance_to_point3(Vec3A::new(point.x, point.y, point.z))
    }

    /// glam-native `signedDistanceToPoint`.
    pub fn signed_distance_to_point3(&self, point: Vec3A) -> (Vec3A, f32) {
        let center = self.center();
        let half = self.half_extents();
        let delta = point - center;

        // Outside/inside test per component, like hkAabbUtil.
        let outside = |d: f32, h: f32| d.abs() - h;
        let x = outside(delta.x, half.x);
        let y = outside(delta.y, half.y);
        let z = outside(delta.z, half.z);
        let max = x.max(y).max(z);
        let gradient = Vec3A::new(
            if x > 0.0 { delta.x.signum() } else { 0.0 },
            if y > 0.0 { delta.y.signum() } else { 0.0 },
            if z > 0.0 { delta.z.signum() } else { 0.0 },
        );
        (gradient, max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_math::Vector4;

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
        assert_eq!(a.min, Vec3A::new(-1.0, 2.0, 0.0));
        assert_eq!(a.max, Vec3A::new(1.0, 5.0, 3.0));

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
        assert_eq!(a.center(), Vec3A::ZERO);
        assert_eq!(a.half_extents(), Vec3A::new(1.0, 2.0, 3.0));
        assert_eq!(a.extents(), Vec3A::new(2.0, 4.0, 6.0));
        assert_eq!(a.volume(), 48.0);

        a.expand_by(1.0);
        assert_eq!(a.min, Vec3A::new(-2.0, -3.0, -4.0));
        assert_eq!(a.max, Vec3A::new(2.0, 3.0, 4.0));

        a.translate(Vec3A::new(5.0, 0.0, 0.0));
        assert_eq!(a.center(), Vec3A::new(5.0, 0.0, 0.0));
    }

    #[test]
    fn vertices() {
        let a = Aabb::new(
            Vector4::from_xyz(0.0, 0.0, 0.0),
            Vector4::from_xyz(1.0, 1.0, 1.0),
        );
        assert_eq!(a.vertex(0), Vec3A::new(0.0, 0.0, 0.0));
        assert_eq!(a.vertex(7), Vec3A::new(1.0, 1.0, 1.0));
        assert_eq!(a.vertex(3), Vec3A::new(1.0, 1.0, 0.0));
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
        assert_eq!(gradient, Vec3A::new(1.0, 0.0, 0.0));

        // Center: negative distance to nearest face.
        let (_, distance) = a.signed_distance_to_point(Vector4::from_xyz(0.5, 0.5, 0.5));
        assert_eq!(distance, -0.5);
    }

    #[test]
    fn default_is_empty() {
        assert_eq!(Aabb::default(), Aabb::EMPTY);
    }
}
