//! Contact manifold types.
//!
//! Mirrors the convention of `hkcdManifold4`: the normal points **from B to A**
//! in world space, contact positions lie on B, and a negative distance means
//! penetration.
//!
//! The surface point on A can be reconstructed with
//! `position + normal * distance`, exactly like the SDK solver does.

use hk_math::Vector4;

/// One contact point of a manifold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPoint {
    /// Contact position on shape B, in world space.
    pub position: Vector4,
    /// Signed distance along the manifold normal: positive = separated,
    /// negative = penetrating.
    pub distance: f32,
}

/// A contact manifold between two convex shapes (`hknpManifold` / `hkcdManifold4`).
///
/// Phase 7 generates at most one point per convex-convex pair (GJK/EPA or the
/// analytic sphere-sphere path). Face clipping for multi-point box/face
/// manifolds is deferred to the solver/manifold phase.
#[derive(Clone, Debug, PartialEq)]
pub struct Manifold {
    /// Unit normal from B to A in world space.
    pub normal: Vector4,
    /// Contact points (positions on B).
    pub points: Vec<ContactPoint>,
}

impl Manifold {
    /// Construct a manifold; the normal is normalized defensively.
    pub fn new(normal: Vector4, points: Vec<ContactPoint>) -> Self {
        let length = normal.length3();
        let normal = if length > 0.0 {
            Vector4::from_xyz(normal.x / length, normal.y / length, normal.z / length)
        } else {
            Vector4::UNIT_X
        };
        Self { normal, points }
    }

    /// Number of generated points.
    pub fn num_points(&self) -> usize {
        self.points.len()
    }

    /// Minimum signed distance over all points (positive = separated).
    pub fn minimum_distance(&self) -> f32 {
        self.points
            .iter()
            .fold(f32::INFINITY, |minimum, point| minimum.min(point.distance))
    }

    /// Reconstruct the contact position on shape A for a point
    /// (`position + normal * distance`).
    pub fn point_on_a(&self, index: usize) -> Option<Vector4> {
        self.points
            .get(index)
            .map(|point| point.position + self.normal * point.distance)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reconstructs_point_on_a() {
        let manifold = Manifold::new(
            Vector4::from_xyz(-1.0, 0.0, 0.0),
            vec![ContactPoint {
                position: Vector4::from_xyz(0.5, 0.0, 0.0),
                distance: -0.5,
            }],
        );
        assert_eq!(manifold.normal, Vector4::from_xyz(-1.0, 0.0, 0.0));
        assert_eq!(manifold.num_points(), 1);
        assert_eq!(manifold.minimum_distance(), -0.5);
        assert_eq!(
            manifold.point_on_a(0).unwrap(),
            Vector4::from_xyz(1.0, 0.0, 0.0)
        );
    }

    #[test]
    fn normalizes_normal() {
        let manifold = Manifold::new(
            Vector4::from_xyz(0.0, 2.0, 0.0),
            vec![ContactPoint {
                position: Vector4::ZERO,
                distance: 1.0,
            }],
        );
        assert_eq!(manifold.normal, Vector4::UNIT_Y);
        assert_eq!(
            manifold.point_on_a(0).unwrap(),
            Vector4::from_xyz(0.0, 1.0, 0.0)
        );
    }
}
