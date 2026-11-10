//! GJK closest-point / intersection query for convex shapes.
//!
//! Port of the GJK core of `hkcdGsk::getClosestPoint`
//! (`Common/Source/Geometry/Internal/Algorithms/Gsk/`). One step of the SDK
//! function is the GJK simplex update; this port runs GJK in world space and
//! returns the closest witness points on both shapes.
//!
//! The Minkowski-difference support vertex is
//! `support_A(direction) - support_B(-direction)`. A resulting point `v` points
//! from the witness point on B to the witness point on A, so the returned
//! `normal` (B → A) is `v / |v|`.

use hk_math::{Transform, Vector4};
use hknp_shape::Shape;

use crate::support::{SupportVertex, support_vertex};

/// Maximum GJK iterations (the SDK uses a feature-based termination loop).
const MAX_ITERATIONS: usize = 64;
/// Relative convergence tolerance.
const EPSILON: f32 = 1e-6;
/// Absolute zero threshold for distances/normals.
const ZERO_EPSILON: f32 = 1e-10;
/// Duplicate support-point threshold.
const DUPLICATE_EPSILON: f32 = 1e-12;

/// Result of a GJK query.
#[derive(Clone, Debug)]
pub struct GjkResult {
    /// Distance between the shapes (0 when intersecting or touching).
    pub distance: f32,
    /// Unit normal from shape B to shape A (world space).
    pub normal: Vector4,
    /// Closest witness point on shape A.
    pub point_a: Vector4,
    /// Closest witness point on shape B.
    pub point_b: Vector4,
    /// Whether the shapes intersect (the simplex then contains the origin).
    pub intersects: bool,
    /// Final Minkowski simplex, used by [`crate::epa`] for penetration.
    pub(crate) simplex: Vec<SupportVertex>,
}

impl GjkResult {
    /// Whether the result is a contact/intersection rather than separated.
    pub fn is_intersecting(&self) -> bool {
        self.intersects
    }
}

/// Closest points between two convex shapes, or `None` when either shape has no
/// support function (composites/meshes).
///
/// The returned normal points from B to A. When the shapes intersect the
/// distance is zero; `crate::epa::penetration` uses the retained simplex to
/// compute depth.
pub fn closest_points(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
) -> Option<GjkResult> {
    if !shape_a.is_convex() || !shape_b.is_convex() {
        return None;
    }

    let mut initial_direction = transform_a.translation - transform_b.translation;
    if initial_direction.length3() <= ZERO_EPSILON {
        initial_direction = Vector4::UNIT_X;
    }
    let first = support_vertex(
        shape_a,
        transform_a,
        shape_b,
        transform_b,
        initial_direction,
    )?;
    let mut simplex: Vec<SupportVertex> = vec![first];
    let mut closest = first.minkowski;

    for _ in 0..MAX_ITERATIONS {
        let distance_squared = closest.length_squared3();
        if distance_squared <= ZERO_EPSILON {
            return Some(intersection_result(simplex, transform_a, transform_b));
        }

        let direction = -closest / distance_squared.sqrt();
        let next = support_vertex(shape_a, transform_a, shape_b, transform_b, direction)?;

        // No progress along the search direction: closest point found.
        let progress = distance_squared - closest.dot3(next.minkowski);
        if progress <= EPSILON * distance_squared {
            return Some(separated_result(&simplex, &reduce_simplex(&simplex)));
        }

        // The support point already exists in the simplex: converged.
        if simplex.iter().any(|vertex| {
            (vertex.minkowski - next.minkowski).length_squared3() <= DUPLICATE_EPSILON
        }) {
            return Some(separated_result(&simplex, &reduce_simplex(&simplex)));
        }

        simplex.push(next);
        let reduction = reduce_simplex(&simplex);
        if reduction.inside {
            return Some(intersection_result(simplex, transform_a, transform_b));
        }
        closest = reduction.point;
        simplex = reduction.kept;
    }

    let reduction = reduce_simplex(&simplex);
    Some(separated_result(&simplex, &reduction))
}

/// Convenience boolean intersection query.
pub fn intersects(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
) -> bool {
    closest_points(shape_a, transform_a, shape_b, transform_b)
        .map(|result| result.intersects)
        .unwrap_or(false)
}

/// Result of reducing a simplex to its closest point to the origin.
#[derive(Clone, Debug)]
pub(crate) struct SimplexReduction {
    /// Closest point on the Minkowski simplex.
    pub point: Vector4,
    /// Whether the simplex contains the origin.
    pub inside: bool,
    /// Witness point on A.
    pub point_a: Vector4,
    /// Witness point on B.
    pub point_b: Vector4,
    /// Vertices with non-zero barycentric weight (the reduced simplex).
    pub kept: Vec<SupportVertex>,
}

fn separated_result(simplex: &[SupportVertex], reduction: &SimplexReduction) -> GjkResult {
    let distance = reduction.point.length3();
    let normal = if distance > ZERO_EPSILON {
        reduction.point / distance
    } else {
        Vector4::UNIT_X
    };
    GjkResult {
        distance,
        normal,
        point_a: reduction.point_a,
        point_b: reduction.point_b,
        intersects: false,
        simplex: if reduction.kept.is_empty() {
            simplex.to_vec()
        } else {
            reduction.kept.clone()
        },
    }
}

fn intersection_result(
    simplex: Vec<SupportVertex>,
    transform_a: &Transform,
    transform_b: &Transform,
) -> GjkResult {
    let center_direction = transform_a.translation - transform_b.translation;
    let normal = if center_direction.length3() > ZERO_EPSILON {
        center_direction.normalized3()
    } else {
        Vector4::UNIT_X
    };
    let last = simplex.last().copied().unwrap_or(SupportVertex {
        minkowski: Vector4::ZERO,
        point_a: Vector4::ZERO,
        point_b: Vector4::ZERO,
    });
    GjkResult {
        distance: 0.0,
        normal,
        point_a: last.point_a,
        point_b: last.point_b,
        intersects: true,
        simplex,
    }
}

fn reduce_simplex(points: &[SupportVertex]) -> SimplexReduction {
    match points.len() {
        0 => SimplexReduction {
            point: Vector4::ZERO,
            inside: false,
            point_a: Vector4::ZERO,
            point_b: Vector4::ZERO,
            kept: Vec::new(),
        },
        1 => single_reduction(points),
        2 => {
            let (point, weights) = closest_point_segment(points[0].minkowski, points[1].minkowski);
            build_reduction(points, &weights, point, false)
        }
        3 => {
            let (point, weights) = closest_point_triangle(
                points[0].minkowski,
                points[1].minkowski,
                points[2].minkowski,
            );
            let touching = point.length_squared3() <= ZERO_EPSILON;
            build_reduction(points, &weights, point, touching)
        }
        _ => reduce_tetrahedron(points),
    }
}

fn single_reduction(points: &[SupportVertex]) -> SimplexReduction {
    SimplexReduction {
        point: points[0].minkowski,
        inside: points[0].minkowski.length_squared3() <= ZERO_EPSILON,
        point_a: points[0].point_a,
        point_b: points[0].point_b,
        kept: vec![points[0]],
    }
}

fn build_reduction(
    points: &[SupportVertex],
    weights: &[f32],
    point: Vector4,
    inside: bool,
) -> SimplexReduction {
    let count = points.len().min(weights.len());
    let mut witness_a = Vector4::ZERO;
    let mut witness_b = Vector4::ZERO;
    let mut kept = Vec::new();
    for index in 0..count {
        let weight = weights[index];
        witness_a += points[index].point_a * weight;
        witness_b += points[index].point_b * weight;
        if weight > EPSILON {
            kept.push(points[index]);
        }
    }
    SimplexReduction {
        point,
        inside,
        point_a: witness_a,
        point_b: witness_b,
        kept,
    }
}

/// Closest point on segment `a..b` to the origin.
fn closest_point_segment(a: Vector4, b: Vector4) -> (Vector4, [f32; 2]) {
    let ab = b - a;
    let length_squared = ab.length_squared3();
    if length_squared <= ZERO_EPSILON {
        return (a, [1.0, 0.0]);
    }
    let t = (-a.dot3(ab) / length_squared).clamp(0.0, 1.0);
    (a + ab * t, [1.0 - t, t])
}

/// Closest point on triangle `a, b, c` to the origin (Ericson region tests).
fn closest_point_triangle(a: Vector4, b: Vector4, c: Vector4) -> (Vector4, [f32; 3]) {
    let normal = (b - a).cross(c - a);
    if normal.length_squared3() <= 1e-20 {
        return closest_degenerate_triangle(a, b, c);
    }

    let ab = b - a;
    let ac = c - a;
    let ap = -a;
    let d1 = ab.dot3(ap);
    let d2 = ac.dot3(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, [1.0, 0.0, 0.0]);
    }

    let bp = -b;
    let d3 = ab.dot3(bp);
    let d4 = ac.dot3(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, [0.0, 1.0, 0.0]);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v, [1.0 - v, v, 0.0]);
    }

    let cp = -c;
    let d5 = ab.dot3(cp);
    let d6 = ac.dot3(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, [0.0, 0.0, 1.0]);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w, [1.0 - w, 0.0, w]);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, [0.0, 1.0 - w, w]);
    }

    let denominator = va + vb + vc;
    if denominator.abs() <= ZERO_EPSILON {
        return closest_degenerate_triangle(a, b, c);
    }
    let inverse = 1.0 / denominator;
    let v = vb * inverse;
    let w = vc * inverse;
    (a + ab * v + ac * w, [1.0 - v - w, v, w])
}

fn closest_degenerate_triangle(a: Vector4, b: Vector4, c: Vector4) -> (Vector4, [f32; 3]) {
    let mut best = (a, [1.0, 0.0, 0.0]);
    let mut best_distance = a.length_squared3();
    let (point, weights) = closest_point_segment(a, b);
    let distance = point.length_squared3();
    if distance < best_distance {
        best = (point, [weights[0], weights[1], 0.0]);
        best_distance = distance;
    }
    let (point, weights) = closest_point_segment(b, c);
    let distance = point.length_squared3();
    if distance < best_distance {
        best = (point, [0.0, weights[0], weights[1]]);
        best_distance = distance;
    }
    let (point, weights) = closest_point_segment(a, c);
    if point.length_squared3() < best_distance {
        best = (point, [weights[0], 0.0, weights[1]]);
    }
    best
}

fn reduce_tetrahedron(points: &[SupportVertex]) -> SimplexReduction {
    debug_assert_eq!(points.len(), 4);
    if let Some(weights) = tetrahedron_weights(
        points[0].minkowski,
        points[1].minkowski,
        points[2].minkowski,
        points[3].minkowski,
    ) {
        if weights.iter().all(|weight| *weight >= -EPSILON) {
            let mut witness_a = Vector4::ZERO;
            let mut witness_b = Vector4::ZERO;
            for index in 0..4 {
                witness_a += points[index].point_a * weights[index];
                witness_b += points[index].point_b * weights[index];
            }
            return SimplexReduction {
                point: Vector4::ZERO,
                inside: true,
                point_a: witness_a,
                point_b: witness_b,
                kept: points.to_vec(),
            };
        }
    }

    let faces = [[0usize, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]];
    let mut best: Option<(Vector4, [f32; 4], f32)> = None;
    for face in faces {
        let (point, weights) = closest_point_triangle(
            points[face[0]].minkowski,
            points[face[1]].minkowski,
            points[face[2]].minkowski,
        );
        let mut tetra_weights = [0.0f32; 4];
        tetra_weights[face[0]] = weights[0];
        tetra_weights[face[1]] = weights[1];
        tetra_weights[face[2]] = weights[2];
        let distance = point.length_squared3();
        if best.is_none_or(|(_, _, best_distance)| distance < best_distance) {
            best = Some((point, tetra_weights, distance));
        }
    }

    let Some((point, weights, _)) = best else {
        return single_reduction(points);
    };
    build_reduction(points, &weights, point, false)
}

/// Barycentric weights of the origin in a tetrahedron (Cramer's rule).
fn tetrahedron_weights(a: Vector4, b: Vector4, c: Vector4, d: Vector4) -> Option<[f32; 4]> {
    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let denominator = ab.dot3(ac.cross(ad));
    if denominator.abs() <= ZERO_EPSILON {
        return None;
    }
    let rhs = -a;
    let wb = rhs.dot3(ac.cross(ad)) / denominator;
    let wc = ab.dot3(rhs.cross(ad)) / denominator;
    let wd = ab.dot3(ac.cross(rhs)) / denominator;
    let wa = 1.0 - wb - wc - wd;
    Some([wa, wb, wc, wd])
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;
    use hknp_shape::{BoxShape, Shape, SphereShape};
    use std::sync::Arc;

    fn sphere(radius: f32) -> Arc<dyn Shape> {
        Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
    }

    fn box_shape(half: f32) -> Arc<dyn Shape> {
        Arc::new(BoxShape::with_convex_radius(Vector4::splat(half), 0.0)) as Arc<dyn Shape>
    }

    fn translation(x: f32, y: f32, z: f32) -> Transform {
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(x, y, z))
    }

    #[test]
    fn separated_spheres_give_distance_and_witnesses() {
        let shape = sphere(1.0);
        let result = closest_points(
            shape.as_ref(),
            &Transform::IDENTITY,
            shape.as_ref(),
            &translation(5.0, 0.0, 0.0),
        )
        .unwrap();
        assert!(!result.intersects);
        assert!(
            (result.distance - 3.0).abs() < 1e-4,
            "distance {}",
            result.distance
        );
        assert!((result.normal - Vector4::from_xyz(-1.0, 0.0, 0.0)).length3() < 1e-4);
        assert!(
            result
                .point_a
                .all_equal3(Vector4::from_xyz(1.0, 0.0, 0.0), 1e-4)
        );
        assert!(
            result
                .point_b
                .all_equal3(Vector4::from_xyz(4.0, 0.0, 0.0), 1e-4)
        );
    }

    #[test]
    fn overlapping_spheres_intersect() {
        let shape = sphere(1.0);
        let result = closest_points(
            shape.as_ref(),
            &Transform::IDENTITY,
            shape.as_ref(),
            &translation(1.0, 0.0, 0.0),
        )
        .unwrap();
        assert!(result.intersects);
        assert_eq!(result.distance, 0.0);
        assert!(!result.simplex.is_empty());
    }

    #[test]
    fn separated_boxes_give_distance() {
        let shape = box_shape(1.0);
        let result = closest_points(
            shape.as_ref(),
            &Transform::IDENTITY,
            shape.as_ref(),
            &translation(4.0, 0.0, 0.0),
        )
        .unwrap();
        assert!(!result.intersects);
        assert!(
            (result.distance - 2.0).abs() < 1e-3,
            "distance {}",
            result.distance
        );
        assert!(
            result
                .normal
                .all_equal3(Vector4::from_xyz(-1.0, 0.0, 0.0), 1e-3)
        );
    }

    #[test]
    fn rotated_box_distance() {
        let shape = box_shape(1.0);
        let rotation = Quaternion::from_axis_angle(Vector4::UNIT_Z, core::f32::consts::FRAC_PI_4);
        let rotated = Transform::from_quaternion(rotation, Vector4::from_xyz(0.0, 4.0, 0.0));
        let result = closest_points(
            shape.as_ref(),
            &Transform::IDENTITY,
            shape.as_ref(),
            &rotated,
        )
        .unwrap();
        assert!(!result.intersects);
        assert!(result.distance > 1.0, "distance {}", result.distance);
        assert!((result.normal.length3() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn non_convex_shape_returns_none() {
        use crate::gjk::tests_support::mesh_shape;
        let mesh = mesh_shape();
        assert!(
            closest_points(
                mesh.as_ref(),
                &Transform::IDENTITY,
                mesh.as_ref(),
                &Transform::IDENTITY
            )
            .is_none()
        );
    }
}

/// Test-only helper shared with the narrow-phase tests.
#[cfg(test)]
pub(crate) mod tests_support {
    use hk_math::Vector4;
    use hknp_shape::{MeshShape, Shape};
    use std::sync::Arc;

    pub(crate) fn mesh_shape() -> Arc<dyn Shape> {
        Arc::new(MeshShape::new(
            vec![Vector4::ZERO, Vector4::UNIT_X, Vector4::UNIT_Y],
            vec![[0, 1, 2]],
        )) as Arc<dyn Shape>
    }
}
