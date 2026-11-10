//! EPA penetration depth for intersecting convex shapes.
//!
//! Extension of the GJK result: once GJK has produced a tetrahedron containing
//! the origin, EPA expands the Minkowski polytope until the closest face is
//! reached. This ports the penetration branch of `hkcdGsk` in simplified form:
//! no feature caching/witness caching, fixed iteration cap and one contact point.
//!
//! Conventions: `depth` is positive penetration along `normal_ba` (the B → A
//! normal); `point_a`/`point_b` are the surface witnesses on A/B facing each
//! other.

use hk_math::{Transform, Vector4};
use hknp_shape::Shape;

use crate::support::{SupportVertex, support_vertex};

/// Penetration data extracted from EPA.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Penetration {
    /// Penetration depth (positive).
    pub depth: f32,
    /// Unit normal from B to A.
    pub normal_ba: Vector4,
    /// Witness point on A.
    pub point_a: Vector4,
    /// Witness point on B.
    pub point_b: Vector4,
}

/// Run EPA on the final GJK simplex.
///
/// Returns `None` when GJK did not produce a full tetrahedron (touching or
/// degenerate configuration); callers then fall back to a support-based
/// estimate.
pub(crate) fn penetration(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
    simplex: &[SupportVertex],
) -> Option<Penetration> {
    let mut vertices = match build_tetrahedron(shape_a, transform_a, shape_b, transform_b, simplex)
    {
        Some(vertices) => vertices,
        None => return sampled_penetration(shape_a, transform_a, shape_b, transform_b),
    };

    let mut faces: Vec<Face> = Vec::new();
    for [i0, i1, i2] in [[0u32, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]] {
        if let Some(face) = Face::new(&vertices, i0, i1, i2) {
            faces.push(face);
        }
    }
    if faces.is_empty() {
        return sampled_penetration(shape_a, transform_a, shape_b, transform_b);
    }

    for _ in 0..64 {
        let Some(face) = faces
            .iter()
            .copied()
            .min_by(|a, b| a.distance.total_cmp(&b.distance))
        else {
            return sampled_penetration(shape_a, transform_a, shape_b, transform_b);
        };

        let Some(support) = support_vertex(shape_a, transform_a, shape_b, transform_b, face.normal)
        else {
            return sampled_penetration(shape_a, transform_a, shape_b, transform_b);
        };
        let support_distance = support.minkowski.dot3(face.normal);

        // The support plane is close enough to the current face: this is the
        // penetration depth.
        if support_distance - face.distance <= 1e-4 {
            return Some(contact_from_face(&vertices, &face));
        }

        // Duplicate support point: terminate with the current face.
        if vertices
            .iter()
            .any(|vertex| (vertex.minkowski - support.minkowski).length_squared3() <= 1e-12)
        {
            return Some(contact_from_face(&vertices, &face));
        }

        let new_index = vertices.len() as u32;
        vertices.push(support);

        let visible: Vec<usize> = faces
            .iter()
            .enumerate()
            .filter(|(_, candidate)| {
                candidate
                    .normal
                    .dot3(support.minkowski - vertices[candidate.i0 as usize].minkowski)
                    > 1e-6
            })
            .map(|(index, _)| index)
            .collect();
        if visible.is_empty() {
            return Some(contact_from_face(&vertices, &face));
        }

        let mut horizon: Vec<(u32, u32)> = Vec::new();
        for index in &visible {
            let candidate = faces[*index];
            for edge in [
                (candidate.i0, candidate.i1),
                (candidate.i1, candidate.i2),
                (candidate.i2, candidate.i0),
            ] {
                if let Some(position) = horizon
                    .iter()
                    .position(|&(x, y)| x == edge.1 && y == edge.0)
                {
                    horizon.remove(position);
                } else {
                    horizon.push(edge);
                }
            }
        }

        faces = faces
            .into_iter()
            .enumerate()
            .filter(|(index, _)| !visible.contains(index))
            .map(|(_, candidate)| candidate)
            .collect();

        for (i0, i1) in horizon {
            if let Some(new_face) = Face::new(&vertices, i0, i1, new_index) {
                faces.push(new_face);
            }
        }
        if faces.is_empty() {
            return Some(contact_from_face(&vertices, &face));
        }
    }

    sampled_penetration(shape_a, transform_a, shape_b, transform_b)
}

/// Build a tetrahedron enclosing the origin from the GJK simplex and a few
/// additional support samples. GJK often terminates with a 2D simplex through
/// the origin for smooth shapes, so EPA cannot start from the simplex alone.
fn build_tetrahedron(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
    simplex: &[SupportVertex],
) -> Option<Vec<SupportVertex>> {
    let mut candidates: Vec<SupportVertex> = Vec::new();
    for vertex in simplex {
        push_unique(&mut candidates, *vertex);
    }

    let axes = [
        Vector4::UNIT_X,
        -Vector4::UNIT_X,
        Vector4::UNIT_Y,
        -Vector4::UNIT_Y,
        Vector4::UNIT_Z,
        -Vector4::UNIT_Z,
    ];
    for direction in axes {
        if let Some(vertex) = support_vertex(shape_a, transform_a, shape_b, transform_b, direction)
        {
            push_unique(&mut candidates, vertex);
        }
    }

    let center_direction = transform_a.translation - transform_b.translation;
    if center_direction.length3() > 1e-8 {
        let normalized = center_direction.normalized3();
        for direction in [normalized, -normalized] {
            if let Some(vertex) =
                support_vertex(shape_a, transform_a, shape_b, transform_b, direction)
            {
                push_unique(&mut candidates, vertex);
            }
        }
    }

    if candidates.len() < 4 {
        return None;
    }

    let mut best: Option<(Vec<SupportVertex>, f32)> = None;
    for a in 0..candidates.len() - 3 {
        for b in a + 1..candidates.len() - 2 {
            for c in b + 1..candidates.len() - 1 {
                for d in c + 1..candidates.len() {
                    let tetra = [candidates[a], candidates[b], candidates[c], candidates[d]];
                    let volume = (tetra[1].minkowski - tetra[0].minkowski).dot3(
                        (tetra[2].minkowski - tetra[0].minkowski)
                            .cross(tetra[3].minkowski - tetra[0].minkowski),
                    );
                    if volume.abs() <= 1e-9 {
                        continue;
                    }
                    let Some(weights) = origin_barycentric(
                        tetra[0].minkowski,
                        tetra[1].minkowski,
                        tetra[2].minkowski,
                        tetra[3].minkowski,
                    ) else {
                        continue;
                    };
                    // Require the origin to be strictly inside, not on a face:
                    // a boundary tetrahedron would make EPA report depth 0.
                    let min_weight = weights.iter().copied().fold(f32::INFINITY, f32::min);
                    if min_weight <= 1e-4 {
                        continue;
                    }
                    let score = min_weight;
                    if best
                        .as_ref()
                        .is_none_or(|(_, best_score)| score > *best_score)
                    {
                        best = Some((tetra.to_vec(), score));
                    }
                }
            }
        }
    }
    best.map(|(tetra, _)| tetra)
}

fn push_unique(candidates: &mut Vec<SupportVertex>, vertex: SupportVertex) {
    if !candidates
        .iter()
        .any(|existing| (existing.minkowski - vertex.minkowski).length_squared3() <= 1e-12)
    {
        candidates.push(vertex);
    }
}

fn origin_barycentric(a: Vector4, b: Vector4, c: Vector4, d: Vector4) -> Option<[f32; 4]> {
    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let denominator = ab.dot3(ac.cross(ad));
    if denominator.abs() <= 1e-10 {
        return None;
    }
    let rhs = -a;
    let wb = rhs.dot3(ac.cross(ad)) / denominator;
    let wc = ab.dot3(rhs.cross(ad)) / denominator;
    let wd = ab.dot3(ac.cross(rhs)) / denominator;
    let wa = 1.0 - wb - wc - wd;
    Some([wa, wb, wc, wd])
}

/// Fallback penetration estimate by sampling support directions.
fn sampled_penetration(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
) -> Option<Penetration> {
    let mut directions = vec![
        Vector4::UNIT_X,
        -Vector4::UNIT_X,
        Vector4::UNIT_Y,
        -Vector4::UNIT_Y,
        Vector4::UNIT_Z,
        -Vector4::UNIT_Z,
    ];
    let center_direction = transform_a.translation - transform_b.translation;
    if center_direction.length3() > 1e-8 {
        let normalized = center_direction.normalized3();
        directions.push(normalized);
        directions.push(-normalized);
    }

    let mut best: Option<Penetration> = None;
    for direction in directions {
        let support = support_vertex(shape_a, transform_a, shape_b, transform_b, direction)?;
        let depth = direction.dot3(support.minkowski);
        if depth < -1e-5 {
            continue;
        }
        let candidate = Penetration {
            depth: depth.max(0.0),
            normal_ba: -direction,
            point_a: support.point_a,
            point_b: support.point_b,
        };
        if best.is_none_or(|current| candidate.depth < current.depth) {
            best = Some(candidate);
        }
    }
    best
}

#[derive(Clone, Copy, Debug)]
struct Face {
    i0: u32,
    i1: u32,
    i2: u32,
    normal: Vector4,
    distance: f32,
}

impl Face {
    fn new(vertices: &[SupportVertex], i0: u32, i1: u32, i2: u32) -> Option<Self> {
        let a = vertices[i0 as usize].minkowski;
        let b = vertices[i1 as usize].minkowski;
        let c = vertices[i2 as usize].minkowski;
        let mut normal = (b - a).cross(c - a);
        let length = normal.length3();
        if length <= 1e-12 {
            return None;
        }
        normal /= length;
        let mut distance = normal.dot3(a);
        if distance < 0.0 {
            normal = -normal;
            distance = -distance;
        }
        Some(Self {
            i0,
            i1,
            i2,
            normal,
            distance,
        })
    }
}

fn contact_from_face(vertices: &[SupportVertex], face: &Face) -> Penetration {
    let a = vertices[face.i0 as usize];
    let b = vertices[face.i1 as usize];
    let c = vertices[face.i2 as usize];
    let point = face.normal * face.distance;
    let weights = triangle_barycentric(a.minkowski, b.minkowski, c.minkowski, point);
    let point_a = a.point_a * weights[0] + b.point_a * weights[1] + c.point_a * weights[2];
    let point_b = a.point_b * weights[0] + b.point_b * weights[1] + c.point_b * weights[2];
    Penetration {
        depth: face.distance,
        normal_ba: -face.normal,
        point_a,
        point_b,
    }
}

/// Barycentric coordinates of a point known to lie on the triangle plane.
fn triangle_barycentric(a: Vector4, b: Vector4, c: Vector4, point: Vector4) -> [f32; 3] {
    let v0 = b - a;
    let v1 = c - a;
    let v2 = point - a;
    let d00 = v0.dot3(v0);
    let d01 = v0.dot3(v1);
    let d11 = v1.dot3(v1);
    let d20 = v2.dot3(v0);
    let d21 = v2.dot3(v1);
    let denominator = d00 * d11 - d01 * d01;
    if denominator.abs() <= 1e-12 {
        return [1.0, 0.0, 0.0];
    }
    let v = (d11 * d20 - d01 * d21) / denominator;
    let w = (d00 * d21 - d01 * d20) / denominator;
    [1.0 - v - w, v, w]
}

/// Render helper for tests: penetration of two shapes, or zero when separated.
#[cfg(test)]
pub(crate) fn penetration_or_zero(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
) -> f32 {
    let result = crate::gjk::closest_points(shape_a, transform_a, shape_b, transform_b).unwrap();
    if !result.intersects {
        return 0.0;
    }
    penetration(shape_a, transform_a, shape_b, transform_b, &result.simplex)
        .map(|penetration| penetration.depth)
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;
    use hknp_shape::{BoxShape, Shape, SphereShape};
    use std::sync::Arc;

    fn translation(x: f32, y: f32, z: f32) -> Transform {
        Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(x, y, z))
    }

    #[test]
    fn overlapping_spheres_have_expected_depth() {
        let shape = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        let depth = penetration_or_zero(
            shape.as_ref(),
            &Transform::IDENTITY,
            shape.as_ref(),
            &translation(1.5, 0.0, 0.0),
        );
        assert!((depth - 0.5).abs() < 1e-3, "depth {depth}");
    }

    #[test]
    fn overlapping_boxes_have_expected_depth() {
        let shape =
            Arc::new(BoxShape::with_convex_radius(Vector4::splat(1.0), 0.0)) as Arc<dyn Shape>;
        let depth = penetration_or_zero(
            shape.as_ref(),
            &Transform::IDENTITY,
            shape.as_ref(),
            &translation(1.5, 0.0, 0.0),
        );
        assert!((depth - 0.5).abs() < 1e-3, "depth {depth}");
    }

    #[test]
    fn separated_shapes_have_zero_depth() {
        let shape = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
        assert_eq!(
            penetration_or_zero(
                shape.as_ref(),
                &Transform::IDENTITY,
                shape.as_ref(),
                &translation(4.0, 0.0, 0.0)
            ),
            0.0
        );
    }

    #[test]
    fn barycentric_weights_reconstruct_point() {
        let a = Vector4::ZERO;
        let b = Vector4::UNIT_X;
        let c = Vector4::UNIT_Y;
        let p = (a + b + c) / 3.0;
        let weights = triangle_barycentric(a, b, c, p);
        assert!((weights[0] + weights[1] + weights[2] - 1.0).abs() < 1e-5);
        let reconstructed = a * weights[0] + b * weights[1] + c * weights[2];
        assert!(reconstructed.all_equal3(p, 1e-5));
    }
}
