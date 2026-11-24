//! Deterministic point-cloud convex hull construction.
//!
//! The original SDK builds hulls through `hkgpConvexHull` (whose implementation
//! is not shipped in the source distribution) and uses
//! `hkcdPlanarGeometryConvexHullUtil` as a robust fixed-precision model. This
//! module ports the needed subset: deduplicate inputs, derive a deterministic
//! initial simplex, then run a QuickHull-style visible-facet/horizon loop with
//! explicit index-order tie-breaks. Random simplification, plane merging,
//! Boolean operations and mesh decimation are intentionally not ported.
//!
//! Lower-dimensional inputs are supported: [`HullDimension::Point`] and
//! [`HullDimension::Line`] return extremal vertices with no faces, while
//! [`HullDimension::Plane`] returns the ordered 2D hull vertices and no faces.
//! Those results still drive `Aabb`/`support` correctly in `phys-shape`, but
//! have no closed volume.

use std::collections::BTreeSet;

use crate::base_math::Vector4;

use crate::base_geometry::weld::{WeldError, WeldOptions, weld_vertices};

/// Default weld tolerance applied before hull construction.
pub const DEFAULT_HULL_TOLERANCE: f32 = 1e-6;

/// Dimension of the constructed hull.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HullDimension {
    Point,
    Line,
    Plane,
    Volume,
}

/// Errors from hull construction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HullError {
    EmptyInput,
    NonFinitePoint,
    Degenerate,
}

impl From<WeldError> for HullError {
    fn from(error: WeldError) -> Self {
        match error {
            WeldError::NonFinitePoint => HullError::NonFinitePoint,
            WeldError::InvalidThreshold | WeldError::IndexOutOfRange => HullError::Degenerate,
        }
    }
}

impl core::fmt::Display for HullError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for HullError {}

/// Vertices and outward-facing triangle faces of a convex hull.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexHull3D {
    vertices: Vec<Vector4>,
    faces: Vec<[u32; 3]>,
    dimension: HullDimension,
}

impl ConvexHull3D {
    #[inline]
    pub fn vertices(&self) -> &[Vector4] {
        &self.vertices
    }

    #[inline]
    pub fn faces(&self) -> &[[u32; 3]] {
        &self.faces
    }

    #[inline]
    pub fn dimension(&self) -> HullDimension {
        self.dimension
    }
}

/// Build a hull with [`DEFAULT_HULL_TOLERANCE`].
pub fn build_convex_hull(points: &[Vector4]) -> Result<ConvexHull3D, HullError> {
    build_convex_hull_with_tolerance(points, DEFAULT_HULL_TOLERANCE)
}

/// Build a hull after welding points within `tolerance`.
pub fn build_convex_hull_with_tolerance(
    points: &[Vector4],
    tolerance: f32,
) -> Result<ConvexHull3D, HullError> {
    let options = WeldOptions {
        threshold: tolerance,
        keep_vertex_order: true,
        ..WeldOptions::default()
    };
    let welded = weld_vertices(points, &[], options)?;
    let points = welded.vertices;
    if points.is_empty() {
        return Err(HullError::EmptyInput);
    }
    if points.len() == 1 {
        return Ok(ConvexHull3D {
            vertices: points,
            faces: Vec::new(),
            dimension: HullDimension::Point,
        });
    }

    let mut minimum = points[0];
    let mut maximum = points[0];
    for point in &points[1..] {
        minimum = minimum.min(*point);
        maximum = maximum.max(*point);
    }
    let extents = [
        maximum.x - minimum.x,
        maximum.y - minimum.y,
        maximum.z - minimum.z,
    ];
    let scale = extents[0].max(extents[1]).max(extents[2]);
    if scale <= f32::EPSILON {
        return Ok(ConvexHull3D {
            vertices: vec![points[0]],
            faces: Vec::new(),
            dimension: HullDimension::Point,
        });
    }
    let epsilon = scale * 1e-6;

    let axis = if extents[0] >= extents[1] && extents[0] >= extents[2] {
        0
    } else if extents[1] >= extents[2] {
        1
    } else {
        2
    };
    let (first, last) = extremes(&points, axis);
    if first == last {
        return Err(HullError::Degenerate);
    }
    let origin = points[first];
    let line = points[last] - origin;
    let line_length_sq = line.length_squared3();
    let unit_line = line.normalized3();

    let mut farthest_index = 0usize;
    let mut farthest_distance_sq = -1.0f32;
    for (index, point) in points.iter().enumerate() {
        let distance_sq = line.cross(*point - origin).length_squared3();
        if distance_sq > farthest_distance_sq {
            farthest_distance_sq = distance_sq;
            farthest_index = index;
        }
    }
    if farthest_distance_sq <= epsilon * epsilon * line_length_sq {
        return Ok(ConvexHull3D {
            vertices: vec![points[first], points[last]],
            faces: Vec::new(),
            dimension: HullDimension::Line,
        });
    }
    let second = farthest_index;
    let plane_normal = line.cross(points[second] - origin).normalized3();

    let mut third = 0usize;
    let mut plane_distance = -1.0f32;
    for (index, point) in points.iter().enumerate() {
        let distance = (plane_normal.dot3(*point - origin)).abs();
        if distance > plane_distance {
            plane_distance = distance;
            third = index;
        }
    }
    if plane_distance <= epsilon {
        let plane_v = plane_normal.cross(unit_line).normalized3();
        return Ok(convex_hull_2d(&points, origin, unit_line, plane_v));
    }

    let simplex = [first as u32, last as u32, second as u32, third as u32];
    let mut interior = Vector4::ZERO;
    for &index in &simplex {
        interior += points[index as usize];
    }
    interior *= 0.25;

    let mut facets = Vec::new();
    for face in [
        [simplex[0], simplex[1], simplex[2]],
        [simplex[0], simplex[1], simplex[3]],
        [simplex[0], simplex[2], simplex[3]],
        [simplex[1], simplex[2], simplex[3]],
    ] {
        if let Some(facet) = make_facet(&points, face, interior) {
            facets.push(facet);
        }
    }
    if facets.len() != 4 {
        return Err(HullError::Degenerate);
    }

    let mut centroid = Vector4::ZERO;
    for point in &points {
        centroid += *point;
    }
    centroid *= 1.0 / points.len() as f32;

    let mut processed = vec![false; points.len()];
    let mut guard = 0usize;
    let guard_limit = points.len() * 4 + 16;
    loop {
        rebuild_outside(&mut facets, &points, &processed, epsilon);
        let Some(facet_index) = facets.iter().position(|facet| !facet.outside.is_empty()) else {
            break;
        };
        let apex = facets[facet_index].outside[0];
        if processed[apex] {
            return Err(HullError::Degenerate);
        }

        let point = points[apex];
        let mut visible = vec![false; facets.len()];
        for (index, facet) in facets.iter().enumerate() {
            visible[index] = signed_distance(facet, point) > epsilon;
        }

        let mut horizon = BTreeSet::new();
        for (index, facet) in facets
            .iter()
            .enumerate()
            .filter(|(index, _)| visible[*index])
        {
            let vertices = facet.vertices;
            for edge in [
                (vertices[0], vertices[1]),
                (vertices[1], vertices[2]),
                (vertices[2], vertices[0]),
            ] {
                let shared_with_visible = facets.iter().enumerate().any(|(other, neighbor)| {
                    other != index && visible[other] && has_reversed_edge(neighbor.vertices, edge)
                });
                if !shared_with_visible {
                    horizon.insert(edge);
                }
            }
        }
        if horizon.is_empty() {
            processed[apex] = true;
            continue;
        }

        let mut next_facets: Vec<Facet> = facets
            .iter()
            .enumerate()
            .filter(|(index, _)| !visible[*index])
            .map(|(_, facet)| facet.clone())
            .collect();
        for (a, b) in horizon {
            if let Some(facet) = make_facet(&points, [a, b, apex as u32], centroid) {
                next_facets.push(facet);
            }
        }
        facets = next_facets;
        processed[apex] = true;
        guard += 1;
        if guard > guard_limit {
            return Err(HullError::Degenerate);
        }
    }

    if facets.is_empty() {
        return Err(HullError::Degenerate);
    }
    let mut used = BTreeSet::new();
    for facet in &facets {
        for &index in &facet.vertices {
            used.insert(index);
        }
    }
    let mut old_to_new = vec![u32::MAX; points.len()];
    let mut vertices = Vec::with_capacity(used.len());
    for (new_index, &old_index) in used.iter().enumerate() {
        old_to_new[old_index as usize] = new_index as u32;
        vertices.push(points[old_index as usize]);
    }
    let faces = facets
        .iter()
        .map(|facet| {
            [
                old_to_new[facet.vertices[0] as usize],
                old_to_new[facet.vertices[1] as usize],
                old_to_new[facet.vertices[2] as usize],
            ]
        })
        .collect();

    Ok(ConvexHull3D {
        vertices,
        faces,
        dimension: HullDimension::Volume,
    })
}

#[derive(Clone, Debug)]
struct Facet {
    vertices: [u32; 3],
    normal: Vector4,
    offset: f32,
    outside: Vec<usize>,
}

fn make_facet(points: &[Vector4], vertices: [u32; 3], interior: Vector4) -> Option<Facet> {
    let a = points[vertices[0] as usize];
    let b = points[vertices[1] as usize];
    let c = points[vertices[2] as usize];
    let normal = (b - a).cross(c - a);
    if normal.length_squared3() <= f32::EPSILON {
        return None;
    }
    let vertices = if normal.dot3(a - interior) < 0.0 {
        [vertices[0], vertices[2], vertices[1]]
    } else {
        vertices
    };
    let a = points[vertices[0] as usize];
    let b = points[vertices[1] as usize];
    let c = points[vertices[2] as usize];
    let mut normal = (b - a).cross(c - a).normalized3();
    normal.w = 0.0;
    Some(Facet {
        vertices,
        offset: -normal.dot3(a),
        normal,
        outside: Vec::new(),
    })
}

fn signed_distance(facet: &Facet, point: Vector4) -> f32 {
    facet.normal.dot3(point) + facet.offset
}

fn has_reversed_edge(vertices: [u32; 3], edge: (u32, u32)) -> bool {
    (vertices[0] == edge.1 && vertices[1] == edge.0)
        || (vertices[1] == edge.1 && vertices[2] == edge.0)
        || (vertices[2] == edge.1 && vertices[0] == edge.0)
}

fn rebuild_outside(facets: &mut [Facet], points: &[Vector4], processed: &[bool], epsilon: f32) {
    for facet in facets.iter_mut() {
        facet.outside.clear();
    }
    for (index, point) in points.iter().enumerate() {
        if processed[index] {
            continue;
        }
        let mut best = epsilon;
        let mut best_facet = None;
        for (facet_index, facet) in facets.iter().enumerate() {
            let distance = signed_distance(facet, *point);
            if distance > best {
                best = distance;
                best_facet = Some(facet_index);
            }
        }
        if let Some(facet_index) = best_facet {
            facets[facet_index].outside.push(index);
            continue;
        }
        for (facet_index, facet) in facets.iter().enumerate() {
            if signed_distance(facet, *point).abs() <= epsilon
                && point_outside_triangle(points, facet, *point, epsilon)
            {
                facets[facet_index].outside.push(index);
                break;
            }
        }
    }
}

fn point_outside_triangle(points: &[Vector4], facet: &Facet, point: Vector4, epsilon: f32) -> bool {
    let a = points[facet.vertices[0] as usize];
    let b = points[facet.vertices[1] as usize];
    let c = points[facet.vertices[2] as usize];
    let n = facet.normal;
    let ab = n.dot3((b - a).cross(point - a));
    let bc = n.dot3((c - b).cross(point - b));
    let ca = n.dot3((a - c).cross(point - c));
    ab < -epsilon || bc < -epsilon || ca < -epsilon
}

fn extremes(points: &[Vector4], axis: usize) -> (usize, usize) {
    let component = |point: Vector4| match axis {
        0 => point.x,
        1 => point.y,
        _ => point.z,
    };
    let mut minimum = 0usize;
    let mut maximum = 0usize;
    for (index, &point) in points.iter().enumerate().skip(1) {
        let value = component(point);
        if value < component(points[minimum]) {
            minimum = index;
        }
        if value > component(points[maximum]) {
            maximum = index;
        }
    }
    (minimum, maximum)
}

fn convex_hull_2d(points: &[Vector4], origin: Vector4, u: Vector4, v: Vector4) -> ConvexHull3D {
    let mut projected: Vec<(f32, f32, usize)> = points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let relative = *point - origin;
            (u.dot3(relative), v.dot3(relative), index)
        })
        .collect();
    projected.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap()
            .then_with(|| a.1.partial_cmp(&b.1).unwrap())
            .then_with(|| a.2.cmp(&b.2))
    });
    if projected.len() <= 2 {
        return ConvexHull3D {
            vertices: projected
                .into_iter()
                .map(|(_, _, index)| points[index])
                .collect(),
            faces: Vec::new(),
            dimension: HullDimension::Plane,
        };
    }

    let mut lower = Vec::new();
    for index in 0..projected.len() {
        while lower.len() >= 2
            && cross2(
                projected[lower[lower.len() - 2]],
                projected[lower[lower.len() - 1]],
                projected[index],
            ) <= 0.0
        {
            lower.pop();
        }
        lower.push(index);
    }
    let mut upper = Vec::new();
    for index in (0..projected.len()).rev() {
        while upper.len() >= 2
            && cross2(
                projected[upper[upper.len() - 2]],
                projected[upper[upper.len() - 1]],
                projected[index],
            ) <= 0.0
        {
            upper.pop();
        }
        upper.push(index);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    let vertices = lower
        .into_iter()
        .map(|position| points[projected[position].2])
        .collect();
    ConvexHull3D {
        vertices,
        faces: Vec::new(),
        dimension: HullDimension::Plane,
    }
}

fn cross2(a: (f32, f32, usize), b: (f32, f32, usize), c: (f32, f32, usize)) -> f32 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: f32, y: f32, z: f32) -> Vector4 {
        Vector4::from_xyz(x, y, z)
    }

    fn cube() -> Vec<Vector4> {
        let h = 1.0;
        vec![
            point(-h, -h, -h),
            point(h, -h, -h),
            point(h, h, -h),
            point(-h, h, -h),
            point(-h, -h, h),
            point(h, -h, h),
            point(h, h, h),
            point(-h, h, h),
        ]
    }

    #[test]
    fn rejects_empty_input() {
        assert_eq!(build_convex_hull(&[]), Err(HullError::EmptyInput));
    }

    #[test]
    fn handles_lower_dimensions() {
        let single = build_convex_hull(&[point(0.0, 0.0, 0.0)]).unwrap();
        assert_eq!(single.dimension(), HullDimension::Point);
        assert_eq!(single.vertices().len(), 1);

        let line = build_convex_hull(&[
            point(0.0, 0.0, 0.0),
            point(2.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
        ])
        .unwrap();
        assert_eq!(line.dimension(), HullDimension::Line);
        assert_eq!(line.vertices().len(), 2);

        let plane = build_convex_hull(&[
            point(0.0, 0.0, 0.0),
            point(2.0, 0.0, 0.0),
            point(2.0, 1.0, 0.0),
            point(0.0, 1.0, 0.0),
            point(1.0, 0.5, 0.0),
        ])
        .unwrap();
        assert_eq!(plane.dimension(), HullDimension::Plane);
        assert_eq!(plane.vertices().len(), 4);
        assert!(plane.faces().is_empty());
    }

    #[test]
    fn builds_cube_with_outward_faces() {
        let hull = build_convex_hull(&cube()).unwrap();
        assert_eq!(hull.dimension(), HullDimension::Volume);
        assert_eq!(hull.vertices().len(), 8);
        assert_eq!(hull.faces().len(), 12);
        let mut six_volume = 0.0f32;
        for &[a, b, c] in hull.faces() {
            let pa = hull.vertices()[a as usize];
            let pb = hull.vertices()[b as usize];
            let pc = hull.vertices()[c as usize];
            six_volume += pa.dot3(pb.cross(pc));
        }
        assert!((six_volume / 6.0 - 8.0).abs() < 1e-3);
    }

    #[test]
    fn ignores_duplicates_and_interior_points() {
        let mut points = cube();
        points.push(point(0.0, 0.0, 0.0));
        points.extend(cube());
        let hull = build_convex_hull(&points).unwrap();
        assert_eq!(hull.dimension(), HullDimension::Volume);
        assert_eq!(hull.vertices().len(), 8);
        assert_eq!(hull.faces().len(), 12);
    }

    #[test]
    fn builds_tetrahedron() {
        let hull = build_convex_hull(&[
            point(0.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(0.0, 1.0, 0.0),
            point(0.0, 0.0, 1.0),
        ])
        .unwrap();
        assert_eq!(hull.dimension(), HullDimension::Volume);
        assert_eq!(hull.faces().len(), 4);
    }

    #[test]
    fn is_deterministic() {
        assert_eq!(build_convex_hull(&cube()), build_convex_hull(&cube()));
    }
}
