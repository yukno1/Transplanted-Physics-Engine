//! Vertex/triangle welding (`hkGeometryUtils::weldVertices` subset).
//!
//! The original helper sorts input vertices by X, sweeps with an absolute
//! Euclidean threshold (default `1e-3`) and emits vertex/triangle remaps. This
//! port keeps that public behaviour but uses an explicit total order so results
//! are reproducible across runs: finite f32 values only, no `RandomState`, and
//! original indices as the final tie-break.

use std::collections::BTreeSet;

use base_math::Vector4;

/// Options for [`weld_vertices`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldOptions {
    /// Maximum Euclidean distance (inclusive) for two vertices to merge.
    pub threshold: f32,
    /// Emit unique vertices in first-occurrence order instead of sweep order.
    pub keep_vertex_order: bool,
    /// Drop triangles with two equal indices after remapping.
    pub remove_degenerate_triangles: bool,
    /// Drop triangles whose index triple was already emitted.
    pub remove_duplicate_triangles: bool,
    /// Treat `(a,b,c)` and `(a,c,b)` as the same triangle when deduplicating.
    pub ignore_winding: bool,
}

impl Default for WeldOptions {
    fn default() -> Self {
        Self {
            threshold: 1e-3,
            keep_vertex_order: false,
            remove_degenerate_triangles: true,
            remove_duplicate_triangles: false,
            ignore_winding: true,
        }
    }
}

/// Errors from [`weld_vertices`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WeldError {
    /// `threshold` must be finite and non-negative.
    InvalidThreshold,
    /// Input positions must contain only finite f32 components.
    NonFinitePoint,
    /// A triangle referenced a vertex outside the input slice.
    IndexOutOfRange,
}

impl core::fmt::Display for WeldError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for WeldError {}

/// Result of a weld operation.
///
/// `vertex_remap` maps every input vertex to a compact index in [`WeldResult::vertices`].
/// `triangle_remap` maps every input triangle to a compact index in
/// [`WeldResult::triangles`], or `-1` when the triangle was removed.
#[derive(Clone, Debug, PartialEq)]
pub struct WeldResult {
    pub vertices: Vec<Vector4>,
    pub triangles: Vec<[u32; 3]>,
    pub vertex_remap: Vec<u32>,
    pub triangle_remap: Vec<i32>,
}

/// Weld coincident vertices and rewrite triangle indices.
pub fn weld_vertices(
    points: &[Vector4],
    triangles: &[[u32; 3]],
    options: WeldOptions,
) -> Result<WeldResult, WeldError> {
    if !options.threshold.is_finite() || options.threshold < 0.0 {
        return Err(WeldError::InvalidThreshold);
    }
    if points
        .iter()
        .any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite())
    {
        return Err(WeldError::NonFinitePoint);
    }

    let mut sorted: Vec<usize> = (0..points.len()).collect();
    sorted.sort_by(|&a, &b| compare_point(points[a], points[b]).then_with(|| a.cmp(&b)));
    let threshold_sq = options.threshold * options.threshold;

    let mut representative: Vec<usize> = (0..points.len()).collect();
    let mut reps: Vec<usize> = Vec::new();
    for &index in &sorted {
        let point = points[index];
        let mut found = None;
        for &rep in &reps {
            if points[rep].x - point.x > options.threshold {
                break;
            }
            if distance_sq3(point, points[rep]) <= threshold_sq {
                found = Some(rep);
                break;
            }
        }
        match found {
            Some(rep) => representative[index] = rep,
            None => {
                reps.push(index);
                representative[index] = index;
            }
        }
    }

    let unique_order: Vec<usize> = if options.keep_vertex_order {
        let mut seen = BTreeSet::new();
        (0..points.len())
            .filter(|&index| seen.insert(representative[index]))
            .collect()
    } else {
        reps
    };

    let mut compact: Vec<usize> = vec![0; points.len()];
    for (new_index, &original) in unique_order.iter().enumerate() {
        compact[original] = new_index;
    }
    let vertex_remap: Vec<u32> = representative
        .iter()
        .map(|&rep| compact[rep] as u32)
        .collect();
    let vertices: Vec<Vector4> = unique_order.iter().map(|&index| points[index]).collect();

    let mut welded_triangles = Vec::new();
    let mut triangle_remap = vec![-1i32; triangles.len()];
    let mut seen_triangles = BTreeSet::new();

    for (triangle_index, triangle) in triangles.iter().enumerate() {
        if triangle.iter().any(|&index| index as usize >= points.len()) {
            return Err(WeldError::IndexOutOfRange);
        }
        let mapped = [
            vertex_remap[triangle[0] as usize],
            vertex_remap[triangle[1] as usize],
            vertex_remap[triangle[2] as usize],
        ];
        if options.remove_degenerate_triangles
            && (mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[0] == mapped[2])
        {
            continue;
        }
        let key = if options.ignore_winding {
            let mut sorted = mapped;
            sorted.sort_unstable();
            (sorted[0], sorted[1], sorted[2])
        } else {
            (mapped[0], mapped[1], mapped[2])
        };
        if options.remove_duplicate_triangles && !seen_triangles.insert(key) {
            continue;
        }
        triangle_remap[triangle_index] = welded_triangles.len() as i32;
        welded_triangles.push(mapped);
    }

    Ok(WeldResult {
        vertices,
        triangles: welded_triangles,
        vertex_remap,
        triangle_remap,
    })
}

fn compare_point(a: Vector4, b: Vector4) -> core::cmp::Ordering {
    a.x.partial_cmp(&b.x)
        .unwrap()
        .then_with(|| a.y.partial_cmp(&b.y).unwrap())
        .then_with(|| a.z.partial_cmp(&b.z).unwrap())
}

fn distance_sq3(a: Vector4, b: Vector4) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    dx * dx + dy * dy + dz * dz
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: f32, y: f32, z: f32) -> Vector4 {
        Vector4::from_xyz(x, y, z)
    }

    #[test]
    fn welds_duplicate_points_and_keeps_first_occurrence_order() {
        let points = vec![
            point(1.0, 0.0, 0.0),
            point(0.0, 0.0, 0.0),
            point(1.0 + 1e-4, 0.0, 0.0),
            point(0.0, 1.0, 0.0),
        ];
        let options = WeldOptions {
            keep_vertex_order: true,
            ..WeldOptions::default()
        };
        let result = weld_vertices(&points, &[], options).unwrap();
        assert_eq!(result.vertices.len(), 3);
        assert_eq!(result.vertices[0], points[0]);
        assert_eq!(result.vertices[1], points[1]);
        assert_eq!(result.vertices[2], points[3]);
        assert_eq!(result.vertex_remap, vec![0, 1, 0, 2]);
    }

    #[test]
    fn remaps_triangles_and_drops_degenerate_faces() {
        let points = vec![
            point(0.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(1.0 + 1e-4, 0.0, 0.0),
            point(0.0, 1.0, 0.0),
        ];
        let triangles = vec![[0, 1, 3], [1, 2, 3]];
        let options = WeldOptions {
            keep_vertex_order: true,
            ..WeldOptions::default()
        };
        let result = weld_vertices(&points, &triangles, options).unwrap();
        assert_eq!(result.triangles, vec![[0, 1, 2]]);
        assert_eq!(result.triangle_remap, vec![0, -1]);
    }

    #[test]
    fn removes_duplicate_triangles_ignoring_winding() {
        let points = vec![
            point(0.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(0.0, 1.0, 0.0),
        ];
        let triangles = vec![[0, 1, 2], [2, 1, 0]];
        let options = WeldOptions {
            remove_duplicate_triangles: true,
            ..WeldOptions::default()
        };
        let result = weld_vertices(&points, &triangles, options).unwrap();
        assert_eq!(result.triangles.len(), 1);
        assert_eq!(result.triangle_remap, vec![0, -1]);
    }

    #[test]
    fn rejects_non_finite_and_bad_indices() {
        assert_eq!(
            weld_vertices(&[point(f32::NAN, 0.0, 0.0)], &[], WeldOptions::default()),
            Err(WeldError::NonFinitePoint)
        );
        assert_eq!(
            weld_vertices(
                &[point(0.0, 0.0, 0.0)],
                &[[0, 1, 2]],
                WeldOptions::default()
            ),
            Err(WeldError::IndexOutOfRange)
        );
    }
}
