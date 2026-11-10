//! Triangle mesh with a simple BVH (`hknpMeshShape`).
//!
//! Simplified port of `Collide/Shape/Composite/Mesh/Mesh/hknpMeshShape.h`. The
//! C++ mesh stores a vertex buffer, per-primitive sections, a bounding-volume
//! hierarchy and optional connectivity. This phase keeps the triangle soup
//! (vertices + triangle indices) and builds a median-split BVH over the triangle
//! AABBs; sections/material ids and compressed-mesh variants are deferred.
//!
//! The BVH is used by `hknp-collide` (Phase 7) for accelerated queries. Nodes are
//! stored in pre-order in one flat array; leaves reference contiguous ranges of
//! [`MeshShape::triangle_order`], which stores triangle indices.

use core::any::Any;

use hk_math::{Aabb, Vector4};
use hknp_types::{CollisionDispatchType, ShapeFlags, ShapeType};

use crate::{Shape, aabb_bounding_radius, aabb_union};

const LEAF_TRIANGLES: usize = 4;

/// One node of the mesh bounding-volume hierarchy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BvhNode {
    /// Bounds enclosing all triangles under this node.
    pub aabb: Aabb,
    /// First index into [`MeshShape::triangle_order`] (leaves only).
    pub first: u32,
    /// Number of triangles (leaves only).
    pub count: u32,
    /// Child node indices `(left, right)`; `None` for leaves.
    pub children: Option<(u32, u32)>,
}

impl BvhNode {
    /// Whether this node is a leaf.
    #[inline]
    pub const fn is_leaf(self) -> bool {
        self.children.is_none()
    }
}

/// Triangle mesh (`hknpMeshShape`).
#[derive(Clone, Debug, PartialEq)]
pub struct MeshShape {
    vertices: Vec<Vector4>,
    triangles: Vec<[u32; 3]>,
    aabb: Aabb,
    nodes: Vec<BvhNode>,
    triangle_order: Vec<u32>,
}

impl MeshShape {
    /// Build a mesh from vertices and triangles.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if the vertex list is empty or a triangle index is
    /// out of range.
    pub fn new(vertices: Vec<Vector4>, triangles: Vec<[u32; 3]>) -> Self {
        debug_assert!(!vertices.is_empty(), "mesh needs vertices");
        for triangle in &triangles {
            for &index in triangle {
                debug_assert!(
                    (index as usize) < vertices.len(),
                    "mesh triangle index out of range"
                );
            }
        }

        let aabb = vertices
            .iter()
            .fold(Aabb::new(vertices[0], vertices[0]), |aabb, vertex| {
                let mut result = aabb;
                result.include_point(*vertex);
                result
            });

        let triangle_aabbs: Vec<Aabb> = triangles
            .iter()
            .map(|&[a, b, c]| {
                let pa = vertices[a as usize];
                let pb = vertices[b as usize];
                let pc = vertices[c as usize];
                Aabb::new(pa.min(pb).min(pc), pa.max(pb).max(pc))
            })
            .collect();

        let mut indices: Vec<u32> = (0..triangles.len() as u32).collect();
        let mut nodes = Vec::new();
        let mut triangle_order = Vec::new();
        if !indices.is_empty() {
            build_bvh(
                &mut nodes,
                &mut triangle_order,
                &mut indices,
                &triangle_aabbs,
            );
        }

        Self {
            vertices,
            triangles,
            aabb,
            nodes,
            triangle_order,
        }
    }

    /// Mesh vertices.
    #[inline]
    pub fn vertices(&self) -> &[Vector4] {
        &self.vertices
    }

    /// Mesh triangles (indices into [`MeshShape::vertices`]).
    #[inline]
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }

    /// BVH nodes (empty when the mesh has no triangles).
    #[inline]
    pub fn bvh_nodes(&self) -> &[BvhNode] {
        &self.nodes
    }

    /// Triangle indices grouped into leaf ranges.
    #[inline]
    pub fn triangle_order(&self) -> &[u32] {
        &self.triangle_order
    }

    /// Triangles referenced by a leaf node (empty for interior nodes).
    pub fn node_triangles(&self, node_index: usize) -> &[u32] {
        let node = self.nodes[node_index];
        if node.is_leaf() {
            &self.triangle_order[node.first as usize..(node.first as usize + node.count as usize)]
        } else {
            &[]
        }
    }
}

impl Shape for MeshShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Mesh
    }

    fn dispatch_type(&self) -> CollisionDispatchType {
        CollisionDispatchType::Composite
    }

    fn flags(&self) -> ShapeFlags {
        ShapeFlags::IS_COMPOSITE_SHAPE
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

/// Recursively build the BVH, returning the node index.
fn build_bvh(
    nodes: &mut Vec<BvhNode>,
    order: &mut Vec<u32>,
    indices: &mut [u32],
    triangle_aabbs: &[Aabb],
) -> u32 {
    let bounds = indices.iter().fold(Aabb::EMPTY, |aabb, &index| {
        aabb_union(&aabb, &triangle_aabbs[index as usize])
    });

    let node_index = nodes.len() as u32;
    nodes.push(BvhNode {
        aabb: bounds,
        first: 0,
        count: 0,
        children: None,
    });

    if indices.len() <= LEAF_TRIANGLES {
        let first = order.len() as u32;
        order.extend_from_slice(indices);
        nodes[node_index as usize] = BvhNode {
            aabb: bounds,
            first,
            count: indices.len() as u32,
            children: None,
        };
        return node_index;
    }

    let extents = bounds.extents();
    let axis = if extents.x >= extents.y && extents.x >= extents.z {
        0
    } else if extents.y >= extents.z {
        1
    } else {
        2
    };

    indices.sort_by(|&a, &b| {
        let ca = triangle_aabbs[a as usize].center().component(axis);
        let cb = triangle_aabbs[b as usize].center().component(axis);
        ca.total_cmp(&cb)
    });

    let middle = indices.len() / 2;
    let (left_indices, right_indices) = indices.split_at_mut(middle);
    let left = build_bvh(nodes, order, left_indices, triangle_aabbs);
    let right = build_bvh(nodes, order, right_indices, triangle_aabbs);

    nodes[node_index as usize] = BvhNode {
        aabb: bounds,
        first: 0,
        count: 0,
        children: Some((left, right)),
    };
    node_index
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad() -> (Vec<Vector4>, Vec<[u32; 3]>) {
        (
            vec![
                Vector4::from_xyz(0.0, 0.0, 0.0),
                Vector4::from_xyz(1.0, 0.0, 0.0),
                Vector4::from_xyz(1.0, 1.0, 0.0),
                Vector4::from_xyz(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
    }

    fn grid(size: usize) -> (Vec<Vector4>, Vec<[u32; 3]>) {
        let mut vertices = Vec::with_capacity(size * size);
        for z in 0..size {
            for x in 0..size {
                vertices.push(Vector4::from_xyz(x as f32, 0.0, z as f32));
            }
        }
        let mut triangles = Vec::new();
        for z in 0..size - 1 {
            for x in 0..size - 1 {
                let i = (z * size + x) as u32;
                let right = i + 1;
                let down = i + size as u32;
                let diagonal = down + 1;
                triangles.push([i, right, diagonal]);
                triangles.push([i, diagonal, down]);
            }
        }
        (vertices, triangles)
    }

    fn collect_leaf_triangles(nodes: &[BvhNode], order: &[u32], node: usize, out: &mut Vec<u32>) {
        let entry = nodes[node];
        match entry.children {
            Some((left, right)) => {
                collect_leaf_triangles(nodes, order, left as usize, out);
                collect_leaf_triangles(nodes, order, right as usize, out);
            }
            None => {
                out.extend_from_slice(
                    &order[entry.first as usize..(entry.first as usize + entry.count as usize)],
                );
            }
        }
    }

    #[test]
    fn quad_mesh_aabb_and_leaves() {
        let (vertices, triangles) = quad();
        let mesh = MeshShape::new(vertices, triangles);
        assert!(mesh.aabb().is_valid());
        assert!(mesh.aabb().min.all_equal3(Vector4::ZERO, 0.0));
        assert!(
            mesh.aabb()
                .max
                .all_equal3(Vector4::from_xyz(1.0, 1.0, 0.0), 0.0)
        );
        assert_eq!(mesh.volume(), None);
        assert_eq!(mesh.bvh_nodes().len(), 1, "two triangles fit in one leaf");

        let mut leaves = Vec::new();
        collect_leaf_triangles(mesh.bvh_nodes(), mesh.triangle_order(), 0, &mut leaves);
        leaves.sort_unstable();
        assert_eq!(leaves, vec![0, 1]);
        assert_eq!(mesh.node_triangles(0).len(), 2);
    }

    #[test]
    fn grid_mesh_bvh_covers_every_triangle_once() {
        let (vertices, triangles) = grid(10);
        let triangle_count = triangles.len();
        let mesh = MeshShape::new(vertices, triangles);

        let mut leaves = Vec::new();
        collect_leaf_triangles(mesh.bvh_nodes(), mesh.triangle_order(), 0, &mut leaves);
        assert_eq!(leaves.len(), triangle_count);
        leaves.sort_unstable();
        for (expected, actual) in (0..triangle_count as u32).zip(leaves) {
            assert_eq!(actual, expected, "each triangle appears exactly once");
        }

        for node in mesh.bvh_nodes() {
            if node.is_leaf() {
                assert!(node.count <= LEAF_TRIANGLES as u32);
            } else {
                let (left, right) = node.children.unwrap();
                assert!(left < mesh.bvh_nodes().len() as u32);
                assert!(right < mesh.bvh_nodes().len() as u32);
                assert!(node.aabb.contains(&mesh.bvh_nodes()[left as usize].aabb));
                assert!(node.aabb.contains(&mesh.bvh_nodes()[right as usize].aabb));
            }
        }
    }

    #[test]
    fn root_aabb_equals_mesh_aabb() {
        let (vertices, triangles) = grid(5);
        let mesh = MeshShape::new(vertices, triangles);
        assert!(mesh.bvh_nodes()[0].aabb.is_equal(&mesh.aabb()));
    }

    #[test]
    fn empty_triangle_mesh_has_no_bvh() {
        let mesh = MeshShape::new(
            vec![Vector4::ZERO, Vector4::from_xyz(1.0, 1.0, 1.0)],
            Vec::new(),
        );
        assert!(mesh.bvh_nodes().is_empty());
        assert_eq!(mesh.triangle_order().len(), 0);
    }
}
