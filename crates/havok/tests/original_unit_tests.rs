//! Ports/mappings of the Rust-relevant original `Physics/Test/UnitTest` cases.
//!
//! See `docs/phase9-test-matrix.md` for the full 41-case status table. The tests
//! here correspond to the `Ported`/`Partial` rows that the current port can
//! express; blocked cases are documented instead of being faked.

use std::sync::Arc;

use havok::body::BodyCinfo;
use havok::math::Vector4;
use havok::np_types::MotionType;
use havok::shape::{
    BoxShape, CapsuleShape, ConvexHullShape, HeightFieldShape, MeshShape, Shape, SphereShape,
};
use havok::world::{World, WorldCinfo};

fn sphere(radius: f32) -> Arc<dyn Shape> {
    Arc::new(SphereShape::new(Vector4::ZERO, radius)) as Arc<dyn Shape>
}

#[test]
fn np_body_allocation_test() {
    // Port of NpBodyAllocationTest's allocation/destruction/reuse core.
    // The SDK also supports a desired hknpBodyIndex; that explicit placement is
    // not ported yet and is documented as a deviation.
    let mut world = World::new(WorldCinfo::new());
    let first = world.create_body(&BodyCinfo::new(sphere(1.0)));
    assert!(first.is_valid());
    assert!(world.destroy_body(first));
    assert!(world.body(first).is_none());

    let second = world.create_body(&BodyCinfo::new(sphere(1.0)));
    assert_ne!(first, second, "slot reuse must bump the serial");
    assert!(world.body(second).is_some());
    assert!(
        world.body(first).is_none(),
        "stale id must not alias the reused slot"
    );
    assert_eq!(world.body_count(), 1);

    let third = world.create_body(&BodyCinfo::new(sphere(1.0)));
    assert_ne!(second, third);
    assert!(world.destroy_body(second));
    assert!(world.destroy_body(third));
    assert_eq!(world.body_count(), 0);
}

#[test]
fn np_box_shape_unit_test() {
    // Port of the AABB/support/volume part of NpBoxShapeUnitTest.
    // shrinkByRadius and makeFromVertices are not ported; the test documents
    // the current convex-radius semantics instead.
    let half = Vector4::from_xyz(1.0, 2.0, 3.0);
    let box_shape = BoxShape::with_convex_radius(half, 0.0);
    let aabb = box_shape.aabb();
    assert!(
        aabb.min
            .all_equal3(Vector4::from_xyz(-1.0, -2.0, -3.0), 0.0)
    );
    assert!(aabb.max.all_equal3(Vector4::from_xyz(1.0, 2.0, 3.0), 0.0));
    assert!((box_shape.volume().unwrap() - 48.0).abs() < 1e-5);
    assert_eq!(
        box_shape
            .support(Vector4::from_xyz(1.0, -1.0, 1.0))
            .unwrap(),
        Vector4::from_xyz(1.0, -2.0, 3.0)
    );

    let inflated = BoxShape::with_convex_radius(half, 0.5);
    let inflated_aabb = inflated.aabb();
    assert!(
        inflated_aabb
            .min
            .all_equal3(Vector4::from_xyz(-1.5, -2.5, -3.5), 1e-6)
    );
    assert!(
        inflated_aabb
            .max
            .all_equal3(Vector4::from_xyz(1.5, 2.5, 3.5), 1e-6)
    );
}

fn assert_bounding_radius_contains_aabb(shape: &dyn Shape) {
    let aabb = shape.aabb();
    let radius = shape.bounding_radius();
    for x in [aabb.min.x, aabb.max.x] {
        for y in [aabb.min.y, aabb.max.y] {
            for z in [aabb.min.z, aabb.max.z] {
                let corner = Vector4::from_xyz(x, y, z);
                assert!(
                    corner.length3() <= radius + 1e-5,
                    "corner {:?} outside bounding radius {radius}",
                    corner
                );
            }
        }
    }
}

#[test]
fn np_bounding_sphere_test() {
    // Port of NpBoundingSphereTest for the convex shapes currently implemented.
    // Scaled convex and compound bounding spheres are blocked by missing
    // shape variants; see the matrix.
    assert_bounding_radius_contains_aabb(&SphereShape::new(Vector4::from_xyz(1.0, 2.0, 3.0), 2.0));
    assert_bounding_radius_contains_aabb(&BoxShape::with_convex_radius(
        Vector4::from_xyz(1.0, 2.0, 3.0),
        0.1,
    ));
    assert_bounding_radius_contains_aabb(&CapsuleShape::new(
        Vector4::from_xyz(0.0, -1.0, 0.0),
        Vector4::from_xyz(0.0, 1.0, 0.0),
        0.5,
    ));
    let hull = ConvexHullShape::new(
        vec![
            Vector4::from_xyz(-1.0, -1.0, -1.0),
            Vector4::from_xyz(1.0, -1.0, -1.0),
            Vector4::from_xyz(1.0, 1.0, -1.0),
            Vector4::from_xyz(-1.0, 1.0, -1.0),
            Vector4::from_xyz(-1.0, -1.0, 1.0),
            Vector4::from_xyz(1.0, -1.0, 1.0),
            Vector4::from_xyz(1.0, 1.0, 1.0),
            Vector4::from_xyz(-1.0, 1.0, 1.0),
        ],
        Vec::new(),
        0.0,
    );
    assert_bounding_radius_contains_aabb(&hull);
}

#[test]
fn np_mesh_shape_unit_test_structural() {
    // Partial port of NpMeshShapeUnitTest: BVH/AABB/triangle coverage.
    // Shape-key/leaf-shape queries are not ported (matrix row 24).
    let size = 5usize;
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
            triangles.push([i, i + 1, i + size as u32 + 1]);
            triangles.push([i, i + size as u32 + 1, i + size as u32]);
        }
    }
    let triangle_count = triangles.len();
    let mesh = MeshShape::new(vertices, triangles);

    assert!(mesh.aabb().is_valid());
    let nodes = mesh.bvh_nodes();
    assert!(!nodes.is_empty());
    assert!(nodes[0].aabb.is_equal(&mesh.aabb()));

    let mut leaf_triangles = Vec::new();
    collect_leaf_triangles(nodes, mesh.triangle_order(), 0, &mut leaf_triangles);
    leaf_triangles.sort_unstable();
    assert_eq!(leaf_triangles.len(), triangle_count);
    for (expected, actual) in (0..triangle_count as u32).zip(leaf_triangles) {
        assert_eq!(expected, actual, "each triangle appears exactly once");
    }
}

fn collect_leaf_triangles(
    nodes: &[havok::shape::BvhNode],
    order: &[u32],
    node: usize,
    output: &mut Vec<u32>,
) {
    let entry = nodes[node];
    match entry.children {
        Some((left, right)) => {
            collect_leaf_triangles(nodes, order, left as usize, output);
            collect_leaf_triangles(nodes, order, right as usize, output);
        }
        None => output.extend_from_slice(
            &order[entry.first as usize..(entry.first as usize + entry.count as usize)],
        ),
    }
}

#[test]
fn np_height_field_shape_unit_test_structural() {
    // Partial port of NpHeightFieldShapeUnitTest: samples/AABB/scale.
    // Shape-key iteration and min/max range queries are not ported.
    let heightfield = HeightFieldShape::new(
        3,
        3,
        Vector4::new(1.0, 1.0, 1.0, 1.0),
        vec![0.0, 1.0, 2.0, 3.0, 8.0, 5.0, 6.0, 7.0, 4.0],
    )
    .expect("height field");
    assert_eq!(heightfield.height(1, 1), 8.0);
    assert_eq!(heightfield.vertex(1, 1), Vector4::from_xyz(1.0, 8.0, 1.0));
    assert_eq!(heightfield.int_size_x(), 3);
    assert_eq!(heightfield.int_size_z(), 3);
    let aabb = heightfield.aabb();
    assert!(aabb.min.all_equal3(Vector4::ZERO, 0.0));
    assert!(aabb.max.all_equal3(Vector4::from_xyz(2.0, 8.0, 2.0), 0.0));
}

#[test]
fn np_many_bodies_test() {
    // Port of the body-count lifecycle part of NpManyBodiesTest.
    // Original also checks capacity and broadphase counts; the port tests
    // creation/step/destruction scale instead.
    let mut world = World::new(WorldCinfo::new());
    let mut static_ids = Vec::new();
    for index in 0..100 {
        let x = (index % 10) as f32 * 2.0;
        let z = (index / 10) as f32 * 2.0;
        static_ids.push(
            world.create_body(
                &BodyCinfo::new(sphere(0.5)).with_position(Vector4::from_xyz(x, 0.0, z)),
            ),
        );
    }
    let mut dynamic_ids = Vec::new();
    for index in 0..10 {
        dynamic_ids.push(
            world.create_body(
                &BodyCinfo::new(sphere(0.25))
                    .with_motion_type(MotionType::Dynamic)
                    .with_mass(1.0)
                    .with_position(Vector4::from_xyz(index as f32 * 3.0, 10.0, 0.0)),
            ),
        );
    }
    assert_eq!(world.body_count(), 110);
    for _ in 0..10 {
        world.step(1.0 / 60.0);
    }
    assert_eq!(world.body_count(), 110);
    for id in dynamic_ids {
        assert!(world.destroy_body(id));
    }
    assert_eq!(world.body_count(), 100);
    for id in static_ids {
        assert!(world.destroy_body(id));
    }
    assert_eq!(world.body_count(), 0);
}
