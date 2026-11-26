use std::sync::Arc;

use proton::body::BodyCinfo;
use proton::extensions::hash_world;
use proton::geometry::{WeldOptions, weld_vertices};
use proton::math::{Vec4, Vec4Ext};
use proton::np_types::MotionType;
use proton::shape::{ConvexHullShape, MassConfig, Shape};
use proton::world::{World, WorldCinfo};

fn cube_points(half: f32) -> Vec<Vec4> {
    vec![
        Vec4::from_xyz(-half, -half, -half),
        Vec4::from_xyz(half, -half, -half),
        Vec4::from_xyz(half, half, -half),
        Vec4::from_xyz(-half, half, -half),
        Vec4::from_xyz(-half, -half, half),
        Vec4::from_xyz(half, -half, half),
        Vec4::from_xyz(half, half, half),
        Vec4::from_xyz(-half, half, half),
    ]
}

#[test]
fn point_cloud_builds_volume_mass_and_convex_radius() {
    let mut points = cube_points(1.0);
    points.extend(cube_points(1.0));
    points.push(Vec4::ZERO);

    let hull = ConvexHullShape::from_points(points, 0.1);
    assert_eq!(hull.vertices().len(), 8);
    assert_eq!(hull.faces().len(), 12);
    assert!((hull.volume().unwrap() - 8.0).abs() < 1e-3);
    assert_eq!(hull.aabb().max.x, 1.1);
    assert_eq!(hull.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap().x, 1.1);

    let mass = hull.mass_properties(MassConfig::from_density(2.0)).unwrap();
    assert!((mass.volume - 8.0).abs() < 1e-3);
    assert!((mass.mass - 16.0).abs() < 1e-2);
}

#[test]
fn welding_remaps_triangles_and_removes_degenerate_faces() {
    let points = vec![
        Vec4::from_xyz(0.0, 0.0, 0.0),
        Vec4::from_xyz(1.0, 0.0, 0.0),
        Vec4::from_xyz(0.0, 1.0, 0.0),
        Vec4::from_xyz(1.0 + 1e-4, 0.0, 0.0),
    ];
    let triangles = vec![[0, 1, 2], [1, 3, 2]];
    let options = WeldOptions {
        keep_vertex_order: true,
        ..WeldOptions::default()
    };
    let result = weld_vertices(&points, &triangles, options).unwrap();

    assert_eq!(result.vertices.len(), 3);
    assert_eq!(result.triangles, vec![[0, 1, 2]]);
    assert_eq!(result.triangle_remap, vec![0, -1]);
}

#[test]
fn lower_dimensional_cloud_stays_queryable() {
    let hull =
        ConvexHullShape::from_points(vec![Vec4::ZERO, Vec4::from_xyz(2.0, 0.0, 0.0)], 0.0);
    assert_eq!(hull.volume(), None);
    assert_eq!(hull.support(Vec4::new(1.0, 0.0, 0.0, 0.0)).unwrap().x, 2.0);
}

fn falling_hull_world() -> World {
    let mut world = World::new(WorldCinfo::new());
    world.set_gravity(Vec4::from_xyz(0.0, -9.81, 0.0));
    let shape = ConvexHullShape::from_points(cube_points(0.5), 0.0);
    world.create_body(
        &BodyCinfo::new(Arc::new(shape) as Arc<dyn Shape>)
            .with_motion_type(MotionType::Dynamic)
            .with_mass(1.0)
            .with_position(Vec4::from_xyz(0.0, 10.0, 0.0)),
    );
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    world
}

#[test]
fn simulated_hull_world_is_bitwise_deterministic() {
    let first = falling_hull_world();
    let second = falling_hull_world();
    assert_eq!(hash_world(&first), hash_world(&second));
}
