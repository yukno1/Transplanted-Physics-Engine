//! Phase 9 benchmark harness (no `criterion`; crates.io is unreachable for this
//! workspace). Run:
//!
//! ```text
//! cargo bench -p havok --bench simulation -- --quick
//! cargo bench -p havok --bench simulation
//! ```

use std::env;
use std::hint::black_box;
use std::sync::Arc;
use std::time::Instant;

use havok::body::BodyCinfo;
use havok::collide::{BodyAabb, BroadPhase, BroadPhaseConfig, closest_points, collide};
use havok::math::{Aabb, Quaternion, Transform, Vector4};
use havok::np_types::{BodyId, MotionType};
use havok::shape::{BoxShape, Shape, SphereShape};
use havok::solver::{ContactConstraint, Solver, SolverBody};
use havok::world::{World, WorldCinfo};

fn main() {
    let quick = env::args().any(|argument| argument == "--quick");

    let broadphase_count = if quick { 64 } else { 512 };
    let broadphase_iterations = if quick { 3 } else { 10 };
    let gjk_iterations = if quick { 200 } else { 2000 };
    let narrowphase_iterations = if quick { 200 } else { 2000 };
    let solver_iterations = if quick { 5 } else { 20 };
    let fall_bodies = if quick { 16 } else { 64 };
    let fall_steps = if quick { 10 } else { 60 };
    let stack_steps = if quick { 30 } else { 120 };

    println!("# hvk-bench v1 quick={quick}");
    println!("# name\tvalue\tunit\tdetail");

    benchmark_broadphase(broadphase_count, broadphase_iterations);
    benchmark_gjk(gjk_iterations);
    benchmark_narrowphase(narrowphase_iterations);
    benchmark_solver(solver_iterations);
    benchmark_world_fall(fall_bodies, fall_steps);
    benchmark_world_stack(stack_steps);
}

fn benchmark_broadphase(count: usize, iterations: u32) {
    let bodies: Vec<BodyAabb> = (0..count)
        .map(|index| {
            let x = (index % 32) as f32 * 0.9;
            let z = (index / 32) as f32 * 0.9;
            BodyAabb::new(
                BodyId::new(index as u64 + 1),
                Aabb::new(
                    Vector4::from_xyz(x - 0.5, -0.5, z - 0.5),
                    Vector4::from_xyz(x + 0.5, 0.5, z + 0.5),
                ),
            )
        })
        .collect();
    let broadphase = BroadPhase::new(BroadPhaseConfig::default());
    let mut pairs = 0;
    let elapsed = measure(iterations, || {
        let result = broadphase.find_pairs(black_box(&bodies));
        pairs = result.len();
        pairs
    });
    report(
        "broadphase_grid",
        elapsed,
        iterations,
        "ms/iter",
        format!("bodies={count} pairs={pairs}"),
    );
}

fn benchmark_gjk(iterations: u32) {
    let shape = Arc::new(SphereShape::new(Vector4::ZERO, 1.0)) as Arc<dyn Shape>;
    let left = Transform::IDENTITY;
    let right = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(5.0, 0.0, 0.0));
    let elapsed = measure(iterations, || {
        let result = closest_points(shape.as_ref(), &left, shape.as_ref(), &right).unwrap();
        black_box(result.distance);
        1
    });
    report(
        "gjk_sphere_pair",
        elapsed,
        iterations,
        "ms/iter",
        String::from("separated"),
    );

    let box_shape =
        Arc::new(BoxShape::with_convex_radius(Vector4::splat(1.0), 0.0)) as Arc<dyn Shape>;
    let elapsed = measure(iterations, || {
        let result = closest_points(box_shape.as_ref(), &left, box_shape.as_ref(), &right).unwrap();
        black_box(result.distance);
        1
    });
    report(
        "gjk_box_pair",
        elapsed,
        iterations,
        "ms/iter",
        String::from("separated"),
    );
}

fn benchmark_narrowphase(iterations: u32) {
    let shape = Arc::new(BoxShape::with_convex_radius(Vector4::splat(1.0), 0.0)) as Arc<dyn Shape>;
    let left = Transform::IDENTITY;
    let right = Transform::from_quaternion(Quaternion::IDENTITY, Vector4::from_xyz(1.5, 0.0, 0.0));
    let elapsed = measure(iterations, || {
        let manifold = collide(shape.as_ref(), &left, shape.as_ref(), &right).unwrap();
        black_box(manifold.points[0].distance);
        manifold.num_points()
    });
    report(
        "narrowphase_box_pairs",
        elapsed,
        iterations,
        "ms/iter",
        String::from("overlapping"),
    );
}

fn benchmark_solver(iterations: u32) {
    let bodies = vec![
        SolverBody::new(
            1.0,
            Vector4::ZERO,
            Vector4::from_xyz(0.0, 1.0, 0.0),
            Vector4::from_xyz(0.0, -1.0, 0.0),
            Vector4::ZERO,
        ),
        SolverBody::static_body(Vector4::ZERO),
    ];
    let contacts = vec![ContactConstraint::new(
        0,
        1,
        Vector4::UNIT_Y,
        Vector4::from_xyz(0.0, 0.5, 0.0),
        0.1,
        0.5,
    )];
    let solver = Solver::default();
    let elapsed = measure(iterations, || {
        let mut working = bodies.clone();
        let stats = solver.solve(&mut working, &contacts, 1.0 / 60.0);
        black_box(working[0].linear_velocity.y);
        stats.contacts
    });
    report(
        "solver_contact_pair",
        elapsed,
        iterations,
        "ms/iter",
        String::from("2 bodies, 1 contact, 4 iterations"),
    );
}

fn benchmark_world_fall(body_count: usize, steps: usize) {
    let mut world = World::new(WorldCinfo::new());
    let ground_shape = Arc::new(BoxShape::with_convex_radius(
        Vector4::from_xyz(6.0, 0.5, 6.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(ground_shape).with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
    );
    let sphere = Arc::new(SphereShape::new(Vector4::ZERO, 0.5)) as Arc<dyn Shape>;
    for index in 0..body_count {
        let x = (index % 16) as f32 * 0.7 - 5.0;
        let z = (index / 16) as f32 * 0.7 - 2.0;
        world.create_body(
            &BodyCinfo::new(Arc::clone(&sphere))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(x, 5.0, z)),
        );
    }
    let start = Instant::now();
    for _ in 0..steps {
        world.step(1.0 / 60.0);
    }
    let elapsed = start.elapsed();
    let contacts = world.last_contact_count();
    report(
        "world_fall_n",
        elapsed,
        steps as u32,
        "ms/step",
        format!("bodies={body_count} contacts={contacts}"),
    );
}

fn benchmark_world_stack(steps: usize) {
    let mut world = World::new(WorldCinfo::new());
    let ground_shape = Arc::new(BoxShape::with_convex_radius(
        Vector4::from_xyz(4.0, 0.5, 4.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(ground_shape).with_position(Vector4::from_xyz(0.0, -0.5, 0.0)),
    );
    let box_shape =
        Arc::new(BoxShape::with_convex_radius(Vector4::splat(0.5), 0.0)) as Arc<dyn Shape>;
    for index in 0..10 {
        world.create_body(
            &BodyCinfo::new(Arc::clone(&box_shape))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vector4::from_xyz(0.0, 1.0 + index as f32 * 1.2, 0.0)),
        );
    }
    let start = Instant::now();
    for _ in 0..steps {
        world.step(1.0 / 60.0);
    }
    let elapsed = start.elapsed();
    let contacts = world.last_contact_count();
    report(
        "world_stack_10",
        elapsed,
        steps as u32,
        "ms/step",
        format!("bodies=10 contacts={contacts}"),
    );
}

fn measure(iterations: u32, mut function: impl FnMut() -> usize) -> std::time::Duration {
    // Warm up once to keep allocator/page faults out of the measured loop.
    black_box(function());
    let start = Instant::now();
    for _ in 0..iterations {
        black_box(function());
    }
    start.elapsed()
}

fn report(name: &str, elapsed: std::time::Duration, units: u32, unit: &str, detail: String) {
    let per_unit = elapsed.as_secs_f64() * 1000.0 / f64::from(units.max(1));
    println!("{name}\t{per_unit:.3}\t{unit}\t{detail}");
}
