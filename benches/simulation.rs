//! Phase 9 benchmark harness (no `criterion`; crates.io is unreachable for this
//! workspace). Run:
//!
//! ```text
//! cargo bench -p proton-engine --bench simulation -- --quick
//! cargo bench -p proton-engine --bench simulation
//! ```

use std::env;
use std::hint::black_box;
use std::sync::Arc;
use std::time::Instant;

use proton::body::BodyCinfo;
use proton::collide::{BodyAabb, BroadPhase, BroadPhaseConfig, closest_points, collide};
use proton::math::{Aabb, Matrix3, Quaternion, Transform, Vector4};
use proton::np_types::{BodyId, MotionType};
use proton::math::{Vec4, Vec4Ext};
use proton::shape::{BoxShape, Shape, SphereShape};
use proton::solver::{ContactConstraint, Solver, SolverBody};
use proton::thread::ThreadPool;
use proton::world::{ExecutorKind, SchedulerConfig, World, WorldCinfo};

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
    let query_iterations = if quick { 200 } else { 2000 };
    let joint_bodies = if quick { 8 } else { 24 };
    let joint_steps = if quick { 10 } else { 60 };
    let math_iterations = if quick { 100_000 } else { 2_000_000 };

    println!("# hvk-bench v1 quick={quick}");
    println!("# name\tvalue\tunit\tdetail");

    benchmark_math_micro(math_iterations);
    benchmark_broadphase(broadphase_count, broadphase_iterations);
    benchmark_gjk(gjk_iterations);
    benchmark_narrowphase(narrowphase_iterations);
    benchmark_solver(solver_iterations);
    benchmark_world_fall(fall_bodies, fall_steps, 1, "world_fall_n");
    benchmark_world_fall(fall_bodies, fall_steps, 4, "world_fall_n_parallel");
    benchmark_world_fall_persistent_pool(fall_bodies, fall_steps, 4);
    benchmark_world_fall_scheduler(
        fall_bodies,
        fall_steps,
        4,
        SchedulerConfig::new(ExecutorKind::Pool, true),
        "world_fall_n_priority_pool",
    );
    benchmark_world_fall_scheduler(
        fall_bodies,
        fall_steps,
        4,
        SchedulerConfig::new(ExecutorKind::WorkStealing, false),
        "world_fall_n_work_stealing",
    );
    benchmark_world_fall_scheduler(
        fall_bodies,
        fall_steps,
        4,
        SchedulerConfig::new(ExecutorKind::WorkStealing, true),
        "world_fall_n_priority_work_stealing",
    );
    benchmark_world_fall_scheduler(
        fall_bodies,
        fall_steps,
        4,
        SchedulerConfig::new(ExecutorKind::Pool, false).with_parallel_contact_rows(true),
        "world_fall_n_parallel_contact_rows",
    );
    benchmark_world_fall_scheduler(
        fall_bodies,
        fall_steps,
        4,
        SchedulerConfig::new(ExecutorKind::Pool, false).with_parallel_writeback(true),
        "world_fall_n_parallel_writeback",
    );
    benchmark_world_fall_scheduler(
        fall_bodies,
        fall_steps,
        4,
        SchedulerConfig::new(ExecutorKind::WorkStealing, false).with_stage_parallelism(true, true),
        "world_fall_n_parallel_stages",
    );
    benchmark_world_stack(stack_steps);
    benchmark_raycast_world(query_iterations);
    benchmark_shape_cast_world(query_iterations);
    benchmark_joint_chain(joint_bodies, joint_steps);
}

fn benchmark_math_micro(iterations: u32) {
    let mut counter = 0u32;
    macro_rules! inputs {
        () => {{
            counter = counter.wrapping_add(1);
            let jitter = (counter & 0xff) as f32 * 1e-4;
            (
                black_box(Vector4::from_xyz(1.25 + jitter, -2.5, 0.75)),
                black_box(Vector4::from_xyz(3.5 - jitter, 0.125, -4.0)),
            )
        }};
    }

    let elapsed = measure(iterations, || {
        let (lhs, rhs) = inputs!();
        let result = lhs + rhs;
        black_box(result.x.to_bits() as usize)
    });
    report_us(
        "math_vector4_add",
        elapsed,
        iterations,
        format!("iterations={iterations}"),
    );

    let elapsed = measure(iterations, || {
        let (lhs, rhs) = inputs!();
        let result = lhs * rhs;
        black_box(result.z.to_bits() as usize)
    });
    report_us(
        "math_vector4_mul",
        elapsed,
        iterations,
        format!("iterations={iterations}"),
    );

    let elapsed = measure(iterations, || {
        let (lhs, rhs) = inputs!();
        black_box(lhs.dot3(rhs).to_bits() as usize)
    });
    report_us(
        "math_vector4_dot3",
        elapsed,
        iterations,
        format!("iterations={iterations}"),
    );

    let elapsed = measure(iterations, || {
        let (lhs, _) = inputs!();
        let result = lhs.normalized3();
        black_box(result.x.to_bits() as usize)
    });
    report_us(
        "math_vector4_normalize3",
        elapsed,
        iterations,
        format!("iterations={iterations}"),
    );

    let matrix = black_box(Matrix3::from_cols(
        Vector4::from_xyz(1.0, 0.25, -0.5),
        Vector4::from_xyz(-0.25, 1.0, 0.125),
        Vector4::from_xyz(0.5, -0.125, 1.0),
    ));
    let elapsed = measure(iterations, || {
        let (lhs, _) = inputs!();
        let q0 = Quaternion::new(lhs.x * 0.1, lhs.y * 0.1, lhs.z * 0.1, 0.9);
        let q1 = Quaternion::new(-0.4, 0.5, 0.2, 0.8);
        let result = q0.mul(black_box(q1));
        black_box(result.w.to_bits() as usize)
    });
    report_us(
        "math_quaternion_mul",
        elapsed,
        iterations,
        format!("iterations={iterations}"),
    );

    let elapsed = measure(iterations, || {
        let (lhs, _) = inputs!();
        let result = matrix.mul_vec(lhs);
        black_box(result.y.to_bits() as usize)
    });
    report_us(
        "math_matrix3_mul_vec",
        elapsed,
        iterations,
        format!("iterations={iterations}"),
    );

    let elapsed = measure(iterations, || {
        let (lhs, rhs) = inputs!();
        let mut aabb = Aabb::new(Vector4::splat(-1.0), Vector4::splat(1.0));
        aabb.include_point(lhs);
        black_box(aabb.contains_point(rhs) as usize)
    });
    report_us(
        "math_aabb_include_point",
        elapsed,
        iterations,
        format!("iterations={iterations}"),
    );
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
    let shape = Arc::new(SphereShape::new(Vec4::ZERO, 1.0)) as Arc<dyn Shape>;
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
        Arc::new(BoxShape::with_convex_radius(Vec4::splat(1.0), 0.0)) as Arc<dyn Shape>;
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
    let shape = Arc::new(BoxShape::with_convex_radius(Vec4::splat(1.0), 0.0)) as Arc<dyn Shape>;
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
            Vec4::ZERO,
            Vec4::from_xyz(0.0, 1.0, 0.0),
            Vec4::from_xyz(0.0, -1.0, 0.0),
            Vec4::ZERO,
        ),
        SolverBody::static_body(Vec4::ZERO),
    ];
    let contacts = vec![ContactConstraint::new(
        0,
        1,
        Vec4::Y,
        Vec4::from_xyz(0.0, 0.5, 0.0),
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

fn build_fall_world(body_count: usize) -> World {
    let mut world = World::new(WorldCinfo::new());
    let ground_shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(6.0, 0.5, 6.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(ground_shape).with_position(Vec4::from_xyz(0.0, -0.5, 0.0)),
    );
    let sphere = Arc::new(SphereShape::new(Vec4::ZERO, 0.5)) as Arc<dyn Shape>;
    for index in 0..body_count {
        let x = (index % 16) as f32 * 0.7 - 5.0;
        let z = (index / 16) as f32 * 0.7 - 2.0;
        world.create_body(
            &BodyCinfo::new(Arc::clone(&sphere))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vec4::from_xyz(x, 5.0, z)),
        );
    }
    world
}

fn benchmark_world_fall(body_count: usize, steps: usize, threads: usize, name: &str) {
    let mut world = build_fall_world(body_count);
    let start = Instant::now();
    for _ in 0..steps {
        if threads > 1 {
            world.step_with_threads(1.0 / 60.0, threads);
        } else {
            world.step(1.0 / 60.0);
        }
    }
    let elapsed = start.elapsed();
    let contacts = world.last_contact_count();
    report(
        name,
        elapsed,
        steps as u32,
        "ms/step",
        format!("bodies={body_count} contacts={contacts}"),
    );
}

fn benchmark_world_fall_persistent_pool(body_count: usize, steps: usize, threads: usize) {
    let mut world = build_fall_world(body_count);
    world.set_thread_pool(Arc::new(ThreadPool::with_name("hk-bench", threads)));
    let start = Instant::now();
    for _ in 0..steps {
        world.step_with_threads(1.0 / 60.0, threads);
    }
    let elapsed = start.elapsed();
    let contacts = world.last_contact_count();
    report(
        "world_fall_n_persistent_pool",
        elapsed,
        steps as u32,
        "ms/step",
        format!("bodies={body_count} contacts={contacts} threads={threads} task-graph"),
    );
}

fn benchmark_world_fall_scheduler(
    body_count: usize,
    steps: usize,
    threads: usize,
    config: SchedulerConfig,
    name: &str,
) {
    let mut world = build_fall_world(body_count);
    world.set_scheduler_config(config);
    let start = Instant::now();
    for _ in 0..steps {
        world.step_with_scheduler(1.0 / 60.0, threads);
    }
    let elapsed = start.elapsed();
    let contacts = world.last_contact_count();
    report(
        name,
        elapsed,
        steps as u32,
        "ms/step",
        format!("bodies={body_count} contacts={contacts} threads={threads} {config:?}"),
    );
}

fn benchmark_world_stack(steps: usize) {
    let mut world = World::new(WorldCinfo::new());
    let ground_shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(4.0, 0.5, 4.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(ground_shape).with_position(Vec4::from_xyz(0.0, -0.5, 0.0)),
    );
    let box_shape =
        Arc::new(BoxShape::with_convex_radius(Vec4::splat(0.5), 0.0)) as Arc<dyn Shape>;
    for index in 0..10 {
        world.create_body(
            &BodyCinfo::new(Arc::clone(&box_shape))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vec4::from_xyz(0.0, 1.0 + index as f32 * 1.2, 0.0)),
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

fn benchmark_raycast_world(iterations: u32) {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vec4::ZERO));
    let ground_shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(6.0, 0.5, 6.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(ground_shape).with_position(Vec4::from_xyz(0.0, -0.5, 0.0)),
    );
    let sphere = Arc::new(SphereShape::new(Vec4::ZERO, 0.5)) as Arc<dyn Shape>;
    for index in 0..16 {
        let x = (index % 4) as f32 * 1.5 - 2.5;
        let z = (index / 4) as f32 * 1.5 - 2.5;
        world.create_body(
            &BodyCinfo::new(Arc::clone(&sphere))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vec4::from_xyz(x, 3.0, z)),
        );
    }
    let mut hits = 0;
    let elapsed = measure(iterations, || {
        let hit = world.raycast(Vec4::from_xyz(0.0, 20.0, 0.0), -Vec4::Y, 30.0);
        if hit.is_some() {
            hits += 1;
        }
        hits
    });
    report(
        "raycast_world",
        elapsed,
        iterations,
        "ms/iter",
        format!("bodies=17 hits={hits}"),
    );
}

fn benchmark_shape_cast_world(iterations: u32) {
    let mut world = World::new(WorldCinfo::new().with_gravity(Vec4::ZERO));
    let ground_shape = Arc::new(BoxShape::with_convex_radius(
        Vec4::from_xyz(6.0, 0.5, 6.0),
        0.0,
    )) as Arc<dyn Shape>;
    world.create_body(
        &BodyCinfo::new(ground_shape).with_position(Vec4::from_xyz(0.0, -0.5, 0.0)),
    );
    let query = Arc::new(SphereShape::new(Vec4::ZERO, 0.5)) as Arc<dyn Shape>;
    let start = Transform::from_quaternion(
        Quaternion::IDENTITY,
        proton::math::Vector4::from_xyz(0.0, 5.0, 0.0),
    );
    let filter = proton::world::QueryFilter::default();
    let mut hits = 0;
    let elapsed = measure(iterations, || {
        let hit = world.cast_shape(query.as_ref(), &start, -Vec4::Y, 10.0, &filter);
        if hit.is_some() {
            hits += 1;
        }
        hits
    });
    report(
        "shape_cast_world",
        elapsed,
        iterations,
        "ms/iter",
        format!("bodies=1 hits={hits}"),
    );
}

fn benchmark_joint_chain(body_count: usize, steps: usize) {
    let mut world = World::new(WorldCinfo::new());
    let sphere = Arc::new(SphereShape::new(Vec4::ZERO, 0.5)) as Arc<dyn Shape>;
    let anchor = world.create_body(
        &BodyCinfo::new(Arc::clone(&sphere)).with_position(Vec4::from_xyz(0.0, 10.0, 0.0)),
    );
    let mut previous = anchor;
    for index in 1..=body_count {
        let body = world.create_body(
            &BodyCinfo::new(Arc::clone(&sphere))
                .with_motion_type(MotionType::Dynamic)
                .with_mass(1.0)
                .with_position(Vec4::from_xyz(0.0, 10.0, index as f32)),
        );
        let motion = world.body(body).expect("live body").motion_id;
        world
            .motion_mut(motion)
            .expect("live motion")
            .inverse_inertia = Vec4::splat(0.5);
        world.create_ragdoll_constraint(
            &proton::world::RagdollCinfo::new(
                body,
                previous,
                Vec4::ZERO,
                Vec4::ZERO,
                Vec4::X,
                Vec4::X,
                Vec4::Z,
                Vec4::Z,
            )
            .with_limits(0.6, -0.3, 0.3),
        );
        previous = body;
    }
    let start = Instant::now();
    for _ in 0..steps {
        world.step(1.0 / 60.0);
    }
    let elapsed = start.elapsed();
    report(
        "joint_chain_ragdoll",
        elapsed,
        steps as u32,
        "ms/step",
        format!("bodies={} joints={body_count}", body_count + 1),
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

fn report_us(name: &str, elapsed: std::time::Duration, units: u32, detail: String) {
    let per_unit = elapsed.as_secs_f64() * 1_000_000.0 / f64::from(units.max(1));
    println!("{name}\t{per_unit:.4}\tus/iter\t{detail}");
}
