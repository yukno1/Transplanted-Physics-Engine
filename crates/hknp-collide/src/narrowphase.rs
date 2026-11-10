//! Narrow phase: convex-convex contact generation.
//!
//! Port of the convex-convex path of `hknpConvexConvexCollisionDetector`
//! (`Collide/NarrowPhase/Detector/ConvexConvex/`). The SDK generates up to four
//! contact points via face clipping; this phase generates one point per pair:
//! analytic sphere-sphere contacts plus GJK/EPA for the general convex case.
//! Multi-point/clipped manifolds are deferred to the solver phase.
//!
//! Conventions follow [`crate::Manifold`]: manifold normal points from B to A,
//! contact positions lie on B, and negative distance means penetration.

use hk_math::{Transform, Vector4};
use hknp_shape::{Shape, SphereShape};
use hknp_types::ShapeType;

use crate::epa;
use crate::gjk;
use crate::manifold::{ContactPoint, Manifold};
use crate::support::world_support;

/// Generate a contact manifold for a pair of shapes.
///
/// Returns `None` when the shapes are separated by more than `tolerance`, when
/// a shape has no convex support function (mesh/height field/compound), or when
/// the penetration estimate fails.
pub fn collide_with_tolerance(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
    tolerance: f32,
) -> Option<Manifold> {
    debug_assert!(tolerance >= 0.0 && tolerance.is_finite());
    if !shape_a.is_convex() || !shape_b.is_convex() {
        return None;
    }

    if let (Some(sphere_a), Some(sphere_b)) = (downcast_sphere(shape_a), downcast_sphere(shape_b)) {
        return sphere_sphere(sphere_a, transform_a, sphere_b, transform_b, tolerance);
    }

    let result = gjk::closest_points(shape_a, transform_a, shape_b, transform_b)?;

    if result.intersects {
        if let Some(penetration) =
            epa::penetration(shape_a, transform_a, shape_b, transform_b, &result.simplex)
        {
            // Reconstruct the signed distance from the EPA witnesses.
            let distance = penetration
                .normal_ba
                .dot3(penetration.point_a - penetration.point_b);
            if distance <= tolerance {
                return Some(Manifold::new(
                    penetration.normal_ba,
                    vec![ContactPoint {
                        position: penetration.point_b,
                        distance,
                    }],
                ));
            }
            return None;
        }
        return support_estimate(shape_a, transform_a, shape_b, transform_b, tolerance);
    }

    if result.distance <= tolerance {
        return Some(Manifold::new(
            result.normal,
            vec![ContactPoint {
                position: result.point_b,
                distance: result.distance,
            }],
        ));
    }
    None
}

/// Generate contacts with zero tolerance (touching or penetrating only).
pub fn collide(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
) -> Option<Manifold> {
    collide_with_tolerance(shape_a, transform_a, shape_b, transform_b, 0.0)
}

fn downcast_sphere(shape: &dyn Shape) -> Option<&SphereShape> {
    if shape.shape_type() == ShapeType::Sphere {
        shape.as_any().downcast_ref::<SphereShape>()
    } else {
        None
    }
}

fn sphere_sphere(
    sphere_a: &SphereShape,
    transform_a: &Transform,
    sphere_b: &SphereShape,
    transform_b: &Transform,
    tolerance: f32,
) -> Option<Manifold> {
    let center_a = transform_a.transform_point(sphere_a.center);
    let center_b = transform_b.transform_point(sphere_b.center);
    let delta = center_a - center_b;
    let center_distance = delta.length3();
    let normal = if center_distance > 1e-8 {
        delta / center_distance
    } else {
        Vector4::UNIT_X
    };
    let separation = center_distance - sphere_a.radius - sphere_b.radius;
    if separation > tolerance {
        return None;
    }
    let position = center_b + normal * sphere_b.radius;
    Some(Manifold::new(
        normal,
        vec![ContactPoint {
            position,
            distance: separation,
        }],
    ))
}

/// Fallback used when GJK reports intersection but EPA has no full tetrahedron.
///
/// Uses the center direction and the extreme support points facing each other.
fn support_estimate(
    shape_a: &dyn Shape,
    transform_a: &Transform,
    shape_b: &dyn Shape,
    transform_b: &Transform,
    tolerance: f32,
) -> Option<Manifold> {
    let center_direction = transform_a.translation - transform_b.translation;
    let normal = if center_direction.length3() > 1e-8 {
        center_direction.normalized3()
    } else {
        Vector4::UNIT_X
    };
    // Support points facing each other: A along -normal, B along +normal.
    let point_a = world_support(shape_a, transform_a, -normal)?;
    let point_b = world_support(shape_b, transform_b, normal)?;
    let distance = normal.dot3(point_a - point_b);
    if distance > tolerance {
        return None;
    }
    Some(Manifold::new(
        normal,
        vec![ContactPoint {
            position: point_b,
            distance,
        }],
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::Quaternion;
    use hknp_shape::{BoxShape, CapsuleShape};
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
    fn overlapping_spheres_generate_analytic_contact() {
        let a = sphere(1.0);
        let b = sphere(1.0);
        let manifold = collide(
            a.as_ref(),
            &Transform::IDENTITY,
            b.as_ref(),
            &translation(1.5, 0.0, 0.0),
        )
        .expect("contact");
        assert_eq!(manifold.num_points(), 1);
        assert!((manifold.normal - Vector4::from_xyz(-1.0, 0.0, 0.0)).length3() < 1e-5);
        let point = manifold.points[0];
        assert!((point.position.x - 0.5).abs() < 1e-5);
        assert!((point.distance + 0.5).abs() < 1e-5);
        assert!(
            manifold
                .point_on_a(0)
                .unwrap()
                .all_equal3(Vector4::from_xyz(1.0, 0.0, 0.0), 1e-5)
        );
    }

    #[test]
    fn separated_spheres_have_no_contact_within_zero_tolerance() {
        let a = sphere(1.0);
        let b = sphere(1.0);
        assert!(
            collide(
                a.as_ref(),
                &Transform::IDENTITY,
                b.as_ref(),
                &translation(3.0, 0.0, 0.0),
            )
            .is_none()
        );
        let manifold = collide_with_tolerance(
            a.as_ref(),
            &Transform::IDENTITY,
            b.as_ref(),
            &translation(3.0, 0.0, 0.0),
            1.0,
        )
        .expect("speculative contact");
        assert_eq!(manifold.points[0].distance, 1.0);
    }

    #[test]
    fn overlapping_boxes_generate_contact() {
        let a = box_shape(1.0);
        let b = box_shape(1.0);
        let manifold = collide(
            a.as_ref(),
            &Transform::IDENTITY,
            b.as_ref(),
            &translation(1.5, 0.0, 0.0),
        )
        .expect("box contact");
        assert_eq!(manifold.num_points(), 1);
        let normal = manifold.normal;
        assert!((normal.x + 1.0).abs() < 1e-2, "normal {normal:?}");
        assert!((manifold.points[0].distance + 0.5).abs() < 1e-2);
    }

    #[test]
    fn separated_boxes_respect_tolerance() {
        let a = box_shape(1.0);
        let b = box_shape(1.0);
        // Gap of 1.0 along X.
        assert!(
            collide(
                a.as_ref(),
                &Transform::IDENTITY,
                b.as_ref(),
                &translation(3.0, 0.0, 0.0),
            )
            .is_none()
        );
        let manifold = collide_with_tolerance(
            a.as_ref(),
            &Transform::IDENTITY,
            b.as_ref(),
            &translation(3.0, 0.0, 0.0),
            1.5,
        )
        .expect("speculative contact");
        assert!((manifold.points[0].distance - 1.0).abs() < 1e-2);
    }

    #[test]
    fn capsules_collide() {
        let a = Arc::new(CapsuleShape::new(
            Vector4::from_xyz(0.0, -1.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            0.5,
        )) as Arc<dyn Shape>;
        let b = Arc::new(CapsuleShape::new(
            Vector4::from_xyz(0.0, -1.0, 0.0),
            Vector4::from_xyz(0.0, 1.0, 0.0),
            0.5,
        )) as Arc<dyn Shape>;
        let manifold = collide(
            a.as_ref(),
            &Transform::IDENTITY,
            b.as_ref(),
            &translation(0.5, 0.0, 0.0),
        )
        .expect("capsule contact");
        assert!((manifold.points[0].distance + 0.5).abs() < 1e-2);
    }

    #[test]
    fn non_convex_shapes_are_skipped() {
        use crate::gjk::tests_support::mesh_shape;
        let mesh = mesh_shape();
        assert!(
            collide(
                mesh.as_ref(),
                &Transform::IDENTITY,
                mesh.as_ref(),
                &Transform::IDENTITY,
            )
            .is_none()
        );
    }
}
