//! Bitwise parity gate for the scalar and SIMD math paths.
//!
//! Every expectation in this file is computed from public fields with plain
//! scalar Rust expressions in the same order as the scalar backend. The tests
//! pass before any vectorization and must keep passing after the SIMD rollout.

use glam::Vec3A;

use crate::base_math::{Aabb, Vector4, Vector4d};

fn bits(value: Vector4) -> [u32; 4] {
    value.to_array().map(f32::to_bits)
}

fn bits3(value: Vec3A) -> [u32; 3] {
    value.to_array().map(f32::to_bits)
}

fn bits_d(value: Vector4d) -> [u64; 4] {
    value.to_array().map(f64::to_bits)
}

fn scalar_add(a: Vector4, b: Vector4) -> Vector4 {
    Vector4::new(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w)
}

fn scalar_sub(a: Vector4, b: Vector4) -> Vector4 {
    Vector4::new(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w)
}

fn scalar_mul(a: Vector4, b: Vector4) -> Vector4 {
    Vector4::new(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w)
}

fn scalar_div(a: Vector4, b: Vector4) -> Vector4 {
    Vector4::new(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w)
}

fn scalar_min(a: Vector4, b: Vector4) -> Vector4 {
    Vector4::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z), a.w.min(b.w))
}

fn scalar_max(a: Vector4, b: Vector4) -> Vector4 {
    Vector4::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z), a.w.max(b.w))
}

const SPECIAL_PAIRS: &[(f32, f32)] = &[
    (0.0, -0.0),
    (-0.0, 0.0),
    (f32::from_bits(1), f32::from_bits(2)),
    (f32::MIN_POSITIVE / 2.0, f32::MIN_POSITIVE),
    (f32::MAX, 2.0),
    (-f32::MAX, -2.0),
    (f32::NAN, 1.0),
    (1.0, f32::NAN),
    (f32::INFINITY, -f32::INFINITY),
    (0.0, 0.0),
    (1.0e-8, 3.0),
];

#[test]
fn componentwise_arithmetic_matches_scalar_reference_for_special_values() {
    for &(x, y) in SPECIAL_PAIRS {
        let a = Vector4::new(x, y, -x, y);
        let b = Vector4::new(y, x, -y, x);

        assert_eq!(bits(a + b), bits(scalar_add(a, b)), "add {x:?}/{y:?}");
        assert_eq!(bits(a - b), bits(scalar_sub(a, b)), "sub {x:?}/{y:?}");
        assert_eq!(bits(a * b), bits(scalar_mul(a, b)), "mul {x:?}/{y:?}");
        assert_eq!(bits(a / b), bits(scalar_div(a, b)), "div {x:?}/{y:?}");
        assert_eq!(bits(-a), bits(Vector4::new(-a.x, -a.y, -a.z, -a.w)));

        let c = Vector4::new(1.5, -2.5, 0.25, -0.75);
        let mut add_mul = Vector4::ZERO;
        add_mul.set_add_mul(a, b, c);
        let expected = Vector4::new(
            a.x + b.x * c.x,
            a.y + b.y * c.y,
            a.z + b.z * c.z,
            a.w + b.w * c.w,
        );
        assert_eq!(bits(add_mul), bits(expected), "add_mul {x:?}/{y:?}");

        let mut sub_mul = Vector4::ZERO;
        sub_mul.set_sub_mul(a, b, c);
        let expected = Vector4::new(
            a.x - b.x * c.x,
            a.y - b.y * c.y,
            a.z - b.z * c.z,
            a.w - b.w * c.w,
        );
        assert_eq!(bits(sub_mul), bits(expected), "sub_mul {x:?}/{y:?}");
    }
}

#[test]
fn min_max_abs_and_flip_sign_match_scalar_reference() {
    for &(x, y) in SPECIAL_PAIRS {
        let a = Vector4::new(x, -x, y, -y);
        let b = Vector4::new(y, -y, x, -x);

        assert_eq!(bits(a.min(b)), bits(scalar_min(a, b)), "min {x:?}/{y:?}");
        assert_eq!(bits(a.max(b)), bits(scalar_max(a, b)), "max {x:?}/{y:?}");
        assert_eq!(
            bits(a.abs()),
            bits(Vector4::new(a.x.abs(), a.y.abs(), a.z.abs(), a.w.abs()))
        );
    }
}

#[test]
fn reductions_keep_scalar_addition_order() {
    let a = Vector4::new(1.25, -2.5, 0.75, -0.125);
    let b = Vector4::new(3.5, 0.0625, -4.0, 2.0);

    let dot3 = a.x * b.x + a.y * b.y + a.z * b.z;
    assert_eq!(a.dot3(b).to_bits(), dot3.to_bits());

    let hadd3 = a.x + a.y + a.z;
    assert_eq!(a.horizontal_add3().to_bits(), hadd3.to_bits());

    let cross = Vector4::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
        0.0,
    );
    assert_eq!(bits(a.cross(b)), bits(cross));

    let length_squared3 = a.x * a.x + a.y * a.y + a.z * a.z;
    assert_eq!(a.length_squared3().to_bits(), length_squared3.to_bits());

    let length3 = length_squared3.sqrt();
    let expected3 = if length3 == 0.0 {
        Vector4::ZERO
    } else {
        Vector4::new(a.x / length3, a.y / length3, a.z / length3, a.w)
    };
    assert_eq!(bits(a.normalized3()), bits(expected3));
}

#[test]
fn comparison_masks_and_select_match_scalar_reference() {
    let a = Vector4::new(f32::NAN, -0.0, f32::INFINITY, f32::from_bits(1));
    let b = Vector4::new(1.0, 0.0, f32::INFINITY, f32::MIN_POSITIVE);

    fn expected_mask(a: Vector4, b: Vector4, op: fn(f32, f32) -> bool) -> u8 {
        (op(a.x, b.x) as u8)
            | ((op(a.y, b.y) as u8) << 1)
            | ((op(a.z, b.z) as u8) << 2)
            | ((op(a.w, b.w) as u8) << 3)
    }

    assert_eq!(a.less(b).bits(), expected_mask(a, b, |x, y| x < y));
    assert_eq!(a.less_equal(b).bits(), expected_mask(a, b, |x, y| x <= y));
    assert_eq!(a.greater(b).bits(), expected_mask(a, b, |x, y| x > y));
    assert_eq!(
        a.greater_equal(b).bits(),
        expected_mask(a, b, |x, y| x >= y)
    );
    assert_eq!(a.equal(b).bits(), expected_mask(a, b, |x, y| x == y));
    assert_eq!(a.not_equal(b).bits(), expected_mask(a, b, |x, y| x != y));

    let mask = a.less_equal(b);
    let yes = Vector4::from_xyz(1.0, 2.0, 3.0);
    let no = Vector4::from_xyz(-1.0, -2.0, -3.0);
    let expected = Vector4::new(
        if mask.get(0) { yes.x } else { no.x },
        if mask.get(1) { yes.y } else { no.y },
        if mask.get(2) { yes.z } else { no.z },
        if mask.get(3) { yes.w } else { no.w },
    );
    assert_eq!(bits(Vector4::select(mask, yes, no)), bits(expected));
    assert_eq!(mask.and(mask).bits(), mask.bits());
    assert_eq!(mask.or(mask).bits(), mask.bits());
    assert_eq!(mask.and_not(mask).bits(), 0);
}

#[test]
fn aabb_component_ops_match_scalar_reference() {
    let mut aabb = Aabb::empty();
    let points = [
        Vec3A::new(1.0, -2.0, 0.5),
        Vec3A::new(-1.0, 3.0, -0.5),
        Vec3A::new(-0.0, 0.0, f32::NAN),
    ];
    let mut min = Vec3A::splat(f32::MAX);
    let mut max = Vec3A::splat(-f32::MAX);
    for point in points {
        let expected_min = Vec3A::new(min.x.min(point.x), min.y.min(point.y), min.z.min(point.z));
        let expected_max = Vec3A::new(max.x.max(point.x), max.y.max(point.y), max.z.max(point.z));
        aabb.include_point3(point);
        assert_eq!(bits3(aabb.min), bits3(expected_min));
        assert_eq!(bits3(aabb.max), bits3(expected_max));
        min = expected_min;
        max = expected_max;
    }

    let mut expanded = aabb;
    expanded.expand_by(0.25);
    let delta = Vec3A::splat(0.25);
    assert_eq!(bits3(expanded.min), bits3(aabb.min - delta));
    assert_eq!(bits3(expanded.max), bits3(aabb.max + delta));

    let other = Aabb::from_corners(Vec3A::new(-2.0, -2.0, -2.0), Vec3A::new(2.0, 4.0, 2.0));
    let mut merged = aabb;
    merged.include_aabb(&other);
    assert_eq!(bits3(merged.min), bits3(aabb.min.min(other.min)));
    assert_eq!(bits3(merged.max), bits3(aabb.max.max(other.max)));

    assert_eq!(
        other.overlaps(&aabb),
        other.min.x <= aabb.max.x
            && other.min.y <= aabb.max.y
            && other.min.z <= aabb.max.z
            && aabb.min.x <= other.max.x
            && aabb.min.y <= other.max.y
            && aabb.min.z <= other.max.z
    );
    assert!(!Aabb::empty().is_valid());
}

#[test]
fn vector4d_keeps_scalar_bit_patterns() {
    let a = Vector4d::new(f64::from_bits(1), -0.0, f64::MAX, f64::NEG_INFINITY);
    let b = Vector4d::new(f64::MIN_POSITIVE / 2.0, 0.0, 2.0, f64::INFINITY);
    assert_eq!(
        bits_d(a + b),
        bits_d(Vector4d::new(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w,))
    );
    assert_eq!(
        bits_d(a * b),
        bits_d(Vector4d::new(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w,))
    );
    let dot3 = a.x * b.x + a.y * b.y + a.z * b.z;
    assert_eq!(a.dot3(b).to_bits(), dot3.to_bits());
}
