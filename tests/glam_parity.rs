//! glam↔legacy bit-parity gate (replaces the retired `simd_parity` SIMD/scalar
//! gate; see `docs/refactor-glam-plan.md`, Phase 2).
//!
//! Every expectation is computed from public fields with plain scalar Rust
//! expressions in the same order as the legacy C++ `Fpu` formulations. The
//! legacy types (`Vector4`, `Quaternion`, `Matrix3`, ...) and the glam-backed
//! implementations (`Vec4Ext`, `QuatExt`, `Mat3Ext`, `Aabb`,
//! `PreciseTransform`) must agree bit-for-bit on `+ - * /` and comparisons;
//! only `sin`/`cos`/`sqrt`-derived values may differ (compared with tolerances
//! there).

use glam::{DVec4, Mat3, Quat, Vec3, Vec3A, Vec4};

use proton::math::{
    Aabb, Mat3Ext, Matrix3, PreciseTransform, QuatExt, Quaternion, Vec4Ext, Vector4,
};

fn bits4(v: Vec4) -> [u32; 4] {
    v.to_array().map(f32::to_bits)
}

fn legacy_bits4(v: Vector4) -> [u32; 4] {
    v.to_array().map(f32::to_bits)
}

fn to_legacy(v: Vec4) -> Vector4 {
    Vector4::new(v.x, v.y, v.z, v.w)
}

/// Special-value pairs: signed zeros, subnormals, infinities, NaN payloads.
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
fn vec4_ext_matches_legacy_bit_for_bit() {
    for &(x, y) in SPECIAL_PAIRS {
        let a = Vec4::new(x, y, -x, y);
        let b = Vec4::new(y, x, -y, x);
        let la = to_legacy(a);
        let lb = to_legacy(b);

        // dot3 / horizontal_add3: fixed scalar order.
        assert_eq!(
            a.dot3(b).to_bits(),
            la.dot3(lb).to_bits(),
            "dot3 {x:?}/{y:?}"
        );
        assert_eq!(
            a.horizontal_add3().to_bits(),
            la.horizontal_add3().to_bits(),
            "horizontal_add3 {x:?}/{y:?}"
        );

        // cross3: legacy `set_cross` scalar formula, w zeroed.
        assert_eq!(bits4(a.cross3(b)), legacy_bits4(la.cross(lb)), "cross3 {x:?}/{y:?}");

        // length_squared3: fixed-order xyz dot with self.
        assert_eq!(
            a.length_squared3().to_bits(),
            la.length_squared3().to_bits(),
            "length_squared3 {x:?}/{y:?}"
        );

        // all_equal3: xyz-only tolerance comparison (bool, no bit-level math).
        let eps = 1e-3;
        assert_eq!(
            a.all_equal3(b, eps),
            la.all_equal3(lb, eps),
            "all_equal3 {x:?}/{y:?}"
        );
        assert_eq!(
            a.all_equal3(a, eps),
            la.all_equal3(la, eps),
            "all_equal3 self {x:?}/{y:?}"
        );

        // Legacy component-wise min/max use f32::min/max (NaN → other operand).
        let min = Vec4::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z), a.w.min(b.w));
        let max = Vec4::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z), a.w.max(b.w));
        assert_eq!(bits4(min), legacy_bits4(la.min(lb)), "min {x:?}/{y:?}");
        assert_eq!(bits4(max), legacy_bits4(la.max(lb)), "max {x:?}/{y:?}");
    }

    // normalize_or_zero3: zero maps to zero, w preserved.
    assert_eq!(Vec4::ZERO.normalize_or_zero3(), Vec4::ZERO);
    let v = Vec4::new(3.0, 4.0, 0.0, 7.0);
    let legacy_n = Vector4::new(3.0, 4.0, 0.0, 7.0);
    let mut legacy_n = legacy_n;
    legacy_n.normalize3();
    assert_eq!(bits4(v.normalize_or_zero3()), legacy_bits4(legacy_n));
}

#[test]
fn quaternion_ops_match_legacy_bit_for_bit() {
    let to_gq = |q: Quaternion| Quat::from_xyzw(q.x, q.y, q.z, q.w);
    let to_lq = |q: Quat| Quaternion::new(q.x, q.y, q.z, q.w);

    for &(x, y) in &[(0.3f32, 0.4f32), (-0.7f32, 0.1f32)] {
        let axis = Vec4::new(1.0, 2.0, 2.0, 0.0).normalize_or_zero3();
        let angle = x + 0.5f32 * y.abs();
        let legacy = Quaternion::from_axis_angle(Vector4::new(axis.x, axis.y, axis.z, 0.0), angle);
        let glam_q = Quat::from_axis_angle(Vec3::new(axis.x, axis.y, axis.z), angle);

        let direction = Vec4::new(1.0, -2.0, 0.5, 0.0);
        let legacy_dir = Vector4::new(direction.x, direction.y, direction.z, direction.w);

        // rotate / rotate_inverse: SDK `_setRotatedDir` formulation.
        assert_eq!(
            bits4(glam_q.rotate_sdk(direction)),
            legacy_bits4(legacy.rotate(legacy_dir)),
            "rotate {x:?}/{y:?}"
        );
        assert_eq!(
            bits4(glam_q.rotate_inverse_sdk(direction)),
            legacy_bits4(legacy.rotate_inverse(legacy_dir)),
            "rotate_inverse {x:?}/{y:?}"
        );

        // Hamilton product.
        let other = Quaternion::from_axis_angle(Vector4::UNIT_X, y);
        let glam_other = Quat::from_axis_angle(Vec3::X, y);
        assert_eq!(
            to_lq(proton::math::quat_mul(glam_q, glam_other)),
            legacy.mul(other),
            "mul {x:?}/{y:?}"
        );

        // from_matrix: 4-branch trace + post normalization.
        let m = legacy.to_matrix();
        let glam_m = Mat3::from_cols(
            Vec3::new(m.col0.x, m.col0.y, m.col0.z),
            Vec3::new(m.col1.x, m.col1.y, m.col1.z),
            Vec3::new(m.col2.x, m.col2.y, m.col2.z),
        );
        let converted = Quat::from_matrix_sdk(glam_m);
        let legacy_converted = Quaternion::from_matrix(&m);
        assert_eq!(
            to_lq(converted),
            legacy_converted,
            "from_matrix {x:?}/{y:?}"
        );

        // Identity must not depend on glam's Quat::default().
        assert_eq!(to_lq(Quat::IDENTITY), Quaternion::IDENTITY);
        let _ = to_gq(legacy);
    }
}

#[test]
fn mat3_ext_matches_legacy_bit_for_bit() {
    let to_gm = |m: &Matrix3| {
        Mat3::from_cols(
            Vec3::new(m.col0.x, m.col0.y, m.col0.z),
            Vec3::new(m.col1.x, m.col1.y, m.col1.z),
            Vec3::new(m.col2.x, m.col2.y, m.col2.z),
        )
    };

    let cases = [
        Matrix3::from_rows(
            Vector4::from_xyz(2.0, 1.0, 0.0),
            Vector4::from_xyz(1.0, 3.0, 1.0),
            Vector4::from_xyz(0.0, 1.0, 2.0),
        ),
        // Singular: eps check fails → zero matrix.
        Matrix3::from_rows(
            Vector4::from_xyz(1.0, 2.0, 3.0),
            Vector4::from_xyz(2.0, 4.0, 6.0),
            Vector4::from_xyz(1.0, 1.0, 1.0),
        ),
        Matrix3::IDENTITY,
    ];

    for legacy_m in cases {
        let glam_m = to_gm(&legacy_m);
        let v = Vector4::new(0.5, -1.5, 2.0, 0.0);
        let gv = Vec3::new(v.x, v.y, v.z);

        // mul_vec: column expansion in the C++ order.
        let legacy_mul = legacy_m.mul_vec(v);
        let glam_mul = glam_m.mul_vec_sdk(gv);
        assert_eq!(glam_mul.x, legacy_mul.x);
        assert_eq!(glam_mul.y, legacy_mul.y);
        assert_eq!(glam_mul.z, legacy_mul.z);

        // inverse: scaled-eps determinant check, singular → zero matrix.
        let (legacy_inv, legacy_det) = legacy_m.inverse();
        let (glam_inv, glam_det) = glam_m.inverse_sdk();
        assert_eq!(glam_det.to_bits(), legacy_det.to_bits());
        assert_eq!(
            glam_inv.x_axis.to_array().map(f32::to_bits),
            [
                legacy_inv.col0.x.to_bits(),
                legacy_inv.col0.y.to_bits(),
                legacy_inv.col0.z.to_bits()
            ]
        );
        assert_eq!(
            glam_inv.y_axis.to_array().map(f32::to_bits),
            [
                legacy_inv.col1.x.to_bits(),
                legacy_inv.col1.y.to_bits(),
                legacy_inv.col1.z.to_bits()
            ]
        );
        assert_eq!(
            glam_inv.z_axis.to_array().map(f32::to_bits),
            [
                legacy_inv.col2.x.to_bits(),
                legacy_inv.col2.y.to_bits(),
                legacy_inv.col2.z.to_bits()
            ]
        );
    }
}

#[test]
fn quat_normalize_matches_legacy_bit_for_bit() {
    for &(x, y) in SPECIAL_PAIRS {
        let normalized = proton::math::quat_normalize(Quat::from_xyzw(x, y, -x, y));
        let mut legacy = Quaternion::new(x, y, -x, y);
        legacy.normalize();
        assert_eq!(
            normalized.x.to_bits(),
            legacy.x.to_bits(),
            "normalize.x {x:?}/{y:?}"
        );
        assert_eq!(
            normalized.y.to_bits(),
            legacy.y.to_bits(),
            "normalize.y {x:?}/{y:?}"
        );
        assert_eq!(
            normalized.z.to_bits(),
            legacy.z.to_bits(),
            "normalize.z {x:?}/{y:?}"
        );
        assert_eq!(
            normalized.w.to_bits(),
            legacy.w.to_bits(),
            "normalize.w {x:?}/{y:?}"
        );
    }

    // Zero maps to zero (HK_SQRT_SET_ZERO).
    assert_eq!(
        proton::math::quat_normalize(Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)),
        Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)
    );
}

#[test]
fn quat_to_matrix_sdk_matches_legacy_bit_for_bit() {
    let to_lm = |m: &Matrix3| -> [[u32; 3]; 3] {
        [
            [m.col0.x.to_bits(), m.col0.y.to_bits(), m.col0.z.to_bits()],
            [m.col1.x.to_bits(), m.col1.y.to_bits(), m.col1.z.to_bits()],
            [m.col2.x.to_bits(), m.col2.y.to_bits(), m.col2.z.to_bits()],
        ]
    };

    for &(x, y) in &[(0.3f32, 0.4f32), (-0.7f32, 0.1f32)] {
        let axis = Vec3::new(1.0, 2.0, 2.0).normalize();
        let angle = x + 0.5 * y.abs();
        let legacy_q =
            Quaternion::from_axis_angle(Vector4::new(axis.x, axis.y, axis.z, 0.0), angle);
        let glam_q = Quat::from_axis_angle(axis, angle);

        let legacy_m = legacy_q.to_matrix();
        let glam_m = glam_q.to_matrix_sdk();
        let expected = to_lm(&legacy_m);
        assert_eq!(glam_m.x_axis.to_array().map(f32::to_bits), expected[0]);
        assert_eq!(glam_m.y_axis.to_array().map(f32::to_bits), expected[1]);
        assert_eq!(glam_m.z_axis.to_array().map(f32::to_bits), expected[2]);
    }
}

#[test]
fn mat3_mul_mat_and_element_match_legacy_bit_for_bit() {
    let to_gm = |m: &Matrix3| {
        Mat3::from_cols(
            Vec3::new(m.col0.x, m.col0.y, m.col0.z),
            Vec3::new(m.col1.x, m.col1.y, m.col1.z),
            Vec3::new(m.col2.x, m.col2.y, m.col2.z),
        )
    };

    let cases = [
        Matrix3::from_rows(
            Vector4::from_xyz(2.0, 1.0, 0.0),
            Vector4::from_xyz(1.0, 3.0, 1.0),
            Vector4::from_xyz(0.0, 1.0, 2.0),
        ),
        Matrix3::IDENTITY,
    ];

    for legacy_m in &cases {
        let glam_m = to_gm(legacy_m);
        let other = &cases[0];

        // mul_mat: each product column via the SDK multiplyVector order.
        let legacy_product = legacy_m.mul_mat(other);
        let glam_product = glam_m.mul_mat_sdk(to_gm(other));
        for column in 0..3 {
            for row in 0..3 {
                assert_eq!(
                    glam_product.element_sdk(row, column).to_bits(),
                    legacy_product.element(row, column).to_bits(),
                    "mul_mat [{row},{column}]"
                );
            }
        }

        // element accessor agreement.
        for row in 0..3 {
            for column in 0..3 {
                assert_eq!(
                    glam_m.element_sdk(row, column).to_bits(),
                    legacy_m.element(row, column).to_bits(),
                    "element [{row},{column}]"
                );
            }
        }
    }

    // set_element mutates in place.
    let mut glam_m = to_gm(&cases[0]);
    glam_m.set_element_sdk(1, 2, -7.5);
    assert_eq!(glam_m.element_sdk(1, 2), -7.5);
}

#[test]
fn aabb_matches_legacy_scalar_reference() {
    // include_point with NaN must keep the non-NaN operand (f32::min/max rules).
    let mut aabb = Aabb::empty();
    aabb.include_point(Vector4::new(1.0, -2.0, 0.5, 0.0));
    aabb.include_point(Vector4::new(-1.0, 3.0, -0.5, 0.0));
    aabb.include_point(Vector4::new(-0.0, 0.0, f32::NAN, 0.0));

    let mut reference_min = [f32::MAX; 3];
    let mut reference_max = [-f32::MAX; 3];
    for p in [
        [1.0f32, -2.0, 0.5],
        [-1.0, 3.0, -0.5],
        [-0.0, 0.0, f32::NAN],
    ] {
        for i in 0..3 {
            reference_min[i] = reference_min[i].min(p[i]);
            reference_max[i] = reference_max[i].max(p[i]);
        }
    }
    assert_eq!(aabb.min.to_array(), reference_min);
    assert_eq!(aabb.max.to_array(), reference_max);

    // expand_by / center / extents arithmetic is exact on the sampled box.
    let box_ = Aabb::from_corners(Vec3A::new(-1.0, -2.0, -3.0), Vec3A::new(1.0, 2.0, 3.0));
    assert_eq!(box_.center().to_array(), [0.0, 0.0, 0.0]);
    assert_eq!(box_.extents().to_array(), [2.0, 4.0, 6.0]);
    assert_eq!(box_.volume(), 48.0);

    // xyz-only overlap semantics, inclusive boundaries.
    let inner = Aabb::from_corners(Vec3A::ZERO, Vec3A::splat(1.0));
    assert!(inner.overlaps(&Aabb::from_corners(Vec3A::splat(1.0), Vec3A::splat(2.0))));
    assert!(inner.contains(&Aabb::from_corners(Vec3A::splat(0.5), Vec3A::splat(1.0))));
    assert!(inner.contains_point3(Vec3A::splat(1.0)));
    assert!(!inner.contains_point3(Vec3A::new(1.5, 0.5, 0.5)));
}

#[test]
fn precise_transform_matches_legacy_scalar_model() {
    // from_translation_d / translation_d against a hand-rolled double-float model.
    let translation = DVec4::new(1_234_567.25, -2_000_003.5, 12.000_000_1, 1.0);
    let precise = PreciseTransform::from_translation_d(Matrix3::IDENTITY, translation);
    let model = [
        translation.x as f32 as f64 + (translation.x - translation.x as f32 as f64),
        translation.y as f32 as f64 + (translation.y - translation.y as f32 as f64),
        translation.z as f32 as f64 + (translation.z - translation.z as f32 as f64),
    ];
    let reconstructed = precise.translation_d();
    let components = reconstructed.to_array();
    for (i, &component) in components[..3].iter().enumerate() {
        assert!(
            (component - model[i]).abs() < 1e-6,
            "axis {i}: {component} vs {}",
            model[i]
        );
    }

    // Compensated add keeps sub-f32 remainders in the low part.
    let mut shifted = PreciseTransform::from_translation_d(
        Matrix3::IDENTITY,
        DVec4::new(10_000_000.0, 0.0, 0.0, 1.0),
    );
    shifted.add_translation(Vec4::new(0.25, 0.0, 0.0, 0.0));
    assert!((shifted.translation_d().x - 10_000_000.25).abs() < 1e-4);
    // The f32 high part cannot hold the remainder; it must live in `low`.
    assert_eq!(shifted.translation.x, 10_000_000.0);
    assert!((shifted.translation_adjustment.x - 0.25).abs() < 1e-6);

    // relative_to reproduces the scalar `_setMulInverseMul` distance.
    let a = PreciseTransform::from_translation_d(
        Matrix3::IDENTITY,
        DVec4::new(1_000_000.0, 0.0, 0.0, 1.0),
    );
    let c = PreciseTransform::from_translation_d(
        Matrix3::IDENTITY,
        DVec4::new(1_000_002.5, 0.0, 0.0, 1.0),
    );
    let relative = a.relative_to(c);
    assert!((relative.translation.x - 2.5).abs() < 1e-3);

    // Double precision round trip through the large-world path.
    let point = DVec4::new(1_000_003.5, 0.0, 0.0, 0.0);
    let local = a.inverse_transform_point_d(point);
    assert!((local.x - 3.5).abs() < 1e-3);
    let back = a.transform_point_d(local);
    assert!((back.x - point.x).abs() < 1e-3);
}

#[test]
fn determinism_smoke_repeated_evaluations_agree() {
    // Same input → bit-identical output across repeated calls (no hidden state).
    let q = Quat::from_axis_angle(Vec3::new(1.0, 2.0, 2.0).normalize(), 0.7);
    let v = Vec4::new(1.0, -2.0, 0.5, 0.0);
    let first = bits4(q.rotate_sdk(v));
    for _ in 0..100 {
        assert_eq!(bits4(q.rotate_sdk(v)), first);
    }
}
