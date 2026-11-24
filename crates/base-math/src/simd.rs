//! Internal component-wise arithmetic helpers.
//!
//! `f32` uses SSE2 on `x86_64` (the platform baseline); every other target uses
//! the scalar reference implementation. `f64` always uses the scalar path.
//!
//! Only component-wise arithmetic lives here. Reductions (`dot`, `cross`,
//! `horizontalAdd`, normalization) keep the scalar addition order so existing
//! `hvk-trace v1` snapshots stay bit-for-bit identical. `min`/`max`, comparison
//! masks and `select` stay scalar in `vector.rs`: SSE `minps` has different
//! NaN/equal rules, and routing AABB comparisons through this helper regressed
//! `math_aabb_include_point` by ~3x in `bench --quick`.
//!
//! Add and multiply are always separate instructions: no FMA contraction and
//! no reassociation is performed anywhere in this module.

pub(crate) trait LaneOps: Copy {
    fn add(a: [Self; 4], b: [Self; 4]) -> [Self; 4];
    fn sub(a: [Self; 4], b: [Self; 4]) -> [Self; 4];
    fn mul(a: [Self; 4], b: [Self; 4]) -> [Self; 4];
    fn div(a: [Self; 4], b: [Self; 4]) -> [Self; 4];
    fn neg(a: [Self; 4]) -> [Self; 4];
    fn add_mul(a: [Self; 4], b: [Self; 4], c: [Self; 4]) -> [Self; 4];
    fn sub_mul(a: [Self; 4], b: [Self; 4], c: [Self; 4]) -> [Self; 4];
    fn abs(a: [Self; 4]) -> [Self; 4];
    fn flip_sign(a: [Self; 4], sign: [Self; 4]) -> [Self; 4];
}

#[inline]
pub(crate) fn splat<T: LaneOps>(value: T) -> [T; 4] {
    [value; 4]
}

#[inline]
pub(crate) fn add<T: LaneOps>(a: [T; 4], b: [T; 4]) -> [T; 4] {
    T::add(a, b)
}

#[inline]
pub(crate) fn sub<T: LaneOps>(a: [T; 4], b: [T; 4]) -> [T; 4] {
    T::sub(a, b)
}

#[inline]
pub(crate) fn mul<T: LaneOps>(a: [T; 4], b: [T; 4]) -> [T; 4] {
    T::mul(a, b)
}

#[inline]
pub(crate) fn div<T: LaneOps>(a: [T; 4], b: [T; 4]) -> [T; 4] {
    T::div(a, b)
}

#[inline]
pub(crate) fn neg<T: LaneOps>(a: [T; 4]) -> [T; 4] {
    T::neg(a)
}

#[inline]
pub(crate) fn add_mul<T: LaneOps>(a: [T; 4], b: [T; 4], c: [T; 4]) -> [T; 4] {
    T::add_mul(a, b, c)
}

#[inline]
pub(crate) fn sub_mul<T: LaneOps>(a: [T; 4], b: [T; 4], c: [T; 4]) -> [T; 4] {
    T::sub_mul(a, b, c)
}

#[inline]
pub(crate) fn abs<T: LaneOps>(a: [T; 4]) -> [T; 4] {
    T::abs(a)
}

#[inline]
pub(crate) fn flip_sign<T: LaneOps>(a: [T; 4], sign: [T; 4]) -> [T; 4] {
    T::flip_sign(a, sign)
}

impl LaneOps for f64 {
    #[inline]
    fn add(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
        [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]]
    }

    #[inline]
    fn sub(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
        [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]]
    }

    #[inline]
    fn mul(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
        [a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]]
    }

    #[inline]
    fn div(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
        [a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]]
    }

    #[inline]
    fn neg(a: [Self; 4]) -> [Self; 4] {
        [-a[0], -a[1], -a[2], -a[3]]
    }

    #[inline]
    fn add_mul(a: [Self; 4], b: [Self; 4], c: [Self; 4]) -> [Self; 4] {
        [
            a[0] + b[0] * c[0],
            a[1] + b[1] * c[1],
            a[2] + b[2] * c[2],
            a[3] + b[3] * c[3],
        ]
    }

    #[inline]
    fn sub_mul(a: [Self; 4], b: [Self; 4], c: [Self; 4]) -> [Self; 4] {
        [
            a[0] - b[0] * c[0],
            a[1] - b[1] * c[1],
            a[2] - b[2] * c[2],
            a[3] - b[3] * c[3],
        ]
    }

    #[inline]
    fn abs(a: [Self; 4]) -> [Self; 4] {
        [a[0].abs(), a[1].abs(), a[2].abs(), a[3].abs()]
    }

    #[inline]
    fn flip_sign(a: [Self; 4], sign: [Self; 4]) -> [Self; 4] {
        [
            if sign[0] < 0.0 { -a[0] } else { a[0] },
            if sign[1] < 0.0 { -a[1] } else { a[1] },
            if sign[2] < 0.0 { -a[2] } else { a[2] },
            if sign[3] < 0.0 { -a[3] } else { a[3] },
        ]
    }
}

#[cfg(target_arch = "x86_64")]
mod f32_lanes {
    use core::arch::x86_64::*;

    use super::LaneOps;

    const SIGN_MASK: u32 = 0x8000_0000;

    #[inline]
    fn load(values: [f32; 4]) -> __m128 {
        unsafe { _mm_loadu_ps(values.as_ptr()) }
    }

    #[inline]
    fn store(values: __m128) -> [f32; 4] {
        let mut result = [0.0; 4];
        unsafe { _mm_storeu_ps(result.as_mut_ptr(), values) };
        result
    }

    #[inline]
    fn sign_mask() -> __m128 {
        unsafe { _mm_castsi128_ps(_mm_set1_epi32(SIGN_MASK as i32)) }
    }

    impl LaneOps for f32 {
        #[inline]
        fn add(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
            unsafe { store(_mm_add_ps(load(a), load(b))) }
        }

        #[inline]
        fn sub(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
            unsafe { store(_mm_sub_ps(load(a), load(b))) }
        }

        #[inline]
        fn mul(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
            unsafe { store(_mm_mul_ps(load(a), load(b))) }
        }

        #[inline]
        fn div(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
            unsafe { store(_mm_div_ps(load(a), load(b))) }
        }

        #[inline]
        fn neg(a: [Self; 4]) -> [Self; 4] {
            unsafe { store(_mm_xor_ps(load(a), sign_mask())) }
        }

        #[inline]
        fn add_mul(a: [Self; 4], b: [Self; 4], c: [Self; 4]) -> [Self; 4] {
            unsafe { store(_mm_add_ps(load(a), _mm_mul_ps(load(b), load(c)))) }
        }

        #[inline]
        fn sub_mul(a: [Self; 4], b: [Self; 4], c: [Self; 4]) -> [Self; 4] {
            unsafe { store(_mm_sub_ps(load(a), _mm_mul_ps(load(b), load(c)))) }
        }

        #[inline]
        fn abs(a: [Self; 4]) -> [Self; 4] {
            unsafe { store(_mm_andnot_ps(sign_mask(), load(a))) }
        }

        #[inline]
        fn flip_sign(a: [Self; 4], sign: [Self; 4]) -> [Self; 4] {
            let negative = unsafe { _mm_cmplt_ps(load(sign), _mm_setzero_ps()) };
            let flip = unsafe { _mm_and_ps(negative, sign_mask()) };
            unsafe { store(_mm_xor_ps(load(a), flip)) }
        }
    }
}

#[cfg(not(target_arch = "x86_64"))]
mod f32_lanes {
    use super::LaneOps;

    impl LaneOps for f32 {
        #[inline]
        fn add(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
            [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]]
        }

        #[inline]
        fn sub(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
            [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]]
        }

        #[inline]
        fn mul(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
            [a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]]
        }

        #[inline]
        fn div(a: [Self; 4], b: [Self; 4]) -> [Self; 4] {
            [a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]]
        }

        #[inline]
        fn neg(a: [Self; 4]) -> [Self; 4] {
            [-a[0], -a[1], -a[2], -a[3]]
        }

        #[inline]
        fn add_mul(a: [Self; 4], b: [Self; 4], c: [Self; 4]) -> [Self; 4] {
            [
                a[0] + b[0] * c[0],
                a[1] + b[1] * c[1],
                a[2] + b[2] * c[2],
                a[3] + b[3] * c[3],
            ]
        }

        #[inline]
        fn sub_mul(a: [Self; 4], b: [Self; 4], c: [Self; 4]) -> [Self; 4] {
            [
                a[0] - b[0] * c[0],
                a[1] - b[1] * c[1],
                a[2] - b[2] * c[2],
                a[3] - b[3] * c[3],
            ]
        }

        #[inline]
        fn abs(a: [Self; 4]) -> [Self; 4] {
            [a[0].abs(), a[1].abs(), a[2].abs(), a[3].abs()]
        }

        #[inline]
        fn flip_sign(a: [Self; 4], sign: [Self; 4]) -> [Self; 4] {
            [
                if sign[0] < 0.0 { -a[0] } else { a[0] },
                if sign[1] < 0.0 { -a[1] } else { a[1] },
                if sign[2] < 0.0 { -a[2] } else { a[2] },
                if sign[3] < 0.0 { -a[3] } else { a[3] },
            ]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn componentwise_add_mul_matches_scalar_values() {
        assert_eq!(
            add([1.0, 2.0, 3.0, 4.0], [4.0, 3.0, 2.0, 1.0]),
            [5.0, 5.0, 5.0, 5.0]
        );
        assert_eq!(
            add_mul(
                [1.0, 2.0, 3.0, 4.0],
                [2.0, 2.0, 2.0, 2.0],
                [0.5, 0.5, 0.5, 0.5]
            ),
            [2.0, 3.0, 4.0, 5.0]
        );
    }
}
