//! Deterministic symmetric 3x3 diagonalization for mass properties.

use base_math::{Matrix3, Quaternion, Vector4};

const EPS: f32 = 1.0e-6;
const MAX_SWEEPS: usize = 12;
const PAIRS: [(usize, usize); 3] = [(0, 1), (0, 2), (1, 2)];

/// Result of diagonalizing a symmetric inertia tensor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiagonalizedSymmetric {
    /// Eigenvalues (diagonal inertia) in ascending order.
    pub diagonal: Vector4,
    /// Orthonormal eigenvectors as matrix columns.
    pub axes: Matrix3,
    /// Rotation from principal-axis space to the original space.
    pub orientation: Quaternion,
}

/// Diagonalize a symmetric matrix with a fixed cyclic Jacobi schedule.
///
/// The rotation order, sweep count, eigenvalue sort and eigenvector sign rules
/// are fixed so identical inputs produce identical frames across runs. The
/// result is deterministic but not guaranteed to match the SDK's eigen
/// ordering bit-for-bit.
pub fn diagonalize_symmetric(matrix: Matrix3) -> DiagonalizedSymmetric {
    let mut a = matrix;
    let mut axes = Matrix3::IDENTITY;

    for _ in 0..MAX_SWEEPS {
        let mut changed = false;
        for (p, q) in PAIRS {
            let off_diagonal = a.element(p, q);
            if off_diagonal.abs() <= EPS {
                continue;
            }
            changed = true;
            let app = a.element(p, p);
            let aqq = a.element(q, q);
            let tau = (aqq - app) / (2.0 * off_diagonal);
            let t = if tau >= 0.0 {
                1.0 / (tau + (1.0 + tau * tau).sqrt())
            } else {
                -1.0 / (-tau + (1.0 + tau * tau).sqrt())
            };
            let c = 1.0 / (1.0 + t * t).sqrt();
            let s = t * c;
            rotate(&mut a, &mut axes, p, q, c, s);
        }
        if !changed {
            break;
        }
    }

    let mut eigen = [
        (a.element(0, 0).max(EPS), 0usize),
        (a.element(1, 1).max(EPS), 1usize),
        (a.element(2, 2).max(EPS), 2usize),
    ];
    eigen.sort_by(|left, right| left.0.total_cmp(&right.0).then(left.1.cmp(&right.1)));

    let mut columns = [axes.col0, axes.col1, axes.col2];
    let mut ordered = [
        columns[eigen[0].1],
        columns[eigen[1].1],
        columns[eigen[2].1],
    ];
    for (index, column) in ordered.iter_mut().enumerate() {
        let largest = column.x.abs().max(column.y.abs()).max(column.z.abs());
        if largest > 0.0 {
            let sign_component = if column.x.abs() == largest {
                column.x
            } else if column.y.abs() == largest {
                column.y
            } else {
                column.z
            };
            if sign_component < 0.0 {
                *column = -*column;
            }
        }
        columns[index] = *column;
    }
    let mut axes = Matrix3::from_cols(columns[0], columns[1], columns[2]);
    let determinant = determinant(&axes);
    if determinant < 0.0 {
        columns[2] = -columns[2];
        axes = Matrix3::from_cols(columns[0], columns[1], columns[2]);
    }
    let mut orientation = axes.to_quaternion();
    if !orientation.w.is_finite() || orientation.length_squared() <= 0.0 {
        orientation = Quaternion::IDENTITY;
    } else {
        orientation.normalize();
    }

    DiagonalizedSymmetric {
        diagonal: Vector4::from_xyz(eigen[0].0, eigen[1].0, eigen[2].0),
        axes,
        orientation,
    }
}

fn rotate(a: &mut Matrix3, axes: &mut Matrix3, p: usize, q: usize, c: f32, s: f32) {
    for k in 0..3 {
        if k == p || k == q {
            continue;
        }
        let akp = a.element(k, p);
        let akq = a.element(k, q);
        let new_kp = c * akp - s * akq;
        let new_kq = s * akp + c * akq;
        a.set_element(k, p, new_kp);
        a.set_element(p, k, new_kp);
        a.set_element(k, q, new_kq);
        a.set_element(q, k, new_kq);
    }
    let app = a.element(p, p);
    let aqq = a.element(q, q);
    let apq = a.element(p, q);
    a.set_element(p, p, c * c * app - 2.0 * s * c * apq + s * s * aqq);
    a.set_element(q, q, s * s * app + 2.0 * s * c * apq + c * c * aqq);
    a.set_element(p, q, 0.0);
    a.set_element(q, p, 0.0);

    let vp = axes.column(p);
    let vq = axes.column(q);
    let new_p = vp * c - vq * s;
    let new_q = vp * s + vq * c;
    let mut columns = [axes.col0, axes.col1, axes.col2];
    columns[p] = new_p;
    columns[q] = new_q;
    *axes = Matrix3::from_cols(columns[0], columns[1], columns[2]);
}

fn determinant(matrix: &Matrix3) -> f32 {
    matrix.col0.dot3(matrix.col1.cross(matrix.col2))
}
