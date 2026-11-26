//! Deterministic symmetric 3x3 diagonalization for mass properties.

use crate::base_math::{Mat3Ext, QuatExt, Vec4Ext, quat_normalize};
use glam::{Mat3, Quat, Vec4};

const EPS: f32 = 1.0e-6;
const MAX_SWEEPS: usize = 12;
const PAIRS: [(usize, usize); 3] = [(0, 1), (0, 2), (1, 2)];

/// Result of diagonalizing a symmetric inertia tensor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiagonalizedSymmetric {
    /// Eigenvalues (diagonal inertia) in ascending order.
    pub diagonal: Vec4,
    /// Orthonormal eigenvectors as matrix columns.
    pub axes: Mat3,
    /// Rotation from principal-axis space to the original space.
    pub orientation: Quat,
}

/// Diagonalize a symmetric matrix with a fixed cyclic Jacobi schedule.
///
/// The rotation order, sweep count, eigenvalue sort and eigenvector sign rules
/// are fixed so identical inputs produce identical frames across runs. The
/// result is deterministic but not guaranteed to match the SDK's eigen
/// ordering bit-for-bit.
pub fn diagonalize_symmetric(matrix: Mat3) -> DiagonalizedSymmetric {
    let mut a = matrix;
    let mut axes = Mat3::IDENTITY;

    for _ in 0..MAX_SWEEPS {
        let mut changed = false;
        for (p, q) in PAIRS {
            let off_diagonal = a.element_sdk(p, q);
            if off_diagonal.abs() <= EPS {
                continue;
            }
            changed = true;
            let app = a.element_sdk(p, p);
            let aqq = a.element_sdk(q, q);
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
        (a.element_sdk(0, 0).max(EPS), 0usize),
        (a.element_sdk(1, 1).max(EPS), 1usize),
        (a.element_sdk(2, 2).max(EPS), 2usize),
    ];
    eigen.sort_by(|left, right| left.0.total_cmp(&right.0).then(left.1.cmp(&right.1)));

    let mut columns = [axes.x_axis, axes.y_axis, axes.z_axis];
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
    let mut axes = Mat3::from_cols(columns[0], columns[1], columns[2]);
    let determinant = determinant(&axes);
    if determinant < 0.0 {
        columns[2] = -columns[2];
        axes = Mat3::from_cols(columns[0], columns[1], columns[2]);
    }
    let mut orientation = Quat::from_matrix_sdk(axes);
    if !orientation.w.is_finite() || orientation.length_squared() <= 0.0 {
        orientation = Quat::IDENTITY;
    } else {
        orientation = quat_normalize(orientation);
    }

    DiagonalizedSymmetric {
        diagonal: Vec4::from_xyz(eigen[0].0, eigen[1].0, eigen[2].0),
        axes,
        orientation,
    }
}

fn rotate(a: &mut Mat3, axes: &mut Mat3, p: usize, q: usize, c: f32, s: f32) {
    for k in 0..3 {
        if k == p || k == q {
            continue;
        }
        let akp = a.element_sdk(k, p);
        let akq = a.element_sdk(k, q);
        let new_kp = c * akp - s * akq;
        let new_kq = s * akp + c * akq;
        a.set_element_sdk(k, p, new_kp);
        a.set_element_sdk(p, k, new_kp);
        a.set_element_sdk(k, q, new_kq);
        a.set_element_sdk(q, k, new_kq);
    }
    let app = a.element_sdk(p, p);
    let aqq = a.element_sdk(q, q);
    let apq = a.element_sdk(p, q);
    a.set_element_sdk(p, p, c * c * app - 2.0 * s * c * apq + s * s * aqq);
    a.set_element_sdk(q, q, s * s * app + 2.0 * s * c * apq + c * c * aqq);
    a.set_element_sdk(p, q, 0.0);
    a.set_element_sdk(q, p, 0.0);

    let vp = column(axes, p);
    let vq = column(axes, q);
    let new_p = vp * c - vq * s;
    let new_q = vp * s + vq * c;
    let mut columns = [axes.x_axis, axes.y_axis, axes.z_axis];
    columns[p] = new_p;
    columns[q] = new_q;
    *axes = Mat3::from_cols(columns[0], columns[1], columns[2]);
}

fn column(matrix: &Mat3, index: usize) -> glam::Vec3 {
    match index {
        0 => matrix.x_axis,
        1 => matrix.y_axis,
        _ => matrix.z_axis,
    }
}

fn determinant(matrix: &Mat3) -> f32 {
    matrix.x_axis.dot(matrix.y_axis.cross(matrix.z_axis))
}
