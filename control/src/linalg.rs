//! Small dense matrix operations on fixed-size arrays, in `f64`.
//!
//! Only what the controller design needs: products, transposes, sums and the inverse of a
//! small square matrix. Matrices are row-major nested arrays, `[[f64; COLS]; ROWS]`.

/// A matrix with `R` rows and `C` columns, row-major.
pub type Matrix<const R: usize, const C: usize> = [[f64; C]; R];

/// The `N × N` identity matrix.
#[must_use]
pub fn identity<const N: usize>() -> Matrix<N, N> {
    let mut m = [[0.0; N]; N];
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }

    m
}

/// The product `a · b`.
#[must_use]
pub fn mul<const R: usize, const K: usize, const C: usize>(
    a: &Matrix<R, K>,
    b: &Matrix<K, C>,
) -> Matrix<R, C> {
    let mut out = [[0.0; C]; R];
    for (out_row, a_row) in out.iter_mut().zip(a) {
        for (k, a_rk) in a_row.iter().enumerate() {
            for (o, b_kc) in out_row.iter_mut().zip(&b[k]) {
                *o += a_rk * b_kc;
            }
        }
    }

    out
}

/// The transpose `aᵀ`.
#[must_use]
pub fn transpose<const R: usize, const C: usize>(a: &Matrix<R, C>) -> Matrix<C, R> {
    let mut out = [[0.0; R]; C];
    for (r, row) in a.iter().enumerate() {
        for (c, value) in row.iter().enumerate() {
            out[c][r] = *value;
        }
    }

    out
}

/// The sum `a + b`.
#[must_use]
pub fn add<const R: usize, const C: usize>(a: &Matrix<R, C>, b: &Matrix<R, C>) -> Matrix<R, C> {
    let mut out = *a;
    for (out_row, b_row) in out.iter_mut().zip(b) {
        for (o, v) in out_row.iter_mut().zip(b_row) {
            *o += v;
        }
    }

    out
}

/// The symmetric part `(a + aᵀ) / 2`, to remove rounding asymmetry from a covariance or a
/// Riccati solution.
#[must_use]
pub fn symmetrize<const N: usize>(a: &Matrix<N, N>) -> Matrix<N, N> {
    let mut out = *a;
    for i in 0..N {
        for j in 0..N {
            out[i][j] = f64::midpoint(a[i][j], a[j][i]);
        }
    }

    out
}

/// The 1-norm (largest absolute column sum).
#[must_use]
pub fn norm1<const R: usize, const C: usize>(a: &Matrix<R, C>) -> f64 {
    (0..C)
        .map(|c| a.iter().map(|row| row[c].abs()).sum::<f64>())
        .fold(0.0, f64::max)
}

/// The inverse of a square matrix, by Gauss-Jordan elimination with partial pivoting.
///
/// Returns `None` if a pivot is smaller than `1e-14` times the 1-norm of `a`, i.e. if `a` is
/// singular or too close to singular to invert in `f64`, or if `a` holds a NaN or an
/// infinity.
#[must_use]
pub fn inverse<const N: usize>(a: &Matrix<N, N>) -> Option<Matrix<N, N>> {
    let scale = norm1(a);
    if !scale.is_finite() || scale == 0.0 {
        return None;
    }

    let mut m = *a;
    let mut inv = identity::<N>();
    for col in 0..N {
        let pivot = pivot_row(&m, col);
        if m[pivot][col].abs() < 1e-14 * scale {
            return None;
        }

        m.swap(col, pivot);
        inv.swap(col, pivot);
        eliminate(&mut m, &mut inv, col);
    }

    Some(inv)
}

/// The row at or below `col` with the largest absolute value in column `col`.
fn pivot_row<const N: usize>(m: &Matrix<N, N>, col: usize) -> usize {
    (col..N)
        .max_by(|&i, &j| m[i][col].abs().total_cmp(&m[j][col].abs()))
        .unwrap_or(col)
}

/// Scales row `col` to a unit pivot and removes column `col` from all other rows, in `m` and
/// in `inv` alike.
fn eliminate<const N: usize>(m: &mut Matrix<N, N>, inv: &mut Matrix<N, N>, col: usize) {
    let p = m[col][col];
    for c in 0..N {
        m[col][c] /= p;
        inv[col][c] /= p;
    }

    for r in (0..N).filter(|&r| r != col) {
        let f = m[r][col];
        for c in 0..N {
            m[r][c] -= f * m[col][c];
            inv[r][c] -= f * inv[col][c];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A product with the inverse gives the identity.
    #[test]
    fn inverse_times_matrix_is_identity() {
        let a = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]];
        let inv = inverse(&a).expect("a is well conditioned");
        let p = mul(&a, &inv);
        for (i, row) in p.iter().enumerate() {
            for (j, v) in row.iter().enumerate() {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((v - expected).abs() < 1e-12, "entry ({i}, {j}) is {v}");
            }
        }
    }

    /// A singular matrix has no inverse.
    #[test]
    fn singular_matrix_has_no_inverse() {
        assert!(inverse(&[[1.0, 2.0], [2.0, 4.0]]).is_none());
    }
}
