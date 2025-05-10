#![allow(dead_code)]

use nalgebra as na;

pub fn assert_approx_eq_float(a: f64, b: f64, tol: f64) {
    assert!(
        (a - b).abs() < tol,
        "Floats {a} and {b} are not approximately equal within tolerance {tol}"
    );
}

pub fn assert_approx_eq_vec(a: na::Vector3<f64>, b: na::Vector3<f64>, tol: f64) {
    assert!(
        (a - b).norm() < tol,
        "Vectors {a:?} and {b:?} are not approximately equal within tolerance {tol}"
    );
}
