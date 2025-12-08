//! Defines the [`Plane`] struct and related helper functions for 3D geometry.
//!
//! This module provides a representation of a 3D plane defined by an origin
//! and two orthogonal axis vectors, along with methods to create and
//! transform objects relative to this plane.

use nalgebra as na;
use scadman::prelude::*;

use crate::to_openscad_rotate_angles;

/// Represents a 3D plane defined by an origin point
/// and two orthogonal axis vectors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    /// The origin point of the plane in 3D space.
    origin: Point3D,
    /// The normalized vector representing the X-axis direction of the plane.
    x_axis: Point3D,
    /// The normalized vector representing the Y-axis direction of the plane.
    /// It is orthogonal to `x_axis`.
    y_axis: Point3D,
}

/// Checks if a vector is approximately the zero vector
/// within a small tolerance.
///
/// # Arguments
///
/// * `v` - The vector to check.
///
/// # Returns
///
/// Returns `true` if the vector's norm is less than `1e-10`,
/// `false` otherwise.
#[inline]
fn is_approx_zero(v: na::Vector3<f64>) -> bool {
    v.norm() < 1e-10
}

/// Checks if two vectors are approximately parallel within a small tolerance.
///
/// # Arguments
///
/// * `x` - The first vector.
/// * `y` - The second vector.
///
/// # Returns
///
/// Returns `true` if the two vectors are approximately parallel
/// (or anti-parallel), `false` otherwise.
/// Returns `false` if either vector is close to zero.
#[inline]
fn is_parallel(x: na::Vector3<f64>, y: na::Vector3<f64>) -> bool {
    let x_norm = x.norm();
    let y_norm = y.norm();

    if x_norm * y_norm < 1e-10 {
        return false;
    }
    // Check if the cosine of the angle is close to 1 or -1
    (x.dot(&y).abs() / x_norm / y_norm - 1.).abs() < 1e-10
}

/// Orthogonalizes the `target` vector with respect to the `axis` vector.
///
/// Returns the normalized component of `target` that is orthogonal to `axis`.
/// This component is in the same plane as `axis` and `target`.
///
/// # Arguments
///
/// * `target` - The vector to be orthogonalized.
/// * `axis` - The vector to orthogonalize against.
///
/// # Returns
///
/// * [`Some(na::Vector3<f64>)`] - Containing the normalized vector
///   representing the component of `target` orthogonal to `axis`.
/// * [`None`] - If `axis` or `target` is a zero vector, or if
///   `target` is parallel to `axis`.
#[inline]
fn orthogonalize(target: na::Vector3<f64>, axis: na::Vector3<f64>) -> Option<na::Vector3<f64>> {
    if axis.norm() < 1e-10 || target.norm() < 1e-10 {
        return None;
    }

    let ax_norm = axis.normalize();

    // Project target onto axis
    let tg_proj_ax = target.dot(&ax_norm) * ax_norm;
    // Get the component of target perpendicular to axis
    let tg_perp = target - tg_proj_ax;
    if tg_perp.norm() < 1e-10 {
        // target was parallel to axis
        return None;
    }

    Some(tg_perp.normalize())
}

impl Plane {
    /// Attempts to create a new `Plane` from an origin and two axis vectors.
    ///
    /// The `x_axis` and `y_axis` vectors are normalized and made orthogonal
    /// during the creation process.
    /// The resulting `x_axis` will be the normalized input `x_axis`,
    /// and the resulting `y_axis` will be the normalized component
    /// of the input `y_axis` that is orthogonal to the input `x_axis`.
    ///
    /// # Arguments
    ///
    /// * `origin` - The origin point of the plane.
    /// * `x_axis` - A vector defining the direction of the plane's X-axis.
    ///   will be normalized.
    /// * `y_axis` - A vector defining the direction of the plane's Y-axis.
    ///   will be orthogonalized respect to `x_axis` and then normalized.
    ///
    /// # Returns
    ///
    /// * [`Some<Self>`] - Containing the new `Plane`
    ///   if the input vectors are valid.
    /// * [`None`] - If either `x_axis` or `y_axis` is a zero vector,
    ///   or if they are parallel
    ///   (meaning a unique orthogonal `y_axis` cannot be determined).
    pub fn try_new(origin: Point3D, x_axis: Point3D, y_axis: Point3D) -> Option<Self> {
        if is_approx_zero(x_axis) || is_approx_zero(y_axis) || is_parallel(x_axis, y_axis) {
            return None;
        }
        Some(Self {
            origin,
            x_axis: x_axis.normalize(),
            y_axis: orthogonalize(y_axis, x_axis)?,
        })
    }

    /// Converts the plane into a Scad modifier that translates and rotates
    /// a target object to align its local coordinate system with this plane.
    ///
    /// The target object's local origin (0,0,0) will be moved to the plane's
    /// origin, and its local X and Y axes will be aligned with the plane's
    /// X and Y axes, respectively.
    ///
    /// # Arguments
    ///
    /// * `target` - The Scad object to apply the transformation to.
    ///
    /// # Returns
    ///
    /// Returns a `ScadObject3D` representing the input `target` object
    /// transformed by the plane's translation and rotation.
    pub fn as_modifier(&self, target: ScadObject3D) -> ScadObject3D {
        let (rot_x, rot_y, rot_z) = {
            // Calculate the Z axis as the cross product of X and Y
            let z_axis = self.x_axis.cross(&self.y_axis);
            // Create a rotation matrix from the orthogonal axes
            let rot_matrix = na::Matrix3::from_columns(&[self.x_axis, self.y_axis, z_axis]);
            let rot = na::Rotation3::from_matrix(&rot_matrix);
            to_openscad_rotate_angles(&rot)
        };

        // Apply rotation first, then translation
        Translate3D::build_with(|tb| {
            let _ = tb.v(self.origin);
        })
        .apply_to(
            Rotate3D::build_with(|rb| {
                // Convert radians to degrees for OpenSCAD
                let _ = rb.rad([rot_x, rot_y, rot_z]);
            })
            .apply_to(target),
        )
        .commented(&format!(
            "as plane o:[{}, {}, {}], x:[{}, {}, {}], y: [{}, {}, {}]",
            self.origin.x,
            self.origin.y,
            self.origin.z,
            self.x_axis.x,
            self.x_axis.y,
            self.x_axis.z,
            self.y_axis.x,
            self.y_axis.y,
            self.y_axis.z,
        ))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::shadow_unrelated)]

    use crate::test_utils::assert_approx_eq_vec;

    use super::*;
    use nalgebra as na;

    const TOLERANCE: f64 = 1e-3; // Use a slightly larger tolerance for comparisons

    #[test]
    fn test_is_approx_zero_vec() {
        assert!(is_approx_zero(na::vector![0., 0., 0.]));
        assert!(is_approx_zero(na::vector![1e-11, -1e-11, 1e-11])); // Within tolerance
        assert!(!is_approx_zero(na::vector![1e-9, 0., 0.])); // Outside tolerance
        assert!(!is_approx_zero(na::vector![1., 0., 0.]));
    }

    #[test]
    fn test_is_parallel() {
        // Parallel
        assert!(is_parallel(
            na::vector![1., 0., 0.],
            na::vector![2., 0., 0.]
        ));
        assert!(is_parallel(
            na::vector![1., 2., 3.],
            na::vector![2., 4., 6.]
        ));
        assert!(is_parallel(
            na::vector![1., 0., 0.],
            na::vector![-2., 0., 0.]
        ));
        assert!(is_parallel(
            na::vector![1., 2., 3.],
            na::vector![-2., -4., -6.]
        ));
        // Orthogonal
        assert!(!is_parallel(
            na::vector![1., 0., 0.],
            na::vector![0., 1., 0.]
        ));
        // Neither parallel nor orthogonal
        assert!(!is_parallel(
            na::vector![1., 1., 0.],
            na::vector![1., 0., 1.]
        ));
        // Zero vector cases
        assert!(!is_parallel(
            na::vector![0., 0., 0.],
            na::vector![1., 0., 0.]
        ));
        assert!(!is_parallel(
            na::vector![1., 0., 0.],
            na::vector![0., 0., 0.]
        ));
        assert!(!is_parallel(
            na::vector![0., 0., 0.],
            na::vector![0., 0., 0.]
        ));
        // Close to parallel
        assert!(is_parallel(
            na::vector![1., 0., 0.],
            na::vector![1. + 1e-11, 0., 0.]
        ));
        assert!(is_parallel(
            na::vector![1., 0., 0.],
            na::vector![-1. - 1e-11, 0., 0.]
        ));
    }

    #[test]
    fn test_orthogonalize() {
        // Orthogonal vectors
        let target = na::vector![0., 1., 0.];
        let axis = na::vector![1., 0., 0.];
        let result = orthogonalize(target, axis);
        assert!(result.is_some());
        assert_approx_eq_vec(result.unwrap(), na::vector![0., 1., 0.], TOLERANCE);

        // Non-orthogonal vectors
        let target = na::vector![1., 1., 0.];
        let axis = na::vector![1., 0., 0.];
        let result = orthogonalize(target, axis);
        assert!(result.is_some());
        assert_approx_eq_vec(result.unwrap(), na::vector![0., 1., 0.], TOLERANCE); // Component orthogonal to axis
        let target = na::vector![1., -1., 0.];
        let axis = na::vector![1., 0., 0.];
        let result = orthogonalize(target, axis);
        assert!(result.is_some());
        assert_approx_eq_vec(result.unwrap(), na::vector![0., -1., 0.], TOLERANCE); // Component orthogonal to axis

        let target = na::vector![1., 1., 1.];
        let axis = na::vector![1., 0., 0.];
        let result = orthogonalize(target, axis);
        assert!(result.is_some());
        assert_approx_eq_vec(
            result.unwrap(),
            na::vector![0., 1., 1.].normalize(),
            TOLERANCE,
        );

        // Parallel vectors
        let target = na::vector![2., 0., 0.];
        let axis = na::vector![1., 0., 0.];
        assert!(orthogonalize(target, axis).is_none());

        // Anti-parallel vectors
        let target = na::vector![-2., 0., 0.];
        let axis = na::vector![1., 0., 0.];
        assert!(orthogonalize(target, axis).is_none());

        // Target is zero
        let target = na::vector![0., 0., 0.];
        let axis = na::vector![1., 0., 0.];
        assert!(orthogonalize(target, axis).is_none());

        // Axis is zero
        let target = na::vector![1., 0., 0.];
        let axis = na::vector![0., 0., 0.];
        assert!(orthogonalize(target, axis).is_none());

        // Both are zero
        let target = na::vector![0., 0., 0.];
        let axis = na::vector![0., 0., 0.];
        assert!(orthogonalize(target, axis).is_none());

        // Target is close to parallel
        let target = na::vector![1. + 1e-11, 1e-12, 0.]; // Slightly off parallel
        let axis = na::vector![1., 0., 0.];
        // The orthogonal component is very small, should be treated as parallel
        assert!(orthogonalize(target, axis).is_none());
    }

    #[test]
    fn test_plane_try_new() {
        let origin = na::vector![1., 2., 3.];

        // Valid orthogonal axes
        let x_axis = na::vector![1., 0., 0.];
        let y_axis = na::vector![0., 1., 0.];
        let plane_opt = Plane::try_new(origin, x_axis, y_axis);
        assert!(plane_opt.is_some());
        let plane = plane_opt.unwrap();
        assert_approx_eq_vec(plane.origin, origin, TOLERANCE);
        assert_approx_eq_vec(plane.x_axis, x_axis.normalize(), TOLERANCE);
        assert_approx_eq_vec(plane.y_axis, y_axis.normalize(), TOLERANCE); // Already orthogonal

        // Valid non-orthogonal axes
        let x_axis = na::vector![1., 0., 0.];
        let y_axis = na::vector![1., 1., 0.]; // Not orthogonal to x_axis
        let plane_opt = Plane::try_new(origin, x_axis, y_axis);
        assert!(plane_opt.is_some());
        let plane = plane_opt.unwrap();
        assert_approx_eq_vec(plane.origin, origin, TOLERANCE);
        // y_axis should be orthogonalized w.r.t x_axis and normalized
        assert_approx_eq_vec(plane.y_axis, na::vector![0., 1., 0.].normalize(), TOLERANCE);

        let x_axis = na::vector![1., 0., 0.];
        let y_axis = na::vector![1., -1., 0.]; // Not orthogonal to x_axis
        let plane_opt = Plane::try_new(origin, x_axis, y_axis);
        assert!(plane_opt.is_some());
        let plane = plane_opt.unwrap();
        assert_approx_eq_vec(plane.origin, origin, TOLERANCE);
        // y_axis should be orthogonalized w.r.t x_axis and normalized
        assert_approx_eq_vec(
            plane.y_axis,
            na::vector![0., -1., 0.].normalize(),
            TOLERANCE,
        );

        // Parallel axes
        let x_axis = na::vector![1., 0., 0.];
        let y_axis = na::vector![2., 0., 0.];
        assert!(Plane::try_new(origin, x_axis, y_axis).is_none());

        // Anti-parallel axes
        let x_axis = na::vector![1., 0., 0.];
        let y_axis = na::vector![-1., 0., 0.];
        assert!(Plane::try_new(origin, x_axis, y_axis).is_none());

        // One zero axis (x)
        let x_axis = na::vector![0., 0., 0.];
        let y_axis = na::vector![0., 1., 0.];
        assert!(Plane::try_new(origin, x_axis, y_axis).is_none());

        // One zero axis (y)
        let x_axis = na::vector![1., 0., 0.];
        let y_axis = na::vector![0., 0., 0.];
        assert!(Plane::try_new(origin, x_axis, y_axis).is_none());

        // Both zero axes
        let x_axis = na::vector![0., 0., 0.];
        let y_axis = na::vector![0., 0., 0.];
        assert!(Plane::try_new(origin, x_axis, y_axis).is_none());

        // Axes that become parallel after normalization/orthogonalization
        let x_axis = na::vector![1., 1., 0.];
        let y_axis = na::vector![2., 2., 0.]; // Parallel to x_axis
        assert!(Plane::try_new(origin, x_axis, y_axis).is_none());
    }
    //
    #[test]
    fn test_plane_as_modifier() {
        let target_object = Sphere::build_with(|sb| {
            let _ = sb.r(2.);
        });

        {
            let origin = na::vector![10., 20., 30.];
            let x_axis = na::vector![0., 0., 1.];
            let y_axis = na::vector![0., 1., 0.];
            let plane = Plane::try_new(origin, x_axis, y_axis).unwrap();

            let modified_object = plane.as_modifier(target_object.clone());
            assert_eq!(
                modified_object.to_code(),
                r"/* as plane o:[10, 20, 30], x:[0, 0, 1], y: [0, 1, 0] */
translate([10, 20, 30])
  rotate(a = [0, -89.99999915, 0])
    sphere(r = 2);
"
            );
        }

        {
            let origin = na::vector![5., -3., 12.];
            let x_axis = na::vector![0.43152905, 0.89608213, 0.10401677];
            let y_axis = na::vector![-0.17069653, 0.19433216, -0.96596983];
            let plane = Plane::try_new(origin, x_axis, y_axis).unwrap();

            let modified_object = plane.as_modifier(target_object);
            assert_eq!(
                modified_object.to_code(),
                r"/* as plane o:[5, -3, 12], x:[0.4315290514800427, 0.89608213307335, 0.10401677035675296], y: [-0.17069653067490384, 0.19433215688782346, -0.9659698272797542] */
translate([5, -3, 12])
  rotate(a = [-76.22499996, -5.97052146, 64.28578488])
    sphere(r = 2);
"
            );
        }
    }
}
