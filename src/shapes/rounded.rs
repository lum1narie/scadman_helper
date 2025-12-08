//! Provides functions for creating rounded shapes.
//!
//! This module contains implementations for generating shapes with rounded edges
//! and corners, such as rounded cuboids.

use itertools::iproduct;
use scadman::prelude::*;

/// Creates a rounded cuboid.
///
/// This function generates a rounded cuboid by hulling spheres,
/// placed at the corners of the cuboid's core
/// (the cuboid size minus twice the radius).
///
/// # Arguments
///
/// * `size` - The total size of the cuboid including the rounded corners.
/// * `radius` - The radius of the rounding spheres.
///   Must be positive and less than or equal to
///   half of the smallest dimension of `size`.
///
/// # Returns
///
/// A [`ScadObject`] representing the rounded cuboid.
///
/// # Panics
///
/// Panics if `radius` is not positive
/// or if any dimension of `size` is less than `radius * 2.`.
pub fn rounded_cuboid(size: Point3D, radius: Unit) -> ScadObject3D {
    assert!(radius > 0.);
    assert!(size.x >= radius * 2.);
    assert!(size.y >= radius * 2.);
    assert!(size.z >= radius * 2.);
    let xs = [radius, size.x - radius];
    let ys = [radius, size.y - radius];
    let zs = [radius, size.z - radius];

    let positions_iter = iproduct!(&xs, &ys, &zs).map(|(&x, &y, &z)| [x, y, z]);

    Hull::new()
        .apply_to_3d(
            positions_iter
                .map(|v| {
                    Translate3D::build_with(|tb| {
                        let _ = tb.v(v);
                    })
                    .apply_to(Sphere::build_with(|sb| {
                        let _ = sb.r(radius).r#fn(64_u64);
                    }))
                })
                .collect::<Vec<_>>(),
        )
        .commented(&format!(
            "rounded_cuboid([{}, {}, {}], {radius})",
            size.x, size.y, size.z
        ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rounded_cuboid() {
        assert_eq!(
            rounded_cuboid(Point3D::new(5., 3., 4.), 1.).to_code(),
            r"/* rounded_cuboid([5, 3, 4], 1) */
hull() {
  translate([1, 1, 1])
    sphere(r = 1, $fn = 64);
  translate([1, 1, 3])
    sphere(r = 1, $fn = 64);
  translate([1, 2, 1])
    sphere(r = 1, $fn = 64);
  translate([1, 2, 3])
    sphere(r = 1, $fn = 64);
  translate([4, 1, 1])
    sphere(r = 1, $fn = 64);
  translate([4, 1, 3])
    sphere(r = 1, $fn = 64);
  translate([4, 2, 1])
    sphere(r = 1, $fn = 64);
  translate([4, 2, 3])
    sphere(r = 1, $fn = 64);
}
"
        );
    }

    #[test]
    #[should_panic]
    fn test_rounded_cuboid_radius_fail() {
        drop(rounded_cuboid(Point3D::new(5., 3., 4.), -0.1));
    }

    #[test]
    #[should_panic]
    fn test_rounded_cuboid_size_fail() {
        drop(rounded_cuboid(Point3D::new(5., 2.9, 4.), 1.5));
    }
}
