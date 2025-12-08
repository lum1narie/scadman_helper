//! Geometry builders.
//!
//! This module provides functions for creating common geometric shapes
//! like cuboids and squares from different input parameters.

use approx::abs_diff_eq;
use scadman::prelude::*;

/// Creates a cuboid defined by two opposite corner points.
///
/// The function calculates the minimum and maximum coordinates
/// from the two input points to determine the size and position of the cuboid.
///
/// # Arguments
///
/// * `p0` - The first corner point of the cuboid.
/// * `p1` - The second corner point of the cuboid (opposite to `p0`).
///
/// # Returns
///
/// A `ScadObject` representing the generated cuboid.
pub fn cuboid_from_to(p0: Point3D, p1: Point3D) -> ScadObject3D {
    let from = Point3D::new(p0.x.min(p1.x), p0.y.min(p1.y), p0.z.min(p1.z));
    let to = Point3D::new(p0.x.max(p1.x), p0.y.max(p1.y), p0.z.max(p1.z));

    let c = Cube::build_with(|cb| {
        let _ = cb.size(to - from).center(false);
    });
    let norm = abs_diff_eq!(from.norm(), 0.);

    if norm {
        c
    } else {
        Translate3D::build_with(|tb| {
            let _ = tb.v(from);
        })
        .apply_to(c)
    }
    .commented(&format!(
        "cuboid_from_to([{}, {}, {}], [{}, {}, {}])",
        p0.x, p0.y, p0.z, p1.x, p1.y, p1.z
    ))
}

/// Creates a square defined by two opposite corner points.
///
/// The function calculates the minimum and maximum coordinates
/// from the two input points to determine the size and position of the square.
///
/// # Arguments
///
/// * `p0` - The first corner point of the square.
/// * `p1` - The second corner point of the square (opposite to `p0`).
///
/// # Returns
///
/// A `ScadObject` representing the generated square.
pub fn square_from_to(p0: Point2D, p1: Point2D) -> ScadObject2D {
    let from = Point2D::new(p0.x.min(p1.x), p0.y.min(p1.y));
    let to = Point2D::new(p0.x.max(p1.x), p0.y.max(p1.y));

    let c = Square::build_with(|sb| {
        let _ = sb.size(to - from).center(false);
    });
    let norm = abs_diff_eq!(from.norm(), 0.);

    if norm {
        c
    } else {
        Translate2D::build_with(|tb| {
            let _ = tb.v(from);
        })
        .apply_to(c)
    }
    .commented(&format!(
        "square_from_to([{}, {}], [{}, {}])",
        p0.x, p0.y, p1.x, p1.y
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cuboid_from_to() {
        let p0 = Point3D::new(1., 4., 3.);
        let p1 = Point3D::new(5., -2., 7.);
        let c = cuboid_from_to(p0, p1);

        assert_eq!(
            c.to_code(),
            r"/* cuboid_from_to([1, 4, 3], [5, -2, 7]) */
translate([1, -2, 3])
  cube(size = [4, 6, 4], center = false);
"
        );
    }

    #[test]
    fn test_square_from_to() {
        let p0 = Point2D::new(1., 4.);
        let p1 = Point2D::new(5., -2.);
        let s = square_from_to(p0, p1);

        assert_eq!(
            s.to_code(),
            r"/* square_from_to([1, 4], [5, -2]) */
translate([1, -2])
  square(size = [4, 6], center = false);
"
        );
    }
}
