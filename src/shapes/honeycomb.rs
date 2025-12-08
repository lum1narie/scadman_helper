//! Provides structures and functions for creating honeycomb patterns.

use std::f64::consts::PI;

use scadman::prelude::*;

use crate::transform::map::map_translate_3d;

const OVERLAP_SMALL: Unit = 5e-2;

/// Represents a honeycomb structure with specified dimensions and parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Honeycomb {
    /// The diameter of each hexagonal cell (distance across parallel edges).
    pub d: Unit,
    /// The thickness of the walls between hexagonal cells.
    pub t: Unit,

    /// The overall width of the honeycomb pattern.
    pub width: Unit,
    /// The overall height of the honeycomb pattern.
    pub height: Unit,
    /// The depth of the honeycomb pattern.
    pub depth: Unit,

    /// The distance from the outer edge of the pattern
    /// to the outermost edge of the hexagonal cells.
    pub vesel: Unit,
}

impl Honeycomb {
    /// Creates a new `HoneyComb` instance.
    ///
    /// # Arguments
    ///
    /// * `d` - The diameter of each hexagonal cell.
    ///   distance across parallel edges).
    /// * `t` - The thickness of the walls between cells.
    /// * `width` - The overall width of the pattern.
    /// * `height` - The overall height of the pattern.
    /// * `depth` - The depth of the pattern.
    /// * `vesel` - The distance from the edge
    ///   the outermost edge of the hexagonal cells.
    ///
    /// # Returns
    ///
    /// A new `Honeycomb` instance.
    pub const fn new(
        d: Unit,
        t: Unit,
        width: Unit,
        height: Unit,
        depth: Unit,
        vesel: Unit,
    ) -> Self {
        Self {
            d,
            t,
            width,
            height,
            depth,
            vesel,
        }
    }

    /// Creates the bounding box for the honeycomb holes.
    ///
    /// This is used to limit the area where hexagons are placed.
    ///
    /// # Returns
    ///
    /// A `ScadObject` representing the bounding box.
    #[inline]
    fn create_boundary(&self) -> ScadObject3D {
        Translate3D::build_with(|tb| {
            let _ =
                tb.v(Point3D::new(0., 0., -self.depth) + Point3D::new(self.vesel, self.vesel, 0.));
        })
        .apply_to(Cube::build_with(|cb| {
            let _ = cb.size(
                Point3D::new(self.width, self.height, self.depth)
                    - 2. * Point3D::new(self.vesel, self.vesel, 0.)
                    + Point3D::new(0., 0., OVERLAP_SMALL),
            );
        }))
    }

    /// Creates a single hexagonal cylinder primitive.
    ///
    /// This primitive is then translated and unioned to form the full pattern.
    ///
    /// # Returns
    ///
    /// A `ScadObject` representing a single hexagonal cylinder.
    #[inline]
    fn create_hex_primitive(&self) -> ScadObject3D {
        let hex_d = self.d / (PI / 6.).cos();
        Translate3D::build_with(|tb| {
            let _ = tb.v(Point3D::new(0., 0., -self.depth) + Point3D::new(0., 0., -OVERLAP_SMALL));
        })
        .apply_to(
            Rotate3D::build_with(|rb| {
                let _ = rb.deg([0., 0., 30.]);
            })
            .apply_to(Cylinder::build_with(|cb| {
                let _ = cb
                    .d(hex_d)
                    .h(3.0_f64.mul_add(OVERLAP_SMALL, self.depth))
                    .r#fn(6_u64);
            })),
        )
    }

    /// Calculates the midpoints for each hexagonal cell
    /// within the defined boundary.
    ///
    /// # Arguments
    ///
    /// * `offset` - The 2D offset to apply to the starting point
    ///   of the hexagon grid.
    ///
    /// # Returns
    ///
    /// A `[Vec<Point3D>]` representing the midpoints of the hexagonal cells.
    /// Returns an empty vector if the `vesel` is too large
    /// for any hexagons to fit.
    fn calculate_hex_mid_points(&self, offset: Point2D) -> Vec<Point3D> {
        if self.vesel >= (self.width.min(self.height) / 2.) {
            return Vec::new();
        }

        // Distance between the two closest midpoints
        let dist = self.d + self.t;
        let hex_r = self.d / (PI / 6.).cos() / 2.;
        // Vector connecting the two closest midpoints along the x-axis
        let v0 = Point3D::new(dist, 0., 0.);
        // Vector connecting the two closest midpoints
        // at 120 degrees to the x-axis
        let v1 = Point3D::new(-dist * (PI / 3.).cos(), dist * (PI / 3.).sin(), 0.);
        let x_lim = [self.vesel - hex_r, self.width - self.vesel + hex_r];
        let y_lim = [self.vesel - hex_r, self.height - self.vesel + hex_r];

        // leftmost midpoint in the row
        let mut left = {
            let mut p = Point3D::new(offset.x, offset.y, 0.);
            if offset.y > 0. {
                p -= v1 * (offset.y / v1.y).ceil();
            }
            if offset.x > 0. {
                p -= v0 * (offset.x / v0.x).ceil();
            }
            p
        };
        let mut result = Vec::<Point3D>::new();

        // send `left` up if the vesel is big enough
        if left.y < y_lim[0] {
            left += v1 * ((y_lim[0] - left.y) / v1.y).ceil();
        }

        // add midpoints for each row
        while left.y <= y_lim[1] - 1e-10 {
            // send `left` right if it is out of the area
            if left.x < x_lim[0] {
                left += v0 * ((x_lim[0] - left.x) / v0.x).ceil();
            }
            let mut p = left;
            // scan the midpoints in the row
            while p.x <= x_lim[1] - 1e-10 {
                result.push(p);
                p += v0;
            }
            // send `left` up
            left += v1;
        }
        result
    }

    /// Generates an OpenSCAD object
    /// representing the holes of the honeycomb pattern.
    ///
    /// # Returns
    ///
    /// A `ScadObject` representing the honeycomb holes.
    pub fn holes_as_primitve(&self) -> ScadObject3D {
        let boundary = self.create_boundary();
        let hex = self.create_hex_primitive();
        let hex_mid_points = self.calculate_hex_mid_points(Point2D::zeros());
        let hexes = map_translate_3d(hex, &hex_mid_points);

        (boundary * Union::new().apply_to_3d(hexes))
            .commented(&format!("{self:?}.holes_as_primitve()"))
    }

    /// Generates an OpenSCAD object
    /// representing the holes of the honeycomb pattern.
    ///
    /// This function generates honeycomb holes
    /// similarly to `holes_as_primitve()`,
    /// but differs in that it allows applying a custom offset
    /// to the hexagon grid layout.
    /// `holes_as_primitve()` implicitly uses an offset of `(0,0)`.
    ///
    /// # Arguments
    ///
    /// * `offset` - A `Point2D` offset to apply to the hexyagon grid.
    ///
    /// # Returns
    ///
    /// A `ScadObject` representing the honeycomb holes.
    pub fn holes_as_primitve_with_offset(&self, offset: Point2D) -> ScadObject3D {
        let boundary = self.create_boundary();
        let hex = self.create_hex_primitive();
        let hex_mid_points = self.calculate_hex_mid_points(offset);
        let hexes = map_translate_3d(hex, &hex_mid_points);

        (boundary * Union::new().apply_to_3d(hexes)).commented(&format!(
            "{self:?}.holes_as_primitve_with_offset([{}, {}])",
            offset.x, offset.y
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::assert_approx_eq_vec;

    // Helper function for comparing vectors of Point3D with a tolerance
    fn assert_vec_point3d_approx_eq(actual: &[Point3D], expected: &[Point3D], tolerance: Unit) {
        actual
            .iter()
            .zip(expected.iter())
            .for_each(|(&a, &b)| assert_approx_eq_vec(a, b, tolerance));
    }

    #[test]
    fn test_calculate_hex_mid_points_basic() {
        let honeycomb = Honeycomb::new(6.0, 2.0, 20.0, 15.0, 5.0, 1.0);
        let offset = Point2D::zeros();
        let mid_points = honeycomb.calculate_hex_mid_points(offset);

        let expected_points = vec![
            Point3D::new(0., 0., 0.),
            Point3D::new(8., 0., 0.),
            Point3D::new(16., 0., 0.),
            Point3D::new(4., 6.92820323, 0.),
            Point3D::new(12., 6.92820323, 0.),
            Point3D::new(20., 6.92820323, 0.),
            Point3D::new(0., 13.85640646, 0.),
            Point3D::new(8., 13.85640646, 0.),
            Point3D::new(16., 13.85640646, 0.),
        ];

        assert_vec_point3d_approx_eq(&mid_points, &expected_points, 1e-8);
    }

    #[test]
    fn test_calculate_hex_mid_points_offset() {
        let honeycomb = Honeycomb::new(6.0, 2.0, 20.0, 15.0, 5.0, 1.0);
        let offset = Point2D::new(4.0, 6.0);
        let mid_points = honeycomb.calculate_hex_mid_points(offset);

        let expected_points = vec![
            Point3D::new(0., -0.9282032302755088, 0.),
            Point3D::new(8., -0.9282032302755088, 0.),
            Point3D::new(16., -0.9282032302755088, 0.),
            Point3D::new(4., 6., 0.),
            Point3D::new(12., 6., 0.),
            Point3D::new(20., 6., 0.),
            Point3D::new(0., 12.928203230275509, 0.0),
            Point3D::new(8., 12.928203230275509, 0.0),
            Point3D::new(16., 12.928203230275509, 0.0),
        ];

        assert_vec_point3d_approx_eq(&mid_points, &expected_points, 1e-8);
    }

    #[test]
    fn test_calculate_hex_mid_points_vesel_too_large() {
        // vesel is 5.0, min(width, height)/2 is 5.0. So vesel >= min/2 is true.
        let honeycomb = Honeycomb::new(6.0, 2.0, 10.0, 10.0, 5.0, 5.0);
        let offset = Point2D::zeros();
        let mid_points = honeycomb.calculate_hex_mid_points(offset);

        assert!(mid_points.is_empty());
    }

    #[test]
    fn test_calculate_hex_mid_points_small_dimensions() {
        let honeycomb = Honeycomb::new(6.0, 2.0, 10.0, 10.0, 5.0, 1.0);
        let offset = Point2D::zeros();
        let mid_points = honeycomb.calculate_hex_mid_points(offset);

        let expected_points = vec![
            Point3D::new(0., 0., 0.),
            Point3D::new(8., 0., 0.),
            Point3D::new(4., 6.92820323, 0.),
            Point3D::new(12., 6.92820323, 0.),
        ];

        assert_vec_point3d_approx_eq(&mid_points, &expected_points, 1e-8);
    }

    #[test]
    fn test_honeycomb() {
        let w = 20.;
        let h = 15.;
        let d = 5.;

        let honeycomb = Honeycomb::new(6.0, 2.0, w, h, d, 1.0);
        let hole = honeycomb.holes_as_primitve();

        assert_eq!(
            hole.to_code(),
            r"/* Honeycomb { d: 6.0, t: 2.0, width: 20.0, height: 15.0, depth: 5.0, vesel: 1.0 }.holes_as_primitve() */
intersection() {
  translate([1, 1, -5])
    cube(size = [18, 13, 5.05]);
  union() {
    translate([0, 0, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([8, 0, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([16, 0, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([4, 6.92820323, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([12, 6.92820323, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([20, 6.92820323, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([0, 13.85640646, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([8, 13.85640646, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([16, 13.85640646, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
  }
}
"
        );
    }

    #[test]
    fn test_holes_as_primitve_with_offset() {
        let w = 20.;
        let h = 15.;
        let d = 5.;

        let honeycomb = Honeycomb::new(6.0, 2.0, w, h, d, 1.0);
        let offset = Point2D::new(4.0, 3.0);
        let hole = honeycomb.holes_as_primitve_with_offset(offset);

        assert_eq!(
            hole.to_code(),
            r"/* Honeycomb { d: 6.0, t: 2.0, width: 20.0, height: 15.0, depth: 5.0, vesel: 1.0 }.holes_as_primitve_with_offset([4, 3]) */
intersection() {
  translate([1, 1, -5])
    cube(size = [18, 13, 5.05]);
  union() {
    translate([4, 3, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([12, 3, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([20, 3, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([0, 9.92820323, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([8, 9.92820323, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([16, 9.92820323, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([4, 16.85640646, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([12, 16.85640646, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
    translate([20, 16.85640646, 0])
      translate([0, 0, -5.05])
        rotate(a = [0, 0, 30])
          cylinder(h = 5.15, d = 6.92820323, $fn = 6);
  }
}
"
        );
    }
}
