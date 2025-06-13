//! Provides structures and functions for creating honeycomb patterns.

use std::f64::consts::PI;

use nalgebra as na;
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
    /// * `d` - The diameter of each hexagonal cell. (distance across parallel edges).
    /// * `t` - The thickness of the walls between cells.
    /// * `width` - The overall width of the pattern.
    /// * `height` - The overall height of the pattern.
    /// * `depth` - The depth of the pattern.
    /// * `vesel` - The distance from the edge
    ///             to the outermost edge of the hexagonal cells.
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
    #[inline]
    fn create_boundary(&self) -> ScadObject {
        modifier_3d(
            Translate3D::build_with(|tb| {
                let _ = tb
                    .v(Point3D::new(0., 0., -self.depth)
                        + Point3D::new(self.vesel, self.vesel, 0.));
            }),
            primitive_3d(Cube::build_with(|cb| {
                let _ = cb.size(
                    Point3D::new(self.width, self.height, self.depth)
                        - 2. * Point3D::new(self.vesel, self.vesel, 0.)
                        + Point3D::new(0., 0., OVERLAP_SMALL),
                );
            })),
        )
    }

    /// Creates a single hexagonal cylinder primitive.
    ///
    /// This primitive is then translated and unioned to form the full pattern.
    #[inline]
    fn create_hex_primitive(&self) -> ScadObject {
        let hex_d = self.d / (PI / 6.).cos();
        modifier_3d(
            Translate3D::build_with(|tb| {
                let _ =
                    tb.v(Point3D::new(0., 0., -self.depth) + Point3D::new(0., 0., -OVERLAP_SMALL));
            }),
            modifier_3d(
                Rotate3D::build_with(|rb| {
                    let _ = rb.deg([0., 0., 30.]);
                }),
                primitive_3d(Cylinder::build_with(|cb| {
                    let _ = cb
                        .d(hex_d)
                        .h(3.0_f64.mul_add(OVERLAP_SMALL, self.depth))
                        .r#fn(6_u64);
                })),
            ),
        )
    }

    /// Calculates the midpoints for each hexagonal cell within the defined boundary.
    ///
    /// Returns an empty vector if the `vesel` is too large for any hexagons to fit.
    fn calculate_hex_mid_points(&self) -> Vec<na::Vector3<f64>> {
        if self.vesel >= (self.width.min(self.height) / 2.) {
            return Vec::new();
        }

        // Distance between the two closest midpoints
        let dist = self.d + self.t;
        let hex_r = self.d / (PI / 6.).cos() / 2.;
        // Vector connecting the two closest midpoints along the x-axis
        let v0 = na::Vector3::new(dist, 0., 0.);
        // Vector connecting the two closest midpoints at 120 degrees to the x-axis
        let v1 = na::Vector3::new(-dist * (PI / 3.).cos(), dist * (PI / 3.).sin(), 0.);
        let x_lim = [self.vesel - hex_r, self.width - self.vesel + hex_r];
        let y_lim = [self.vesel - hex_r, self.height - self.vesel + hex_r];

        // leftmost midpoint in the row
        let mut left = na::Vector3::zeros();
        let mut result = Vec::<na::Vector3<f64>>::new();

        // send `left` up if the vesel is big enough
        while left.y < y_lim[0] - 1e-10 {
            left += v1;
        }

        // add midpoints for each row
        while left.y <= y_lim[1] - 1e-10 {
            // send `left` right if it is out of the area
            while left.x < x_lim[0] - 1e-10 {
                left += v0;
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

    /// Generates an OpenSCAD object representing the holes of the honeycomb pattern.
    pub fn holes_as_primitve(&self) -> ScadObject {
        let boundary = self.create_boundary();
        let hex = self.create_hex_primitive();
        let hex_mid_points = self.calculate_hex_mid_points();
        let hexes = map_translate_3d(&hex, &hex_mid_points);

        (boundary * modifier_3d(Union::new(), block_3d(&hexes)))
            .commented(&format!("{self:?}.holes_as_primitve()"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
