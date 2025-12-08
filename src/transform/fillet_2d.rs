//! High-level utilities for creating smooth, rounded corners (fillets) on 2D shapes.
//! This module provides helpers that combine offset operations
//! to produce inward, outward, and composed fillets.

use scadman::prelude::*;

/// Apply an inward fillet to a 2D object.
///
/// Arguments:
/// * `target` - The input 2D object to fillet.
/// * `r` - The fillet radius (positive value).
///
/// Returns:
/// * A new `ScadObject` representing the inward-fillet result.
pub fn inner_fillet(target: ScadObject2D, r: Unit) -> ScadObject2D {
    Offset::build_with(|ob| {
        let _ = ob.r(-r).r#fn(24_u64);
    })
    .apply_to(
        Offset::build_with(|ob| {
            let _ = ob.r(r).r#fn(24_u64);
        })
        .apply_to(target),
    )
    .commented(&format!("inner_fillet({r})"))
}

/// Apply an outward fillet to a 2D object.
///
/// Arguments:
/// * `target` - The input 2D object to fillet.
/// * `r` - The fillet radius (positive value).
///
/// Returns:
/// * A new `ScadObject` representing the outward-fillet result.
pub fn outer_fillet(target: ScadObject2D, r: Unit) -> ScadObject2D {
    Offset::build_with(|ob| {
        let _ = ob.r(r).r#fn(24_u64);
    })
    .apply_to(
        Offset::build_with(|ob| {
            let _ = ob.r(-r).r#fn(24_u64);
        })
        .apply_to(target),
    )
    .commented(&format!("outer_fillet({r})"))
}

/// Apply an outward then inward fillet (normalize with rounded corners).
///
/// Arguments:
/// * `target` - The input 2D object to fillet.
/// * `r` - The fillet radius (positive value).
///
/// Returns:
/// * A new `ScadObject` representing the composed fillet result.
pub fn both_fillet(target: ScadObject2D, r: Unit) -> ScadObject2D {
    inner_fillet(outer_fillet(target, r), r)
}

#[cfg(test)]
mod tests {
    use crate::transform::map::map_translate_2d;

    use super::*;

    fn make_test_target() -> ScadObject2D {
        let sq = Square::build_with(|sb| {
            let _ = sb.size([2.0, 2.0]);
        });

        Hull::new().apply_to_2d(ScadObject2D::from(map_translate_2d(
            sq.clone(),
            &[Point2D::new(0., 0.), Point2D::new(10., 10.)],
        ))) + Hull::new().apply_to_2d(ScadObject2D::from(map_translate_2d(
            sq,
            &[Point2D::new(10., 0.), Point2D::new(0., 10.)],
        )))
    }

    #[test]
    fn test_inner_fillet_basic() {
        let target = make_test_target();
        let r = 0.5;
        assert_eq!(
            inner_fillet(target, r).to_code(),
            r"/* inner_fillet(0.5) */
offset(r = -0.5, $fn = 24)
  offset(r = 0.5, $fn = 24)
    union() {
      hull() {
        translate([0, 0])
          square(size = [2, 2]);
        translate([10, 10])
          square(size = [2, 2]);
      }
      hull() {
        translate([10, 0])
          square(size = [2, 2]);
        translate([0, 10])
          square(size = [2, 2]);
      }
    }
"
        );
    }

    #[test]
    fn test_outer_fillet_basic() {
        let target = make_test_target();
        let r = 1.0;
        assert_eq!(
            outer_fillet(target, r).to_code(),
            r"/* outer_fillet(1) */
offset(r = 1, $fn = 24)
  offset(r = -1, $fn = 24)
    union() {
      hull() {
        translate([0, 0])
          square(size = [2, 2]);
        translate([10, 10])
          square(size = [2, 2]);
      }
      hull() {
        translate([10, 0])
          square(size = [2, 2]);
        translate([0, 10])
          square(size = [2, 2]);
      }
    }
"
        );
    }

    #[test]
    fn test_both_fillet_basic() {
        let target = make_test_target();
        let r = 0.75;
        assert_eq!(
            both_fillet(target, r).to_code(),
            r"/* inner_fillet(0.75) */
offset(r = -0.75, $fn = 24)
  offset(r = 0.75, $fn = 24)
    /* outer_fillet(0.75) */
    offset(r = 0.75, $fn = 24)
      offset(r = -0.75, $fn = 24)
        union() {
          hull() {
            translate([0, 0])
              square(size = [2, 2]);
            translate([10, 10])
              square(size = [2, 2]);
          }
          hull() {
            translate([10, 0])
              square(size = [2, 2]);
            translate([0, 10])
              square(size = [2, 2]);
          }
        }
"
        );
    }
}
