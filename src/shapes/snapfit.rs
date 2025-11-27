//! Snap-fit connector building blocks.
//!
//! This module provides  connector shapes and their corresponding cutting geometries
//! (holes / voids), and supplies small utilities (fillets, offsets, and a tiny overlap)
//! to make boolean operations robust.

use crate::geometry::builders::square_from_to;
use crate::transform::fillet_2d::{both_fillet, outer_fillet};
use scadman::prelude::*;

/// A small overlap used in boolean operation
const SMALL_OVERLAP: f64 = 0.05;

/// A snap-fit inserter.
///
/// This struct contains the parameters for generating a inserting
/// snap-fit connector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SnapFitInserter {
    /// The width of the snap arm.
    snap_width: f64,
    /// The total width of the base of snap-fit connector.
    width: f64,
    /// The depth of the base of the connector.
    base_depth: f64,
    /// The depth of the notch.
    notch_depth: f64,
    /// The total depth of the connector.
    /// Must be >= `base_depth` + `notch_depth`.
    depth: f64,
    /// The width of the notch overhang.
    notch_over_width: f64,
    /// The thickness of the connector.
    thickness: f64,
    /// The radius of the notch fillet.
    notch_fillet_size: f64,

    /// The clearance between the snap-fit and the hole.
    /// The hole is widen from the otehr parameters.
    clearance: f64,
}

impl SnapFitInserter {
    /// Validates the parameters of the snap-fit inserter.
    ///
    /// # Arguments
    ///
    /// * `val` - The value to validate.
    ///
    /// # Returns
    ///
    /// `true` if the parameters are valid, `false` otherwise.
    fn validate(val: &Self) -> bool {
        let is_positive = (val.snap_width >= 0.)
            && (val.width >= 0.)
            && (val.base_depth >= 0.)
            && (val.notch_depth >= 0.)
            && (val.depth >= 0.)
            && (val.notch_over_width >= 0.)
            && (val.thickness >= 0.)
            && (val.notch_fillet_size >= 0.)
            && (val.clearance >= 0.);
        let depth_enough = val.depth >= (val.base_depth + val.notch_depth);
        is_positive && depth_enough
    }

    /// Try to construct a new snap-fit inserter.
    ///
    /// # Returns
    ///
    /// `Some<Self>` if the parameters are valid, `None` otherwise.
    #[allow(clippy::too_many_arguments)]
    pub fn try_new(
        snap_width: f64,
        width: f64,
        base_depth: f64,
        notch_depth: f64,
        depth: f64,
        notch_over_width: f64,
        thickness: f64,
        notch_fillet_size: f64,
        clearance: f64,
    ) -> Option<Self> {
        let val = Self {
            snap_width,
            width,
            base_depth,
            notch_depth,
            depth,
            notch_over_width,
            thickness,
            notch_fillet_size,
            clearance,
        };
        Self::validate(&val).then_some(val)
    }

    /// Produce the SCAD primitive for the snap-fit.
    ///
    /// Returns:
    /// * `ScadObject` representing the solid inserter shape.
    pub fn snapfit_as_primitive(&self) -> ScadObject {
        let shape_2d = {
            let left = {
                let x_a0 = -self.notch_over_width;
                let x_a1 = 0.;
                let x_a2 = self.snap_width;
                let y_a0 = -SMALL_OVERLAP;
                let y_a1 = self.depth - self.notch_depth;
                let y_a2 = self.depth;

                let x_a0i = 2.0_f64.mul_add(self.notch_fillet_size, x_a0);
                let y_a0i = ((x_a0i - x_a0) / (x_a1 - x_a0)).mul_add(y_a2 - y_a1, y_a1);

                let base_p = vec![
                    Point2D::new(x_a1, y_a0),
                    Point2D::new(x_a2, y_a0),
                    Point2D::new(x_a2, y_a2),
                    Point2D::new(x_a1, y_a2),
                    Point2D::new(x_a0i, y_a0i),
                    Point2D::new(x_a1, y_a1),
                ];
                let round_p = vec![
                    Point2D::new(x_a0, y_a1),
                    Point2D::new(x_a2, y_a1),
                    Point2D::new(x_a1, y_a2),
                ];

                let base = primitive_2d(Polygon::build_with(|pb| {
                    let _ = pb.points(base_p);
                }));

                let round = outer_fillet(
                    &primitive_2d(Polygon::build_with(|pb| {
                        let _ = pb.points(round_p);
                    })),
                    self.notch_fillet_size,
                );
                base + round
            };

            let right = modifier_2d(
                Translate2D::build_with(|tb| {
                    let _ = tb.v(Point2D::new(self.width, 0.));
                }),
                modifier_2d(
                    Mirror2D::build_with(|mb| {
                        let _ = mb.v(Point2D::x());
                    }),
                    left.clone(),
                ),
            );
            let base = square_from_to(
                Point2D::new(0., 0.) - Point2D::new(0., SMALL_OVERLAP),
                Point2D::new(self.width, self.base_depth),
            );

            left + right + base
        };

        let shape = modifier_3d(
            LinearExtrude::build_with(|lb| {
                let _ = lb.height(self.thickness);
            }),
            shape_2d,
        );

        modifier_3d_commented(
            Mirror3D::build_with(|mb| {
                let _ = mb.v(Point3D::y());
            }),
            modifier_3d(
                Rotate3D::build_with(|rb| {
                    let _ = rb.deg([90., 0., 0.]);
                }),
                shape,
            ),
            &format!("{self:?}.snapfit_as_primitive()"),
        )
    }

    /// Produce the SCAD primitive used to cut the hole for this snap-fit.
    ///
    /// Returns:
    /// * `ScadObject` representing the hole shape.
    pub fn hole_as_primitive(&self) -> ScadObject {
        let shape_2d = {
            let base = square_from_to(
                Point2D::zeros()
                    + Point2D::new(-self.clearance, 0.)
                    + Point2D::new(0., -SMALL_OVERLAP),
                Point2D::new(self.width, self.depth) + Point2D::new(self.clearance, self.clearance),
            );
            let notch = square_from_to(
                Point2D::new(-self.notch_over_width, self.depth - self.notch_depth)
                    + Point2D::new(-self.clearance, 0.),
                Point2D::new(self.width + self.notch_over_width, self.depth)
                    + Point2D::new(self.clearance, self.clearance),
            );
            base + notch
        };

        let shape = modifier_3d(
            Translate3D::build_with(|tb| {
                let _ = tb.v(Point3D::new(0., 0., -self.clearance));
            }),
            modifier_3d(
                LinearExtrude::build_with(|lb| {
                    let _ = lb.height(2.0_f64.mul_add(self.clearance, self.thickness));
                }),
                shape_2d,
            ),
        );
        modifier_3d_commented(
            Rotate3D::build_with(|rb| {
                let _ = rb.deg([-90., 0., 0.]);
            }),
            shape,
            &format!("{self:?}.hole_as_primitive()"),
        )
    }
}

/// A nail-style snap-fit inserter.
///
/// This struct contains parameters for generating a nail-shaped
/// snap-fit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SnapFitNail {
    /// The width of the nail.
    width: f64,
    /// Longitudinal length of the nail. Length from the nail tip to its base.
    length: f64,
    /// Thickness of the nail body plate.
    thickness: f64,

    /// Length of the notch.
    notch_length: f64,
    /// Depth of the notch. Same direction as `thickness`.
    notch_depth: f64,
    /// Radius/size used to fillet the notch corner.
    notch_fillet_size: f64,

    /// Extra clearance applied to the hole geometry.
    clearance: f64,
}

impl SnapFitNail {
    /// Validate the snap-fit nail parameters.
    ///
    /// Arguments:
    /// * `val` - The value to validate.
    ///
    /// Returns:
    /// `true` if the parameters are valid, `false` otherwise.
    fn validate(val: &Self) -> bool {
        let is_positive = (val.width >= 0.)
            && (val.length >= 0.)
            && (val.thickness >= 0.)
            && (val.notch_length >= 0.)
            && (val.notch_depth >= 0.)
            && (val.notch_fillet_size >= 0.)
            && (val.clearance >= 0.);
        let base_enough = val.length > val.notch_length;
        let fillet_enough = val.notch_fillet_size
            <= [
                val.notch_depth,
                val.thickness / 2.,
                val.notch_length / 2.,
                (val.length - val.notch_length) / 2.,
            ]
            .into_iter()
            .reduce(f64::max)
            .unwrap_or_default();
        is_positive && base_enough && fillet_enough
    }

    /// Try to construct a new `SnapFitNail`.
    ///
    /// Returns:
    /// * `Some(SnapFitNail)` if parameters are valid, otherwise `None`.
    pub fn try_new(
        width: f64,
        length: f64,
        thickness: f64,
        notch_length: f64,
        notch_depth: f64,
        notch_fillet_size: f64,
        clearance: f64,
    ) -> Option<Self> {
        let val = Self {
            width,
            length,
            thickness,
            notch_length,
            notch_depth,
            notch_fillet_size,
            clearance,
        };
        Self::validate(&val).then_some(val)
    }

    /// Produce the SCAD primitive for the snap-fit.
    ///
    /// Returns:
    /// * `ScadObject` representing the solid nail part.
    pub fn snapfit_as_primitive(&self) -> ScadObject {
        let shape_2d = {
            let x_a0 = -SMALL_OVERLAP;
            let x_a1 = self.length - self.notch_length;
            let x_a2 = self.length;
            let y_a0 = 0.;
            let y_a1 = self.thickness;
            let y_a2 = self.thickness + self.notch_depth;
            let p = vec![
                Point2D::new(x_a0, y_a0),
                Point2D::new(x_a2, y_a0),
                Point2D::new(x_a2, y_a1),
                Point2D::new(x_a1, y_a2),
                Point2D::new(x_a1, y_a1),
                Point2D::new(x_a0, y_a1),
            ];

            let base_shape = primitive_2d(Polygon::build_with(|pb| {
                let _ = pb.points(p);
            }));

            let end_shape = modifier_2d(
                Translate2D::build_with(|tb| {
                    let _ = tb.v(Point2D::new(-SMALL_OVERLAP, 0.));
                }),
                primitive_2d(Square::build_with(|sb| {
                    let _ = sb.size([
                        2.0_f64.mul_add(SMALL_OVERLAP, self.notch_fillet_size),
                        self.thickness,
                    ]);
                })),
            );

            end_shape + both_fillet(&base_shape, self.notch_fillet_size)
        };

        let shape = modifier_3d(
            LinearExtrude::build_with(|lb| {
                let _ = lb.height(self.width);
            }),
            shape_2d,
        );

        modifier_3d_commented(
            Mirror3D::build_with(|mb| {
                let _ = mb.v(Point3D::x());
            }),
            modifier_3d(
                Rotate3D::build_with(|rb| {
                    let _ = rb.deg([0., -90., 0.]);
                }),
                shape,
            ),
            &format!("{self:?}.snapfit_as_primitive()"),
        )
    }

    /// Produce the SCAD primitive used to cut the hole for this nail.
    ///
    /// Returns:
    /// * `ScadObject` representing the hole shape.
    pub fn hole_as_primitive(&self) -> ScadObject {
        let shape_2d = {
            let x_a0 = -SMALL_OVERLAP;
            let x_a1 = self.length - self.notch_length - self.clearance - self.notch_fillet_size;
            let x_a2 = self.length - self.notch_length - self.clearance;
            let x_a3 = self.length + self.clearance;
            let x_a2i = x_a3 - self.notch_fillet_size;
            let y_a0 = 0.;
            let y_a1 = self.thickness + self.clearance;
            let y_a2 = self.thickness + self.notch_fillet_size;
            let y_a3 = self.thickness + self.notch_depth + self.clearance;
            let y_a2i = y_a3 - self.notch_fillet_size;
            let p = vec![
                Point2D::new(x_a0, y_a0),
                Point2D::new(x_a3, y_a0),
                Point2D::new(x_a3, y_a2i),
                Point2D::new(x_a2i, y_a3),
                Point2D::new(x_a2, y_a3),
                Point2D::new(x_a2, y_a2),
                Point2D::new(x_a1, y_a1),
                Point2D::new(x_a0, y_a1),
            ];

            let base_shape = primitive_2d(Polygon::build_with(|pb| {
                let _ = pb.points(p);
            }));

            let round_shape = modifier_2d(
                Translate2D::build_with(|tb| {
                    let _ = tb.v([x_a2i, y_a2i]);
                }),
                primitive_2d(Circle::build_with(|cb| {
                    let _ = cb.r(self.notch_fillet_size);
                })),
            );

            base_shape + round_shape
        };

        let shape = modifier_3d(
            Translate3D::build_with(|tb| {
                let _ = tb.v([0., 0., -self.clearance / 2.]);
            }),
            modifier_3d(
                LinearExtrude::build_with(|lb| {
                    let _ = lb.height(self.width + self.clearance);
                }),
                shape_2d,
            ),
        );

        modifier_3d_commented(
            Rotate3D::build_with(|rb| {
                let _ = rb.deg([0., 90., 0.]);
            }),
            shape,
            &format!("{self:?}.hole_as_primitive()"),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_inserter_snapfit() {
        let snap_ins = SnapFitInserter::try_new(1.8, 6., 2., 3., 10., 1.5, 1.8, 0.5, 0.3).unwrap();
        assert_eq!(
            snap_ins.snapfit_as_primitive().to_code(),
            r"/* SnapFitInserter { snap_width: 1.8, width: 6.0, base_depth: 2.0, notch_depth: 3.0, depth: 10.0, notch_over_width: 1.5, thickness: 1.8, notch_fillet_size: 0.5, clearance: 0.3 }.snapfit_as_primitive() */
mirror([0, 1, 0])
  rotate(a = [90, 0, 0])
    linear_extrude(height = 1.8)
      union() {
        polygon(points = [[0, -0.05], [1.8, -0.05], [1.8, 10], [0, 10], [-0.5, 9], [0, 7]]);
        /* outer_fillet(0.5) */
        offset(r = 0.5, $fn = 24)
          offset(r = -0.5, $fn = 24)
            polygon(points = [[-1.5, 7], [1.8, 7], [0, 10]]);
        translate([6, 0])
          mirror([1, 0])
            union() {
              polygon(points = [[0, -0.05], [1.8, -0.05], [1.8, 10], [0, 10], [-0.5, 9], [0, 7]]);
              /* outer_fillet(0.5) */
              offset(r = 0.5, $fn = 24)
                offset(r = -0.5, $fn = 24)
                  polygon(points = [[-1.5, 7], [1.8, 7], [0, 10]]);
            }
        /* square_from_to([0, -0.05], [6, 2]) */
        translate([0, -0.05])
          square(size = [6, 2.05], center = false);
      }
"
        );
        assert_eq!(
            snap_ins.hole_as_primitive().to_code(),
            r"/* SnapFitInserter { snap_width: 1.8, width: 6.0, base_depth: 2.0, notch_depth: 3.0, depth: 10.0, notch_over_width: 1.5, thickness: 1.8, notch_fillet_size: 0.5, clearance: 0.3 }.hole_as_primitive() */
rotate(a = [-90, 0, 0])
  translate([0, 0, -0.3])
    linear_extrude(height = 2.4)
      union() {
        /* square_from_to([-0.3, -0.05], [6.3, 10.3]) */
        translate([-0.3, -0.05])
          square(size = [6.6, 10.35], center = false);
        /* square_from_to([-1.8, 7], [7.8, 10.3]) */
        translate([-1.8, 7])
          square(size = [9.6, 3.3], center = false);
      }
"
        );
    }

    #[test]
    fn test_nail_snapfit() {
        let snap_nail = SnapFitNail::try_new(10., 10., 3., 5., 2., 0.5, 0.3).unwrap();
        assert_eq!(
            snap_nail.snapfit_as_primitive().to_code(),
            r"/* SnapFitNail { width: 10.0, length: 10.0, thickness: 3.0, notch_length: 5.0, notch_depth: 2.0, notch_fillet_size: 0.5, clearance: 0.3 }.snapfit_as_primitive() */
mirror([1, 0, 0])
  rotate(a = [0, -90, 0])
    linear_extrude(height = 10)
      union() {
        translate([-0.05, 0])
          square(size = [0.6, 3]);
        /* inner_fillet(0.5) */
        offset(r = -0.5, $fn = 24)
          offset(r = 0.5, $fn = 24)
            /* outer_fillet(0.5) */
            offset(r = 0.5, $fn = 24)
              offset(r = -0.5, $fn = 24)
                polygon(points = [[-0.05, 0], [10, 0], [10, 3], [5, 5], [5, 3], [-0.05, 3]]);
      }
"
        );
        assert_eq!(
            snap_nail.hole_as_primitive().to_code(),
            r"/* SnapFitNail { width: 10.0, length: 10.0, thickness: 3.0, notch_length: 5.0, notch_depth: 2.0, notch_fillet_size: 0.5, clearance: 0.3 }.hole_as_primitive() */
rotate(a = [0, 90, 0])
  translate([0, 0, -0.15])
    linear_extrude(height = 10.3)
      union() {
        polygon(points = [[-0.05, 0], [10.3, 0], [10.3, 4.8], [9.8, 5.3], [4.7, 5.3], [4.7, 3.5], [4.2, 3.3], [-0.05, 3.3]]);
        translate([9.8, 4.8])
          circle(r = 0.5);
      }
"
        );
    }
}
