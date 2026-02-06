//! Utilities for applying edge fillets to 3D shapes.

use scadman::prelude::*;

use crate::geometry::plane::Plane;

use nalgebra as na;

const PARALLEL_EPS: Unit = 1e-10;
const SMALL_OVERLAP: f64 = 0.025;

/// The default resolution used for 3D fillet arcs.
pub const DEFAULT_FILLET_FN: u64 = 64;

/// Applies a convex fillet to one edge of a 3D shape.
///
/// `from` and `to` define the target edge.
///
/// `y_fwd` is the local Y direction of one face touching the edge when
/// the local X direction is `from -> to`.
///
/// `y_inv` is the local Y direction of the other touching face when
/// the local X direction is `to -> from`.
///
/// The `Plane` axis direction follows the same orientation rules as
/// [`Plane::try_new`].
///
/// # Arguments
///
/// - `target`: Base shape to be filleted.
/// - `from`: Start point of the edge.
/// - `to`: End point of the edge.
/// - `y_fwd`: Face direction vector for the `from -> to` frame.
/// - `y_inv`: Face direction vector for the `to -> from` frame.
/// - `r`: Fillet radius.
///
/// # Returns
///
/// A new shape with the convex fillet cut from `target`.
///
/// # Panics
///
/// Panics if:
/// - `from` and `to` are identical (zero-length edge),
/// - either plane cannot be constructed from the given vectors.
///
/// # Notes
///
/// `y_fwd` and `y_inv` must point to the real face directions as seen
/// from `from`.
///
/// If either vector is flipped to the opposite direction, this function
/// may cut the wrong side and produce unexpected geometry.
pub fn convex_fillet_edge(
    target: ScadObject3D,
    from: Point3D,
    to: Point3D,
    y_fwd: Point3D,
    y_inv: Point3D,
    r: f64,
) -> ScadObject3D {
    const N: u64 = 64;
    let e = to - from;
    let n = e.norm();
    assert!(e.norm() > 1e-10);

    let p0 = Plane::try_new(from, e, y_fwd).expect("invalid plane with y_fwd");
    let p1 = Plane::try_new(from, e, y_inv).expect("invalid plane with y_inv");

    let rot = na::Rotation3::rotation_between(&Point3D::z(), &e).unwrap();
    let inv_rot = rot.inverse();

    let v0 = (inv_rot * p0.y_axis()).xy();
    let v1 = (inv_rot * p1.y_axis()).xy();

    let n0 = v0.norm();
    let n1 = v1.norm();
    let b = v0 / n0 + v1 / n1;
    let cos_theta = v0.dot(&v1) / (n0 * n1);
    let lo = r / ((1.0 - cos_theta) / 2.0).sqrt();

    let o = b / b.norm() * lo;

    let lh = r * (1. + cos_theta) / (1. - cos_theta.powi(2)).sqrt();
    let h0 = v0 / n0 * lh;
    let h1 = v1 / n1 * lh;

    let void_shape_2d = {
        let outer = Polygon::build_with(|pb| {
            let _ = pb.points(vec![(-o / o.norm() * SMALL_OVERLAP), h0, o, h1]);
        });
        let inner = Translate2D::build_with(|tb| {
            let _ = tb.v(o);
        })
        .apply_to(Circle::build_with(|cb| {
            let _ = cb.r(r).r#fn(DEFAULT_FILLET_FN);
        }));
        outer - inner
    };
    let void_plane = Plane::try_new(from, rot * Point3D::x(), rot * Point3D::y()).unwrap();
    let void_shape = void_plane.as_modifier(
        Translate3D::build_with(|tb| {
            let _ = tb.v([0., 0., -SMALL_OVERLAP]);
        })
        .apply_to(
            LinearExtrude::build_with(|eb| {
                let _ = eb.height(n + 2. * SMALL_OVERLAP);
            })
            .apply_to(void_shape_2d),
        ),
    );

    (target - void_shape).commented(&format!(
        "convex_fillet_edge from:({}, {}, {}), to:({}, {}, {}), y_fwd:({}, {}, {}), y_inv:({}, {}, {}), r:{}",
        from.x, from.y, from.z, to.x, to.y, to.z, y_fwd.x, y_fwd.y, y_fwd.z, y_inv.x, y_inv.y, y_inv.z, r
    ))
}

/// TODO: doc
pub fn concave_fillet_edge(
    target: ScadObject3D,
    from: Point3D,
    to: Point3D,
    y0: Point3D,
    y1: Point3D,
    r: f64,
) -> ScadObject3D {
    const N: u64 = 64;
    todo!()
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum FilletKind {
    Convex,
    Concave,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convex_fillet_edge_basic() {
        let target = Cube::build_with(|cb| {
            let _ = cb.size([5.0, 5.0, 5.0]);
        });
        let from = Point3D::new(0., 0., 0.);
        let to = Point3D::new(5., 0., 0.);
        let y1 = Point3D::new(0., 0., 1.);
        let y2 = Point3D::new(0., 1., 0.);
        let r = 1.0;

        let code = convex_fillet_edge(target, from, to, y1, y2, r).to_code();

        assert_eq!(
            code,
            r"/* convex_fillet_edge from:(0, 0, 0), to:(5, 0, 0), y_fwd:(0, 0, 1), y_inv:(0, 1, 0), r:1 */
difference() {
  cube(size = [5, 5, 5]);
  /* as plane o:[0, 0, 0], x:[0.00000000000000006123233995736766, 0, -1], y:[0, 1, 0] */
  translate([0, 0, 0])
    rotate(a = [0, 90, 0])
      translate([0, 0, -0.025])
        linear_extrude(height = 5.05)
          difference() {
            polygon(points = [[0.01767767, -0.01767767], [-1, 0], [-1, 1], [0, 1]]);
            translate([-1, 1])
              circle(r = 1, $fn = 64);
          }
}
"
        );

        let target = {
            let shape_2d = Polygon::build_with(|pb| {
                let _ = pb.points(vec![[0., 0.], [-5., 5.], [0., 5.], [5., 0.]]);
            });
            Mirror3D::build_with(|mb| {
                let _ = mb.v([0., 1., 0.]);
            })
            .apply_to(
                Rotate3D::build_with(|rb| {
                    let _ = rb.deg([90., 0., 0.]);
                })
                .apply_to(
                    LinearExtrude::build_with(|eb| {
                        let _ = eb.height(5.);
                    })
                    .apply_to(shape_2d),
                ),
            )
        };
        let from_0 = Point3D::new(0., 0., 5.);
        let to_0 = Point3D::new(0., 5., 5.);
        let y1_0 = Point3D::new(1., 0., -1.);
        let y2_0 = Point3D::new(-1., 0., 0.);
        let r_0 = 1.0;

        let from_1 = Point3D::new(-5., 5., 5.);
        let to_1 = Point3D::new(-5., 0., 5.);
        let y1_1 = Point3D::new(1., 0., 0.);
        let y2_1 = Point3D::new(1., 0., -1.);
        let r_1 = 1.0;

        let from_2 = Point3D::new(1.0, 5., 5.);
        let to_2 = Point3D::new(-5.0, 5., 5.);
        let y1_2 = Point3D::new(0., -1., 0.);
        let y2_2 = Point3D::new(0., 0., -1.);
        let r_2 = 1.0;

        let shape = convex_fillet_edge(target, from_0, to_0, y1_0, y2_0, r_0);
        let shape = convex_fillet_edge(shape, from_1, to_1, y1_1, y2_1, r_1);
        let shape = convex_fillet_edge(shape, from_2, to_2, y1_2, y2_2, r_2);

        assert_eq!(shape.to_code(), r"/* convex_fillet_edge from:(1, 5, 5), to:(-5, 5, 5), y_fwd:(0, -1, 0), y_inv:(0, 0, -1), r:1 */
difference() {
  /* convex_fillet_edge from:(-5, 5, 5), to:(-5, 0, 5), y_fwd:(1, 0, 0), y_inv:(1, 0, -1), r:1 */
  difference() {
    /* convex_fillet_edge from:(0, 0, 5), to:(0, 5, 5), y_fwd:(1, 0, -1), y_inv:(-1, 0, 0), r:1 */
    difference() {
      mirror([0, 1, 0])
        rotate(a = [90, 0, 0])
          linear_extrude(height = 5)
            polygon(points = [[0, 0], [-5, 5], [0, 5], [5, 0]]);
      /* as plane o:[0, 0, 5], x:[1, 0, 0], y:[0, 0.00000000000000006123233995736766, -1] */
      translate([0, 0, 5])
        rotate(a = [-90, 0, 0])
          translate([0, 0, -0.025])
            linear_extrude(height = 5.05)
              difference() {
                polygon(points = [[0.00956709, -0.02309699], [0.29289322, 0.29289322], [-0.41421356, 1], [-0.41421356, 0]]);
                translate([-0.41421356, 1])
                  circle(r = 1, $fn = 64);
              }
    }
    /* as plane o:[-5, 5, 5], x:[1, 0, 0], y:[0, 0.00000000000000006123233995736766, 1] */
    translate([-5, 5, 5])
      rotate(a = [90, 0, 0])
        translate([0, 0, -0.025])
          linear_extrude(height = 5.05)
            difference() {
              polygon(points = [[-0.02309699, 0.00956709], [2.41421356, 0], [2.41421356, -1], [1.70710678, -1.70710678]]);
              translate([2.41421356, -1])
                circle(r = 1, $fn = 64);
            }
  }
  /* as plane o:[1, 5, 5], x:[0.00000000000000006123233995736766, 0, 1], y:[0, 1, 0] */
  translate([1, 5, 5])
    rotate(a = [0, -90, 0])
      translate([0, 0, -0.025])
        linear_extrude(height = 6.05)
          difference() {
            polygon(points = [[0.01767767, 0.01767767], [0, -1], [-1, -1], [-1, 0]]);
            translate([-1, -1])
              circle(r = 1, $fn = 64);
          }
}
");
    }

    #[test]
    fn test_concave_fillet_edge_basic() {}
}
