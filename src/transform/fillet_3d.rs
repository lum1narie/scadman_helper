//! Utilities for applying edge fillets to 3D shapes.

use scadman::prelude::*;

use crate::geometry::plane::Plane;

use nalgebra as na;

const PARALLEL_EPS: Unit = 1e-10;
const SMALL_OVERLAP: f64 = 0.025;

/// The default resolution used for 3D fillet arcs.
pub const DEFAULT_FILLET_FN: u64 = 64;

/// Computes the edge-aligned frame and projected face directions.
///
/// # Arguments
///
/// - `from`: Start point of the edge.
/// - `to`: End point of the edge.
/// - `y_fwd`: Face direction in the `from -> to` frame.
/// - `y_inv`: Face direction in the `to -> from` frame.
///
/// # Returns
///
/// `(edge_len, rotation, v0, v1)` where:
/// - `edge_len` is the edge length.
/// - `rotation` maps local XY plane to the edge-aligned plane.
/// - `v0`, `v1` are projected 2D face directions in local coordinates.
///
/// # Panics
///
/// Panics if:
/// - `from` and `to` are identical (zero-length edge),
/// - either plane cannot be constructed from the given vectors.
fn edge_frame(
    from: Point3D,
    to: Point3D,
    y_fwd: Point3D,
    y_inv: Point3D,
) -> (f64, na::Rotation3<f64>, na::Vector2<f64>, na::Vector2<f64>) {
    let edge = to - from;
    let edge_len = edge.norm();
    assert!(edge_len > PARALLEL_EPS, "edge length must be non-zero");

    let p0 = Plane::try_new(from, edge, y_fwd).expect("invalid plane with y_fwd");
    let p1 = Plane::try_new(from, edge, y_inv).expect("invalid plane with y_inv");

    let rot =
        na::Rotation3::rotation_between(&Point3D::z(), &edge).expect("invalid rotation for edge");
    let inv_rot = rot.inverse();
    let v0 = (inv_rot * p0.y_axis()).xy();
    let v1 = (inv_rot * p1.y_axis()).xy();

    (edge_len, rot, v0, v1)
}

/// Computes key 2D points of the fillet profile from two face vectors.
///
/// # Arguments
///
/// - `v0`: First face direction projected to local XY.
/// - `v1`: Second face direction projected to local XY.
/// - `r`: Fillet radius.
///
/// # Returns
///
/// `(o, h0, h1)` where:
/// - `o` is the fillet circle center in local XY.
/// - `h0`, `h1` are tangent points on each face direction.
///
/// # Panics
///
/// Panics if:
/// - either direction vector is degenerate,
/// - the two vectors are nearly parallel or anti-parallel.
fn fillet_profile_points(
    v0: na::Vector2<f64>,
    v1: na::Vector2<f64>,
    r: f64,
) -> (na::Vector2<f64>, na::Vector2<f64>, na::Vector2<f64>) {
    let n0 = v0.norm();
    let n1 = v1.norm();
    assert!(n0 > PARALLEL_EPS, "v0 must be non-zero");
    assert!(n1 > PARALLEL_EPS, "v1 must be non-zero");

    let u0 = v0 / n0;
    let u1 = v1 / n1;
    let bisector = u0 + u1;
    let bisector_norm = bisector.norm();
    assert!(
        bisector_norm > PARALLEL_EPS,
        "face directions are nearly opposite"
    );

    let cos_theta = u0.dot(&u1).clamp(-1.0, 1.0);
    let half_angle = ((1.0 - cos_theta) / 2.0).sqrt();
    let sin_theta = cos_theta.mul_add(-cos_theta, 1.0).sqrt();
    assert!(
        half_angle > PARALLEL_EPS && sin_theta > PARALLEL_EPS,
        "face directions are nearly parallel"
    );

    let lo = r / half_angle;
    let o = bisector / bisector_norm * lo;

    let lh = r * (1.0 + cos_theta) / sin_theta;
    let h0 = u0 * lh;
    let h1 = u1 * lh;

    (o, h0, h1)
}

/// Builds the 2D fillet wedge used for both convex and concave shapes.
///
/// # Arguments
///
/// - `o`: Fillet center in local XY.
/// - `h0`: Tangent point on first face.
/// - `h1`: Tangent point on second face.
/// - `r`: Fillet radius.
///
/// # Returns
///
/// A 2D shape representing the wedge minus the inner arc circle.
fn fillet_shape_2d(
    o: na::Vector2<f64>,
    h0: na::Vector2<f64>,
    h1: na::Vector2<f64>,
    r: f64,
) -> ScadObject2D {
    let outer = Polygon::build_with(|pb| {
        let overlap = -o / o.norm() * SMALL_OVERLAP;
        let _ = pb.points(vec![overlap, h0, o, h1]);
    });
    let inner = Translate2D::build_with(|tb| {
        let _ = tb.v(o);
    })
    .apply_to(Circle::build_with(|cb| {
        let _ = cb.r(r).r#fn(DEFAULT_FILLET_FN);
    }));

    outer - inner
}

fn fillet_plane(from: Point3D, rot: na::Rotation3<f64>) -> Plane {
    Plane::try_new(from, rot * Point3D::x(), rot * Point3D::y()).expect("invalid fillet plane")
}

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
    let (edge_len, rot, v0, v1) = edge_frame(from, to, y_fwd, y_inv);
    let (o, h0, h1) = fillet_profile_points(v0, v1, r);
    let void_shape_2d = fillet_shape_2d(o, h0, h1, r);
    let void_plane = fillet_plane(from, rot);
    let void_shape = void_plane.as_modifier(
        Translate3D::build_with(|tb| {
            let _ = tb.v([0., 0., -SMALL_OVERLAP]);
        })
        .apply_to(
            LinearExtrude::build_with(|eb| {
                let _ = eb.height(2.0_f64.mul_add(SMALL_OVERLAP, edge_len));
            })
            .apply_to(void_shape_2d),
        ),
    );

    (target - void_shape).commented(&format!(
        "convex_fillet_edge from:({}, {}, {}), to:({}, {}, {}), y_fwd:({}, {}, {}), y_inv:({}, {}, {}), r:{}",
        from.x, from.y, from.z, to.x, to.y, to.z, y_fwd.x, y_fwd.y, y_fwd.z, y_inv.x, y_inv.y, y_inv.z, r
    ))
}

/// Applies a concave fillet to one edge of a 3D shape.
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
/// A new shape with the concave fillet added to `target`.
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
pub fn concave_fillet_edge(
    target: ScadObject3D,
    from: Point3D,
    to: Point3D,
    y_fwd: Point3D,
    y_inv: Point3D,
    r: f64,
) -> ScadObject3D {
    let (edge_len, rot, v0, v1) = edge_frame(from, to, y_fwd, y_inv);
    let (o, h0, h1) = fillet_profile_points(v0, v1, r);
    let fill_shape_2d = fillet_shape_2d(o, h0, h1, r);
    let fill_plane = fillet_plane(from, rot);
    let fill_shape = fill_plane.as_modifier(
        LinearExtrude::build_with(|eb| {
            let _ = eb.height(edge_len);
        })
        .apply_to(fill_shape_2d),
    );

    (target + fill_shape).commented(&format!(
        "concave_fillet_edge from:({}, {}, {}), to:({}, {}, {}), y_fwd:({}, {}, {}), y_inv:({}, {}, {}), r:{}",
        from.x, from.y, from.z, to.x, to.y, to.z, y_fwd.x, y_fwd.y, y_fwd.z, y_inv.x, y_inv.y, y_inv.z, r
    ))
}

#[cfg(test)]
mod tests {
    use crate::geometry::builders::square_from_to;
    use std::panic::AssertUnwindSafe;

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

        assert_eq!(
            shape.to_code(),
            r"/* convex_fillet_edge from:(1, 5, 5), to:(-5, 5, 5), y_fwd:(0, -1, 0), y_inv:(0, 0, -1), r:1 */
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
"
        );
    }

    #[test]
    fn test_concave_fillet_edge_basic() {
        let target = {
            let shape_2d = Polygon::build_with(|pb| {
                let _ = pb.points(vec![[-5., 0.], [0., 0.], [10., 10.], [5., 10.]]);
            }) + square_from_to(Point2D::new(-5., 0.), Point2D::new(10., 5.));

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
        let y1_0 = Point3D::new(-1., 0., 0.);
        let y2_0 = Point3D::new(1., 0., 1.);
        let r_0 = 1.0;

        let from_1 = Point3D::new(5., 0., 5.);
        let to_1 = Point3D::new(5., 5., 5.);
        let y1_1 = Point3D::new(1., 0., 1.);
        let y2_1 = Point3D::new(1., 0., 0.);
        let r_1 = 1.0;

        let target = concave_fillet_edge(target, from_0, to_0, y1_0, y2_0, r_0);
        let target = concave_fillet_edge(target, from_1, to_1, y1_1, y2_1, r_1);

        assert_eq!(
            target.to_code(),
            r"/* concave_fillet_edge from:(5, 0, 5), to:(5, 5, 5), y_fwd:(1, 0, 1), y_inv:(1, 0, 0), r:1 */
union() {
  /* concave_fillet_edge from:(0, 0, 5), to:(0, 5, 5), y_fwd:(-1, 0, 0), y_inv:(1, 0, 1), r:1 */
  union() {
    mirror([0, 1, 0])
      rotate(a = [90, 0, 0])
        linear_extrude(height = 5)
          union() {
            polygon(points = [[-5, 0], [0, 0], [10, 10], [5, 10]]);
            /* square_from_to([-5, 0], [10, 5]) */
            translate([-5, 0])
              square(size = [15, 5], center = false);
          }
    /* as plane o:[0, 0, 5], x:[1, 0, 0], y:[0, 0.00000000000000006123233995736766, -1] */
    translate([0, 0, 5])
      rotate(a = [-90, 0, 0])
        linear_extrude(height = 5)
          difference() {
            polygon(points = [[0.00956709, 0.02309699], [-0.41421356, 0], [-0.41421356, -1], [0.29289322, -0.29289322]]);
            translate([-0.41421356, -1])
              circle(r = 1, $fn = 64);
          }
  }
  /* as plane o:[5, 0, 5], x:[1, 0, 0], y:[0, 0.00000000000000006123233995736766, -1] */
  translate([5, 0, 5])
    rotate(a = [-90, 0, 0])
      linear_extrude(height = 5)
        difference() {
          polygon(points = [[-0.02309699, 0.00956709], [1.70710678, -1.70710678], [2.41421356, -1], [2.41421356, 0]]);
          translate([2.41421356, -1])
            circle(r = 1, $fn = 64);
        }
}
"
        );
    }

    #[test]
    fn test_convex_fillet_edge_panics_on_parallel_faces() {
        let target = Cube::build_with(|cb| {
            let _ = cb.size([5.0, 5.0, 5.0]);
        });
        let from = Point3D::new(0.0, 0.0, 0.0);
        let to = Point3D::new(5.0, 0.0, 0.0);
        let y_fwd = Point3D::new(0.0, 1.0, 0.0);
        let y_inv = Point3D::new(0.0, 2.0, 0.0);

        let result = std::panic::catch_unwind(AssertUnwindSafe(|| {
            drop(convex_fillet_edge(target, from, to, y_fwd, y_inv, 1.0));
        }));

        assert!(result.is_err());
    }

    #[test]
    fn test_concave_fillet_edge_panics_on_opposite_faces() {
        let target = Cube::build_with(|cb| {
            let _ = cb.size([5.0, 5.0, 5.0]);
        });
        let from = Point3D::new(0.0, 0.0, 0.0);
        let to = Point3D::new(5.0, 0.0, 0.0);
        let y_fwd = Point3D::new(0.0, 1.0, 0.0);
        let y_inv = Point3D::new(0.0, -1.0, 0.0);

        let result = std::panic::catch_unwind(AssertUnwindSafe(|| {
            drop(concave_fillet_edge(target, from, to, y_fwd, y_inv, 1.0));
        }));

        assert!(result.is_err());
    }
}
