//! Provides structures and functions for creating OpenSCAD objects
//! representing screws and nuts, including their corresponding holes and slits.

use std::f64::consts::PI;

use float_cmp::approx_eq;
use nalgebra as na;
use scadman::prelude::*;

use crate::to_openscad_rotate_angles;

const SMALL_OVERLAP: f64 = 0.025;

/// Applies rotation to a [`ScadObject`] based on a direction vector.
/// The object is assumed to be initially aligned with the Z-axis.
fn apply_direction_rotation(object: ScadObject, direction: Point3D) -> ScadObject {
    let z_vec = na::Vector3::<f64>::z();
    let dir_norm = direction.normalize();
    let angle = z_vec.angle(&dir_norm);

    if approx_eq!(Unit, angle, 0.) {
        object
    } else if approx_eq!(Unit, angle, PI) {
        modifier_3d(
            Rotate3D::build_with(|rb| {
                let _ = rb.deg([180., 0., 0.]);
            }),
            object,
        )
    } else {
        let (rx, ry, rz) =
            to_openscad_rotate_angles(&na::Rotation3::rotation_between(&z_vec, &dir_norm).unwrap());
        modifier_3d(
            Rotate3D::build_with(|rb| {
                let _ = rb.rad([rx, ry, rz]);
            }),
            object,
        )
    }
}

/// Applies rotation to a [`ScadObject`] based on a cut vector for a side slit.
/// The object is assumed to be initially aligned with the X-axis.
fn apply_slit_rotation(object: ScadObject, cut_vector: Point3D) -> ScadObject {
    let hole_v = Point3D::x() * cut_vector.norm();
    let angle = cut_vector.angle(&hole_v);

    if approx_eq!(Unit, angle, 0.) {
        object
    } else if approx_eq!(Unit, angle, PI) {
        modifier_3d(
            Rotate3D::build_with(|rb| {
                let _ = rb.deg([0., 0., 180.]);
            }),
            object,
        )
    } else {
        let (rx, ry, rz) = to_openscad_rotate_angles(
            &na::Rotation3::rotation_between(&hole_v, &cut_vector).unwrap(),
        );
        modifier_3d(
            Rotate3D::build_with(|rb| {
                let _ = rb.rad([rx, ry, rz]);
            }),
            object,
        )
    }
}

/// Represents a screw with a body and a head.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Screw {
    /// The diameter of the screw body.
    pub diameter: Unit,
    /// The diameter of the screw head.
    pub head_diameter: Unit,
}

impl Screw {
    /// Creates a new [`Screw`] instance.
    ///
    /// # Arguments
    ///
    /// * `diameter` - The diameter of the screw body.
    /// * `head_diameter` - The diameter of the screw head.
    pub const fn new(diameter: Unit, head_diameter: Unit) -> Self {
        Self {
            diameter,
            head_diameter,
        }
    }

    /// Creates the basic cylinder for the screw body hole,
    /// including EPS and overlap.
    fn create_body_hole_cylinder(&self, depth: Unit, eps: Unit) -> ScadObject {
        let c = primitive_3d(Cylinder::build_with(|cb| {
            let _ = cb
                .h(depth + SMALL_OVERLAP)
                .d(self.diameter + eps)
                .center(false)
                .r#fn(64_u64);
        }));
        modifier_3d(
            Translate3D::build_with(|tb| {
                let _ = tb.v([0., 0., -SMALL_OVERLAP]);
            }),
            c,
        )
    }

    /// Creates the basic hexagonal cylinder for the screw head hole,
    /// including EPS and overlap.
    fn create_head_hole_cylinder(&self, head_depth: Unit, eps: Unit) -> ScadObject {
        let c = primitive_3d(Cylinder::build_with(|cb| {
            let _ = cb
                .h(head_depth + SMALL_OVERLAP)
                .d(self.head_diameter.mul_add((PI / 6.).cos(), eps))
                .center(false)
                .r#fn(6_u64);
        }));
        modifier_3d(
            Translate3D::build_with(|tb| {
                let _ = tb.v([0., 0., -SMALL_OVERLAP]);
            }),
            c,
        )
    }

    /// Creates the body cylinder for the complete screw hole,
    /// including overlap and translation relative to head.
    fn create_hole_body_cylinder(&self, depth: Unit, head_depth: Unit) -> ScadObject {
        let c = primitive_3d(Cylinder::build_with(|cb| {
            let _ = cb
                .h(depth + SMALL_OVERLAP)
                // No EPS here
                .d(self.diameter)
                .center(false)
                .r#fn(64_u64);
        }));
        modifier_3d(
            Translate3D::build_with(|tb| {
                // Translate relative to head
                let _ = tb.v([0., 0., head_depth - SMALL_OVERLAP]);
            }),
            c,
        )
    }

    /// Generates an OpenSCAD object representing a hole for the screw body.
    ///
    /// This hole is a simple cylinder with a diameter
    /// slightly larger than the screw body diameter.
    ///
    /// # Arguments
    ///
    /// * `depth` - The depth of the screw body hole. Must be positive.
    /// * `position` - The position of the top of the screw body hole.
    /// * `direction` - The vector indicating the direction of the hole.
    ///                 Must be non-zero.
    /// * `eps` - A small value added to the diameter for clearance.
    ///
    /// # Panics
    ///
    /// This function will panic if:
    ///
    /// * `depth` is not positive.
    /// * `direction` is a zero vector.
    pub fn to_body_hole(
        &self,
        depth: Unit,
        position: Point3D,
        direction: Point3D,
        eps: Unit,
    ) -> ScadObject {
        assert!(depth > 0., "depth must be positive");
        assert!(
            direction != Point3D::zeros(),
            "direction must be non-zero vector"
        );

        let hole = self.create_body_hole_cylinder(depth, eps);
        let rot = apply_direction_rotation(hole, direction);

        if approx_eq!(Unit, position.norm(), 0.) {
            rot
        } else {
            modifier_3d(
                Translate3D::build_with(|tb| {
                    let _ = tb.v(position);
                }),
                rot,
            )
        }
        .commented(&format!(
            "{self:?}.to_body_hole({depth}, [{}, {}, {}], [{}, {}, {}], {eps})",
            position.x, position.y, position.z, direction.x, direction.y, direction.z,
        ))
    }

    /// Generates an OpenSCAD object representing
    /// a complete screw hole (body + head).
    ///
    /// This hole consists of a larger hexagonal prism for the head
    /// and a smaller cylinder for the body.
    ///
    /// # Arguments
    ///
    /// * `depth` - The depth of the screw body hole. Must be positive.
    /// * `head_depth` - The depth of the screw head hole. Must be positive.
    /// * `position` - The position of the top of the screw hole.
    /// * `direction` - The vector indicating the direction of the hole.
    ///                 This vector should point
    ///                 from the top of the screw hole towards the bottom.
    ///                 Must be non-zero.
    /// * `eps` - A small value added to the diameter for clearance.
    ///
    /// # Panics
    ///
    /// This function will panic if:
    ///
    /// * `depth` is not positive.
    /// * `head_depth` is not positive.
    /// * `direction` is a zero vector.
    pub fn to_hole(
        &self,
        depth: Unit,
        head_depth: Unit,
        position: Point3D,
        direction: Point3D,
        eps: Unit,
    ) -> ScadObject {
        assert!(depth > 0., "depth must be positive");
        assert!(head_depth > 0., "head_depth must be positive");
        assert!(
            direction != Point3D::zeros(),
            "direction must be non-zero vector"
        );

        let head = self.create_head_hole_cylinder(head_depth, eps);
        let body = self.create_hole_body_cylinder(depth, head_depth);

        let combined_hole = head + body;
        let rot = apply_direction_rotation(combined_hole, direction);

        if approx_eq!(Unit, position.norm(), 0.) {
            rot
        } else {
            modifier_3d(
                Translate3D::build_with(|tb| {
                    let _ = tb.v(position);
                }),
                rot,
            )
        }
        .commented(&format!(
            "{self:?}.to_hole({depth}, {head_depth}, [{}, {}, {}], [{}, {}, {}], {eps})",
            position.x, position.y, position.z, direction.x, direction.y, direction.z,
        ))
    }
}

/// Represents a hexagonal nut.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Nut {
    /// The distance across the parallel faces of the hexagon.
    pub width_across_faces: Unit,
    /// The thickness of the nut.
    pub thickness: Unit,
}

impl Nut {
    /// Creates a new [`Nut`] instance.
    ///
    /// # Arguments
    ///
    /// * `width_across_faces` - The distance
    ///   across the parallel faces of the hexagon.
    /// * `thickness` - The thickness of the nut.
    pub const fn new(width_across_faces: Unit, thickness: Unit) -> Self {
        Self {
            width_across_faces,
            thickness,
        }
    }

    /// Creates the basic hexagonal cylinder for the nut void,
    /// including EPS and overlap.
    fn create_nut_void_cylinder(&self, thickness: Unit, eps: Unit) -> ScadObject {
        let c = primitive_3d(Cylinder::build_with(|cb| {
            let _ = cb
                .h(thickness + SMALL_OVERLAP + eps)
                .d(self.width_across_faces.mul_add((PI / 6.).cos(), eps))
                .center(false)
                .r#fn(6_u64);
        }));
        modifier_3d(
            Translate3D::build_with(|tb| {
                let _ = tb.v([0., 0., -SMALL_OVERLAP]);
            }),
            c,
        )
    }

    /// Creates the basic hexagonal cylinder for the nut side slit,
    /// including EPS and overlap, centered.
    fn create_slit_nut_cylinder(&self, eps: Unit) -> ScadObject {
        primitive_3d(Cylinder::build_with(|cb| {
            let _ = cb
                .h(self.thickness + SMALL_OVERLAP + eps)
                .d(self.width_across_faces.mul_add((PI / 6.).cos(), eps))
                // Center is true for the slit hulling
                .center(true)
                .r#fn(6_u64);
        }))
    }

    /// Generates an OpenSCAD object representing
    /// a void (hole) shaped like the nut.
    ///
    /// This void is a hexagonal prism.
    ///
    /// # Arguments
    ///
    /// * `position` - The position of the center of the nut void.
    /// * `direction` - The vector indicating the direction
    ///                 of the nut's axis (perpendicular to the faces).
    ///                 Must be non-zero.
    /// * `eps` - A small value added to the diameter for clearance.
    pub fn to_void(&self, position: Point3D, direction: Point3D, eps: Unit) -> ScadObject {
        assert!(
            direction != Point3D::zeros(),
            "direction must be non-zero vector"
        );
        let hole = self.create_nut_void_cylinder(self.thickness, eps);
        let rot = apply_direction_rotation(hole, direction);

        if approx_eq!(Unit, position.norm(), 0.) {
            rot
        } else {
            modifier_3d(
                Translate3D::build_with(|tb| {
                    let _ = tb.v(position);
                }),
                rot,
            )
        }
        .commented(&format!(
            "{self:?}.to_void([{}, {}, {}], [{}, {}, {}], {eps})",
            position.x, position.y, position.z, direction.x, direction.y, direction.z,
        ))
    }

    /// Generates an OpenSCAD object representing a side slit
    /// suitable for inserting a nut.
    ///
    /// This slit is created by hulling two hexagonal prisms,
    /// creating an elongated hexagonal shape.
    ///
    /// # Arguments
    ///
    /// * `position` - The starting position of the slit.
    /// * `cut_until` - The ending position of the slit.
    ///                 The slit will extend from `position` to `cut_until`.
    ///                 Must be different from `position`.
    /// * `eps` - A small value added to the diameter for clearance.
    ///
    /// # Panics
    ///
    /// This function will panic if `position` and `cut_until` are the same.
    pub fn to_side_slit(&self, position: Point3D, cut_until: Point3D, eps: Unit) -> ScadObject {
        assert!(
            cut_until != position,
            "`position` and `cut_until` must be different"
        );

        let cut_v = cut_until - position;
        let hole_v = Point3D::x() * cut_v.norm();

        let basic_nut_cylinder = self.create_slit_nut_cylinder(eps);

        let c0 = basic_nut_cylinder.clone();
        let c1 = modifier_3d(
            Translate3D::build_with(|tb| {
                let _ = tb.v(hole_v);
            }),
            basic_nut_cylinder,
        );
        let hole = modifier_3d(Hull::new(), block_3d(&[c0, c1]));

        let rot = apply_slit_rotation(hole, cut_v);

        if approx_eq!(Unit, position.norm(), 0.) {
            rot
        } else {
            modifier_3d(
                Translate3D::build_with(|tb| {
                    let _ = tb.v(position);
                }),
                rot,
            )
        }
        .commented(&format!(
            "{self:?}.to_void([{}, {}, {}], [{}, {}, {}], {eps})",
            position.x, position.y, position.z, cut_until.x, cut_until.y, cut_until.z,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra as na;

    #[test]
    fn test_apply_direction_rotation() {
        let dummy_object = primitive_3d(Sphere::build_with(|sb| {
            let _ = sb.r(1.0);
        }));

        // Test no rotation (direction [0, 0, 1])
        let rotated_obj_z = apply_direction_rotation(dummy_object.clone(), na::vector![0., 0., 1.]);
        assert_eq!(rotated_obj_z.to_code(), dummy_object.to_code());

        // Test 180 degree rotation (direction [0, 0, -1])
        let rotated_obj_neg_z =
            apply_direction_rotation(dummy_object.clone(), na::vector![0., 0., -1.]);
        assert_eq!(
            rotated_obj_neg_z.to_code(),
            r"rotate(a = [180, 0, 0])
  sphere(r = 1);
"
        );

        // Test 90 degree rotation (direction [1, 0, 0])
        let rotated_obj_x = apply_direction_rotation(dummy_object.clone(), na::vector![1., 0., 0.]);
        assert_eq!(
            rotated_obj_x.to_code(),
            r"rotate(a = [-0, 90, 0])
  sphere(r = 1);
"
        );

        // Test 90 degree rotation (direction [0, 1, 0])
        let rotated_obj_y = apply_direction_rotation(dummy_object.clone(), na::vector![0., 1., 0.]);
        assert_eq!(
            rotated_obj_y.to_code(),
            r"rotate(a = [-90, 0, 0])
  sphere(r = 1);
"
        );

        // Test with a non-axis direction (e.g., [1, 1, 0])
        let rotated_obj_diag = apply_direction_rotation(dummy_object, na::vector![1., 1., 0.]);
        let expected_code = r"rotate(a = [-90, 45, -45])
  sphere(r = 1);
"
        .to_string();
        assert_eq!(rotated_obj_diag.to_code(), expected_code);
    }

    #[test]
    fn test_apply_slit_rotation() {
        let dummy_object = primitive_3d(Cube::build_with(|cb| {
            let _ = cb.size([1., 1., 1.]);
        }));

        // Test no rotation (cut_vector [1, 0, 0])
        let rotated_obj_x = apply_slit_rotation(dummy_object.clone(), na::vector![1., 0., 0.]);
        assert_eq!(rotated_obj_x.to_code(), dummy_object.to_code());

        // Test 180 degree rotation (cut_vector [-1, 0, 0])
        let rotated_obj_neg_x = apply_slit_rotation(dummy_object.clone(), na::vector![-1., 0., 0.]);
        assert_eq!(
            rotated_obj_neg_x.to_code(),
            r"rotate(a = [0, 0, 180])
  cube(size = [1, 1, 1]);
"
        );

        // Test 90 degree rotation (cut_vector [0, 1, 0])
        let rotated_obj_y = apply_slit_rotation(dummy_object.clone(), na::vector![0., 1., 0.]);
        assert_eq!(
            rotated_obj_y.to_code(),
            r"rotate(a = [0, 0, 90])
  cube(size = [1, 1, 1]);
"
        );

        // Test 90 degree rotation (cut_vector [0, -1, 0])
        let rotated_obj_neg_y = apply_slit_rotation(dummy_object.clone(), na::vector![0., -1., 0.]);
        assert_eq!(
            rotated_obj_neg_y.to_code(),
            r"rotate(a = [0, 0, -90])
  cube(size = [1, 1, 1]);
"
        );

        // Test with a non-axis cut_vector (e.g., [0, 1, 1])
        let rotated_obj_diag = apply_slit_rotation(dummy_object, na::vector![0., 1., 1.]);
        assert_eq!(
            rotated_obj_diag.to_code(),
            r"rotate(a = [-45, -45, 90])
  cube(size = [1, 1, 1]);
"
        );
    }

    #[test]
    fn test_screw_to_body_hole() {
        let screw = Screw::new(3.0, 6.0);
        let depth = 10.0;
        let position = na::vector![0., 0., 0.];
        let direction = na::vector![0., 0., 1.];
        let eps = 0.3;

        let hole = screw.to_body_hole(depth, position, direction, eps);
        assert_eq!(
            hole.to_code(),
            r"/* Screw { diameter: 3.0, head_diameter: 6.0 }.to_body_hole(10, [0, 0, 0], [0, 0, 1], 0.3) */
translate([0, 0, -0.025])
  cylinder(h = 10.025, d = 3.3, center = false, $fn = 64);
"
        );

        // Test with different position and direction
        let position_shifted = na::vector![10., 20., 30.];
        let direction_x = na::vector![1., 0., 0.];
        let hole_shifted = screw.to_body_hole(depth, position_shifted, direction_x, eps);
        assert_eq!(
            hole_shifted.to_code(),
            r"/* Screw { diameter: 3.0, head_diameter: 6.0 }.to_body_hole(10, [10, 20, 30], [1, 0, 0], 0.3) */
translate([10, 20, 30])
  rotate(a = [-0, 90, 0])
    translate([0, 0, -0.025])
      cylinder(h = 10.025, d = 3.3, center = false, $fn = 64);
"
        );
    }

    #[test]
    #[should_panic(expected = "depth must be positive")]
    fn test_screw_to_body_hole_panic_depth() {
        let screw = Screw::new(3.0, 6.0);
        let depth = 0.0; // Invalid depth
        let position = na::vector![0., 0., 0.];
        let direction = na::vector![0., 0., 1.];
        let eps = 0.3;
        drop(screw.to_body_hole(depth, position, direction, eps));
    }

    #[test]
    #[should_panic(expected = "direction must be non-zero vector")]
    fn test_screw_to_body_hole_panic_direction() {
        let screw = Screw::new(3.0, 6.0);
        let depth = 10.0;
        let position = na::vector![0., 0., 0.];
        let direction = na::vector![0., 0., 0.]; // Invalid direction
        let eps = 0.3;
        drop(screw.to_body_hole(depth, position, direction, eps));
    }

    #[test]
    fn test_screw_to_hole() {
        let screw = Screw::new(3.0, 6.0);
        let depth = 10.0;
        let head_depth = 2.0;
        let position = na::vector![0., 0., 0.];
        let direction = na::vector![0., 0., -1.];
        let eps = 0.3;

        let hole = screw.to_hole(depth, head_depth, position, direction, eps);
        assert_eq!(
            hole.to_code(),
            r"/* Screw { diameter: 3.0, head_diameter: 6.0 }.to_hole(10, 2, [0, 0, 0], [0, 0, -1], 0.3) */
rotate(a = [180, 0, 0])
  union() {
    translate([0, 0, -0.025])
      cylinder(h = 2.025, d = 5.49615242, center = false, $fn = 6);
    translate([0, 0, 1.975])
      cylinder(h = 10.025, d = 3, center = false, $fn = 64);
  }
"
        );

        // Test with different position and direction
        let position_shifted = na::vector![10., 20., 30.];
        let direction_x = na::vector![1., 0., 0.];
        let hole_shifted = screw.to_hole(depth, head_depth, position_shifted, direction_x, eps);
        assert_eq!(
            hole_shifted.to_code(),
            r"/* Screw { diameter: 3.0, head_diameter: 6.0 }.to_hole(10, 2, [10, 20, 30], [1, 0, 0], 0.3) */
translate([10, 20, 30])
  rotate(a = [-0, 90, 0])
    union() {
      translate([0, 0, -0.025])
        cylinder(h = 2.025, d = 5.49615242, center = false, $fn = 6);
      translate([0, 0, 1.975])
        cylinder(h = 10.025, d = 3, center = false, $fn = 64);
    }
"
        );
    }

    #[test]
    #[should_panic(expected = "depth must be positive")]
    fn test_screw_to_hole_panic_depth() {
        let screw = Screw::new(3.0, 6.0);
        let depth = 0.0; // Invalid depth
        let head_depth = 5.0;
        let position = na::vector![0., 0., 0.];
        let direction = na::vector![0., 0., 1.];
        let eps = 0.3;
        drop(screw.to_hole(depth, head_depth, position, direction, eps));
    }

    #[test]
    #[should_panic(expected = "head_depth must be positive")]
    fn test_screw_to_hole_panic_head_depth() {
        let screw = Screw::new(3.0, 6.0);
        let depth = 10.0;
        let head_depth = 0.0; // Invalid head_depth
        let position = na::vector![0., 0., 0.];
        let direction = na::vector![0., 0., 1.];
        let eps = 0.3;
        drop(screw.to_hole(depth, head_depth, position, direction, eps));
    }

    #[test]
    #[should_panic(expected = "direction must be non-zero vector")]
    fn test_screw_to_hole_panic_direction() {
        let screw = Screw::new(3.0, 6.0);
        let depth = 10.0;
        let head_depth = 5.0;
        let position = na::vector![0., 0., 0.];
        let direction = na::vector![0., 0., 0.]; // Invalid direction
        let eps = 0.3;
        drop(screw.to_hole(depth, head_depth, position, direction, eps));
    }

    #[test]
    fn test_nut_to_void() {
        let nut = Nut::new(10.0, 5.0);
        let position = na::vector![1., 2., 3.];
        let direction = na::vector![0., 0., -1.]; // Z direction
        let eps = 0.3;

        let nut_void = nut.to_void(position, direction, eps);
        assert_eq!(
            nut_void.to_code(),
            r"/* Nut { width_across_faces: 10.0, thickness: 5.0 }.to_void([1, 2, 3], [0, 0, -1], 0.3) */
translate([1, 2, 3])
  rotate(a = [180, 0, 0])
    translate([0, 0, -0.025])
      cylinder(h = 5.325, d = 8.96025404, center = false, $fn = 6);
"
        );

        // Test with different position and direction (X direction)
        let position_shifted = na::vector![10., 20., 30.];
        let direction_x = na::vector![1., 0., 0.];
        let nut_void_shifted = nut.to_void(position_shifted, direction_x, eps);
        assert_eq!(
            nut_void_shifted.to_code(),
            r"/* Nut { width_across_faces: 10.0, thickness: 5.0 }.to_void([10, 20, 30], [1, 0, 0], 0.3) */
translate([10, 20, 30])
  rotate(a = [-0, 90, 0])
    translate([0, 0, -0.025])
      cylinder(h = 5.325, d = 8.96025404, center = false, $fn = 6);
"
        );
    }

    #[test]
    #[should_panic(expected = "direction must be non-zero vector")]
    fn test_nut_to_void_panic_direction() {
        let nut = Nut::new(10.0, 5.0);
        let position = na::vector![0., 0., 0.];
        let direction = na::vector![0., 0., 0.]; // Invalid direction
        let eps = 0.3;
        drop(nut.to_void(position, direction, eps));
    }

    #[test]
    fn test_nut_to_side_slit() {
        let nut = Nut::new(10.0, 5.0);
        let position = na::vector![0., 0., 0.];
        let cut_until = na::vector![10., 0., 0.]; // Cut 10 units in the X direction
        let eps = 0.3;

        let slit = nut.to_side_slit(position, cut_until, eps);
        assert_eq!(
            slit.to_code(),
            r"/* Nut { width_across_faces: 10.0, thickness: 5.0 }.to_void([0, 0, 0], [10, 0, 0], 0.3) */
hull() {
  cylinder(h = 5.325, d = 8.96025404, center = true, $fn = 6);
  translate([10, 0, 0])
    cylinder(h = 5.325, d = 8.96025404, center = true, $fn = 6);
}
"
        );

        // Test with different position and cut direction (Y direction)
        let position_shifted = na::vector![1., 2., 3.];
        let cut_until_shifted = na::vector![1., 12., 3.];
        let slit_shifted = nut.to_side_slit(position_shifted, cut_until_shifted, eps);
        assert_eq!(
            slit_shifted.to_code(),
            r"/* Nut { width_across_faces: 10.0, thickness: 5.0 }.to_void([1, 2, 3], [1, 12, 3], 0.3) */
translate([1, 2, 3])
  rotate(a = [0, 0, 90])
    hull() {
      cylinder(h = 5.325, d = 8.96025404, center = true, $fn = 6);
      translate([10, 0, 0])
        cylinder(h = 5.325, d = 8.96025404, center = true, $fn = 6);
    }
"
        );
    }

    #[test]
    #[should_panic(expected = "`position` and `cut_until` must be different")]
    fn test_nut_to_side_slit_panic_same_position() {
        let nut = Nut::new(10.0, 5.0);
        let position = na::vector![1., -2., 3.];
        let cut_until = na::vector![1., -2., 3.]; // Same position
        let eps = 0.3;
        drop(nut.to_side_slit(position, cut_until, eps));
    }
}
