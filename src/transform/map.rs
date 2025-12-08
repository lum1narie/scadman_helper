//! Provides mapping functions for applying transformations to OpenSCAD objects.

use scadman::prelude::*;

/// Applies a translation to an object for each point in a list.
///
/// It creates a new [`ScadObject`] for each vector in the `points` slice,
/// applying a translation transformation
/// based on that vector to a clone of the original object.
///
/// # Arguments
///
/// * `object` - The [`ScadObject`] to be translated.
/// * `points` - A slice of [`na::Vector3<Unit>`]
///   representing the translation vectors.
///
/// # Returns
///
/// A `Vec<ScadObject>` where each element is the original object
/// translated by one of the vectors from the `points` slice.
#[inline]
pub fn map_translate_3d(object: ScadObject3D, points: &[Point3D]) -> Vec<ScadObject3D> {
    points
        .iter()
        .map(|&p| {
            Translate3D::build_with(|tb| {
                let _ = tb.v(p);
            })
            .apply_to(object.clone())
        })
        .collect::<Vec<_>>()
}

/// Applies a translation to an object for each point in a list.
///
/// It creates a new [`ScadObject`] for each vector in the `points` slice,
/// applying a translation transformation
/// based on that vector to a clone of the original object.
///
/// # Arguments
///
/// * `object` - The [`ScadObject`] to be translated.
/// * `points` - A slice of [`na::Vector2<Unit>`]
///   representing the translation vectors.
///
/// # Returns
///
/// A `Vec<ScadObject>` where each element is the original object
/// translated by one of the vectors from the `points` slice.
#[inline]
pub fn map_translate_2d(object: ScadObject2D, points: &[Point2D]) -> Vec<ScadObject2D> {
    points
        .iter()
        .map(|&p| {
            Translate2D::build_with(|tb| {
                let _ = tb.v(p);
            })
            .apply_to(object.clone())
        })
        .collect::<Vec<_>>()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_map_translate_3d() {
        let cb = Cube::build_with(|cb| {
            let _ = cb.size(10.);
        });
        let points = [
            Point3D::new(0., 0., 0.),
            Point3D::new(20., 0., 0.),
            Point3D::new(0., 20., 0.),
            Point3D::new(0., 0., 20.),
        ];

        assert_eq!(
            ScadObject3D::from(map_translate_3d(cb, &points)).to_code(),
            r"{
  translate([0, 0, 0])
    cube(size = 10);
  translate([20, 0, 0])
    cube(size = 10);
  translate([0, 20, 0])
    cube(size = 10);
  translate([0, 0, 20])
    cube(size = 10);
}
"
        );
    }

    #[test]
    fn test_map_translate_2d() {
        let sq = Square::build_with(|sb| {
            let _ = sb.size(10.);
        });
        let points = [
            Point2D::new(0., 0.),
            Point2D::new(20., 0.),
            Point2D::new(0., 20.),
        ];

        assert_eq!(
            ScadObject2D::from(map_translate_2d(sq, &points)).to_code(),
            r"{
  translate([0, 0])
    square(size = 10);
  translate([20, 0])
    square(size = 10);
  translate([0, 20])
    square(size = 10);
}
"
        );
    }
}
