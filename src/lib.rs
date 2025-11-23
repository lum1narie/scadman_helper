//! Helper library for creating 3D drawings using [`scadman`].
//!
//! This crate provides tools and structures to define 3D geometry and shapes,
//! and convert them into OpenSCAD syntax using the [`scadman`] crate.

use nalgebra as na;
use scadman::prelude::*;

pub mod geometry;
pub mod shapes;
pub mod transform;

#[cfg(test)]
mod test_utils;

/// Converts a 3D rotation matrix into Euler angles (XYZ) in radians.
///
/// # Arguments
///
/// * `r` - A reference to a nalgebra 3D rotation matrix ([`nalgebra::Rotation3<Unit>`]).
///
/// # Returns
///
/// A tuple of three [`scadman::Unit`] values representing the extrinsic Euler angles (XYZ).
///
/// # Details
///
/// The function uses the [`nalgebra::Rotation3<Unit>::euler_angles_ordered()`] method
/// with the standard X, Y, Z axes order to extract the angles.
/// The angles are returned in the order corresponding
/// to rotations around the X, Y, and Z axes respectively.
#[inline]
pub fn to_openscad_rotate_angles(r: &na::Rotation3<Unit>) -> (Unit, Unit, Unit) {
    static ORDER: [na::Unit<na::Vector3<Unit>>; 3] = [
        na::Unit::new_unchecked(na::Vector3::new(1.0, 0.0, 0.0)),
        na::Unit::new_unchecked(na::Vector3::new(0.0, 1.0, 0.0)),
        na::Unit::new_unchecked(na::Vector3::new(0.0, 0.0, 1.0)),
    ];
    let (angles, _) = r.euler_angles_ordered(ORDER, true);
    (angles[0], angles[1], angles[2])
}
