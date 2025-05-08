use nalgebra as na;
use scadman::prelude::*;

pub mod geometry;
pub mod shapes;

/// Converts a 3D rotation matrix into Euler angles (XYZ) in radians.
///
/// # Arguments
///
/// * `r` - A reference to a nalgebra 3D rotation matrix ([`Rotation3<Unit>`]).
///
/// # Returns
///
/// A tuple of three [`scadman::Unit`] values representing the extrinsic Euler angles (XYZ).
///
/// # Details
///
/// The function uses the [`nalgebra::euler_angles_ordered`] method
/// with the standard X, Y, Z axes order to extract the angles.
/// The angles are returned in the order corresponding
/// to rotations around the X, Y, and Z axes respectively.
#[inline]
pub fn to_openscad_rotate_angles(r: &na::Rotation3<Unit>) -> (Unit, Unit, Unit) {
    static n: [na::Unit<na::Vector3<Unit>>; 3] = [
        na::Unit::new_unchecked(na::Vector3::new(1.0, 0.0, 0.0)),
        na::Unit::new_unchecked(na::Vector3::new(0.0, 1.0, 0.0)),
        na::Unit::new_unchecked(na::Vector3::new(0.0, 0.0, 1.0)),
    ];
    let (angles, _) = r.euler_angles_ordered(n, true);
    (angles[0], angles[1], angles[2])
}
