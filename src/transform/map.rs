//! Provides mapping functions for applying transformations to OpenSCAD objects.

use nalgebra as na;
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
///              representing the translation vectors.
///
/// # Returns
///
/// A `Vec<ScadObject>` where each element is the original object
/// translated by one of the vectors from the `points` slice.
#[inline]
pub fn map_translate_3d(object: &ScadObject, points: &[na::Vector3<Unit>]) -> Vec<ScadObject> {
    points
        .iter()
        .map(|&p| {
            modifier_3d(
                Translate3D::build_with(|tb| {
                    let _ = tb.v(p);
                }),
                object.clone(),
            )
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
///              representing the translation vectors.
///
/// # Returns
///
/// A `Vec<ScadObject>` where each element is the original object
/// translated by one of the vectors from the `points` slice.
#[inline]
pub fn map_translate_2d(object: &ScadObject, points: &[na::Vector2<Unit>]) -> Vec<ScadObject> {
    points
        .iter()
        .map(|&p| {
            modifier_2d(
                Translate2D::build_with(|tb| {
                    let _ = tb.v(p);
                }),
                object.clone(),
            )
        })
        .collect::<Vec<_>>()
}
