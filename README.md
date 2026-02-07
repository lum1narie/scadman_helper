# scad-rs

`scad-rs` is a Rust library for generating OpenSCAD geometry and shapes.

## Overview

This library provides the following features:

[Documentation](https://docs.rs/scad-rs)

- Definition and manipulation of 3D geometry
- Generation of common shapes such as honeycombs, screws, and rounded cubes
- Object transformations (using translation, rotation, etc.)

## Modules

- `geometry`: Handles geometric elements such as planes.
- `shapes`: Generates predefined shapes (honeycombs, screws, etc.).
- `transform`: Provides object transformation functionalities.
  - Includes 3D edge fillet helpers. Inputs with near-parallel or
    opposite face vectors are invalid and may panic.
