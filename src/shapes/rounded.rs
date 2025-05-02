use itertools::iproduct;
use scadman::prelude::*;

pub fn rounded_cuboid(size: Point3D, radius: Unit) -> ScadObject {
    assert!(radius > 0.);
    assert!(size.x >= radius * 2.);
    assert!(size.y >= radius * 2.);
    assert!(size.z >= radius * 2.);
    let xs = [radius, size.x - radius];
    let ys = [radius, size.y - radius];
    let zs = [radius, size.z - radius];

    let positions = iproduct!(&xs, &ys, &zs)
        .map(|(&x, &y, &z)| [x, y, z])
        .collect::<Vec<_>>();

    modifier_3d_commented(
        Hull::new(),
        block_3d(
            &positions
                .into_iter()
                .map(|v| {
                    modifier_3d(
                        Translate3D::build_with(|tb| {
                            let _ = tb.v(v);
                        }),
                        primitive_3d(Sphere::build_with(|sb| {
                            let _ = sb.r(radius).r#fn(64_u64);
                        })),
                    )
                })
                .collect::<Vec<_>>(),
        ),
        &format!(
            "rounded_cuboid([{}, {}, {}], {radius})",
            size.x, size.y, size.z
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rounded_cuboid() {
        assert_eq!(
            rounded_cuboid(Point3D::new(5., 3., 4.), 1.).to_code(),
            r#"/* rounded_cuboid([5, 3, 4], 1) */
hull() {
  translate([1, 1, 1])
    sphere(r = 1, $fn = 64);
  translate([1, 1, 3])
    sphere(r = 1, $fn = 64);
  translate([1, 2, 1])
    sphere(r = 1, $fn = 64);
  translate([1, 2, 3])
    sphere(r = 1, $fn = 64);
  translate([4, 1, 1])
    sphere(r = 1, $fn = 64);
  translate([4, 1, 3])
    sphere(r = 1, $fn = 64);
  translate([4, 2, 1])
    sphere(r = 1, $fn = 64);
  translate([4, 2, 3])
    sphere(r = 1, $fn = 64);
}
"#
        );
    }

    #[test]
    #[should_panic]
    fn test_rounded_cuboid_radius_fail() {
        drop(rounded_cuboid(Point3D::new(5., 3., 4.), -0.1));
    }

    #[test]
    #[should_panic]
    fn test_rounded_cuboid_size_fail() {
        drop(rounded_cuboid(Point3D::new(5., 2.9, 4.), 1.5));
    }
}
