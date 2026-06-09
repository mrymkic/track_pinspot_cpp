include <../params.scad>;

module bearing_block_common() {
    difference() {
        union() {
            translate(
                [-bearing_block_width_mm / 2, -bearing_block_depth_mm / 2, 0]
            ) cube([bearing_block_width_mm, bearing_block_depth_mm, bearing_block_height_mm]);

            for (x = [-bearing_mount_spacing_x_mm / 2, bearing_mount_spacing_x_mm / 2]) {
                translate([x, 0, bearing_block_height_mm / 2])
                    cylinder(h = bearing_block_height_mm, d = 14, center = true);
            }
        }

        translate([0, 0, -1]) cylinder(h = bearing_block_height_mm + 2, d = shaft_bore_diameter_mm);

        for (x = [-bearing_mount_spacing_x_mm / 2, bearing_mount_spacing_x_mm / 2]) {
            translate([x, 0, -1]) cylinder(h = bearing_block_height_mm + 2, d = m5_clearance_mm);
        }
    }
}

module bearing_block_top() {
    bearing_block_common();
}

module bearing_block_bottom() {
    bearing_block_common();
}

module shaft() {
    cylinder(h = shaft_length_mm, d = shaft_diameter_mm);
}

module shaft_stop_collar() {
    difference() {
        cylinder(h = shaft_collar_thickness_mm, d = shaft_collar_outer_diameter_mm);

        translate([0, 0, -1]) cylinder(h = shaft_collar_thickness_mm + 2, d = shaft_bore_diameter_mm);

        translate([0, -shaft_collar_slit_width_mm / 2, -1])
            cube([
                shaft_collar_outer_diameter_mm / 2 + 1,
                shaft_collar_slit_width_mm,
                shaft_collar_thickness_mm + 2
            ]);

        translate([0, shaft_collar_outer_diameter_mm / 4, shaft_collar_thickness_mm / 2])
            rotate([0, 90, 0])
                cylinder(
                    h = shaft_collar_outer_diameter_mm + 2,
                    d = shaft_collar_bolt_diameter_mm,
                    center = true
                );
    }
}

module shaft_with_collars() {
    shaft();

    translate([0, 0, shaft_end_margin_mm - shaft_collar_thickness_mm / 2]) shaft_stop_collar();
    translate([0, 0, shaft_length_mm - shaft_end_margin_mm - shaft_collar_thickness_mm / 2])
        shaft_stop_collar();
}

module shaft_print_layout() {
    shaft();
    translate([shaft_collar_outer_diameter_mm + 10, 0, 0]) shaft_stop_collar();
    translate([2 * (shaft_collar_outer_diameter_mm + 10), 0, 0]) shaft_stop_collar();
}
