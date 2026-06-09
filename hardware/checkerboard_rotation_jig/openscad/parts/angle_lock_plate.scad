include <../params.scad>;

function polar_from_back(radius_mm, angle_deg) = [sin(angle_deg) * radius_mm, -cos(angle_deg) * radius_mm];

module ring_sector_2d(inner_radius_mm, outer_radius_mm, start_deg, end_deg, step_deg = 2) {
    polygon(points = concat(
        [for (angle = [start_deg : step_deg : end_deg]) polar_from_back(outer_radius_mm, angle)],
        [for (angle = [end_deg : -step_deg : start_deg]) polar_from_back(inner_radius_mm, angle)]
    ));
}

module angle_lock_plate() {
    start_deg = min(angle_list_deg) - 8;
    end_deg = max(angle_list_deg) + 8;

    difference() {
        union() {
            linear_extrude(height = angle_plate_thickness_mm)
                ring_sector_2d(
                    angle_plate_radius_mm - angle_plate_band_width_mm / 2,
                    angle_plate_radius_mm + angle_plate_band_width_mm / 2,
                    start_deg,
                    end_deg
                );

            cylinder(h = angle_plate_hub_height_mm, d = angle_plate_hub_outer_diameter_mm);

            for (brace_angle = [start_deg, 0, end_deg]) {
                hull() {
                    translate([0, 0, angle_plate_thickness_mm / 2])
                        cylinder(h = angle_plate_thickness_mm, d = angle_plate_hub_outer_diameter_mm - 6, center = true);

                    translate([
                        polar_from_back(angle_plate_radius_mm, brace_angle)[0],
                        polar_from_back(angle_plate_radius_mm, brace_angle)[1],
                        angle_plate_thickness_mm / 2
                    ]) cylinder(h = angle_plate_thickness_mm, d = 12, center = true);
                }
            }
        }

        translate([0, 0, -1]) cylinder(h = angle_plate_hub_height_mm + 2, d = shaft_bore_diameter_mm);

        translate([0, -angle_plate_hub_slit_width_mm / 2, -1])
            cube([
                angle_plate_hub_outer_diameter_mm / 2 + 1,
                angle_plate_hub_slit_width_mm,
                angle_plate_hub_height_mm + 2
            ]);

        translate([0, angle_plate_hub_outer_diameter_mm / 4, angle_plate_hub_height_mm / 2])
            rotate([0, 90, 0])
                cylinder(h = angle_plate_hub_outer_diameter_mm + 2, d = m4_clearance_mm, center = true);

        for (angle = angle_list_deg) {
            translate([
                polar_from_back(angle_plate_radius_mm, angle)[0],
                polar_from_back(angle_plate_radius_mm, angle)[1],
                -1
            ]) cylinder(h = angle_plate_thickness_mm + 2, d = angle_pin_diameter_mm + angle_pin_clearance_mm);
        }
    }
}

module angle_lock_pin_knob() {
    difference() {
        union() {
            cylinder(h = angle_pin_handle_thickness_mm, d = angle_pin_handle_diameter_mm);
            translate([0, 0, angle_pin_handle_thickness_mm]) cylinder(h = angle_pin_length_mm, d = angle_pin_diameter_mm);
            translate([0, 0, angle_pin_handle_thickness_mm + angle_pin_length_mm])
                cylinder(h = 4, d1 = angle_pin_diameter_mm * 1.4, d2 = angle_pin_diameter_mm);
        }

        translate([0, 0, angle_pin_handle_thickness_mm / 2])
            rotate([90, 0, 0])
                cylinder(h = angle_pin_handle_diameter_mm + 2, d = 3, center = true);
    }
}

module angle_lock_plate_set() {
    angle_lock_plate();
    translate([angle_plate_radius_mm + 30, 0, 0]) angle_lock_pin_knob();
}
