include <../params.scad>;

module board_clamp_pressure_bar() {
    difference() {
        translate(
            [-clamp_front_bar_width_mm / 2, -clamp_front_bar_thickness_mm / 2, 0]
        ) cube([
            clamp_front_bar_width_mm,
            clamp_front_bar_thickness_mm,
            clamp_front_bar_height_mm
        ]);

        for (x = [-clamp_bolt_spacing_x_mm / 2, clamp_bolt_spacing_x_mm / 2]) {
            translate([x, 0, clamp_front_bar_height_mm / 2])
                rotate([90, 0, 0])
                    cylinder(
                        h = clamp_front_bar_thickness_mm + 2,
                        d = clamp_bolt_diameter_mm,
                        center = true
                    );
        }
    }
}

module board_clamp_carrier() {
    rear_plate_center_y = clamp_spine_reach_mm + clamp_back_plate_thickness_mm / 2;
    plate_center_z = clamp_back_plate_height_mm / 2;
    hub_center_z = clamp_back_plate_height_mm / 2;
    bridge_depth = max(12, rear_plate_center_y - clamp_hub_outer_diameter_mm / 4);

    difference() {
        union() {
            translate(
                [board_axis_offset_x_mm, rear_plate_center_y, plate_center_z]
            ) cube([
                clamp_back_plate_width_mm,
                clamp_back_plate_thickness_mm,
                clamp_back_plate_height_mm
            ], center = true);

            translate([0, 0, hub_center_z])
                cylinder(h = clamp_hub_height_mm, d = clamp_hub_outer_diameter_mm, center = true);

            hull() {
                translate([0, clamp_hub_outer_diameter_mm / 4, hub_center_z])
                    cylinder(h = clamp_hub_height_mm - 6, d = clamp_hub_outer_diameter_mm - 6, center = true);

                translate([
                    board_axis_offset_x_mm / 2,
                    rear_plate_center_y - clamp_back_plate_thickness_mm / 2,
                    plate_center_z
                ]) cube([
                    abs(board_axis_offset_x_mm) + 24,
                    bridge_depth,
                    clamp_back_plate_height_mm * 0.5
                ], center = true);
            }
        }

        translate([0, 0, hub_center_z])
            cylinder(h = clamp_hub_height_mm + 2, d = shaft_bore_diameter_mm, center = true);

        translate([0, -clamp_hub_slit_width_mm / 2, hub_center_z - clamp_hub_height_mm / 2 - 1])
            cube([
                clamp_hub_outer_diameter_mm / 2 + 1,
                clamp_hub_slit_width_mm,
                clamp_hub_height_mm + 2
            ]);

        translate([0, clamp_hub_outer_diameter_mm / 4, hub_center_z])
            rotate([0, 90, 0])
                cylinder(
                    h = clamp_hub_outer_diameter_mm + 2,
                    d = clamp_hub_bolt_diameter_mm,
                    center = true
                );

        translate([board_axis_offset_x_mm, rear_plate_center_y, plate_center_z])
            cube([28, clamp_back_plate_thickness_mm + 2, clamp_back_plate_height_mm - 48], center = true);

        for (z = [
            clamp_bar_edge_margin_mm,
            clamp_back_plate_height_mm / 2,
            clamp_back_plate_height_mm - clamp_bar_edge_margin_mm
        ]) {
            for (x = [board_axis_offset_x_mm - clamp_bolt_spacing_x_mm / 2,
                      board_axis_offset_x_mm + clamp_bolt_spacing_x_mm / 2]) {
                translate([x, rear_plate_center_y, z])
                    rotate([90, 0, 0])
                        cylinder(
                            h = clamp_back_plate_thickness_mm + clamp_board_gap_mm + clamp_front_bar_thickness_mm + 2,
                            d = clamp_bolt_diameter_mm,
                            center = true
                        );
            }
        }
    }
}

module board_clamp_assembly() {
    board_bar_center_y = board_front_bar_center_y_mm;

    board_clamp_carrier();

    for (z_center = [
        clamp_bar_edge_margin_mm + clamp_front_bar_height_mm / 2,
        clamp_back_plate_height_mm / 2,
        clamp_back_plate_height_mm - clamp_bar_edge_margin_mm - clamp_front_bar_height_mm / 2
    ]) {
        translate([board_axis_offset_x_mm, board_bar_center_y, z_center - clamp_front_bar_height_mm / 2])
            board_clamp_pressure_bar();
    }
}

module board_clamp() {
    board_clamp_carrier();

    for (i = [0 : 2]) {
        translate([
            clamp_back_plate_width_mm + 20 + i * (clamp_front_bar_width_mm + 8),
            0,
            0
        ]) board_clamp_pressure_bar();
    }
}
