include <../params.scad>;
use <shaft_and_bearing.scad>;

module frame_rib_pair(arm_center_y, arm_center_z, spine_center_y) {
    for (x_sign = [-1, 1]) {
        hull() {
            translate([
                x_sign * (frame_spine_width_mm / 2 - frame_rib_thickness_mm / 2),
                spine_center_y,
                arm_center_z - 10
            ]) cube([frame_rib_thickness_mm, frame_spine_thickness_mm, 20], center = true);

            translate([
                x_sign * (frame_arm_width_mm / 2 - frame_rib_thickness_mm / 2),
                arm_center_y,
                arm_center_z
            ]) cube([frame_rib_thickness_mm, 18, frame_arm_thickness_mm], center = true);
        }
    }
}

module vertical_axis_frame() {
    spine_center_y = -(bearing_block_depth_mm / 2 + frame_arm_reach_mm + frame_spine_thickness_mm / 2);
    spine_front_y = spine_center_y + frame_spine_thickness_mm / 2;
    arm_front_y = bearing_block_depth_mm / 2;
    arm_depth = arm_front_y - spine_front_y;
    arm_center_y = (arm_front_y + spine_front_y) / 2;
    lower_arm_center_z =
        lower_bearing_center_z_mm - bearing_block_height_mm / 2 - frame_arm_thickness_mm / 2;
    upper_arm_center_z =
        upper_bearing_center_z_mm - bearing_block_height_mm / 2 - frame_arm_thickness_mm / 2;
    pin_arm_center_y = (-angle_plate_radius_mm + spine_front_y) / 2;
    pin_arm_depth = -angle_plate_radius_mm - spine_front_y + 16;
    pin_arm_center_z = angle_plate_bottom_z_mm + angle_plate_hub_height_mm - frame_arm_thickness_mm / 2;

    difference() {
        union() {
            translate(
                [-frame_foot_width_mm / 2, -frame_foot_depth_mm / 2, 0]
            ) cube([frame_foot_width_mm, frame_foot_depth_mm, frame_foot_thickness_mm]);

            translate(
                [-frame_spine_width_mm / 2, spine_center_y - frame_spine_thickness_mm / 2, frame_foot_thickness_mm]
            ) cube([
                frame_spine_width_mm,
                frame_spine_thickness_mm,
                frame_total_height_mm - frame_foot_thickness_mm
            ]);

            for (arm_center_z = [lower_arm_center_z, upper_arm_center_z]) {
                translate(
                    [-frame_arm_width_mm / 2, arm_center_y - arm_depth / 2, arm_center_z - frame_arm_thickness_mm / 2]
                ) cube([frame_arm_width_mm, arm_depth, frame_arm_thickness_mm]);

                frame_rib_pair(arm_center_y, arm_center_z, spine_center_y);
            }

            hull() {
                translate([0, 0, frame_foot_thickness_mm / 2])
                    cube([frame_foot_width_mm - 12, frame_foot_depth_mm - 10, frame_foot_thickness_mm], center = true);

                translate([0, spine_center_y, frame_foot_thickness_mm + 26])
                    cube([frame_spine_width_mm - 12, frame_spine_thickness_mm, 24], center = true);
            }

            translate([0, pin_arm_center_y, pin_arm_center_z])
                cube([18, pin_arm_depth, frame_arm_thickness_mm], center = true);

            translate([0, -angle_plate_radius_mm, angle_plate_bottom_z_mm + angle_pin_guide_height_mm / 2])
                cylinder(h = angle_pin_guide_height_mm, d = angle_pin_diameter_mm + 12, center = true);
        }

        for (x = [-frame_mount_hole_spacing_mm / 2, frame_mount_hole_spacing_mm / 2]) {
            translate([x, 0, -1]) cylinder(h = frame_foot_thickness_mm + 2, d = m5_clearance_mm);
        }

        for (arm_center_z = [lower_arm_center_z, upper_arm_center_z]) {
            translate([0, 0, arm_center_z])
                cylinder(h = frame_arm_thickness_mm + 2, d = shaft_bore_diameter_mm + 4, center = true);

            for (x = [-bearing_mount_spacing_x_mm / 2, bearing_mount_spacing_x_mm / 2]) {
                translate([x, 0, arm_center_z])
                    cylinder(h = frame_arm_thickness_mm + 2, d = m5_clearance_mm, center = true);
            }
        }

        translate([0, -angle_plate_radius_mm, angle_plate_bottom_z_mm - 1])
            cylinder(
                h = angle_pin_guide_height_mm + 2,
                d = angle_pin_diameter_mm + angle_pin_clearance_mm,
                center = false
            );
    }
}

module frame_assembly() {
    vertical_axis_frame();
    translate([0, 0, lower_bearing_center_z_mm - bearing_block_height_mm / 2]) bearing_block_bottom();
    translate([0, 0, upper_bearing_center_z_mm - bearing_block_height_mm / 2]) bearing_block_top();
}

module frame_print_layout() {
    vertical_axis_frame();
    translate([frame_spine_width_mm + 28, 0, 0]) bearing_block_bottom();
    translate([frame_spine_width_mm + bearing_block_width_mm + 44, 0, 0]) bearing_block_top();
}
