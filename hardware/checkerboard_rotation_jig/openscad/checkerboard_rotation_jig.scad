include <params.scad>;
use <parts/mic_stand_mount.scad>;
use <parts/vertical_axis_frame.scad>;
use <parts/shaft_and_bearing.scad>;
use <parts/board_clamp.scad>;
use <parts/angle_lock_plate.scad>;

part_name = is_undef(part) ? "assembly" : part;

module checkerboard_reference() {
    color([0.95, 0.95, 0.95, 0.5])
        translate([
            board_axis_offset_x_mm - board_width_mm / 2,
            board_center_y_mm - board_thickness_mm / 2,
            board_center_z_mm - board_height_mm / 2
        ]) cube([board_width_mm, board_thickness_mm, board_height_mm]);
}

module assembly() {
    color([0.2, 0.2, 0.2]) translate([0, 0, -stand_mount_height_mm]) mic_stand_mount();

    color([0.7, 0.7, 0.72]) frame_assembly();

    color([0.82, 0.82, 0.86])
        translate([0, 0, shaft_center_z_mm - shaft_length_mm / 2]) shaft_with_collars();

    color([0.85, 0.45, 0.2])
        translate([0, 0, board_center_z_mm - clamp_back_plate_height_mm / 2]) board_clamp_assembly();

    color([0.25, 0.45, 0.8]) translate([0, 0, angle_plate_bottom_z_mm]) angle_lock_plate();

    color([0.9, 0.55, 0.1])
        translate([0, -angle_plate_radius_mm, angle_plate_bottom_z_mm + angle_plate_thickness_mm])
            angle_lock_pin_knob();

    checkerboard_reference();
}

if (part_name == "assembly") {
    assembly();
} else if (part_name == "stand_mount") {
    mic_stand_mount();
} else if (part_name == "frame") {
    frame_print_layout();
} else if (part_name == "shaft") {
    shaft_print_layout();
} else if (part_name == "board_clamp") {
    board_clamp();
} else if (part_name == "angle_lock_plate") {
    angle_lock_plate_set();
} else {
    assembly();
}
