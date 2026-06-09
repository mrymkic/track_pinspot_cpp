$fn = 96;

// Board
board_width_mm = 297;
board_height_mm = 210;
board_thickness_mm = 5;
board_axis_offset_x_mm = 0;

// Shaft
shaft_diameter_mm = 12;
shaft_clearance_mm = 0.4;
shaft_length_mm = board_height_mm + 40;

// Bearing
bearing_block_width_mm = 36;
bearing_block_depth_mm = 28;
bearing_block_height_mm = 24;

// Stand mount
stand_adapter_outer_diameter_mm = 18;
stand_clamp_clearance_mm = 0.5;
stand_clamp_bolt_diameter_mm = 5;
stand_mount_width_mm = 72;
stand_mount_depth_mm = 54;
stand_mount_height_mm = 24;
stand_mount_slot_width_mm = 4;

// Angle lock
angle_list_deg = [-30, -20, -10, 0, 10, 20, 30];
angle_plate_radius_mm = 80;
angle_pin_diameter_mm = 5;
angle_pin_clearance_mm = 0.3;

// Fasteners
m3_clearance_mm = 3.4;
m4_clearance_mm = 4.5;
m5_clearance_mm = 5.5;
m6_clearance_mm = 6.6;

// Print tolerance
general_clearance_mm = 0.3;

// Frame
frame_spine_width_mm = 60;
frame_spine_thickness_mm = 10;
frame_arm_thickness_mm = 10;
frame_arm_reach_mm = 34;
frame_arm_width_mm = 54;
frame_foot_width_mm = 72;
frame_foot_depth_mm = 54;
frame_foot_thickness_mm = 10;
frame_rib_thickness_mm = 8;
frame_mount_hole_spacing_mm = 42;

// Board clamp
clamp_back_plate_width_mm = 84;
clamp_back_plate_height_mm = min(board_height_mm - 20, 170);
clamp_back_plate_thickness_mm = 6;
clamp_front_bar_width_mm = 88;
clamp_front_bar_height_mm = 18;
clamp_front_bar_thickness_mm = 8;
clamp_bolt_spacing_x_mm = 56;
clamp_bolt_diameter_mm = m4_clearance_mm;
clamp_bar_edge_margin_mm = 18;
clamp_spine_reach_mm = 24;
clamp_hub_outer_diameter_mm = shaft_diameter_mm + 20;
clamp_hub_height_mm = 24;
clamp_hub_slit_width_mm = 4;
clamp_hub_bolt_diameter_mm = m4_clearance_mm;

// Angle plate
angle_plate_band_width_mm = 18;
angle_plate_thickness_mm = 6;
angle_plate_hub_outer_diameter_mm = shaft_diameter_mm + 26;
angle_plate_hub_height_mm = 16;
angle_plate_hub_slit_width_mm = 4;
angle_pin_guide_height_mm = 22;
angle_pin_handle_diameter_mm = 24;
angle_pin_handle_thickness_mm = 8;
angle_pin_length_mm = 22;

// Shaft collars
shaft_collar_outer_diameter_mm = shaft_diameter_mm + 14;
shaft_collar_thickness_mm = 6;
shaft_collar_slit_width_mm = 3;
shaft_collar_bolt_diameter_mm = m3_clearance_mm;
shaft_end_margin_mm = 18;

// Derived geometry
shaft_bore_diameter_mm = shaft_diameter_mm + shaft_clearance_mm;
clamp_board_gap_mm = board_thickness_mm + 2 * general_clearance_mm;
board_center_y_mm =
    clamp_spine_reach_mm + clamp_back_plate_thickness_mm / 2 + clamp_board_gap_mm / 2;
board_front_bar_center_y_mm =
    clamp_spine_reach_mm
    + clamp_back_plate_thickness_mm
    + clamp_board_gap_mm
    + clamp_front_bar_thickness_mm / 2;
lower_bearing_center_z_mm = frame_foot_thickness_mm + 24 + bearing_block_height_mm / 2;
upper_bearing_center_z_mm =
    lower_bearing_center_z_mm + shaft_length_mm - 2 * shaft_end_margin_mm;
shaft_center_z_mm = (lower_bearing_center_z_mm + upper_bearing_center_z_mm) / 2;
board_center_z_mm = shaft_center_z_mm;
frame_total_height_mm = upper_bearing_center_z_mm + bearing_block_height_mm / 2 + 32;
angle_plate_bottom_z_mm =
    lower_bearing_center_z_mm - bearing_block_height_mm / 2 - angle_plate_hub_height_mm - 6;
bearing_mount_spacing_x_mm = bearing_block_width_mm - 14;
