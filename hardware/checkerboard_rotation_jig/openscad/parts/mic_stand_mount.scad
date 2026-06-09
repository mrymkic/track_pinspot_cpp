include <../params.scad>;

module mic_stand_mount() {
    adapter_hole_d = stand_adapter_outer_diameter_mm + 2 * stand_clamp_clearance_mm;
    clamp_hole_x = stand_mount_width_mm / 2 - 10;

    difference() {
        translate(
            [-stand_mount_width_mm / 2, -stand_mount_depth_mm / 2, 0]
        ) cube([stand_mount_width_mm, stand_mount_depth_mm, stand_mount_height_mm]);

        translate([0, 0, -1]) cylinder(h = stand_mount_height_mm + 2, d = adapter_hole_d);

        translate([0, -stand_mount_slot_width_mm / 2, -1])
            cube([
                stand_mount_width_mm / 2 + 1,
                stand_mount_slot_width_mm,
                stand_mount_height_mm + 2
            ]);

        translate([clamp_hole_x, 0, stand_mount_height_mm / 2])
            rotate([90, 0, 0])
                cylinder(
                    h = stand_mount_depth_mm + 2,
                    d = stand_clamp_bolt_diameter_mm + general_clearance_mm,
                    center = true
                );

        translate([clamp_hole_x, stand_mount_depth_mm / 2 - 4, stand_mount_height_mm / 2])
            rotate([90, 0, 0])
                cylinder(h = 5, d = stand_clamp_bolt_diameter_mm * 2.2, center = true, $fn = 6);

        for (x = [-frame_mount_hole_spacing_mm / 2, frame_mount_hole_spacing_mm / 2]) {
            translate([x, 0, -1]) cylinder(h = stand_mount_height_mm + 2, d = m5_clearance_mm);
        }
    }
}
