#include "../include/occupancy_grid_to_laser_scan_converter.h"

#include "../include/laser_beam_emulator.h"
#include "../include/utilities.h"

void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
			 PGMMapDrawer &drawing_map,
			 size_t color,
			 sensor_msgs::LaserScan &laser_scan,
			 const MapPosition &origin_position,
			 double angle_min,
			 double angle_max,
			 double angle_increment,
			 double range_min,
			 double range_max) {

	laser_scan.header.seq      = occupancy_grid.header.seq;
	laser_scan.header.frame_id = "1";

	laser_scan.angle_min       = angle_min;
	laser_scan.angle_max       = angle_max;
	laser_scan.angle_increment = angle_increment;

	laser_scan.time_increment = 0;
	laser_scan.scan_time      = 0;

	laser_scan.range_min = range_min;
	laser_scan.range_max = range_max;

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, color);

	laser_scan.ranges.clear();
	laser_scan.intensities.clear();

	laser_scan.header.stamp = get_current_time();
	double distance_to_barrier;

	for (double route_angle = angle_min; route_angle <= angle_max; route_angle += angle_increment) {
		distance_to_barrier = laser_emulator.emit_laser_beam(origin_position, route_angle);

		if (distance_to_barrier != -1 && range_min <= distance_to_barrier && distance_to_barrier <= range_max) {
			laser_scan.ranges.push_back(distance_to_barrier);
		} else {
			laser_scan.ranges.push_back(-1);
		}

		laser_scan.intensities.push_back(0);
	}
}
