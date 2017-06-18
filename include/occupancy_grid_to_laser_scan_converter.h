#ifndef OCCUPANCY_GRID_TO_LASER_SCAN_CONVERTER_H
#define OCCUPANCY_GRID_TO_LASER_SCAN_CONVERTER_H

#include <stddef.h>

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "../include/pgm_map_drawer.h"
#include "../include/utilities.h"

void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
			 PGMMapDrawer &map_loader,
			 size_t color,
			 sensor_msgs::LaserScan &laser_scan,
			 const MapPosition &position,
			 double angle_min,
			 double angle_max,
			 double angle_increment,
			 double range_min,
			 double range_max);

#endif
