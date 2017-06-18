/*
 * LaserScanConverter.h
 *
 *  Created on: May 3, 2017
 *      Author: alexander
 */

#ifndef LASERSCANCONVERTER_H_
#define LASERSCANCONVERTER_H_

#include <stdint.h>

#include "nav_msgs/OccupancyGrid.h"

#include "pgm_map_drawer.h"
#include "utilities.h"

class LaserBeamEmulator {
public:
	LaserBeamEmulator(const nav_msgs::OccupancyGrid &occupancy_grid, PGMMapDrawer &drawing_map, uint32_t visited_color);
	virtual ~LaserBeamEmulator() = default;

	double emit_laser_beam(const MapPosition &origin_position, double route_angle);
public:
	GridPosition get_start_cell(const MapPosition &map_position, double route_angle) const; // :)

	double cot(double angle) const;

	bool is_in_field(const GridPosition &cell) const;
	bool is_free(const GridPosition &cell) const;
	int8_t get_occupancy_of(const GridPosition &cell) const;

	DIRECTION get_horizontal_direction(double angle) const;
	DIRECTION get_vertical_direction(double angle) const;
	DIRECTION get_oblique_direction(DIRECTION horizontal_direction, DIRECTION vertical_direction) const;

	uint32_t get_left_cell(double x_position) const;
	uint32_t get_right_cell(double x_position) const;
	uint32_t get_bottom_cell(double y_position) const;
	uint32_t get_top_cell(double y_position) const;

	double get_left_bound(const GridPosition &cell) const;
	double get_right_bound(const GridPosition &cell) const;
	double get_bottom_bound(const GridPosition &cell) const;
	double get_top_bound(const GridPosition &cell) const;

	DIRECTION get_side_to_walk(const MapPosition &map_position, const GridPosition &grid_position, float route_angle, DIRECTION horizontal_direction, DIRECTION vertical_direction) const;
	void move_through_cell(MapPosition &map_position, GridPosition &grid_position, double route_angle) const;

	void move_to_horizontal_bound(MapPosition &position, double route_angle, double horizontal_bound) const;
	void move_to_vertical_bound(MapPosition &position, double route_angle, double vertical_bound) const;
	void move_to_corner(MapPosition &map_position, double horizontal_bound, double vertical_bound) const;

	void move_to_next_cell(GridPosition &grid_position, DIRECTION side_to_walk) const;
private:
	PGMMapDrawer &drawing_map_;
	uint32_t visited_color_;

	float resolution_;

	uint32_t width_, height_;
	const int8_t *occupancy_;
};

#endif /* LASERSCANCONVERTER_H_ */
