/*
 * LaserScanConverter.cpp
 *
 *  Created on: May 3, 2017
 *      Author: alexander
 */

#include <cmath>
#include <iostream>

#include "../include/laser_beam_emulator.h"

LaserBeamEmulator::LaserBeamEmulator(const nav_msgs::OccupancyGrid &occupancy_grid, PGMMapDrawer &drawing_map, uint32_t visited_color)
									  : drawing_map_(drawing_map),
										visited_color_(visited_color),
										resolution_(occupancy_grid.info.resolution),
										width_(occupancy_grid.info.width),
										height_(occupancy_grid.info.height),
										occupancy_(reinterpret_cast<const int8_t *>(occupancy_grid.data.data())) { }

double LaserBeamEmulator::emit_laser_beam(const MapPosition &origin_position, double route_angle) {
	GridPosition current_cell      = get_start_cell(origin_position, route_angle);
	MapPosition  current_position  = origin_position;

	while (is_in_field(current_cell) && is_free(current_cell)) {
		drawing_map_.paint_cell(current_cell.y, current_cell.x, visited_color_);
		move_through_cell(current_position, current_cell, route_angle);
	}

	double distance_to_barrier;

	if (get_occupancy_of(current_cell) == -1) { return -1; }

	double horizontal_distance_to_barrier = current_position.x - origin_position.x,
		   vertical_distance_to_barrier   = current_position.y - origin_position.y;

	distance_to_barrier = sqrt(horizontal_distance_to_barrier * horizontal_distance_to_barrier +
							   vertical_distance_to_barrier * vertical_distance_to_barrier);

	return distance_to_barrier;
}

GridPosition LaserBeamEmulator::get_start_cell(const MapPosition &position, double route_angle) const {
	GridPosition start_cell;

	DIRECTION horizontal_direction = get_horizontal_direction(route_angle),
			  vertical_direction   = get_vertical_direction(route_angle);

	start_cell.x = horizontal_direction == DIRECTION::LEFT ? get_left_cell(position.x)
														   : get_right_cell(position.x);

	start_cell.y = vertical_direction == DIRECTION::BOTTOM ? get_bottom_cell(position.y)
										                   : get_top_cell(position.y);

	return start_cell;
}

bool LaserBeamEmulator ::is_in_field(const GridPosition &cell) const {
	return 0 < cell.x && cell.x <= width_ &&
		   0 < cell.y && cell.y <= height_;
}

bool LaserBeamEmulator::is_free(const GridPosition &cell) const {
	return occupancy_[(cell.y - 1) * width_ + cell.x - 1] == 0;
}

int8_t LaserBeamEmulator::get_occupancy_of(const GridPosition &cell) const {
	return is_in_field(cell) ? occupancy_[(cell.y - 1) * width_ + cell.x - 1] : -1;
}

DIRECTION LaserBeamEmulator::get_horizontal_direction(double angle) const {
	return 0 <= cos(angle) ? DIRECTION::RIGHT
						   : DIRECTION::LEFT;
}

DIRECTION LaserBeamEmulator::get_vertical_direction(double angle) const {
	return 0 <= sin(angle) ? DIRECTION::TOP
						   : DIRECTION::BOTTOM;
}

DIRECTION LaserBeamEmulator::get_oblique_direction(DIRECTION horizontal_direction, DIRECTION vertical_direction) const {
	if (vertical_direction == DIRECTION::TOP    && horizontal_direction == DIRECTION::LEFT)  { return DIRECTION::TOP_LEFT;     }
	if (vertical_direction == DIRECTION::TOP    && horizontal_direction == DIRECTION::RIGHT) { return DIRECTION::TOP_RIGHT;    }
	if (vertical_direction == DIRECTION::BOTTOM && horizontal_direction == DIRECTION::LEFT)  { return DIRECTION::BOTTOM_LEFT;  }
	if (vertical_direction == DIRECTION::BOTTOM && horizontal_direction == DIRECTION::RIGHT) { return DIRECTION::BOTTOM_RIGHT; }

	return DIRECTION::ERROR;
}

uint32_t LaserBeamEmulator::get_left_cell(double x_position) const {
	return static_cast<uint32_t>(floor(x_position / resolution_ + 1));
}

uint32_t LaserBeamEmulator::get_right_cell(double x_position) const {
	return static_cast<uint32_t>(ceil(x_position / resolution_));
}

uint32_t LaserBeamEmulator::get_bottom_cell(double y_position) const {
	return static_cast<uint32_t>(floor(y_position / resolution_ + 1));
}

uint32_t LaserBeamEmulator::get_top_cell(double y_position) const {
	return static_cast<uint32_t>(ceil(y_position / resolution_));
}

double LaserBeamEmulator::get_left_bound(const GridPosition &cell) const {
	return (cell.x - 1) * resolution_;
}

double LaserBeamEmulator::get_right_bound(const GridPosition &cell) const {
	return cell.x * resolution_;
}

double LaserBeamEmulator::get_bottom_bound(const GridPosition &cell) const {
	return (cell.y - 1) * resolution_;
}

double LaserBeamEmulator::get_top_bound(const GridPosition &cell) const {
	return cell.y * resolution_;
}

void LaserBeamEmulator::move_to_next_cell(GridPosition &grid_position, DIRECTION route) const {
	if (route == DIRECTION::LEFT || route == DIRECTION::TOP_LEFT || route == DIRECTION::BOTTOM_LEFT) {
		grid_position.x--;
	}

	if (route == DIRECTION::RIGHT || route == DIRECTION::TOP_RIGHT || route == DIRECTION::BOTTOM_RIGHT) {
		grid_position.x++;
	}

	if (route == DIRECTION::BOTTOM || route == DIRECTION::BOTTOM_LEFT || route == DIRECTION::BOTTOM_RIGHT) {
		grid_position.y--;
	}

	if (route == DIRECTION::TOP || route == DIRECTION::TOP_LEFT || route == DIRECTION::TOP_RIGHT) {
		grid_position.y++;
	}
}

double LaserBeamEmulator::cot(double angle) const {
	return cos(angle) / sin(angle);
}

void LaserBeamEmulator::move_through_cell(MapPosition &position, GridPosition &cell, double route_angle) const {
	DIRECTION horizontal_direction = get_horizontal_direction(route_angle),
			  vertical_direction   = get_vertical_direction(route_angle);

	double horizontal_bound = horizontal_direction == DIRECTION::LEFT ? get_left_bound(cell)
												     				  : get_right_bound(cell);

	double vertical_bound = vertical_direction == DIRECTION::BOTTOM ? get_bottom_bound(cell)
																    : get_top_bound(cell);

	double horizontal_distance_to_bound = horizontal_bound - position.x,
		   vertical_distance_to_bound   = vertical_bound - position.y;

	double angle_to_corner  = atan2(vertical_distance_to_bound, horizontal_distance_to_bound),
	       angle_difference = std::abs(cos(angle_to_corner)) - std::abs(cos(route_angle));

	if (angle_difference > 0) {
		move_to_vertical_bound(position, route_angle, vertical_bound);
		move_to_next_cell(cell, vertical_direction);
	} else if (angle_difference < 0) {
		move_to_horizontal_bound(position, route_angle, horizontal_bound);
		move_to_next_cell(cell, horizontal_direction);
	} else {
		move_to_corner(position, horizontal_bound, vertical_bound);
		move_to_next_cell(cell, get_oblique_direction(horizontal_direction, vertical_direction));
	}
}

void LaserBeamEmulator::move_to_horizontal_bound(MapPosition &position, double route_angle, double horizontal_bound) const {
	double horizontal_distance = horizontal_bound - position.x;

	position.x = horizontal_bound;
	position.y += horizontal_distance * tan(route_angle);
}

void LaserBeamEmulator::move_to_vertical_bound(MapPosition &position, double route_angle, double vertical_bound) const {
	double vertical_distance = vertical_bound - position.y;

	position.x += vertical_distance * cot(route_angle);
	position.y = vertical_bound;
}

void LaserBeamEmulator::move_to_corner(MapPosition &map_position, double horizontal_bound, double vertical_bound) const {
	map_position.x = horizontal_bound;
	map_position.y = vertical_bound;
}
