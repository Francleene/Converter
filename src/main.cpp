#include <iostream>
#include <string>
#include <cstdlib>
#include <stdexcept>

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "../include/laser_beam_emulator.h"
#include "../include/occupancy_grid_to_laser_scan_converter.h"
#include "../include/pgm_map_drawer.h"
#include "../include/utilities.h"

class UserInterface {
public:
	static void run();
private:
	static uint32_t convert_to_uint32_t(const std::string &string);
	static double   convert_to_double(const std::string &string);

	static PGMMapDrawer get_drawing_map();

	static std::string get_output_yaml_file_name();
	static std::string get_output_pgm_file_name();

	static uint32_t get_laser_beam_color(uint32_t max_gray_color);
	static MapPosition get_origin_position(double max_horizontal_pos, double max_vertical_pos);

	static double get_angle_min();
	static double get_angle_max();
	static double get_angle_increment();

	static double get_range_min();
	static double get_range_max();

	static bool get_is_print_ranges_array();

	static void print_input_occupancy_grid(const nav_msgs::OccupancyGrid &occupancy_grid);
	static void print_output_laser_scan(const sensor_msgs::LaserScan &laser, bool is_print_ranges_array);
};

uint32_t UserInterface::convert_to_uint32_t(const std::string &string) {
	uint32_t res_number;
	std::string::size_type tail;

	res_number = std::stoul(string, &tail);
	if (tail != string.size()) {
		throw std::runtime_error("Entered number isn't double");
	}

	return res_number;
}

double UserInterface::convert_to_double(const std::string &string) {
	double res_number;
	std::string::size_type tail;

	res_number = std::stod(string, &tail);
	if (tail != string.size()) {
		throw std::runtime_error("Entered number isn't double");
	}

	return res_number;
}

void UserInterface::print_input_occupancy_grid(const nav_msgs::OccupancyGrid &occupancy_grid) {
	std::cout << std::endl;
	std::cout << "=== INPUT OCCUPANCY GRID ===" << std::endl << std::endl;

	std::cout << "Header:" << std::endl << std::endl;

	std::cout << "| seq: " << occupancy_grid.header.seq << std::endl;
	std::cout << "| stamp: (sec: " << occupancy_grid.header.stamp.sec
			         << ", nsec: " << occupancy_grid.header.stamp.nsec << ")" << std::endl;
	std::cout << "| frame_id: " << occupancy_grid.header.frame_id << std::endl << std::endl;

	std::cout << "Info: " << std::endl << std::endl;

	std::cout << "| map_load_time: (sec: " << occupancy_grid.info.map_load_time.sec
			                <<  ", nsec: " << occupancy_grid.info.map_load_time.nsec << ")" << std::endl;
	std::cout << "| resolution: " << occupancy_grid.info.resolution << std::endl;
	std::cout << "| width: " << occupancy_grid.info.width << std::endl;
	std::cout << "| height: " << occupancy_grid.info.height << std::endl;
	std::cout << "| origin.position: (x: " << occupancy_grid.info.origin.position.x
			                    << ", y: " << occupancy_grid.info.origin.position.y
							    << ", z: " << occupancy_grid.info.origin.position.z << ")" << std::endl;

	std::cout << "| origin.orientation: (x: " << occupancy_grid.info.origin.orientation.x
			                       << ", y: " << occupancy_grid.info.origin.orientation.y
								   << ", z: " << occupancy_grid.info.origin.orientation.z
								   << ", w: " << occupancy_grid.info.origin.orientation.w << ")" << std::endl;
}

void UserInterface::print_output_laser_scan(const sensor_msgs::LaserScan &laser_scan, bool is_print_ranges_array) {
	std::cout << std::endl;
	std::cout << "=== OUTPUT LASER SCAN ===" << std::endl << std::endl;

	std::cout << "Header:" << std::endl << std::endl;

	std::cout << "| seq: " << laser_scan.header.seq << std::endl;
	std::cout << "| stamp: (sec: " << laser_scan.header.stamp.sec
			         << ", nsec: " << laser_scan.header.stamp.nsec << ")" << std::endl;
	std::cout << "| frame_id: " << laser_scan.header.frame_id << std::endl << std::endl;

	std::cout << "angle_min: " << laser_scan.angle_min << std::endl;
	std::cout << "angle_max: " << laser_scan.angle_max << std::endl;
	std::cout << "angle_increment: " << laser_scan.angle_increment << std::endl << std::endl;

	std::cout << "time_increment: " << laser_scan.time_increment << std::endl;
	std::cout << "scan_time: " << laser_scan.scan_time << std::endl << std::endl;

	std::cout << "range_min: " << laser_scan.range_min << std::endl;
	std::cout << "range_max: " << laser_scan.range_max << std::endl;

	if (!is_print_ranges_array) { return; }

	std::cout << std::endl;
	std::cout << "ranges:" << std::endl << std::endl;

	if (!laser_scan.ranges.empty()) { std::cout << laser_scan.ranges[0]; }

	for (size_t i = 1; i < laser_scan.ranges.size(); ++i) {
		std::cout << ", " << laser_scan.ranges[i];
	}

	std::cout << std::endl;
}

double UserInterface::get_range_max() {
	std::cout << "Enter max distance to barrier (range_max): ";

	std::string range_max_str;
	std::cin >> range_max_str;

	return convert_to_double(range_max_str);
}

double UserInterface::get_range_min() {
	std::cout << "Enter min distance to barrier (range_min): ";

	std::string range_min_str;
	std::cin >> range_min_str;

	return convert_to_double(range_min_str);
}

double UserInterface::get_angle_increment() {
	std::cout << "Enter increment of emiting laser beam (angle_increment): ";

	std::string angle_increment_str;
	std::cin >> angle_increment_str;

	return convert_to_double(angle_increment_str);
}

double UserInterface::get_angle_max() {
	std::cout << "Enter end laser beam angle (angle_max): ";

	std::string angle_max_str;
	std::cin >> angle_max_str;

	return convert_to_double(angle_max_str);
}

double UserInterface::get_angle_min() {
	std::cout << "Enter begin laser beam angle (angle_min): ";

	std::string angle_min_str;
	std::cin >> angle_min_str;

	return convert_to_double(angle_min_str);
}

MapPosition UserInterface::get_origin_position(double max_horizontal_pos, double max_vertical_pos) {
	std::cout << "Enter origin position (x=0.." << max_horizontal_pos << ", y=0.." << max_vertical_pos << "): ";

	MapPosition origin_position;

	std::string x_pos_str, y_pos_str;
	std::cin >> x_pos_str >> y_pos_str;

	origin_position.x = convert_to_double(x_pos_str);
	origin_position.y = convert_to_double(y_pos_str);

	if (origin_position.x < 0 || max_horizontal_pos < origin_position.x ||
		origin_position.y < 0 || max_vertical_pos < origin_position.y) {
		throw std::runtime_error("Error: entered position is out of range");
	}

	return origin_position;
}

uint32_t UserInterface::get_laser_beam_color(uint32_t max_gray_value) {
	std::cout << "Enter color of laser beam (0.." << max_gray_value << "): ";

	std::string laser_beam_color_str;
	std::cin >> laser_beam_color_str;

	uint32_t laser_beam_color = convert_to_uint32_t(laser_beam_color_str);

	if (max_gray_value < laser_beam_color) {
		throw std::runtime_error("Entered color of laser beam is out of range");
	}

	return laser_beam_color;
}

std::string UserInterface::get_output_yaml_file_name() {
	std::cout << "Enter output yaml filename: ";

	std::string output_yaml_file_name;
	std::cin >> output_yaml_file_name;

	return output_yaml_file_name;
}

std::string UserInterface::get_output_pgm_file_name() {
	std::cout << "Enter output pgm filename: ";

	std::string output_pgm_file_name;
	std::cin >> output_pgm_file_name;

	return output_pgm_file_name;
}

PGMMapDrawer UserInterface::get_drawing_map() {
	std::string input_yaml_file_name;

	std::cout << "Enter input yaml filename: ";
	std::cin >> input_yaml_file_name;

	PGMMapDrawer drawing_map(input_yaml_file_name);

	return drawing_map;
}

bool UserInterface::get_is_print_ranges_array() {
	std::string is_print_ranges_array_str;

	std::cout << "Does the program need to print the ranges array (type yes - does, something else - doesn't)? ";
	std::cin >> is_print_ranges_array_str;

	return is_print_ranges_array_str == "yes";
}

void UserInterface::run() {
	PGMMapDrawer drawing_map = get_drawing_map();
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	std::string output_yaml_file_name = get_output_yaml_file_name();
	std::string output_pgm_file_name  = get_output_pgm_file_name();

	if (output_yaml_file_name == output_pgm_file_name) {
		throw std::runtime_error("pgm and yaml output files cannot have the same filenames");
	}

	uint32_t max_gray_value   = drawing_map.get_max_gray_value();
	uint32_t laser_beam_color = get_laser_beam_color(max_gray_value);

	double max_horizontal_pos = drawing_map.get_max_horizontal_position(),
		   max_vertical_pos   = drawing_map.get_max_vertical_position();
	MapPosition origin_position = get_origin_position(max_horizontal_pos, max_vertical_pos);

	double angle_min       = get_angle_min(),
		   angle_max       = get_angle_max(),
		   angle_increment = get_angle_increment();

	double range_min = get_range_min(),
	       range_max = get_range_max();

	bool is_print_ranges_array = get_is_print_ranges_array();

	sensor_msgs::LaserScan laser_scan;

	convert(occupancy_grid, drawing_map, laser_beam_color, laser_scan, origin_position, angle_min, angle_max, angle_increment, range_min, range_max);

	drawing_map.save_to_yaml_pgm(output_yaml_file_name, output_pgm_file_name);

	print_input_occupancy_grid(occupancy_grid);
	print_output_laser_scan(laser_scan, is_print_ranges_array);
}

#include <fstream>

int main() {
	try {
		UserInterface::run();
	}

	catch (std::exception &exception) {
		std::cerr << "Error: " << exception.what() << std::endl;
	}

	return 0;
}
