#ifndef UTILITIES_H
#define UTILITIES_H

#include <string>

#include "ros/time.h"

enum class DIRECTION {
	NONE, LEFT, RIGHT, TOP, BOTTOM, TOP_RIGHT, BOTTOM_RIGHT, BOTTOM_LEFT, TOP_LEFT, ERROR
};

struct GridPosition {
	uint32_t x, y;
};

struct MapPosition {
	double x, y;
};

ros::Time get_current_time();
bool is_equals(double first_number, double second_number);

bool check_yaml_is_correct(const std::string &yaml_file_name,
						   double correct_resolution,
						   geometry_msgs::Point correct_origin,
						   double correct_occupied_thresh,
						   double correct_free_thresh,
						   bool correct_negate);
bool compare_files(const std::string &first_file_name, const std::string &second_file_name);

#endif
