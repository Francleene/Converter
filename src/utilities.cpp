#include <fstream>

#include "geometry_msgs/Point.h"

#include "../include/utilities.h"

const double epsilon = 0.0000001;

ros::Time get_current_time() {
	ros::Time current_time;

	current_time.sec  = time(NULL);
	current_time.nsec = 0;

	return current_time;
}

bool is_equals(double first_number, double second_number) {
	return abs(first_number - second_number) < epsilon;
}

static std::string try_to_load_string(std::ifstream &fin) {
	std::string input;

	if (fin >> input) {
		return input;
	}

	throw std::runtime_error("Unable to read string");
}

static double try_to_load_double(std::ifstream &fin) {
	double input;

	if (fin >> input) {
		return input;
	}

	throw std::runtime_error("Unable to read double");
}

static bool try_to_load_bool(std::ifstream &fin) {
	int input;

	if (fin >> input) {
		if (input == 0) { return false; }
		if (input == 1) { return true;  }

		throw std::runtime_error("Invalid value of bool");
	}

	throw std::runtime_error("Unable to read bool");
}

bool check_yaml_is_correct(const std::string &yaml_file_name,
						   double correct_resolution,
						   geometry_msgs::Point correct_origin,
						   double correct_occupied_thresh,
						   double correct_free_thresh,
						   bool correct_negate) {

	bool have_image = false, have_resolution = false, have_origin_position = false,
		 have_occupied_thresh = false, have_free_thresh = false, have_negate = false;

	std::ifstream yaml_input(yaml_file_name);
	if (!yaml_input) {
		throw std::runtime_error("Unable to open " + yaml_file_name);
	}

	std::string input_pgm_file_name;
	double input_resolution;
	geometry_msgs::Point input_origin;
	double input_occupied_thresh, input_free_thresh;
	bool input_negate;

	std::string current_parameter;
	while (yaml_input >> current_parameter) {
		if (current_parameter == "image:") {
			have_image = true;
			input_pgm_file_name = try_to_load_string(yaml_input);
		} else if (current_parameter == "resolution:") {
			have_resolution = true;
			input_resolution = try_to_load_double(yaml_input);
		} else if (current_parameter == "origin:") {
			have_origin_position = true;

			yaml_input.ignore(2);
			input_origin.x = try_to_load_double(yaml_input);

			yaml_input.ignore(2);
			input_origin.y = try_to_load_double(yaml_input);

			yaml_input.ignore(2);
			input_origin.z = try_to_load_double(yaml_input);

			yaml_input.ignore(1);
		} else if (current_parameter == "occupied_thresh:") {
			have_occupied_thresh = true;
			input_occupied_thresh = try_to_load_double(yaml_input);
		} else if (current_parameter == "free_thresh:") {
			have_free_thresh = true;
			input_free_thresh = try_to_load_double(yaml_input);
		} else if (current_parameter == "negate:") {
			have_negate = true;
			input_negate = try_to_load_bool(yaml_input);
		} else {
			throw std::runtime_error("Unknown parameter in yaml file");
		}
	}

	if (!have_image || !have_resolution || !have_origin_position ||
		!have_occupied_thresh ||  !have_free_thresh || !have_negate) {
		throw std::runtime_error("Have no enough data in yaml file");
	}

	if (!yaml_input.eof()) {
		throw std::runtime_error("There are problems with file. EOF hasn't been reached");
	}

	if (!is_equals(input_resolution, correct_resolution))           { return false; }
	if (!is_equals(input_origin.x, correct_origin.x))               { return false; }
	if (!is_equals(input_origin.y, correct_origin.y))               { return false; }
	if (!is_equals(input_origin.z, correct_origin.z))               { return false; }
	if (!is_equals(input_occupied_thresh, correct_occupied_thresh)) { return false; }
	if (!is_equals(input_free_thresh, correct_free_thresh))         { return false; }
	if (input_negate != correct_negate)                             { return false; }

	return true;
}

bool compare_files(const std::string &first_file_name, const std::string &second_file_name) {
	std::ifstream first_input(first_file_name), second_input(second_file_name);

	if (!first_input || !second_input) {
		return true;
	}

	char first_char, second_char;
	while ((first_input >> first_char) && (second_input >> second_char)) {
		if (first_char != second_char) {
			return false;
		}
	}

	return !first_input.eof() || !second_input.eof();
}
