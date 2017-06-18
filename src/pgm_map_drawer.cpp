#include <cstdlib>
#include <iostream>
#include <fstream>

#include "../include/pgm_map_drawer.h"
#include "../include/utilities.h"

PGMMapDrawer::PGMMapDrawer(const std::string &yaml_file_name) {
	is_empty_ = yaml_file_name.empty();

	if (!is_empty_) {
		load_from_yaml_and_pgm(yaml_file_name);
	}
}

void PGMMapDrawer::save_to_yaml_pgm(const std::string &yaml_file_name, const std::string &pgm_file_name) const {
	if (is_empty_) {
		throw std::runtime_error("Empty drawing map cannot be saved");
	}

	save_yaml_part(yaml_file_name, pgm_file_name);
	save_pgm_part(pgm_file_name);
}

nav_msgs::OccupancyGrid PGMMapDrawer::convert_to_occupancy_grid() const {
	if (is_empty_) {
		throw std::runtime_error("Empty drawing map cannot be converted to occupancy grid");
	}

	nav_msgs::OccupancyGrid occupancy_grid;

	fill_occupancy_grid_header(occupancy_grid.header);
	fill_occupancy_grid_info(occupancy_grid.info);

	occupancy_grid.header.stamp = get_current_time();
	fill_occupancy_grid_data(occupancy_grid.data);
	occupancy_grid.info.map_load_time = get_current_time();

	return occupancy_grid;
}

bool PGMMapDrawer::is_empty() const {
	return is_empty_;
}

void PGMMapDrawer::paint_cell(uint32_t row, uint32_t column, uint32_t color) {
	if (is_empty_) { return; }

	data_[(row - 1) * width_ + column - 1] = color;
}

double PGMMapDrawer::get_max_horizontal_position() const {
	if (is_empty_) {
		throw std::runtime_error("Empty drawing map has no max horizontal position");
	}

	return width_ * resolution_;
}

double PGMMapDrawer::get_max_vertical_position() const {
	if (is_empty_) {
		throw std::runtime_error("Empty drawing map has no max vertical position");
	}

	return height_ * resolution_;
}

uint32_t PGMMapDrawer::get_max_gray_value() const {
	if (is_empty_) {
		throw std::runtime_error("Empty drawing map has no max gray value");
	}

	return max_gray_value_;
}

std::string PGMMapDrawer::try_to_load_string(std::ifstream &fin) const {
	std::string input;

	if (fin >> input) {
		return input;
	}

	throw std::runtime_error("Unable to read string");
}

double PGMMapDrawer::try_to_load_double(std::ifstream &fin) const {
	double input;

	if (fin >> input) {
		return input;
	}

	throw std::runtime_error("Unable to read double");
}

uint32_t PGMMapDrawer::try_to_load_uint32_t(std::ifstream &fin) const {
	uint32_t input;

	if (fin >> input) {
		return input;
	}

	throw std::runtime_error("Unable to read uint32_t");
}

bool PGMMapDrawer::try_to_load_bool(std::ifstream &fin) const {
	int input;

	if (fin >> input) {
		if (input == 0) { return false; }
		if (input == 1) { return true;  }

		throw std::runtime_error("Invalid value of bool");
	}

	throw std::runtime_error("Unable to read bool");
}

void PGMMapDrawer::load_from_yaml_and_pgm(const std::string &yaml_file_name) {
	std::string pgm_file_name;

	load_from_yaml(yaml_file_name, pgm_file_name);
	load_from_pgm(pgm_file_name);
}

void PGMMapDrawer::load_from_yaml(const std::string &yaml_file_name,
								  std::string &pgm_file_name) {

	bool have_image = false, have_resolution = false, have_origin_position = false,
		 have_occupied_thresh = false, have_free_thresh = false, have_negate = false;

	std::ifstream yaml_input(yaml_file_name);
	if (!yaml_input) {
		throw std::runtime_error("Unable to open " + yaml_file_name);
	}

	std::string current_parameter;
	while (yaml_input >> current_parameter) {
		if (current_parameter == "image:") {
			have_image = true;
			pgm_file_name = try_to_load_string(yaml_input);
		} else if (current_parameter == "resolution:") {
			have_resolution = true;
			resolution_ = try_to_load_double(yaml_input);
		} else if (current_parameter == "origin:") {
			have_origin_position = true;

			yaml_input.ignore(2);
			origin_position_.x = try_to_load_double(yaml_input);

			yaml_input.ignore(2);
			origin_position_.y = try_to_load_double(yaml_input);

			yaml_input.ignore(2);
			origin_position_.z = try_to_load_double(yaml_input);

			yaml_input.ignore(1);
		} else if (current_parameter == "occupied_thresh:") {
			have_occupied_thresh = true;
			occupied_thresh_ = try_to_load_double(yaml_input);
		} else if (current_parameter == "free_thresh:") {
			have_free_thresh = true;
			free_thresh_ = try_to_load_double(yaml_input);
		} else if (current_parameter == "negate:") {
			have_negate = true;
			negate_ = try_to_load_bool(yaml_input);
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
}

void PGMMapDrawer::load_from_pgm(const std::string &pgm_file_name) {
	std::ifstream pgm_input(pgm_file_name);
	if (!pgm_input) {
		throw std::runtime_error("Unable to open " + pgm_file_name);
	}

	std::string format = try_to_load_string(pgm_input);

	if (format != "P5") {
		throw std::runtime_error(pgm_file_name + " isn't pgm format");
	}

	width_  = try_to_load_uint32_t(pgm_input);
	height_ = try_to_load_uint32_t(pgm_input);
	uint32_t map_size = width_ * height_;

	max_gray_value_ = try_to_load_uint32_t(pgm_input);
	uint32_t number_len = max_gray_value_ < 256 ? 1 : 2;

	pgm_input.ignore();

	uint32_t first_digit, second_digit;

	for (uint32_t i = 0; i < map_size; ++i) {
		first_digit  = number_len == 1 ? 0 : pgm_input.get();
		second_digit = pgm_input.get();

		data_.push_back(first_digit * 256 + second_digit);
	}
}

void PGMMapDrawer::fill_occupancy_grid_header(std_msgs::Header &header) const {
	header.seq      = 0;
	header.frame_id = "1";
}

void PGMMapDrawer::fill_occupancy_grid_info(nav_msgs::MapMetaData &info) const {
	info.resolution      = resolution_;
	info.width           = width_;
	info.height          = height_;
	info.origin.position = origin_position_;
}

void PGMMapDrawer::fill_occupancy_grid_data(std::vector<int8_t> &data) const {
	int8_t occupied_value = static_cast<int8_t>(!negate_),
		   free_value     = static_cast<int8_t>(negate_);

	data.clear();

	double cur_thresh;
	for (size_t i = 0; i < data_.size(); ++i) {
		cur_thresh = static_cast<double>(max_gray_value_ - data_[i]) / max_gray_value_;

		if (occupied_thresh_ <= cur_thresh) {
			data.push_back(occupied_value);
		} else if (cur_thresh <= free_thresh_) {
			data.push_back(free_value);
		} else {
			data.push_back(-1);
		}
	}
}

void PGMMapDrawer::fill_occupancy_grid_origin(geometry_msgs::Pose &origin) const {
	origin.position = origin_position_;

	origin.orientation.x = 0;
	origin.orientation.y = 0;
	origin.orientation.z = 0;
	origin.orientation.w = 0;
}

void PGMMapDrawer::save_yaml_part(const std::string &yaml_file_name, const std::string &pgm_file_name) const {
	std::ofstream yaml_output;
	yaml_output.exceptions(std::ofstream::failbit | std::ofstream::badbit);

	yaml_output.open(yaml_file_name);

	yaml_output << "image: " << pgm_file_name << std::endl;
	yaml_output << "resolution: " << resolution_ << std::endl;

	yaml_output << "origin: [" << origin_position_.x << ", "
							   << origin_position_.y << ", "
							   << origin_position_.z << "]" << std::endl;
	yaml_output << "occupied_thresh: " << occupied_thresh_ << std::endl;
	yaml_output << "free_thresh: " << free_thresh_ << std::endl;
	yaml_output << "negate: " << negate_ << std::endl;
}

void PGMMapDrawer::save_pgm_part(const std::string &pgm_file_name) const {
	std::ofstream pgm_output;
	pgm_output.exceptions(std::ofstream::failbit | std::ofstream::badbit);

	pgm_output.open(pgm_file_name);

	pgm_output << "P5" << std::endl;
	pgm_output << width_ << ' ' << height_ << std::endl;
	pgm_output << max_gray_value_ << std::endl;

	uint32_t number_digit = max_gray_value_ < 256 ? 1 : 2;

	uint8_t first_digit, second_digit;
	for (size_t i = 0; i < data_.size(); ++i) {
		first_digit  = data_[i] / 256;
		second_digit = data_[i] % 256;

		pgm_output.write(reinterpret_cast<char *>(&first_digit), (number_digit - 1) * sizeof(uint8_t));
		pgm_output.write(reinterpret_cast<char *>(&second_digit), sizeof(uint8_t));
	}
}
