#ifndef PGMMAPLOADER_H
#define PGMMAPLOADER_H

#include "nav_msgs/OccupancyGrid.h"

class PGMMapDrawer {
public:
	PGMMapDrawer(const std::string &yaml_file_name);

	void save_to_yaml_pgm(const std::string &yaml_file_name, const std::string &pgm_file_name) const;
	nav_msgs::OccupancyGrid convert_to_occupancy_grid() const;

	bool is_empty() const;
	void paint_cell(uint32_t row, uint32_t column, uint32_t color);

	double get_max_horizontal_position() const;
	double get_max_vertical_position() const;
	uint32_t get_max_gray_value() const;
private:
	std::string try_to_load_string(std::ifstream &fin) const;
	double      try_to_load_double(std::ifstream &fin) const;
	uint32_t    try_to_load_uint32_t(std::ifstream &fin) const;
	char        try_to_load_char(std::ifstream &fin) const;
	bool        try_to_load_bool(std::ifstream &fin) const;

	void load_from_yaml_and_pgm(const std::string &yaml_file_name);
	void load_from_yaml(const std::string &yaml_file_name, std::string &pgm_file_name);
	void load_from_pgm(const std::string &pgm_file_name);

	void fill_occupancy_grid_header(std_msgs::Header &header) const;
	void fill_occupancy_grid_info(nav_msgs::MapMetaData &info) const;
	void fill_occupancy_grid_data(std::vector<int8_t> &data) const;

	void fill_occupancy_grid_origin(geometry_msgs::Pose &origin) const;

	void save_yaml_part(const std::string &yaml_file_name, const std::string &pgm_file_name) const;
	void save_pgm_part(const std::string &pgm_file_name) const;

public:
	bool is_empty_;

	geometry_msgs::Point origin_position_;

	bool negate_;
	double occupied_thresh_, free_thresh_, resolution_;
	uint32_t width_, height_, max_gray_value_;

	std::vector<size_t> data_;
};

#endif
