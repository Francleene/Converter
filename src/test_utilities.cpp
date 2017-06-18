#include <iostream>

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "../include/laser_beam_emulator.h"
#include "../include/occupancy_grid_to_laser_scan_converter.h"
#include "../include/pgm_map_drawer.h"
#include "../include/test_utilities.h"
#include "../include/utilities.h"

#define DO_CHECK(EXPR) check(EXPR, __FUNCTION__, __FILE__, __LINE__)

size_t Test::number_failed_tests;
size_t Test::total_number_tests;

void Test::check(bool expression, const char *function, const char *file_name, size_t line_number) {
	if (!expression) {
		std::cout << "$ function: " << function << ", file: " << file_name << ", line: " << line_number << " failed" << std::endl;
		++number_failed_tests;
	}

	++total_number_tests;
}

void Test::show_final_results() {
	std::cout << std::endl;
	std::cout << "===== FINAL RESULTS =====" << std::endl;
	std::cout << std::endl;

	std::cout << "Total number of tests: " << total_number_tests << std::endl;
	std::cout << "Number of failed tests: " << number_failed_tests << std::endl;
}

void TestSaveMap::run_all_tests() {
	std::cout << "=====> test on SAVE MAP started..." << std::endl << std::endl;

	test_save_no_free_space_map();
	test_save_mixed_map();

	std::cout << std::endl;
	std::cout << "=====> test on SAVE MAP finished!" << std::endl << std::endl;
}

void TestSaveMap::test_save_no_free_space_map() {
	PGMMapDrawer drawing_map("src_test/no_free_space_map.yaml");
	drawing_map.save_to_yaml_pgm("src_test/out_save_no_free_space_map.yaml", "src_test/out_save_no_free_space_map.pgm");
	
	geometry_msgs::Point origin_position;
	origin_position.x = -10;
	origin_position.y = -10;
	origin_position.z = 0;

	DO_CHECK(check_yaml_is_correct("src_test/out_save_no_free_space_map.yaml", 0.05, origin_position, 0.65, 0.196, 0));
	std::system("diff src_test/no_free_space_map.pgm src_test/out_save_no_free_space_map.pgm");

	std::cout << "$ test SAVE NO FREE SPACE passed" << std::endl;
}

void TestSaveMap::test_save_mixed_map(){
	PGMMapDrawer drawing_map("src_test/mixed_map.yaml");
	drawing_map.save_to_yaml_pgm("src_test/out_save_mixed_map.yaml", "src_test/out_save_mixed_map.pgm");

	geometry_msgs::Point origin_position;
	origin_position.x = -10;
	origin_position.y = -10;
	origin_position.z = 0;

	DO_CHECK(check_yaml_is_correct("src_test/out_save_mixed_map.yaml", 0.05, origin_position, 0.65, 0.196, 0));
	std::system("diff src_test/mixed_map.pgm src_test/out_save_mixed_map.pgm");

	std::cout << "$ test SAVE NO FREE SPACE passed" << std::endl;
}

void TestOccupancyGridConverter::run_all_tests() {
	std::cout << "=====> test OCCUPANCY GRID CONVERTER started..." << std::endl << std::endl;

	test_convert_empty_map();
	test_convert_no_free_space_map();
	test_convert_mixed_map();

	std::cout << std::endl;
	std::cout << "=====> test OCCUPANCY GRID CONVERTER finished!" << std::endl << std::endl;
}

void TestOccupancyGridConverter::test_convert_empty_map() {
	PGMMapDrawer drawing_map("src_test/empty_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	DO_CHECK(occupancy_grid.info.width == 100);
	DO_CHECK(occupancy_grid.info.height == 200);
	DO_CHECK(is_equals(occupancy_grid.info.resolution, 0.05));

	DO_CHECK(is_equals(occupancy_grid.info.origin.position.x, -10));
	DO_CHECK(is_equals(occupancy_grid.info.origin.position.y, -10));
	DO_CHECK(is_equals(occupancy_grid.info.origin.position.z, 0));

	std::vector<int8_t> ans_data;
	for (size_t i = 0; i < 20000; ++i) {
		ans_data.push_back(0);
	}

	DO_CHECK(occupancy_grid.data == ans_data);

	std::cout << "$ test CONVERT EMPTY MAP finished!" << std::endl;
}

void TestOccupancyGridConverter::test_convert_no_free_space_map() {
	PGMMapDrawer drawing_map("src_test/no_free_space_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	DO_CHECK(occupancy_grid.info.width == 200);
	DO_CHECK(occupancy_grid.info.height == 100);
	DO_CHECK(is_equals(occupancy_grid.info.resolution, 0.05));

	DO_CHECK(is_equals(occupancy_grid.info.origin.position.x, -10));
	DO_CHECK(is_equals(occupancy_grid.info.origin.position.y, -10));
	DO_CHECK(is_equals(occupancy_grid.info.origin.position.z, 0));

	std::vector<int8_t> ans_data;
	for (size_t i = 0; i < 20000; ++i) {
		ans_data.push_back(1);
	}

	DO_CHECK(occupancy_grid.data == ans_data);

	std::cout << "$ test CONVERT NO FREE SPACE MAP finished!" << std::endl;
}

void TestOccupancyGridConverter::test_convert_mixed_map() {
	PGMMapDrawer drawing_map("src_test/chess_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	DO_CHECK(occupancy_grid.info.width == 270);
	DO_CHECK(occupancy_grid.info.height == 360);
	DO_CHECK(is_equals(occupancy_grid.info.resolution, 0.05));

	DO_CHECK(is_equals(occupancy_grid.info.origin.position.x, -10));
	DO_CHECK(is_equals(occupancy_grid.info.origin.position.y, -10));
	DO_CHECK(is_equals(occupancy_grid.info.origin.position.z, 0));

	std::vector<int8_t> ans_data;
	for (size_t outer_block = 0; outer_block < 4; ++outer_block) {
		for (int i = 0; i < 90; ++i) {
			for (size_t inner_block = 0; inner_block < 3; ++inner_block) {
				for (size_t j = 0; j < 90; ++j) {
					ans_data.push_back(((outer_block + inner_block + 1) % 2));
				}
			}
		}
	}

	DO_CHECK(occupancy_grid.data == ans_data);

	std::cout << "$ test CONVERT MIXED MAP finished!" << std::endl;
}

void TestPaintCell::run_all_tests() {
	std::cout << "=====> test PAINT CELL started..." << std::endl << std::endl;
	
	test_paint_one_cell();
	test_paint_one_rectangle();
	test_paint_three_rectangles();

	test_paint_white_rectangle();
	test_paint_gray_rectangle();

	std::cout << std::endl;
	std::cout << "=====> test PAINT CELL finished!" << std::endl << std::endl;;
}

void TestPaintCell::test_paint_one_cell() {
	PGMMapDrawer drawing_map("src_test/light_gray_map.yaml");

	drawing_map.paint_cell(49, 49, 0);
	drawing_map.save_to_yaml_pgm("src_test/out_paint_one_cell.yaml", "src_test/out_paint_one_cell.pgm");

	std::system("diff src_test/out_paint_one_cell.pgm src_test/expected_paint_one_cell.pgm");

	std::cout << "$ test PAINT ONE CELL finished!" << std::endl;
}

void TestPaintCell::test_paint_one_rectangle() {
	PGMMapDrawer drawing_map("src_test/light_gray_map.yaml");

	draw_rectangle(drawing_map, 20, 20, 50, 80, 0);

	drawing_map.save_to_yaml_pgm("src_test/out_paint_one_rectangle.yaml", "src_test/out_paint_one_rectangle.pgm");

	std::system("diff src_test/out_paint_one_rectangle.pgm src_test/expected_paint_one_rectangle.pgm");

	std::cout << "$ test PAINT ONE RECTANGLE finished!" << std::endl;
}

void TestPaintCell::test_paint_three_rectangles() {
	PGMMapDrawer drawing_map("src_test/light_gray_map.yaml");

	draw_rectangle(drawing_map, 20, 20, 60, 40, 0);
	draw_rectangle(drawing_map, 40, 30, 80, 70, 0);
	draw_rectangle(drawing_map, 50, 10, 70, 35, 0);

	drawing_map.save_to_yaml_pgm("src_test/out_paint_three_rectangles.yaml", "src_test/out_paint_three_rectangles.pgm");

	std::system("diff src_test/out_paint_three_rectangles.pgm src_test/expected_paint_three_rectangles.pgm");

	std::cout << "$ test PAINT THREE RECTANGLES finished!" << std::endl;
}

void TestPaintCell::test_paint_white_rectangle() {
	PGMMapDrawer drawing_map("src_test/light_gray_map.yaml");

	draw_rectangle(drawing_map, 20, 20, 50, 80, 255);

	drawing_map.save_to_yaml_pgm("src_test/out_paint_white_rectangle.yaml", "src_test/out_paint_white_rectangle.pgm");

	std::system("diff src_test/out_paint_white_rectangle.pgm src_test/expected_paint_white_rectangle.pgm");

	std::cout << "$ test PAINT WHITE RECTANGLE finished!" << std::endl;
}

void TestPaintCell::test_paint_gray_rectangle() {
	PGMMapDrawer drawing_map("src_test/light_gray_map.yaml");

	draw_rectangle(drawing_map, 20, 20, 50, 80, 85);

	drawing_map.save_to_yaml_pgm("src_test/out_paing_gray_rectangle.yaml", "src_test/out_paint_gray_rectangle.pgm");

	std::system("diff src_test/out_paint_gray_rectangle.pgm src_test/expected_paint_gray_rectangle.pgm");

	std::cout << "$ test PAINT GRAY RECTANGLE finished!" << std::endl;
}

void TestPaintCell::draw_rectangle(PGMMapDrawer &drawing_map, uint32_t left, uint32_t bottom, uint32_t right, uint32_t top, uint32_t color) {
	for (uint32_t y = bottom; y <= top; ++y) {
		for (uint32_t x = left; x <= right; ++x) {
			drawing_map.paint_cell(y, x, color);
		}
	}
}

void TestEmitLaserBeam::test_distance_horizontal() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, 85);
	MapPosition map_pos = {3.95, 3};

	double distance = laser_emulator.emit_laser_beam(map_pos, 0);

	DO_CHECK(is_equals(distance, 1.0));
	
	std::cout << "$ test DISTANCE HORIZONTAL finished!" << std::endl;
}

void TestEmitLaserBeam::test_distance_vertical() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, 85);
	MapPosition map_pos = {3.95, 3};

	double distance = laser_emulator.emit_laser_beam(map_pos, -M_PI / 2);

	DO_CHECK(is_equals(distance, 2));
	
	std::cout << "$ test DISTANCE VERTICAL finished!" << std::endl;
}

void TestEmitLaserBeam::test_distance_oblique() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, 85);
	MapPosition map_pos = {3.95, 3};

	double distance = laser_emulator.emit_laser_beam(map_pos, -M_PI / 3);

	DO_CHECK(is_equals(distance, 2.0));
	
	std::cout << "$ test DISTANCE OBLIQUE finished!" << std::endl;
}

void TestEmitLaserBeam::test_distance_no_barrier() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, 85);
	MapPosition map_pos = {3.95, 2};

	double distance = laser_emulator.emit_laser_beam(map_pos, M_PI / 2);

	DO_CHECK(is_equals(distance, -1.0));
	
	std::cout << "$ test DISTANCE NO BARRIER finished!" << std::endl;
}

void TestEmitLaserBeam::run_all_tests() {
	std::cout << "=====> test EMIT LASER BEAM started..." << std::endl << std::endl;
	
	test_distance_horizontal();
	test_distance_vertical();
	test_distance_oblique();
	test_distance_no_barrier();

	test_draw_horizontal();
	test_draw_vertical();
	test_draw_oblique();
	test_draw_no_barrier();
	
	std::cout << std::endl;
	std::cout << "=====> test EMIT LASER BEAM finished!" << std::endl << std::endl;
}

void TestEmitLaserBeam::test_draw_horizontal() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, 85);
	MapPosition map_pos = {3.95, 3};

	laser_emulator.emit_laser_beam(map_pos, 0);
	drawing_map.save_to_yaml_pgm("src_test/out_draw_horizontal.yaml", "src_test/out_draw_horizontal.pgm");

	std::system("diff src_test/out_draw_horizontal.pgm src_test/expected_draw_horizontal.pgm");
	
	std::cout << "$ test DRAW HORIZONTAL finished!" << std::endl;
}

void TestEmitLaserBeam::test_draw_vertical() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, 85);
	MapPosition map_pos = {3.95, 3};

	laser_emulator.emit_laser_beam(map_pos, -M_PI / 2);
	drawing_map.save_to_yaml_pgm("src_test/out_draw_vertical.yaml", "src_test/out_draw_vertical.pgm");

	std::system("diff src_test/out_draw_vertical.pgm src_test/expected_draw_vertical.pgm");
	
	std::cout << "$ test DRAW VERTICAL finished!" << std::endl;
}

void TestEmitLaserBeam::test_draw_oblique() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, 85);
	MapPosition map_pos = {3.95, 3};

	laser_emulator.emit_laser_beam(map_pos, -M_PI / 3);
	drawing_map.save_to_yaml_pgm("src_test/out_draw_oblique.yaml", "src_test/out_draw_oblique.pgm");

	std::system("diff src_test/out_draw_oblique.pgm src_test/expected_draw_oblique.pgm");
	
	std::cout << "$ test DRAW OBLIQUE finished!" << std::endl;
}

void TestEmitLaserBeam::test_draw_no_barrier() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	LaserBeamEmulator laser_emulator(occupancy_grid, drawing_map, 85);
	MapPosition map_pos = {3.95, 2};

	laser_emulator.emit_laser_beam(map_pos, M_PI / 2);
	drawing_map.save_to_yaml_pgm("src_test/out_draw_no_barrier.yaml", "src_test/out_draw_no_barrier.pgm");

	std::system("diff src_test/out_draw_no_barrier.pgm src_test/expected_draw_no_barrier.pgm");
	
	std::cout << "$ test DRAW NO BARRIER finished!" << std::endl;
}

void TestConverter::run_all_tests() {
	std::cout << "=====> test CONVERTER started..." << std::endl << std::endl;
	
	test_scan_top_barrier();
	test_scan_right_barrier();
	test_scan_top_right_corner();
	test_scan_no_barrier();
	
	std::cout << std::endl;
	std::cout << "=====> test CONVERTER finished!" << std::endl << std::endl;
};

void TestConverter::test_scan_top_barrier() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	MapPosition map_pos = {3.95, 2};

	sensor_msgs::LaserScan laser_scan;
	convert(occupancy_grid, drawing_map, 85, laser_scan, map_pos, -3 * M_PI / 4, -M_PI / 4, M_PI / 16, 0, 10000);

	DO_CHECK(is_equals(laser_scan.angle_min, -3 * M_PI / 4));
	DO_CHECK(is_equals(laser_scan.angle_max, -M_PI / 4));
	DO_CHECK(is_equals(laser_scan.angle_increment, M_PI / 8));

	DO_CHECK(is_equals(laser_scan.range_min, 0));
	DO_CHECK(is_equals(laser_scan.range_max, 10000));

	double angle = -3 * M_PI / 4;
	for (size_t i = 0; i < laser_scan.ranges.size(); ++i) {
		DO_CHECK( is_equals(laser_scan.ranges[i], 1 / cos( abs(angle + M_PI / 2) ) ) );
		angle += M_PI / 16;
	}
	
	std::cout << "$ test SCAN TOP BARRIER finished!" << std::endl;
}

void TestConverter::test_scan_right_barrier() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	MapPosition map_pos = {3.95, 3};

	sensor_msgs::LaserScan laser_scan;
	convert(occupancy_grid, drawing_map, 85, laser_scan, map_pos, -M_PI / 4, M_PI / 4, M_PI / 16, 0, 10000);

	DO_CHECK(is_equals(laser_scan.angle_min, -M_PI / 4));
	DO_CHECK(is_equals(laser_scan.angle_max, M_PI / 4));
	DO_CHECK(is_equals(laser_scan.angle_increment, M_PI / 16));

	DO_CHECK(is_equals(laser_scan.range_min, 0));
	DO_CHECK(is_equals(laser_scan.range_max, 10000));

	double angle = -M_PI / 4;
	for (size_t i = 0; i < laser_scan.ranges.size(); ++i) {
		DO_CHECK(is_equals(laser_scan.ranges[i], 1 / cos(angle)));
		angle += M_PI / 16;
	}
	
	std::cout << "$ test SCAN RIGHT BARRIER finished!" << std::endl;
}

void TestConverter::test_scan_top_right_corner() {
	PGMMapDrawer drawing_map("src_test/simple_right_top_barrier_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	MapPosition map_pos = {3.95, 3};

	sensor_msgs::LaserScan laser_scan;
	convert(occupancy_grid, drawing_map, 85, laser_scan, map_pos, -M_PI / 2, 0, M_PI / 16, 0, 10000);

	DO_CHECK(is_equals(laser_scan.angle_min, -M_PI / 2));
	DO_CHECK(is_equals(laser_scan.angle_max, 0));
	DO_CHECK(is_equals(laser_scan.angle_increment, M_PI / 16));

	DO_CHECK(is_equals(laser_scan.range_min, 0));
	DO_CHECK(is_equals(laser_scan.range_max, 10000));

	double angle = -M_PI / 2, distance_to_corner = sqrt(2);
	for (size_t i = 0; i < laser_scan.ranges.size(); ++i) {
		DO_CHECK(is_equals(laser_scan.ranges[i], distance_to_corner / cos(angle + M_PI / 4)));
		angle += M_PI / 16;
	}
	
	std::cout << "$ test SCAN TOP RIGHT CORNER finished!" << std::endl;
}

void TestConverter::test_scan_no_barrier() {
	PGMMapDrawer drawing_map("src_test/empty_map.yaml");
	nav_msgs::OccupancyGrid occupancy_grid = drawing_map.convert_to_occupancy_grid();

	MapPosition map_pos = {3, 3};

	sensor_msgs::LaserScan laser_scan;
	convert(occupancy_grid, drawing_map, 85, laser_scan, map_pos, -M_PI / 2, 0, M_PI / 16, 0, 10000);

	DO_CHECK(is_equals(laser_scan.angle_min, -M_PI / 2));
	DO_CHECK(is_equals(laser_scan.angle_max, 0));
	DO_CHECK(is_equals(laser_scan.angle_increment, M_PI / 16));

	DO_CHECK(is_equals(laser_scan.range_min, 0));
	DO_CHECK(is_equals(laser_scan.range_max, 10000));

	for (size_t i = 0; i < laser_scan.ranges.size(); ++i) {
		DO_CHECK(is_equals(laser_scan.ranges[i], -1));
	}
	
	std::cout << "$ test SCAN NO BARRIER finished!" << std::endl;
}
