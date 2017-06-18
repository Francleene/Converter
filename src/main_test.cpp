#include "test_utilities.h"

int main() {
	TestSaveMap save_map_tests;
	save_map_tests.run_all_tests();

	TestOccupancyGridConverter occupancy_grid_converter_tests;
	occupancy_grid_converter_tests.run_all_tests();

	TestPaintCell paint_cell_tests;
	paint_cell_tests.run_all_tests();

	TestEmitLaserBeam emit_laser_beam_tests;
	emit_laser_beam_tests.run_all_tests();

	TestConverter converter_tests;
	converter_tests.run_all_tests();

	Test::show_final_results();

	return 0;
}
