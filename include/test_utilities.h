#include "pgm_map_drawer.h"

class Test {
public:
	static void check(bool expression, const char *function, const char *file_name, size_t line_number);
	static void show_final_results();

	virtual void run_all_tests() = 0;
protected:
	static size_t number_failed_tests;
	static size_t total_number_tests;
};

class TestSaveMap : public Test {
public:
	void run_all_tests();
private:
	void test_save_no_free_space_map();
	void test_save_mixed_map();
};

class TestOccupancyGridConverter : public Test {
public:
	void run_all_tests();
private:
	void test_convert_empty_map();
	void test_convert_no_free_space_map();
	void test_convert_mixed_map();
};

class TestPaintCell : public Test {
public:
	void run_all_tests();
private:
	void test_paint_one_cell();
	void test_paint_one_rectangle();
	void test_paint_three_rectangles();

	void test_paint_white_rectangle();
	void test_paint_gray_rectangle();

	void draw_rectangle(PGMMapDrawer &drawing_map, uint32_t left, uint32_t bottom, uint32_t right, uint32_t top, uint32_t color);
};

class TestEmitLaserBeam : public Test {
public:
	void run_all_tests();
private:
	void test_distance_horizontal();
	void test_distance_vertical();
	void test_distance_oblique();
	void test_distance_no_barrier();

	void test_draw_horizontal();
	void test_draw_vertical();
	void test_draw_oblique();
	void test_draw_no_barrier();
};

class TestConverter : public Test {
public:
	void run_all_tests();
private:
	void test_scan_top_barrier();
	void test_scan_right_barrier();
	void test_scan_top_right_corner();
	void test_scan_no_barrier();
};
