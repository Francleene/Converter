FLAGS = -std=c++11 -Wall -Iinclude -I"you path to kinetic ROS include folder"

all: make_dir convert_build

test: make_dir test_build

clean:
	rm -rf bin
	rm -rf converter
	rm -rf test

make_dir:
	mkdir -p bin

convert_build: bin/main.o bin/laser_beam_emulator.o bin/occupancy_grid_to_laser_scan_converter.o bin/pgm_map_drawer.o bin/test_utilities.o bin/utilities.o
	g++ $(FLAGS) bin/main.o bin/laser_beam_emulator.o bin/occupancy_grid_to_laser_scan_converter.o bin/pgm_map_drawer.o bin/test_utilities.o bin/utilities.o -o converter

test_build: bin/main_test.o bin/laser_beam_emulator.o bin/occupancy_grid_to_laser_scan_converter.o bin/pgm_map_drawer.o bin/test_utilities.o bin/utilities.o
	g++ $(FLAGS) bin/main_test.o bin/laser_beam_emulator.o bin/occupancy_grid_to_laser_scan_converter.o bin/pgm_map_drawer.o bin/test_utilities.o bin/utilities.o -o test

bin/main.o: src/main.cpp
	g++ $(FLAGS) -c src/main.cpp -o bin/main.o

bin/main_test.o: src/main_test.cpp
	g++ $(FLAGS) -c src/main_test.cpp -o bin/main_test.o

bin/laser_beam_emulator.o: src/laser_beam_emulator.cpp	
	g++ $(FLAGS) -c src/laser_beam_emulator.cpp -o bin/laser_beam_emulator.o

bin/occupancy_grid_to_laser_scan_converter.o: src/occupancy_grid_to_laser_scan_converter.cpp
	g++ $(FLAGS) -c src/occupancy_grid_to_laser_scan_converter.cpp -o bin/occupancy_grid_to_laser_scan_converter.o

bin/pgm_map_drawer.o: src/pgm_map_drawer.cpp
	g++ $(FLAGS) -c src/pgm_map_drawer.cpp -o bin/pgm_map_drawer.o

bin/test_utilities.o: src/test_utilities.cpp
	g++ $(FLAGS) -c src/test_utilities.cpp -o bin/test_utilities.o

bin/utilities.o: src/test_utilities.cpp
	g++ $(FLAGS) -c src/utilities.cpp -o bin/utilities.o
