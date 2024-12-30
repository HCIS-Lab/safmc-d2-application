.PHONY : all clean
SHELL := /usr/bin/bash

all:
	colcon build --packages-select agent
	source install/setup.bash
	ros2 run agent agent

clean:
	rm -rf install build log
