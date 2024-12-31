.PHONY : all clean
SHELL := /usr/bin/bash

all:
	colcon build --packages-select agent agent_msgs
	source install/setup.bash
	ros2 run agent agent

clean:
	rm -rf install build log
