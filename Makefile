SHELL := /usr/bin/bash
.PHONY : all clean

all:
	git submodule update --init --recursive
	colcon build --packages-select px4_msgs agent agent_msgs
	source install/setup.bash
	ros2 run agent agent

clean:
	rm -rf install build log
