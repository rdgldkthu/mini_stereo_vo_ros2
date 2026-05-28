.PHONY: build sim bag eval clean

build:
	colcon build --symlink-install

sim:
	@-pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null; sleep 1
	ros2 launch mini_stereo_vo_ros2 stereo_vo_gazebo.launch.py

bag:
	ros2 bag record -o bags/$(shell date +%s) /vo/odometry /left/image_raw /model/turtlebot4/odometry

eval:
	PATH="$(CURDIR)/.venv/bin:$$PATH" python3 scripts/bag2tum.py $(shell ls -td bags/*/ 2>/dev/null | head -1)

clean:
	rm -rf build/ install/ log/
