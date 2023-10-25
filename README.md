slam_gmapping
=============

This package has been ported to ROS 2 from [melodic](https://github.com/ros-perception/slam_gmapping) with some help from the initial work by [allenh1](https://github.com/allenh1/slam_gmapping).

## What has not been ported
* Tests
* the [startReplay](https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/src/slam_gmapping.cpp#L282) function

## Dependencies
* This package is dependent on the [openslam_gmapping](https://github.com/b-it-bots/openslam_gmapping) package. Clone that repository as well before building.

## Usage
* Make sure the robot is running (i.e. it is publishing laser scans, odometry and TF)
* For the youBot, launch: `ros2 launch gmapping slam_gmapping.launch.py`
* To save the map run `ros2 run nav2_map_server map_saver_cli -f /path/to/map`
