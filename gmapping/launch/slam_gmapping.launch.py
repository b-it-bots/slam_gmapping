#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():

    gmapping_node = Node(
            package='gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[
                {"base_frame": "base_footprint"},
                {"map_frame": "map"},
                {"odom_frame": "odom"},
                {"map_update_interval": 5.0},
                {"maxUrange": 4.5},
                {"sigma": 0.05},
                {"kernelSize": 1},
                {"lstep": 0.05},
                {"astep": 0.05},
                {"iterations": 5},
                {"lsigma": 0.075},
                {"ogain": 3.0},
                {"lskip": 0},
                {"srr": 0.1},
                {"srt": 0.2},
                {"str": 0.1},
                {"stt": 0.2},
                {"linearUpdate": 1.0},
                {"angularUpdate": 0.5},
                {"temporalUpdate": -1.0},
                {"resampleThreshold": 0.5},
                {"particles": 30},
                {"xmin": -5.0},
                {"ymin": -5.0},
                {"xmax": 5.0},
                {"ymax": 5.0},
                {"delta": 0.02},
                {"llssamplerange": 0.01},
                {"llsamplestep": 0.01},
                {"lasamplerange": 0.005},
                {"lasampestep": 0.005},
                {"transform_publish_period": 0.05 },
                {"occ_thresh": 0.25},
                {"maxRange": 5.0},
                {"use_sim_time": True}
                ],
            remappings=[('scan', '/scan_front')],
            #prefix=['gdb -ex start --args']
    )
    return LaunchDescription([
        gmapping_node,
    ])
