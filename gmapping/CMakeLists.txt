cmake_minimum_required(VERSION 3.5)
project(gmapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(openslam_gmapping REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(src)


add_executable(slam_gmapping src/slam_gmapping.cpp src/main.cpp)
ament_target_dependencies(slam_gmapping
    geometry_msgs
    nav_msgs
    openslam_gmapping
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    sensor_msgs
)
target_link_libraries(slam_gmapping ${Boost_LIBRARIES})

install(TARGETS slam_gmapping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(
    geometry_msgs
    nav_msgs
    openslam_gmapping
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    sensor_msgs
)
ament_package()
