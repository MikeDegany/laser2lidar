cmake_minimum_required(VERSION 3.11)
project(laser2lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

###############################################################################
# Use FetchContent to automatically download and build laser_geometry.
# Note:
#  - We do not call find_package(laser_geometry) since we want to pull it in.
#  - The GIT_TAG "ros2" is assumed to be a branch or tag for ROS 2 support.
#  - Disabling testing can help avoid build issues.
###############################################################################
include(FetchContent)
FetchContent_Declare(
  laser_geometry
  GIT_REPOSITORY https://github.com/ros-perception/laser_geometry.git
  GIT_TAG ros2          # Adjust this if needed (e.g., to a specific tag)
  CMAKE_ARGS -DBUILD_TESTING=OFF
)
FetchContent_MakeAvailable(laser_geometry)
###############################################################################


add_executable(laser2lidar_node src/laser2lidar.cpp)


ament_target_dependencies(laser2lidar_node
  rclcpp
  sensor_msgs
  tf2_ros
)

# Link with the laser_geometry library.
target_link_libraries(laser2lidar_node
  laser_geometry
)

target_include_directories(laser2lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(TARGETS
  laser2lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable some lint checks if needed.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

