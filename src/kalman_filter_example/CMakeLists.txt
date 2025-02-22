cmake_minimum_required(VERSION 3.8)
project(kalman_filter_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include eigen headers
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(noisy_sensor src/sensor_simulator.cpp)
add_executable(kalman_2d src/kalman_filter_2d.cpp)
add_executable(sim_gps_imu src/gps_imu_simulator.cpp)
ament_target_dependencies(noisy_sensor rclcpp sensor_msgs)
ament_target_dependencies(kalman_2d rclcpp sensor_msgs geometry_msgs Eigen3)
ament_target_dependencies(sim_gps_imu rclcpp sensor_msgs)

install(TARGETS 
        noisy_sensor
        kalman_2d
        sim_gps_imu
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
