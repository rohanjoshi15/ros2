cmake_minimum_required(VERSION 3.8)
project(rover_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)





add_executable(lidar_processor src/lidar_processor.cpp)
ament_target_dependencies(lidar_processor rclcpp sensor_msgs)

add_executable(proximity_warning src/proximity_warning.cpp)
ament_target_dependencies(proximity_warning rclcpp std_msgs)

add_executable(emergency_stop src/emergency_stop.cpp)
ament_target_dependencies(emergency_stop rclcpp std_msgs geometry_msgs)

install(TARGETS
  lidar_processor
  proximity_warning
  emergency_stop
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch model rviz
  DESTINATION share/${PROJECT_NAME}
)




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
