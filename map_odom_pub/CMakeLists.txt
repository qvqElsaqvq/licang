cmake_minimum_required(VERSION 3.8)
project(map_odom_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
        include
)

add_executable(map_odom_pub_node src/map_odom_pub_node.cpp src/map_odom_pub.cpp)
ament_target_dependencies(map_odom_pub_node
        rclcpp
        sensor_msgs
        tf2_ros
        tf2_eigen
        std_msgs
        tf2_sensor_msgs
        tf2_geometry_msgs
)

install(TARGETS map_odom_pub_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY yaml launch
        DESTINATION share/${PROJECT_NAME})

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
