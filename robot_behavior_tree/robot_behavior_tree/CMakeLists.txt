cmake_minimum_required(VERSION 3.8)
project(robot_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(robot_serial REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)
set(dependencies
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  robot_msgs
  robot_serial
  geometry_msgs
)

add_library(nav2_keep_away_from_obstacles_action_bt_node SHARED plugins/action/keep_away_from_obstacles_action.cpp)
list(APPEND plugin_libs nav2_keep_away_from_obstacles_action_bt_node)

add_library(nav2_user_force_success_decorator_bt_node SHARED plugins/decorator/user_force_success.cpp)
list(APPEND plugin_libs nav2_user_force_success_decorator_bt_node)

add_library(nav2_user_force_failure_decorator_bt_node SHARED plugins/decorator/user_force_failure.cpp)
list(APPEND plugin_libs nav2_user_force_failure_decorator_bt_node)

add_library(nav2_update_goal_action_bt_node SHARED plugins/action/update_goal.cpp)
list(APPEND plugin_libs nav2_update_goal_action_bt_node)

add_library(nav2_send_decision_action_bt_node SHARED plugins/action/send_decision.cpp)
list(APPEND plugin_libs nav2_send_decision_action_bt_node)

add_library(nav2_robot_status_action_bt_node SHARED plugins/action/robot_status.cpp)
list(APPEND plugin_libs nav2_robot_status_action_bt_node)

add_library(nav2_send_qrcode_action_bt_node SHARED plugins/action/send_qrcode.cpp)
list(APPEND plugin_libs nav2_send_qrcode_action_bt_node)

add_library(nav2_change_goal_action_bt_node SHARED plugins/action/change_goal.cpp)
list(APPEND plugin_libs nav2_change_goal_action_bt_node)

add_library(nav2_printf_action_bt_node SHARED plugins/action/printf.cpp)
list(APPEND plugin_libs nav2_printf_action_bt_node)

add_library(nav2_match_status_action_bt_node SHARED plugins/action/match_status.cpp)
list(APPEND plugin_libs nav2_match_status_action_bt_node)

add_library(nav2_send_position_error_action_bt_node SHARED plugins/action/send_position_error.cpp)
list(APPEND plugin_libs nav2_send_position_error_action_bt_node)

add_library(nav2_receive_init_error_action_bt_node SHARED plugins/action/receive_init_error.cpp)
list(APPEND plugin_libs nav2_receive_init_error_action_bt_node)

add_library(nav2_if_adjust_condition_bt_node SHARED plugins/condition/if_adjust.cpp)
list(APPEND plugin_libs nav2_if_adjust_condition_bt_node)

add_library(nav2_if_finish_condition_bt_node SHARED plugins/condition/if_finish.cpp)
list(APPEND plugin_libs nav2_if_finish_condition_bt_node)

add_library(nav2_if_catch_ball_condition_bt_node SHARED plugins/condition/if_catch_ball.cpp)
list(APPEND plugin_libs nav2_if_catch_ball_condition_bt_node)

add_library(nav2_is_goal_reached_condition_bt_node SHARED plugins/condition/is_goal_reached.cpp)
list(APPEND plugin_libs nav2_is_goal_reached_condition_bt_node)

add_library(nav2_is_goal_update_condition_bt_node SHARED plugins/condition/is_goal_update.cpp)
list(APPEND plugin_libs nav2_is_goal_update_condition_bt_node)

add_library(nav2_if_navigation_condition_bt_node SHARED plugins/condition/if_navigation.cpp)
list(APPEND plugin_libs nav2_if_navigation_condition_bt_node)

add_library(nav2_if_match_finish_condition_bt_node SHARED plugins/condition/if_match_finish.cpp)
list(APPEND plugin_libs nav2_if_match_finish_condition_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
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

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)
ament_export_dependencies(${dependencies})


ament_package()
