cmake_minimum_required(VERSION 3.5)
project(hsrb_robot_local_planner_node)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_manipulation_types_bridge REQUIRED)
find_package(tmc_manipulation_util REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(tmc_robot_local_planner REQUIRED)
find_package(tmc_robot_local_planner_utils REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/${PROJECT_NAME}/hsrb_joint_trajectories_publisher.cpp
  src/${PROJECT_NAME}/node.cpp
  src/${PROJECT_NAME}/utils.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_lib
  control_msgs
  nav_msgs
  pluginlib
  rclcpp
  tf2
  tf2_eigen
  tf2_ros
  sensor_msgs
  tmc_planning_msgs
  tmc_manipulation_types
  tmc_manipulation_types_bridge
  tmc_manipulation_util
  tmc_robot_kinematics_model
  tmc_robot_local_planner
  tmc_robot_local_planner_utils
  tmc_utils
  trajectory_msgs
)

add_executable(node_with_action src/${PROJECT_NAME}/node_with_action_main.cpp)
target_link_libraries(node_with_action ${PROJECT_NAME}_lib)
ament_target_dependencies(node_with_action rclcpp)

add_executable(node_with_plugin src/${PROJECT_NAME}/node_with_plugin_main.cpp)
target_link_libraries(node_with_plugin ${PROJECT_NAME}_lib)
ament_target_dependencies(node_with_plugin rclcpp)

add_library(plugins_for_hsrb_robot_local_planner_node_tests SHARED test/plugins_for_tests.cpp)
target_include_directories(plugins_for_hsrb_robot_local_planner_node_tests PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(plugins_for_hsrb_robot_local_planner_node_tests builtin_interfaces pluginlib tmc_robot_local_planner)
pluginlib_export_plugin_description_file(tmc_robot_local_planner test/plugins_for_tests.xml)

install(TARGETS node_with_action node_with_plugin ${PROJECT_NAME}_lib plugins_for_hsrb_robot_local_planner_node_tests
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(displacement_checker_test test/displacement_checker-test.cpp)
  ament_target_dependencies(displacement_checker_test tmc_manipulation_tests)
  target_link_libraries(displacement_checker_test ${PROJECT_NAME}_lib)

  ament_add_gtest(hsrb_robot_local_planner_with_action_test test/robot_local_planner_node_with_action-test.cpp)
  ament_target_dependencies(hsrb_robot_local_planner_with_action_test tmc_manipulation_tests)
  target_link_libraries(hsrb_robot_local_planner_with_action_test ${PROJECT_NAME}_lib)

  ament_add_gtest(hsrb_robot_local_planner_with_plugin_test test/robot_local_planner_node_with_plugin-test.cpp)
  ament_target_dependencies(hsrb_robot_local_planner_with_plugin_test tmc_manipulation_tests)
  target_link_libraries(hsrb_robot_local_planner_with_plugin_test ${PROJECT_NAME}_lib)
endif()

ament_export_include_directories()
ament_export_libraries()
ament_export_dependencies()

ament_package()
