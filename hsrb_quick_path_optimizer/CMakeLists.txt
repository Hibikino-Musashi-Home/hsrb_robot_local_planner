cmake_minimum_required(VERSION 3.5)
project(hsrb_quick_path_optimizer)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(hsrb_base_controllers REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tmc_robot_local_planner REQUIRED)
find_package(tmc_robot_local_planner_utils REQUIRED)
find_package(tmc_timeopt REQUIRED)
find_package(tmc_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/base_kinematics_ros.cpp
  src/base_kinematics.cpp
  src/conversions.cpp
  src/joint_limit.cpp
  src/optimizer_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  angles
  hsrb_base_controllers
  moveit_msgs
  pluginlib
  rclcpp
  tf2_eigen
  tf2_geometry_msgs
  tmc_robot_local_planner
  tmc_robot_local_planner_utils
  tmc_timeopt
  tmc_utils
)

add_executable(optimizer_node src/node_main.cpp)
target_link_libraries(optimizer_node ${PROJECT_NAME})
ament_target_dependencies(optimizer_node rclcpp)

pluginlib_export_plugin_description_file(tmc_robot_local_planner optimzier_plugins.xml)

install(TARGETS ${PROJECT_NAME} optimizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_base_kinematics_ros test/base_kinematics_ros-test.cpp)
  target_link_libraries(test_base_kinematics_ros ${PROJECT_NAME})

  ament_add_gtest(test_base_kinematics test/base_kinematics-test.cpp)
  target_link_libraries(test_base_kinematics ${PROJECT_NAME})

  ament_add_gmock(test_conversions test/conversions-test.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME})

  ament_add_gtest(test_joint_limit test/joint_limit-test.cpp)
  target_link_libraries(test_joint_limit ${PROJECT_NAME})

  ament_add_gtest(test_optimizer_node test/optimizer_node-test.cpp)
  target_link_libraries(test_optimizer_node ${PROJECT_NAME})
endif()


ament_export_include_directories()
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  angles
  hsrb_base_controllers
  moveit_msgs
  rclcpp
  tf2_eigen
  tmc_robot_local_planner
  tmc_robot_local_planner_utils
  tmc_timeopt
  tmc_utils
)

ament_package()
