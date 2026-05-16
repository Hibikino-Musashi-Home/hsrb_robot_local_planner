cmake_minimum_required(VERSION 3.5)
project(hsrb_rlp_interface_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(hsrb_rlp_interface_py)

# Install python scripts
install(PROGRAMS script/irlp
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_robot_local_planner_interface
    test/robot_local_planner_interface-test.py)
endif()

ament_package()
