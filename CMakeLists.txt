cmake_minimum_required(VERSION 3.8)
project(clifford_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Defining ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Installing custom ROS2 Python codes
ament_python_install_package(scripts)
install(PROGRAMS
  scripts/joint_group_position_publisher.py
  scripts/inverse_kinematics_publisher.py
  scripts/serial_port_writer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installing additional directories for the ROS2 package
install(
  DIRECTORY config description launch meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
