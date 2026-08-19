cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0009 NEW) # Evita el warning
project(clase5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(hello_moveit
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)
install(TARGETS hello_moveit DESTINATION lib/${PROJECT_NAME})

# Copy necessary files to designated locations in the project
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install (
  DIRECTORY robot_description
  DESTINATION share/${PROJECT_NAME}
)

install (
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install (
  DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/hello_moveit.py
  scripts/hello_moveit_obstacles.py
  scripts/move_across_desk.py
  scripts/move_to_joints.py
  scripts/move_to_pose.py
  DESTINATION lib/${PROJECT_NAME}
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
