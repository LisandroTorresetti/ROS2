cmake_minimum_required(VERSION 3.8)
project(tp2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "action/SplitText.action"
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(talker src/counter_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs std_srvs)

add_executable(listener src/counter_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs std_srvs)

add_executable(actionserver src/action_server.cpp)
ament_target_dependencies(actionserver rclcpp rclcpp_action rclcpp_components std_msgs std_srvs action_msgs)

add_executable(textpublisher src/action_client.cpp)
ament_target_dependencies(textpublisher rclcpp rclcpp_action rclcpp_components std_msgs std_srvs action_msgs)

install(TARGETS
        talker
        listener
        actionserver
        textpublisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
