cmake_minimum_required(VERSION 3.8)
project(composablenode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(publisher SHARED
  src/pub.cpp)
ament_target_dependencies(publisher rclcpp rclcpp_components std_msgs)

add_library(subscriber SHARED
  src/sub.cpp)
ament_target_dependencies(subscriber rclcpp rclcpp_components std_msgs)

rclcpp_components_register_node(
  publisher
  PLUGIN "composable_node_tutorials::HeaderPublisherNode"
  EXECUTABLE publisher_node
)

rclcpp_components_register_node(
  subscriber
  PLUGIN "composable_node_tutorials::HeaderSubscriberNode"
  EXECUTABLE subscriber_node
)

install(TARGETS publisher subscriber
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()