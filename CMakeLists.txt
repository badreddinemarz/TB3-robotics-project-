cmake_minimum_required(VERSION 3.8)
project(warehouse)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(sensor_msgs REQUIRED)
# Navigator node
add_executable(odom_to_tf src/odom_to_tf.cpp)
add_executable(send_goal  src/send_goal.cpp)
add_executable(robot_navigator src/robot_navigator.cpp)
ament_target_dependencies(robot_navigator
  rclcpp
  sensor_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(send_goal
  rclcpp
  sensor_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(odom_to_tf
  rclcpp
  nav2_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS robot_navigator
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS odom_to_tf
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS send_goal
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
