cmake_minimum_required(VERSION 3.10)
project(kr_trackers_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(kr_trackers REQUIRED)
find_package(kr_tracker_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

set(plugins_dependencies
  pluginlib
)

set(node_dependencies
  ${plugins_dependencies}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rcutils
  kr_mav_msgs
  kr_trackers
  kr_tracker_msgs
  nav_msgs
)

add_library(${PROJECT_NAME} SHARED src/trackers_manager.cpp src/lifecycle_manager.cpp)
ament_target_dependencies(${PROJECT_NAME} ${node_dependencies} lifecycle_msgs)
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
rclcpp_components_register_nodes(${PROJECT_NAME} "TrackersManager")
rclcpp_components_register_nodes(${PROJECT_NAME} "TrackersManagerLifecycleManager")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
  
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
