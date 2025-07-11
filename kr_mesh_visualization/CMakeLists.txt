cmake_minimum_required(VERSION 3.10)
project(kr_mesh_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/mesh_visualization.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs nav_msgs visualization_msgs)

install(DIRECTORY
  mesh
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
