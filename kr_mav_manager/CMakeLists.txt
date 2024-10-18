cmake_minimum_required(VERSION 3.10)
project(kr_mav_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(kr_tracker_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"srv/Circle.srv"
"srv/CompoundLissajous.srv"
"srv/GoalTimed.srv"
"srv/Lissajous.srv"
"srv/Vec4.srv"
DEPENDENCIES builtin_interfaces
)

include_directories(include/)

## Create library
add_library(manager src/manager.cpp)

ament_export_targets(manager HAS_LIBRARY_TARGET)
ament_target_dependencies(manager rclcpp rclcpp_action nav_msgs std_msgs sensor_msgs kr_tracker_msgs kr_mav_msgs tf2 tf2_geometry_msgs)
target_link_libraries(manager Eigen3::Eigen)
ament_export_dependencies(manager rclcpp rclcpp_action nav_msgs std_msgs sensor_msgs kr_mav_msgs kr_tracker_msgs Eigen3::Eigen)

install(
  TARGETS manager
  EXPORT manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
##

add_executable(mav_services src/mav_services.cpp)
ament_target_dependencies(mav_services std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp") # to link local msg/srv/action definitions (humble and above)
target_link_libraries(mav_services manager "${cpp_typesupport_target}") 

install(TARGETS
mav_services
DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
