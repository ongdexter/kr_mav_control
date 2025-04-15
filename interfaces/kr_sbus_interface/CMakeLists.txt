cmake_minimum_required(VERSION 3.10)
project(kr_sbus_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             kr_mav_msgs
             nav_msgs
             nodelet
             tf_conversions)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

catkin_package(
LIBRARIES
${PROJECT_NAME}
CATKIN_DEPENDS
roscpp
kr_mav_msgs
nav_msgs
nodelet
tf_conversions
DEPENDS
EIGEN3)

add_library(
  ${PROJECT_NAME}
  src/so3cmd_to_sbus_nodelet.cpp
  src/sbus_bridge.cpp
  src/sbus_msg.cpp
  src/sbus_serial_port.cpp
  src/thrust_mapping.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
