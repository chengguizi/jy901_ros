cmake_minimum_required(VERSION 3.5)
project(jy901_ros)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  serial
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES rs2_ros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

add_executable(imu 
  src/main.cpp
  src/jy901.cpp
)

add_executable(offline_calibration 
  src/offline_calibration.cpp
)


target_include_directories(imu
  PRIVATE
    include
    ${YAML_CPP_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
) # to find ros/ros.h

target_include_directories(offline_calibration
  PRIVATE
    ${YAML_CPP_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
) # to find ros/ros.h

target_link_libraries(imu
  PRIVATE
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

target_link_libraries(offline_calibration
  PRIVATE
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)