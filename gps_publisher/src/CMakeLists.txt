cmake_minimum_required(VERSION 3.0.2)
project(gps_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gps_publisher src/gps_publisher.cpp)

## Specify libraries to link the executable target against
target_link_libraries(gps_publisher
  ${catkin_LIBRARIES}
)

## Add install targets
install(TARGETS gps_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
