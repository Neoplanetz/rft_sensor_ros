cmake_minimum_required(VERSION 2.8.3)
project(rft_sensor)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}") ##AGGIUNTA

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
   src/robotous/robotous_sensor.cpp
   src/robotous/RFT_IF_PACKET_Rev1.0.cpp
   src/serial/serial_port.cpp
)

## Declare a C++ executable
add_executable(read_rftdata src/read_rftdata.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(read_rftdata
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)

## Mark executables and/or libraries for installation
install(TARGETS read_rftdata ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
