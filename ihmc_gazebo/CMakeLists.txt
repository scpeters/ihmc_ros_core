cmake_minimum_required(VERSION 2.8.3)
project(ihmc_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)


# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# Find boost after gazebo
find_package(Boost REQUIRED system thread)

link_directories(${GAZEBO_LIBRARY_DIRS})
#message("Gazebo includes: ${GAZEBO_INCLUDE_DIRS}")
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
include_directories(${roscpp_INCLUDE_DIRS})
message("roscpp includes: ${roscpp_INCLUDE_DIRS}")
link_directories(${roscpp_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
    DRCSimIHMCPlugin
    TCPServer

  CATKIN_DEPENDS 
    roscpp 
    gazebo_ros 
)

#add_library(${PROJECT_NAME} src/.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_library(TCPServer SHARED
  gazebo_plugin/TCPServer.cc
  gazebo_plugin/TCPConnection.cc
)
add_executable(ServerTest gazebo_plugin/ServerTest.cc)
target_link_libraries(ServerTest ${Boost_LIBRARIES} pthread TCPServer)

add_library(DRCSimIHMCPlugin SHARED gazebo_plugin/DRCSimIHMCPlugin.cpp)
target_link_libraries(DRCSimIHMCPlugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} TCPServer)


# Install Gazebo launch files and world files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(TARGETS
  DRCSimIHMCPlugin
  ServerTest
  TCPServer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
