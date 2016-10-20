cmake_minimum_required(VERSION 2.8.3)
project(ihmc_ros_java_adapter)

find_package(catkin REQUIRED COMPONENTS

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ihmc_msgs
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

install(FILES
  build.gradle
  settings.gradle
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
 gradlew
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY gradle
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY buildSrc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)