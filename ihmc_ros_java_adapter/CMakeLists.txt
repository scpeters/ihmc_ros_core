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

install(DIRECTORY .
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.gradle"
  PATTERN "gradle/*"
  PATTERN "buildSrc/*"
  PATTERN "gradlew"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                    GROUP_EXECUTE GROUP_READ
)
