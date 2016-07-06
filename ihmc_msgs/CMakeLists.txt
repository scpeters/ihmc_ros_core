cmake_minimum_required(VERSION 2.8.3)
project(ihmc_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  geometry_msgs
)

 add_message_files(
   FILES
   AbortWalkingRosMessage.msg
   ArmDesiredAccelerationsRosMessage.msg
   ArmTrajectoryRosMessage.msg
   ChestTrajectoryRosMessage.msg
   EndEffectorLoadBearingRosMessage.msg
   EuclideanTrajectoryPointRosMessage.msg
   FootstepDataListRosMessage.msg
   FootstepDataRosMessage.msg
   FootstepStatusRosMessage.msg
   FootTrajectoryRosMessage.msg
   GoHomeRosMessage.msg
   HandDesiredConfigurationRosMessage.msg
   HandTrajectoryRosMessage.msg
   HeadTrajectoryRosMessage.msg
   HighLevelStateChangeStatusRosMessage.msg
   HighLevelStateRosMessage.msg
   LastReceivedMessage.msg
   NeckDesiredAccelerationsRosMessage.msg
   NeckTrajectoryRosMessage.msg
   OneDoFJointTrajectoryRosMessage.msg
   PauseWalkingRosMessage.msg
   PelvisHeightTrajectoryRosMessage.msg
   PelvisOrientationTrajectoryRosMessage.msg
   PelvisTrajectoryRosMessage.msg
   Point2dRosMessage.msg
   SE3TrajectoryPointRosMessage.msg
   SO3TrajectoryPointRosMessage.msg
   StopAllTrajectoryRosMessage.msg
   SupportPolygonRosMessage.msg
   TrajectoryPoint1DRosMessage.msg
   WalkingStatusRosMessage.msg
   WholeBodyTrajectoryRosMessage.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ihmc_msgs
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
