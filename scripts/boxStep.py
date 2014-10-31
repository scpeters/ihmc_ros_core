#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from ihmc_msgs.msg import FootstepDataMessage
from ihmc_msgs.msg import FootstepDataListMessage
from ihmc_msgs.msg import FootstepStatusMessage
from ihmc_msgs.msg import SquareDataMessage
import tf.transformations as tr
import math 

x = 0
y = 0
z = 0.085 
footStepStatusCounter = 0
desiredStepsSentCounter = 0
footYOffset = 0.15
LEFT = 0
RIGHT = 1

def walkInPlace(numSteps):
    global footStepDataListPub, RIGHT, r 
        
    if not rospy.is_shutdown():
        while footStepDataListPub.get_num_connections() == 0:
            print 'waiting for subsciber'
            r.sleep()
        msg = FootstepDataListMessage()
        #the time spent in double-support when executing footsteps 
        msg.transferTime = 1.5
        
        #the time spent in single-support when executing footsteps
        msg.swingTime = 1.5 
        
        # trajectory Waypoint Generation Method
        # DEFAULT = 0
        # BY_BOX = 1
        # STEP_ON_OR_OFF = 2
        # NO_STEP = 3
        # LOW_HEIGHT = 4
        msg.trajectoryWaypointGenerationMethod = 0
        
        #set trajectoryBox if waypoint generation is BY_BOX
        #footStepList.trajectoryBoxData = SquareDataMessage()
        #footStepList.trajectoryBoxData.lengthX = 0
        #footStepList.trajectoryBoxData.widthY = 0
        #footStepList.trajectoryBoxData.location.x = 0
        #footStepList.trajectoryBoxData.location.y = 0
        #footStepList.trajectoryBoxData.location.z = 0
        #footStepList.trajectoryBoxData.rotation.x = 0
        #footStepList.trajectoryBoxData.rotation.y = 0
        #footStepList.trajectoryBoxData.rotation.z = 0
        #footStepList.trajectoryBoxData.rotation.w = 1
        msg.footstepDataList = createFootSteps(RIGHT, numSteps, 0,0,0, False)
        footStepDataListPub.publish(msg)
        print 'walk in place msg sent'
        waitForFootstepsToFinish()

def boxStep():
    global RIGHT, LEFT, r, footStepDataListPub

    if not rospy.is_shutdown():
        while footStepDataListPub.get_num_connections() == 0:
            print 'waiting for subsciber'
            r.sleep()
        msg = FootstepDataListMessage()
        #the time spent in double-support when executing footsteps 
        msg.transferTime = 1.5

        #the time spent in single-support when executing footsteps
        msg.swingTime = 1.5

        # trajectory Waypoint Generation Method
        # DEFAULT = 0
        # BY_BOX = 1
        # STEP_ON_OR_OFF = 2
        # NO_STEP = 3
        # LOW_HEIGHT = 4
        msg.trajectoryWaypointGenerationMethod = 0

        #set trajectoryBox if waypoint generation is BY_BOX
        #footStepList.trajectoryBoxData = SquareDataMessage()
        #footStepList.trajectoryBoxData.lengthX = 0
        #footStepList.trajectoryBoxData.widthY = 0
        #footStepList.trajectoryBoxData.location.x = 0
        #footStepList.trajectoryBoxData.location.y = 0
        #footStepList.trajectoryBoxData.location.z = 0
        #footStepList.trajectoryBoxData.rotation.x = 0
        #footStepList.trajectoryBoxData.rotation.y = 0
        #footStepList.trajectoryBoxData.rotation.z = 0
        #footStepList.trajectoryBoxData.rotation.w = 1
        
        msg.footstepDataList.extend(createFootSteps(LEFT, 4, 0.25, 0, 0, False))
        msg.footstepDataList.extend(createFootSteps(LEFT, 1, 0, 0, 0, False))
        
        msg.footstepDataList.extend(createFootSteps(LEFT, 4, 0, 0.25, 0, True))
        
        msg.footstepDataList.extend(createFootSteps(LEFT, 4, -0.25, 0, 0, False))
        msg.footstepDataList.extend(createFootSteps(LEFT, 1, 0, 0, 0, False))
        
        msg.footstepDataList.extend(createFootSteps(RIGHT, 4, 0, -0.25, 0, True))
        
        footStepDataListPub.publish(msg)
        print 'sent box step footstep list'
        waitForFootstepsToFinish()

def createFootSteps(foot_to_start_with, num_steps, x_slope, y_slope, z_slope, side_step):
    footStepList = []
    global x, y, z, desiredStepsSentCounter, footYOffset
    i = 0
    if side_step:
        num_steps *= 2
    
    while i < num_steps:
        
        desiredStepsSentCounter += 1    
        # Footsteps are in world frame
        footStepList.append(FootstepDataMessage())
        #Robot Side Left Foot = 0, Right Foot = 1
        current_foot = (i + foot_to_start_with) % 2
        footStepList[i].robotSide = current_foot
        yOffset = footYOffset if current_foot == LEFT else -footYOffset
        
        x += x_slope
        footStepList[i].location.x = x   
        y += y_slope 
        footStepList[i].location.y = y + yOffset 
        z += z_slope
        footStepList[i].location.z = z  

        footStepList[i].orientation.x = 0 
        footStepList[i].orientation.y = 0
        footStepList[i].orientation.z = 0
        footStepList[i].orientation.w = 1
        i += 1
        
        if side_step:
            footStepList.append(FootstepDataMessage())
            current_foot = (i + foot_to_start_with) % 2
            footStepList[i].robotSide = current_foot
            yOffset = footYOffset if current_foot == LEFT else -footYOffset
            
            footStepList[i].location.x = x
            footStepList[i].location.y = y + yOffset
            footStepList[i].location.z = z

            footStepList[i].orientation.x = 0
            footStepList[i].orientation.y = 0
            footStepList[i].orientation.z = 0
            footStepList[i].orientation.w = 1
            i += 1


    return footStepList

def recv_status_msg(msg):
    global footStepStatusCounter
    if msg.status == 1:
        footStepStatusCounter += 1 

def waitForFootstepsToFinish():
    global footStepStatusCounter
    global desiredStepsSentCounter
    while footStepStatusCounter < desiredStepsSentCounter:
        r.sleep()
    print 'finished set of steps'

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_box_step')
        footStepStatusSub = rospy.Subscriber('/atlas/outputs/ihmc_msgs/FootstepStatusMessage', FootstepStatusMessage, recv_status_msg)
        footStepDataListPub = rospy.Publisher('/atlas/inputs/ihmc_msgs/FootstepDataListMessage',FootstepDataListMessage, queue_size=2)
        r = rospy.Rate(10) # 10hz     
        walkInPlace(6)
        boxStep()
        rospy.spin() 
    except rospy.ROSInterruptException: pass
