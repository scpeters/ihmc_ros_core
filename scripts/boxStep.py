#!/usr/bin/env python
# license removed for brevity
import rospy
from  rigidBodyTransform import RigidBodyTransform
from geometry_msgs.msg import PoseStamped
from ihmc_msgs.msg import FootstepDataMessage
from ihmc_msgs.msg import FootstepDataListMessage
from ihmc_msgs.msg import FootstepStatusMessage
from ihmc_msgs.msg import SquareDataMessage
import math 

pelvis_transform = RigidBodyTransform()
foot_y_offset = 0.15
foot_z_offset = -0.705
left_offset =  [0, foot_y_offset, foot_z_offset]
right_offset = [0,-foot_y_offset, foot_z_offset]
footstep_offset_Vectors = [left_offset, right_offset]
lock = False
footstep_status_recv_counter = 0
desired_steps_sent_counter = 0
LEFT = 0
RIGHT = 1

def walkInPlace(num_steps):
    global footStepDataList_pub, RIGHT, r, lock 
        
    if not rospy.is_shutdown():
        while footStepDataList_pub.get_num_connections() == 0:
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
        # msg.trajectoryWaypointGenerationMethod = 0
        
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
        lock = True
        msg.footstepDataList = createFootStepList(RIGHT, num_steps, 0,0,0, False)
        lock = False
        footStepDataList_pub.publish(msg)
        print 'walk in place msg sent'
        waitForFootstepsToFinish()

def boxStep():
    global RIGHT, LEFT, r, footStepDataList_pub

    if not rospy.is_shutdown():
        while footStepDataList_pub.get_num_connections() == 0:
            print 'waiting for subsciber'
            r.sleep()
            
        msg = FootstepDataListMessage()
        msg.transferTime = 1.5
        msg.swingTime = 1.5
        #msg.trajectoryWaypointGenerationMethod = 0
        
        lock = True
        msg.footstepDataList.extend(createFootStepList(LEFT, 4, 0.25, 0, 0, False))
        msg.footstepDataList.extend(createFootStepList(LEFT, 1, 0, 0, 0, False))
        msg.footstepDataList.extend(createFootStepList(LEFT, 4, 0, 0.25, 0, True))  
        msg.footstepDataList.extend(createFootStepList(LEFT, 4, -0.25, 0, 0, False))
        msg.footstepDataList.extend(createFootStepList(LEFT, 1, 0, 0, 0, False))       
        msg.footstepDataList.extend(createFootStepList(RIGHT, 4, 0, -0.25, 0, True))
        lock = False
        
        footStepDataList_pub.publish(msg)
        print 'sent box step footstep list'
        waitForFootstepsToFinish()

'''
Takes in number of steps and a distance to walk each step
Will take double the number of side steps, since the second step is just a catch up
'''
def createFootStepList(foot_to_start_with, num_steps, x_slope, y_slope, z_slope, side_step):
    global desired_steps_sent_counter 
    footStepList = []
   
    slopeTransform = RigidBodyTransform(None, [x_slope, y_slope, z_slope])
    
    i = 0
    if side_step:
        num_steps *= 2

    while i < num_steps:
        desired_steps_sent_counter += 1    
        current_foot = (i + foot_to_start_with) % 2
        footStepList.append(createFootStep(current_foot, slopeTransform))
        i += 1
        
        if side_step:
            current_foot = (i + foot_to_start_with) % 2
            footStepList.append(createFootStep(current_foot, None))
            i += 1
    return footStepList


'''
uses the current pelvis pose 
and shifts the feet to the ground and out
to the sides.

offset is the relative offset from the pelvis
side: Left Foot = 0, Right Foot = 1
'''
def createFootStep(side, offset):
    footstep = FootstepDataMessage()
    footstep.robotSide = side 
    
    if offset is not None:
        pelvis_transform.transform(offset)
    
    foot_offset_vector = footstep_offset_Vectors[side]      
    footstep_shift = pelvis_transform.transformVector(foot_offset_vector)

    pelvis_transform.packTranslation(footstep.location)
    footstep.location.x += footstep_shift[0]
    footstep.location.y += footstep_shift[1]
    footstep.location.z += footstep_shift[2]
    pelvis_transform.packRotation(footstep.orientation)
    return footstep

'''
footStepStatus sends 0 for started and 1 for finished taking a step
'''
def recv_status_msg(msg):
    global footstep_status_recv_counter
    if msg.status == 1:
        footstep_status_recv_counter += 1 

def waitForFootstepsToFinish():
    global footstep_status_recv_counter, desired_steps_sent_counter
    while footstep_status_recv_counter < desired_steps_sent_counter:
        r.sleep()
    print 'finished set of steps'

def recv_rootpose(msg):
    global pelvis_transform, lock
    if lock is not True:
        pelvis_transform.setRotationFromQuaternion(msg.pose.orientation)
        pelvis_transform.setTranslationFromVector(msg.pose.position)

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_box_step')
        footStepStatusSub = rospy.Subscriber('/atlas/outputs/ihmc_msgs/FootstepStatusMessage', FootstepStatusMessage, recv_status_msg)
        rootPoseSub = rospy.Subscriber('/atlas/outputs/rootPose', PoseStamped, recv_rootpose)
        footStepDataList_pub = rospy.Publisher('/atlas/inputs/ihmc_msgs/FootstepDataListMessage',FootstepDataListMessage, queue_size=2)
        r = rospy.Rate(10) # 10hz   
        r.sleep()  
        walkInPlace(6)
        boxStep()
        rospy.spin() 
    except rospy.ROSInterruptException: pass
