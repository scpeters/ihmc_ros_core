#!/usr/bin/env python

import time
import rospy
import tf2_ros

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusMessage
from ihmc_msgs.msg import RobotConfigurationDataMessage
from ihmc_msgs.msg import FootstepDataListMessage
from ihmc_msgs.msg import FootstepDataMessage

from rigidBodyTransform import RigidBodyTransform

LEFT = 0
RIGHT = 1

def stepInPlace():
    msg = FootstepDataListMessage()
    msg.transferTime = 1.5
    msg.swingTime = 1.5

    msg.footstepDataList.append(createFootStepInPlace(LEFT))
    msg.footstepDataList.append(createFootStepInPlace(RIGHT))
    msg.footstepDataList.append(createFootStepInPlace(LEFT))
    msg.footstepDataList.append(createFootStepInPlace(RIGHT))

    footStepListPublisher.publish(msg)
    print 'walking in place...'
    waitForFootsteps(len(msg.footstepDataList))

def boxStep():
    msg = FootstepDataListMessage()
    msg.transferTime = 1.5
    msg.swingTime = 1.5

    # walk forward starting LEFT
    msg.footstepDataList.append(createFootStepOffset(LEFT, [0.2, 0.0, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(RIGHT, [0.4, 0.0, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(LEFT, [0.4, 0.0, 0.0]))

    # walk left starting LEFT
    msg.footstepDataList.append(createFootStepOffset(LEFT, [0.4, 0.2, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(RIGHT, [0.4, 0.2, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(LEFT, [0.4, 0.4, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(RIGHT, [0.4, 0.4, 0.0]))

    # walk back starting LEFT
    msg.footstepDataList.append(createFootStepOffset(LEFT, [0.2, 0.4, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(RIGHT, [0.0, 0.4, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(LEFT, [0.0, 0.4, 0.0]))

    # walk right starting RIGHT
    msg.footstepDataList.append(createFootStepOffset(RIGHT, [0.0, 0.2, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(LEFT, [0.0, 0.2, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(RIGHT, [0.0, 0.0, 0.0]))
    msg.footstepDataList.append(createFootStepOffset(LEFT, [0.0, 0.0, 0.0]))

    footStepListPublisher.publish(msg)
    print 'box stepping...'
    waitForFootsteps(len(msg.footstepDataList))

def createFootStepInPlace(stepSide):
    footstep = FootstepDataMessage()
    footstep.robotSide = stepSide

    if stepSide == LEFT:
        foot_frame = 'l_foot'
    else:
        foot_frame = 'r_foot'

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep

def createFootStepOffset(stepSide, offset):
    footstep = createFootStepInPlace(stepSide)

    footWorldRotation = RigidBodyTransform()
    footWorldRotation.setRotationFromQuaternion(footstep.orientation)
    transformedOffset = footWorldRotation.transformVector(offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    print 'finished set of steps'

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_box_step')

        footStepStatusSubscriber = rospy.Subscriber('/ihmc_msgs/atlas/output/footstep_status', FootstepStatusMessage, recievedFootStepStatus)
        footStepListPublisher = rospy.Publisher('/ihmc_msgs/atlas/control/footstep_list', FootstepDataListMessage, queue_size=1)
        
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if footStepListPublisher.get_num_connections() == 0:
            print 'waiting for subsciber...'
            while footStepListPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            stepInPlace()

        time.sleep(1)

        if not rospy.is_shutdown():
            boxStep()

    except rospy.ROSInterruptException:
        pass