## LookAtPacketMessage
# This message specifies a point in world coordinates for the robot to look at.
# The controller will attempt to orient the head such that the point is withinthe FOV of the camera using a combination of neck and pelvis joints. If thepoint is outside the combined range of the available joints, the robot willturn as close as it can to the point without taking any steps.

geometry_msgs/Vector3 lookAtPoint

