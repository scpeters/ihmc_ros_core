#!/usr/bin/env python
import math
class RigidBodyTransform:

    def __init__(self, rot = None, trans = None):
        
        self.row0 = [1,0,0,0]
        self.row1 = [0,1,0,0]
        self.row2 = [0,0,1,0]
        self.row3 = [0,0,0,1]

        if rot is not None:
            self.setRotation(rot[0], rot[1], rot[2], rot[3])
        if trans is not None:
            self.setTranslation(trans[0], trans[1], trans[2])

    def setRotation(self, qx, qy, qz, qw):

        yy2 = 2.0 * qy * qy
        zz2 = 2.0 * qz * qz
        xx2 = 2.0 * qx * qx
        xy2 = 2.0 * qx * qy
        wz2 = 2.0 * qw * qz
        xz2 = 2.0 * qx * qz
        wy2 = 2.0 * qw * qy
        yz2 = 2.0 * qy * qz
        wx2 = 2.0 * qw * qx

        self.row0[0] = (1.0 - yy2 - zz2)
        self.row0[1] = (xy2 - wz2)
        self.row0[2] = (xz2 + wy2)
        self.row1[0] = (xy2 + wz2)
        self.row1[1] = (1.0 - xx2 - zz2)
        self.row1[2] = (yz2 - wx2)
        self.row2[0] = (xz2 - wy2)
        self.row2[1] = (yz2 + wx2)
        self.row2[2] = (1.0 - xx2 - yy2)

    def setRotationFromQuaternion(self, quaternion):
        self.setRotation(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    
    def setTranslationFromVector(self, vector):
        self.row0[3] = vector.x
        self.row1[3] = vector.y
        self.row2[3] = vector.z

    def setTranslation(self, x, y, z):
        self.row0[3] = x
        self.row1[3] = y
        self.row2[3] = z

    def transform(self, transform2):
        tmp00 = self.row0[0] * transform2.row0[0] + self.row0[1] * transform2.row1[0] + self.row0[2] * transform2.row2[0]
        tmp01 = self.row0[0] * transform2.row0[1] + self.row0[1] * transform2.row1[1] + self.row0[2] * transform2.row2[1]
        tmp02 = self.row0[0] * transform2.row0[2] + self.row0[1] * transform2.row1[2] + self.row0[2] * transform2.row2[2]
        tmp03 = self.row0[0] * transform2.row0[3] + self.row0[1] * transform2.row1[3] + self.row0[2] * transform2.row2[3] + self.row0[3]

        tmp10 = self.row1[0] * transform2.row0[0] + self.row1[1] * transform2.row1[0] + self.row1[2] * transform2.row2[0]
        tmp11 = self.row1[0] * transform2.row0[1] + self.row1[1] * transform2.row1[1] + self.row1[2] * transform2.row2[1]
        tmp12 = self.row1[0] * transform2.row0[2] + self.row1[1] * transform2.row1[2] + self.row1[2] * transform2.row2[2]
        tmp13 = self.row1[0] * transform2.row0[3] + self.row1[1] * transform2.row1[3] + self.row1[2] * transform2.row2[3] + self.row1[3]

        tmp20 = self.row2[0] * transform2.row0[0] + self.row2[1] * transform2.row1[0] + self.row2[2] * transform2.row2[0]
        tmp21 = self.row2[0] * transform2.row0[1] + self.row2[1] * transform2.row1[1] + self.row2[2] * transform2.row2[1]
        tmp22 = self.row2[0] * transform2.row0[2] + self.row2[1] * transform2.row1[2] + self.row2[2] * transform2.row2[2]
        tmp23 = self.row2[0] * transform2.row0[3] + self.row2[1] * transform2.row1[3] + self.row2[2] * transform2.row2[3] + self.row2[3]

        self.row0[0] = tmp00
        self.row0[1] = tmp01
        self.row0[2] = tmp02
        self.row0[3] = tmp03
        self.row1[0] = tmp10
        self.row1[1] = tmp11
        self.row1[2] = tmp12
        self.row1[3] = tmp13
        self.row2[0] = tmp20
        self.row2[1] = tmp21
        self.row2[2] = tmp22
        self.row2[3] = tmp23
   
    def packTranslation(self, trans):
        trans.x = self.row0[3]
        trans.y = self.row1[3]
        trans.z = self.row2[3]

    def packRotation(self, quat):
        trace = self.row0[0] + self.row1[1] + self.row2[2]
        val = 0

        if trace > 0.0:
           val = math.sqrt(trace + 1.0) * 2.0
           quat.x = (self.row2[1] - self.row1[2]) / val
           quat.y = (self.row0[2] - self.row2[0]) / val
           quat.z = (self.row1[0] - self.row0[1]) / val
           quat.w = 0.25 * val
        elif self.row1[1] > self.row2[2]:
           val = math.sqrt(1.0 + self.row1[1] - self.row0[0] - self.row2[2]) * 2.0
           quat.x = (self.row0[1] + self.row1[0]) / val
           quat.y = 0.25 * val
           quat.z = (self.row1[2] + self.row2[1]) / val
           quat.w = (self.row0[2] - self.row2[0]) / val
        elif self.row0[0] > self.row1[1] and self.row0[0] > self.row2[2]:
           val = math.sqrt(1.0 + self.row0[0] - self.row1[1] - self.row2[2]) * 2.0
           quat.x = 0.25 * val
           quat.y = (self.row0[1] + self.row1[0]) / val
           quat.z = (self.row0[2] + self.row2[0]) / val
           quat.w = (self.row2[1] - self.row1[2]) / val
        else:
           val = math.sqrt(1.0 + self.row2[2] - self.row0[0] - self.row1[1]) * 2.0
           quat.x = (self.row0[2] + self.row2[0]) / val
           quat.y = (self.row1[2] + self.row2[1]) / val
           quat.z = 0.25 * val
           quat.w = (self.row1[0] - self.row0[1]) / val

    def transformVector(self, vector):
        x = self.row0[0] * vector[0] + self.row0[1] * vector[1] + self.row0[2] * vector[2];
        y = self.row1[0] * vector[0] + self.row1[1] * vector[1] + self.row1[2] * vector[2];
        z = self.row2[0] * vector[0] + self.row2[1] * vector[1] + self.row2[2] * vector[2];
        return [x,y,z]

