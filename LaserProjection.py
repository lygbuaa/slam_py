#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import rospy, time
from glog import get_glogger
from geometry_msgs.msg import Pose

glogger = get_glogger(0, __file__)

# project sensor_msgs/LaserScan.msg to sensor_msgs/PointCloud.msg
class LaserProjection:
    timestamp = 0
    id = 0
    angle_min = 0
    angle_max = 0
    angle_increment = 0
    time_increment = 0
    scan_time = 0
    range_min = 0
    range_max = 60
    points_3d = []
    points_2d = []
    origin = None

    def __init__(self, origin):
        self.origin = origin
        # print self.origin
        # self.points_3d.append((0, 0, 0))
        # self.points_3d.append((1, 0, 0))
        # self.points_3d.append((2, 0, 0))
        # print self.points_3d
        # self.points_2d = np.mat(self.points_3d, dtype=np.float32)[: , 0:2].tolist()
        # print self.points_2d

    def project(self, scan):
        sita = scan.angle_min
        del self.points_3d[:]
        # del self.points_2d[:]
        # glogger.info("projection start, id = %d", scan.header.seq)
        # t_start = time.time()
        for r in scan.ranges:
            point = (r*math.cos(sita), r*math.sin(sita), 0)
            self.points_3d.append(point)
            sita += scan.angle_increment
        # glogger.info( "projection done, time consume = %f us", (time.time() - t_start)*1e6 )
        self.points_2d = np.mat(self.points_3d, dtype=np.float32)[: , 0:2]
        return self.points_2d

if __name__ == '__main__':
    origin = Pose()
    origin.position.x = 1
    origin.position.y = 2
    origin.position.z = 3
    origin.orientation.x = 0.1
    origin.orientation.y = 0.2
    origin.orientation.z = 0.3
    origin.orientation.w = 0.4

    # glogger.info("pose: %f",pose.position.x)
    lp = LaserProjection(origin)
    # sita = math.pi / 6
    # print math.cos(sita), math.sin(sita)
