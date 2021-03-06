#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import rospy, time
from glog import get_glogger

glogger = get_glogger(0, __file__)

# project sensor_msgs/LaserScan.msg to sensor_msgs/PointCloud.msg
class LaserProjection:
    angle_min = 0
    angle_max = 0
    angle_increment = 0
    time_increment = 0
    range_min = 0.5
    range_max = 15.0
    points_3d = []
    points_2d = None

    def __init__(self):
        range_min = 0.5
        range_max = 15.0

    # origin: tuple(0, 0, 0),  rotation: np.matrix(3,3)
    def project(self, scan, origin=(0, 0, 0), dcm=np.eye(3), downsample = 1):
        sita = scan.angle_min
        del self.points_3d[:]
        for i, r in enumerate(scan.ranges):
            if i%downsample != 0 or r < self.range_min or r > self.range_max:
                continue
            point = (r*np.cos(sita), r*np.sin(sita), 0)
            # point = np.matmul(dcm, point)
            # limit laser beam height to (-0.5, 0.5)m
            if point[2] < -0.5 or point[2] > 0.5:
                continue
            point = np.add(point, origin)
            self.points_3d.append(point.tolist())
            sita = scan.angle_min + scan.angle_increment*i
        # self.points_2d = np.array(self.points_3d, dtype=np.double)[: , 0:2]
        points_3d = np.array(self.points_3d, dtype=np.double)[: , 0:2]
        self.points_2d = np.ascontiguousarray(points_3d, dtype=np.double)
        return self.points_2d

if __name__ == '__main__':
    # glogger.info("pose: %f",pose.position.x)
    # lp = LaserProjection(origin)
    # sita = math.pi / 6
    # print math.cos(sita), math.sin(sita)

    x1 = np.eye(3)
    print "x1", x1
    x2 = (1,2,3)
    print "x2", x2
    x3 = np.add(x1, x2)
    print "x3", x3
    x4 = np.matmul(x1, x2)
    print "x4", x4
