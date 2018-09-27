#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy, time, math, tf
from time import ctime,sleep
from glog import get_glogger

class DataContainer:
    scan_ts = None
    points_2d = None
    points_2d_ref = None
    pose_2d = None
    pose_2d_ref = None
    scan_origin = None
    trans = None
    dcm = None
    euler = None

    def __init__(self):
        self.scan_ts = 0
        self.points_2d = np.array( (0, 0), dtype=np.double)
        self.points_2d_ref = np.array( (0, 0), dtype=np.double)
        self.scan_origin = (0, 0, 0) # (x, y, z)
        self.pose_2d = (0, 0, 0) # (x, y, yaw)
        self.pose_2d_ref = (0, 0, 0) # (x, y, yaw)
        self.trans = (0, 0, 0) # (x, y, z)
        self.dcm = np.eye(3) # rotation matrix
        self.euler = (0, 0, 0) # (pitch, roll, yaw)

if __name__ == '__main__':
    data = DataContainer()
    print data.points_2d
    print data.dcm
