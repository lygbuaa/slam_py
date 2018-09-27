#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math, time, ctypes

class CSM_ICP:
    so = None
    icp = None
    pose_2d = ()
    dcm_2d = None
    trans_2d = None
    last_points_2d = None
    pose_old = None
    pose_new = None

    def __init__(self):
        self.so = ctypes.CDLL('./csm/AndreaCensi/csm/build/sm/libcsm_py.so')
        self.icp = self.so.csm_py
        array_1d_double = np.ctypeslib.ndpointer(dtype=np.double, ndim=1, flags='CONTIGUOUS')
        array_2d_double = np.ctypeslib.ndpointer(dtype=np.double, ndim=2, flags='CONTIGUOUS')
        self.icp.argtypes = [array_2d_double, ctypes.c_int, array_2d_double, ctypes.c_int, array_1d_double, array_1d_double]
        self.icp.restype = None
        self.dcm_2d = np.eye(2)
        self.trans_2d = [0, 0]
        self.pose_2d = [0, 0, 0]
        self.pose_old = np.empty(3, dtype=np.double)
        self.pose_new = np.empty(3, dtype=np.double)

    def match(self, points_2d):
        if self.last_points_2d is not None:
            self.icp(self.last_points_2d, len(self.last_points_2d), points_2d, len(points_2d), self.pose_old, self.pose_new)
            self.pose_2d[0] += self.pose_new[0]
            self.pose_2d[1] += self.pose_new[1]
            self.pose_2d[2] += self.pose_new[2]
            if(self.pose_new[0] > 0.1 or self.pose_new[1] > 0.1):
                print "outstanding pose inc :", self.pose_new
                print "now pose_2d is:", self.pose_2d
        self.last_points_2d = points_2d
        return self.pose_2d

if __name__ == "__main__":
    exit(0)
