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
            sita = self.pose_new[2]
            dcm_inc = np.array( [[np.cos(sita), -1*np.sin(sita)], [np.sin(sita), np.cos(sita)]], dtype=np.double )
            self.dcm_2d = np.matmul(dcm_inc, self.dcm_2d)
            trans_inc = np.array( [self.pose_new[0], self.pose_new[1]], dtype=np.double )
            trans_map = np.matmul(self.dcm_2d.T, -1*trans_inc)
            self.pose_2d[0] += trans_map[0]
            self.pose_2d[1] += trans_map[1]
            self.pose_2d[2] = np.arcsin(self.dcm_2d[1, 0])
            if(self.pose_new[0] > 0.1 or self.pose_new[1] > 0.1):
                print "outstanding pose inc :", self.pose_new
                print "now pose_2d is:", self.pose_2d
        self.last_points_2d = points_2d
        return self.pose_2d

if __name__ == "__main__":
    exit(0)
