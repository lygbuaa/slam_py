#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import time
from sklearn.neighbors import NearestNeighbors
from glog import get_glogger

glogger = get_glogger(0, __file__)

# scan matcher: basic ICP algorithm
class SM_ICP:
    pose_2d = ()
    dcm_2d = None
    trans_2d = None
    last_points_2d = None

    def __init__(self):
        self.dcm_2d = np.eye(2)
        self.trans_2d = [0, 0]
        self.pose_2d = [0, 0, 0]

    def best_fit_transform(self, A, B):
        '''
        Calculates the least-squares best-fit transform between corresponding 3D points A->B
        Input:
          A: Nx2 numpy array of corresponding 3D points
          B: Nx2 numpy array of corresponding 3D points
        Returns:
          T: 3x3 homogeneous transformation matrix
          R: 2x2 rotation matrix
          t: 2x1 column vector
        '''
        assert len(A) == len(B)
        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        W = np.dot(BB.T, AA)
        U, s, VT = np.linalg.svd(W)
        R = np.dot(U, VT)

        # special reflection case
        if np.linalg.det(R) < 0:
           VT[1,:] *= -1
           R = np.dot(U, VT)
        # translation
        t = centroid_B.T - np.dot(R, centroid_A.T)

        # homogeneous transformation
        T = np.identity(3)
        T[0:2, 0:2] = R
        T[0:2, 2] = t
        return T, R, t

    # Naive algorithm, time complexity is O(N^2)
    def nearest_neighbor(self, src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nx2 array of points
            dst: Nx2 array of points
        Output:
            distances: Euclidean distances (errors) of the nearest neighbor
            indecies: dst indecies of the nearest neighbor
        '''
        indecies = np.zeros(src.shape[0], dtype=np.int)
        distances = np.zeros(src.shape[0])
        for i, s in enumerate(src):
            min_dist = np.inf
            for j, d in enumerate(dst):
                dist = np.linalg.norm(s-d)
                if dist < min_dist:
                    min_dist = dist
                    indecies[i] = j
                    distances[i] = dist
        return distances, indecies

    # this method don't work well, though it's time complexity perform is good
    def nearest_neighbor2(self, src, dst):
        #Find the nearest neighbours between the current source and the
        #destination cloudpoint
        indecies = np.zeros(src.shape[0], dtype=np.int)
        distances = np.zeros(src.shape[0])
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(dst)
        distances, indices = nbrs.kneighbors(src)
        return distances, indecies

    def icp(self, A, B, init_pose=None, max_iterations=50, tolerance=0.001):
        '''
        The Iterative Closest Point method
        Input:
            A: Nx2 numpy array of source 2D points
            B: Nx2 numpy array of destination 2D point
            init_pose: 3x3 homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria
        Output:
            T: final homogeneous transformation
            distances: Euclidean distances (errors) of the nearest neighbor
        '''
        # make points homogeneous, copy them so as to maintain the originals
        src = np.ones((3, A.shape[0]))
        dst = np.ones((3, B.shape[0]))
        src[0:2,:] = np.copy(A.T)
        dst[0:2,:] = np.copy(B.T)

        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)

        prev_error = 0

        for i in range(max_iterations):
            # find the nearest neighbours between the current source and destination points
            # ts = time.time()
            distances, indices = self.nearest_neighbor(src[0:2,:].T, dst[0:2,:].T)
            # glogger.info("nn time:%f", (time.time()-ts)*1e6)
            # ts = time.time()
            # compute the transformation between the current source and nearest destination points
            T,_,_ = self.best_fit_transform(src[0:2,:].T, dst[0:2,indices].T)
            # glogger.info("bft time:%f", (time.time()-ts)*1e6)

            # update the current source
            # refer to "Introduction to Robotics" Chapter2 P28. Spatial description and transformations
            src = np.dot(T, src)

            # check error
            mean_error = np.sum(distances) / distances.size
            if abs(prev_error-mean_error) < tolerance:
                # print "iteration break at: ", i
                break
            prev_error = mean_error
        # calculcate final tranformation
        T, R, t = self.best_fit_transform(A, src[0:2,:].T)
        # return T, distances
        # print "distance.average too big: ", np.average(distances)
        return R, t

    def match(self, points_2d):
        if self.last_points_2d is not None:
            dcm_inc, trans_inc = self.icp(self.last_points_2d, points_2d)
            self.dcm_2d = np.matmul(self.dcm_2d, dcm_inc)
            self.pose_2d[0] += trans_inc[0]
            self.pose_2d[1] += trans_inc[1]
            self.pose_2d[2] = np.arcsin(self.dcm_2d[1, 0])
        self.last_points_2d = points_2d
        return self.pose_2d

if __name__ == "__main__":
    A = np.random.randint(0,100,(10,2))    # 20 points for test
    A = np.array(A, dtype=np.float32)[: , 0:2]
    rotz = lambda theta: np.array([[np.cos(theta),-np.sin(theta)],
                                       [np.sin(theta),np.cos(theta)]
                                  ])
    trans = np.array([1.11, 2.22])
    B = A.dot(rotz(np.pi/30).T) + trans
    C = np.append(B, np.mat((-1, -1)), axis=0)
    icp = SM_ICP()
    glogger.info("icp start")
    pose_2d = icp.match(A)
    pose_2d = icp.match(C)
    # R, t = icp.icp(A, C)
    glogger.info("icp end")
    np.set_printoptions(precision=3,suppress=True)
    print pose_2d
    # print "sita: ", np.arcsin(R[1, 0])*57.3
    # print distances
