#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy, time, math
from time import ctime,sleep
from glog import get_glogger

# project sensor_msgs/LaserScan.msg to sensor_msgs/PointCloud.msg
class Plotter:
    pointcloud = np.zeros( (1040,2) )
    robot = np.zeros( (40,2) )
    trajectory = np.zeros((1,2))
    trajectory_2 = np.zeros((1,2))
    pose_2d = (0, 0, 0)
    angle_min = -1*math.pi/2
    angle_max = math.pi/2

    fig = plt.figure()
    axes = fig.add_subplot(111, autoscale_on=True)
    lines = []
    aini = None

    def __init__(self):
        xdata = []
        ydata = []
        line, = self.axes.plot(xdata, ydata, 'r.') # point cloud
        self.lines.append(line)
        line, = self.axes.plot(xdata, ydata, 'b+') # plot_robot
        self.lines.append(line)
        line, = self.axes.plot(xdata, ydata, 'go') # trajectory
        self.lines.append(line)
        self.axes.set_autoscale_on(True)
        self.axes.relim()
        self.axes.autoscale_view(True,True,True)
        self.ani = animation.FuncAnimation(self.fig, self.update, self.data_gen, interval=1*200)

    def show(self):
        plt.show()

    def set_angle(self, min, max):
        self.angle_max = max
        self.angle_min = min

    def update(self, data):
        # xdata = np.random.rand(1040)*1000
        # ydata = np.random.rand(1040)*100
        xdata = data[0][:,0]
        ydata = data[0][:,1]
        self.lines[0].set_data(xdata, ydata)
        xdata = data[1][:,0]
        ydata = data[1][:,1]
        self.lines[1].set_data(xdata, ydata)
        xdata = data[2][:,0]
        ydata = data[2][:,1]
        self.lines[2].set_data(xdata, ydata)
        self.axes.axis('equal')
        self.axes.relim()
        self.axes.autoscale_view(True,True,True)
        return self.lines,

    def data_gen(self):
        while True:
            # rospy.loginfo("gen points %d", len(scan))
            yield [self.pointcloud, self.robot, self.trajectory]

    def plot_trajectory(self, pose_2d=(0,0,0)):
        self.plot_robot(pose_2d)
        self.trajectory = np.append(self.trajectory, np.mat(pose_2d[0:2]), axis=0)

    def plot_scan(self, points):
        self.pointcloud = points
        # self.plot_robot(pose_2d)
        # print points[: , 0].

    def plot_robot(self, pose_2d):
        # del self.robot[:]
        x0 = pose_2d[0]
        y0 = pose_2d[1]
        yaw = pose_2d[2]
        robot = np.zeros( (1,2) )

        for r in range(20):
            point = [(x0 + r/4.0*np.cos(yaw), y0 + r/4.0*np.sin(yaw))]
            robot = np.append(robot, np.mat(point), axis=0)
        for r in range(10):
            point = [(x0 + r/4.0*np.cos(yaw + self.angle_max), y0 + r/4.0*np.sin(yaw + self.angle_max))]
            robot = np.append(robot, np.mat(point), axis=0)
        for r in range(10):
            point = [(x0 + r/4.0*np.cos(yaw + self.angle_min), y0 + r/4.0*np.sin(yaw + self.angle_min))]
            robot = np.append(robot, np.mat(point), axis=0)

        # self.data = np.append(self.data, robot, axis=0)
        self.robot = robot
        # print points[: , 0]

if __name__ == '__main__':
    glogger = get_glogger(0, __file__)
    # plotter = Plotter()
