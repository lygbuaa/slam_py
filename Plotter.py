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
    # data = np.random.rand(1040)*100
    # xdata = np.random.rand(1040)*100
    # ydata = np.random.rand(1040)*100
    data = np.zeros( (1040,2) )
    robot = np.zeros( (40,2) )
    xdata = []
    ydata = []
    fig = plt.figure()
    axes = fig.add_subplot(111, autoscale_on=True)
    line, = axes.plot(xdata, ydata, 'r*')
    aini = None

    def __init__(self):
        self.axes.set_autoscale_on(True)
        self.axes.relim()
        self.axes.autoscale_view(True,True,True)
        self.ani = animation.FuncAnimation(self.fig, self.update, self.data_gen, interval=1*100)
        # self.axes.set_autoscale_on(True)

    def show(self):
        # plt.autoscale(enable=True, axis='both', tight=None)
        plt.show()

    def update(self, data):
        # self.line.set_ydata(data)
        # xdata = np.random.rand(1040)*1000
        # ydata = np.random.rand(1040)*100
        # data = np.ones( (1040,2) )
        xdata = data[:,0]
        ydata = data[:,1]
        # print xdata, ydata
        self.line.set_data(xdata, ydata)
        self.axes.relim()
        self.axes.autoscale_view(True,True,True)
        return self.line,

    def data_gen(self):
        while True:
            # rospy.loginfo("gen points %d", len(scan))
            yield self.data

    def plot_scan(self, points):
        self.data = points
        self.plot_robot(0, 0, 0)
        # print points[: , 0].

    def plot_robot(self, x0, y0, yaw):
        # del self.robot[:]
        robot = np.zeros( (1,2) )
        for r in range(20):
            point = [(r/4.0*np.cos(yaw), r/4.0*np.sin(yaw))]
            robot = np.append(robot, np.mat(point), axis=0)
        for r in range(19):
            point = [((r-9)/4.0*np.cos(yaw + math.pi/2), (r-9)/4.0*np.sin(yaw + math.pi/2))]
            robot = np.append(robot, np.mat(point), axis=0)
        self.data = np.append(self.data, robot, axis=0)
        # print points[: , 0]

if __name__ == '__main__':
    glogger = get_glogger(0, __file__)
    # plotter = Plotter()
    head = np.zeros( (10,2) )
    tail = np.ones( (2,2) )
    head = np.append(head, tail, axis=0)
    print head
