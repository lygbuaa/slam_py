#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy, time, math
import threading
from time import ctime,sleep
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from LaserProjection import LaserProjection
from glog import get_glogger
from Plotter import Plotter

# sensor_msgs/LaserScan.msg
def scan_callback(msg):
    global scan, plotter
    origin = Pose()
    lp = LaserProjection(origin)
    # scan = msg.ranges
    # plotter.plot_scan(scan)
    t_start = time.time()
    glogger.info("projection start, id = %d", msg.header.seq)
    points = lp.project(msg)
    plotter.plot_scan(points)
    glogger.info( "projection done, time consume = %f us", (time.time() - t_start)*1e6 )

def listener():
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    # pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    glogger = get_glogger(0, __file__)
    rospy.init_node('slam_py', anonymous=True)
    sub = threading.Thread(target = listener, args = ())
    sub.start()
    plotter = Plotter()
    plotter.show()
