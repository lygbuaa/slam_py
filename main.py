#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy, time, math, tf
import threading
from time import ctime,sleep
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from LaserProjection import LaserProjection
from glog import get_glogger
from Plotter import Plotter
from DataContainer import DataContainer
from ICP import SM_ICP

# geometry_msgs/PoseWithCovarianceStamped.msg
def pose_callback(msg):
    global plotter, data
    # glogger.info("get pose, position = (%f, %f, %f)", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    data.scan_origin = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    # dcm = msg.pose.pose.orientation
    q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    data.euler = tf.transformations.euler_from_quaternion(q)
    dcm = tf.transformations.quaternion_matrix(q)
    data.dcm = dcm[0:3, 0:3]
    yaw = data.euler[2]
    data.pose_2d = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
    plotter.plot_trajectory( data.pose_2d )
    return 0

# sensor_msgs/LaserScan.msg
def scan_callback(msg):
    global plotter, dt, data, icp
    data.scan_ts = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
    lp = LaserProjection()
    t_start = time.time()
    if msg.header.seq % 50 == 0:
        glogger.info("projection start, id = %d", msg.header.seq)
    data.points_2d = lp.project(msg, data.scan_origin, data.dcm, downsample = 5)
    plotter.set_angle(msg.angle_min, msg.angle_max)
    plotter.plot_scan(data.points_2d)
    pose_2d = icp.match(data.points_2d)
    plotter.plot_trajectory2( pose_2d )
    dt += (time.time() - t_start)*1e6 # time consume in us
    if msg.header.seq % 50 == 0:
        glogger.info( "%d points projected, time consume = %f us", data.points_2d.shape[0], dt/50.0 )
        dt = 0

def listener():
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, pose_callback)
    # pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
    rospy.spin()

def tf_broadcaster():
    global data
    tf_ = tf.TransformListener()
    time.sleep(1)
    while not rospy.is_shutdown():
        try:
            tf_.waitForTransform("/nav", "/base_link", data.scan_ts, rospy.Duration.from_sec(0.5))
            (trans,rot) = tf_.lookupTransform("/nav", "/base_link", tt)
            # data.dcm = tf.transformations.quaternion_matrix(rot)
            data.euler = tf.transformations.euler_from_quaternion(rot)
            #print "trans:", trans
            #print "q:", rot
            #print "matrix", tf.transformations.quaternion_matrix(rot)
        except:
            glogger.error("can't listen tf!")
        # time.sleep(1)
    glogger.info("tf broadcaster quit.")

if __name__ == '__main__':
    global plotter, dt, data, icp
    data = DataContainer()
    icp = SM_ICP()
    dt = 0
    glogger = get_glogger(0, __file__)
    rospy.init_node('slam_py', anonymous=True)
    tt = rospy.Time.now()
    sub1 = threading.Thread(target = listener, args = ())
    sub1.start()
    # sub2 = threading.Thread(target = tf_broadcaster, args = ())
    # sub2.start()
    plotter = Plotter()
    plotter.show()
