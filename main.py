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

# geometry_msgs/PoseWithCovarianceStamped.msg
def pose_callback(msg):
    global plotter, pose_3d, dcm, yaw
    # glogger.info("get pose, position = (%f, %f, %f)", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    pose_3d = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    # dcm = msg.pose.pose.orientation
    q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(q)
    dcm = tf.transformations.quaternion_matrix(q)
    yaw = euler[2]
    pose_2d = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
    plotter.plot_trajectory( pose_2d )
    return 0

# sensor_msgs/LaserScan.msg
def scan_callback(msg):
    global plotter, pose_3d, dcm, tt
    tt = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
    lp = LaserProjection()
    if msg.header.seq % 25 == 0:
        t_start = time.time()
        glogger.info("projection start, id = %d", msg.header.seq)
    points = lp.project(msg, pose_3d, dcm[0:3, 0:3])
    plotter.set_angle(msg.angle_min, msg.angle_max)
    plotter.plot_scan(points)
    if msg.header.seq % 25 == 0:
        glogger.info( "%d points projected, time consume = %f us", points.shape[0], (time.time() - t_start)*1e6 )

def listener():
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, pose_callback)
    # pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
    rospy.spin()

def tf_broadcaster():
    global tt, dcm, yaw
    tf_ = tf.TransformListener()
    time.sleep(1)
    while not rospy.is_shutdown():
        try:
            tf_.waitForTransform("/nav", "/base_link", tt, rospy.Duration.from_sec(0.5))
            (trans,rot) = tf_.lookupTransform("/nav", "/base_link", tt)
            # dcm = tf.transformations.quaternion_matrix(rot)
            euler = tf.transformations.euler_from_quaternion(rot)
            # yaw = euler[2]
            #print "trans:", trans
            #print "q:", rot
            #print "matrix", tf.transformations.quaternion_matrix(rot)
        except:
            glogger.error("can't listen tf!")
        # time.sleep(1)
    glogger.info("tf broadcaster quit.")

if __name__ == '__main__':
    global plotter, pose_3d, dcm, tt, yaw
    pose_3d = (0, 0, 0)
    dcm = np.eye(3)
    yaw = 0
    glogger = get_glogger(0, __file__)
    rospy.init_node('slam_py', anonymous=True)
    tt = rospy.Time.now()
    sub1 = threading.Thread(target = listener, args = ())
    sub1.start()
    sub2 = threading.Thread(target = tf_broadcaster, args = ())
    sub2.start()
    plotter = Plotter()
    plotter.show()
