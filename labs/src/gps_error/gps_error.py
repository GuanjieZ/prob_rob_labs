#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
import numpy as np
from math import *


class EKFError:
    def __init__(self):
        self.gt_pose = Subscriber('/jackal/ground_truth/pose', PoseStamped)
        self.ekf_pose = Subscriber('/ekf_pose', PoseWithCovarianceStamped)
        self.position_error_pub = rospy.Publisher('/position_error', Float64, queue_size = 1)
        self.orientation_error_pub = rospy.Publisher('/orientation_error', Float64, queue_size = 1)

        self.sync = ApproximateTimeSynchronizer([self.gt_pose, self.ekf_pose], queue_size = 10, slop = 0.1, allow_headerless = True)
        self.sync.registerCallback(self.calc_pose_error)

        self.position_error = Float64()
        self.orientation_error = Float64()

    def calc_pose_error(self, gt_pose, ekf_pose):
        x_gt = gt_pose.pose.position.x
        y_gt = gt_pose.pose.position.y

        x_ekf = ekf_pose.pose.pose.position.x
        y_ekf = ekf_pose.pose.pose.position.y

        self.position_error = np.linalg.norm(np.array([x_gt, y_gt]) - np.array([x_ekf, y_ekf]))
 
        gt_ang = gt_pose.pose.orientation  
        _, _, gt_yaw = euler_from_quaternion([gt_ang.x, gt_ang.y, gt_ang.z, gt_ang.w])

        ekf_ang = ekf_pose.pose.pose.orientation
        _, _, ekf_yaw = euler_from_quaternion([ekf_ang.x, ekf_ang.y, ekf_ang.z, ekf_ang.w])

        self.orientation_error = atan2(sin(gt_yaw - ekf_yaw), cos(gt_yaw - ekf_yaw))

        self.position_error_pub.publish(self.position_error)
        self.orientation_error_pub.publish(self.orientation_error)


def main():
    rospy.init_node('gps_error')
    rospy.loginfo('starting gps_error')
    EKFError()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
