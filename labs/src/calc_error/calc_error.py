#!/usr/bin/env python

import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import Float64
from math import *

class Calc_Error:
    def __init__(self):
        self.gt_pose = Subscriber('/jackal/ground_truth/pose', PoseStamped, queue_size = 1)
        self.odom = Subscriber('/ekf_odom', Odometry, queue_size = 1)
        self.jackal_odom = Subscriber('/jackal_velocity_controller/odom', Odometry, queue_size = 1)
        self.jackal_odom_filtered = Subscriber('/odometry/filtered', Odometry, queue_size = 1)
        self.sync = ApproximateTimeSynchronizer([
                    self.gt_pose, self.odom, self.jackal_odom, self.jackal_odom_filtered],
                    queue_size = 10, slop = 0.04, allow_headerless = True)
        self.sync.registerCallback(self.callback)
        self.pub_pos_error = rospy.Publisher('/Position_Error', Float64, queue_size = 1)
        self.pub_ang_error = rospy.Publisher('/Angular_Error', Float64, queue_size = 1)
        self.pub_pos_error_jo = rospy.Publisher('/Position_Error_jo', Float64, queue_size = 1)
        self.pub_ang_error_jo = rospy.Publisher('/Angular_Error_jo', Float64, queue_size = 1)
        self.pub_pos_error_jof = rospy.Publisher('/Position_Error_jof', Float64, queue_size = 1)
        self.pub_ang_error_jof = rospy.Publisher('/Angular_Error_jof', Float64, queue_size = 1)
        
        self.dis_error = Float64()
        self.ang_error = Float64()
        self.dis_error_jo = Float64()
        self.ang_error_jo = Float64()
        self.dis_error_jof = Float64()
        self.ang_error_jof = Float64()


    def callback(self, gt_pose_msg, odom_msg, jackal_odom_msg, jackal_odom_filtered_msg):
        gt_pos = gt_pose_msg.pose.position
        odom_pos = odom_msg.pose.pose.position
        gt_ang = gt_pose_msg.pose.orientation
        gt_roll, gt_pitch, gt_yaw = euler_from_quaternion([gt_ang.x, gt_ang.y, gt_ang.z, gt_ang.w])
        odom_ang = odom_msg.pose.pose.orientation
        odom_roll, odom_pitch, odom_yaw = euler_from_quaternion([odom_ang.x, odom_ang.y, odom_ang.z, odom_ang.w])

        self.dis_error = np.linalg.norm(np.array([gt_pos.x, gt_pos.y, gt_pos.z]) - np.array([odom_pos.x, odom_pos.y, odom_pos.z]))
        self.ang_error = atan2(sin(gt_yaw - odom_yaw), cos(gt_yaw - odom_yaw))
        
        self.pub_pos_error.publish(self.dis_error)
        self.pub_ang_error.publish(self.ang_error)

        # Jackal_odom
        gt_pos = gt_pose_msg.pose.position
        odom_pos = jackal_odom_msg.pose.pose.position
        gt_ang = gt_pose_msg.pose.orientation
        gt_roll, gt_pitch, gt_yaw = euler_from_quaternion([gt_ang.x, gt_ang.y, gt_ang.z, gt_ang.w])
        odom_ang = jackal_odom_msg.pose.pose.orientation
        odom_roll, odom_pitch, odom_yaw = euler_from_quaternion([odom_ang.x, odom_ang.y, odom_ang.z, odom_ang.w])

        self.dis_error_jo = np.linalg.norm(np.array([gt_pos.x, gt_pos.y, gt_pos.z]) - np.array([odom_pos.x, odom_pos.y, odom_pos.z]))
        self.ang_error_jo = atan2(sin(gt_yaw - odom_yaw), cos(gt_yaw - odom_yaw))

        self.pub_pos_error_jo.publish(self.dis_error_jo)
        self.pub_ang_error_jo.publish(self.ang_error_jo)

        # Jackal_odom_filtered
        gt_pos = gt_pose_msg.pose.position
        odom_pos = jackal_odom_filtered_msg.pose.pose.position
        gt_ang = gt_pose_msg.pose.orientation
        gt_roll, gt_pitch, gt_yaw = euler_from_quaternion([gt_ang.x, gt_ang.y, gt_ang.z, gt_ang.w])
        odom_ang = jackal_odom_filtered_msg.pose.pose.orientation
        odom_roll, odom_pitch, odom_yaw = euler_from_quaternion([odom_ang.x, odom_ang.y, odom_ang.z, odom_ang.w])

        self.dis_error_jof = np.linalg.norm(np.array([gt_pos.x, gt_pos.y, gt_pos.z]) - np.array([odom_pos.x, odom_pos.y, odom_pos.z]))
        self.ang_error_jof = atan2(sin(gt_yaw - odom_yaw), cos(gt_yaw - odom_yaw))

        self.pub_pos_error_jof.publish(self.dis_error_jof)
        self.pub_ang_error_jof.publish(self.ang_error_jof)




def main():
    rospy.init_node('calc_error')
    rospy.loginfo('starting calc_error')
    calc = Calc_Error()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
