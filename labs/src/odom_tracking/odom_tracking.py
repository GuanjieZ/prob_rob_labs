#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Empty

class OdomTracker:
    def __init__(self):
        self.joint_state_sub = Subscriber('/joint_states', JointState)
        self.imu_sub = Subscriber('/imu/data', Imu)
        self.cmd_vel_sub = Subscriber('jackal_velocity_controller/cmd_vel', Twist)
        self.covariance_reset_sub = rospy.Subscriber('/covariance_trigger', Empty, self.covariance_reset)

        self.odom_pub = rospy.Publisher('/ekf_odom', Odometry, queue_size=10)

        self.sync = ApproximateTimeSynchronizer([
                self.joint_state_sub, self.imu_sub, self.cmd_vel_sub],
        queue_size = 10, slop = 0.04, allow_headerless = True)
        self.sync.registerCallback(self.callback)
        self.r_w = 0.098
        self.R = 0.27
        self.prev_time = None

        self.state = np.zeros((5,1))
        self.sigma_x = np.eye(5) * 0.01

    def callback(self, joint_state_msg, imu_msg, cmd_vel_msg):

        current_time = joint_state_msg.header.stamp
        delta_t = (current_time - self.prev_time).to_sec() if self.prev_time is not None else 0.0

        u_v = cmd_vel_msg.linear.x
        u_w = cmd_vel_msg.angular.z

        w_fl = joint_state_msg.velocity[0]
        w_fr = joint_state_msg.velocity[1]
        w_rl = joint_state_msg.velocity[2]
        w_rr = joint_state_msg.velocity[3]

        w_g = imu_msg.angular_velocity.z

        self.prev_time = current_time

        self.ekf(delta_t, u_v, u_w, w_fl, w_fr, w_rl, w_rr, w_g)

    def covariance_reset(self, msg):
        self.sigma_x[1][1] = 0
        self.sigma_x[2][2] = 0

    def ekf(self, delta_t, u_v, u_w, w_fl, w_fr, w_rl, w_rr, w_g):
        theta = self.state[0][0]
        x = self.state[1][0]
        y = self.state[2][0]
        v = self.state[3][0]
        omega = self.state[4][0]
        #u = np.array([[u_v], [u_w]])
        a_v = 0.1**(delta_t/0.25)
        a_w = 0.1**(delta_t/0.14)
        G_v = 1
        G_w = 1.34

        x_bar = np.zeros((5,1))
        x_bar[0][0] = theta + omega*delta_t
        x_bar[1][0] = x + v*delta_t*np.cos(theta)
        x_bar[2][0] = y + v*delta_t*np.sin(theta)
        x_bar[3][0] = a_v*v + G_v*(1-a_v)*u_v
        x_bar[4][0] = a_w*omega + G_w*(1-a_w)*u_w

        G_x = np.array([
                [1, 0, 0, 0,                     delta_t],
                [0, 1, 0, delta_t*np.cos(theta), 0],
                [0, 0, 1, delta_t*np.sin(theta), 0],
                [0, 0, 0, a_v,                   0],
                [0, 0, 0, 0,                     a_w],
        ])

        G_u = np.array([
                [0,           0],
                [0,           0],
                [0,           0],
                [G_v*(1-a_v), 0],
                [0,           G_w*(1-a_w)],
        ])

        #rospy.loginfo(B)
        '''
        G_x = np.array([
                [0, 0, 0,  0,                     0],
                [0, 0, 0, -delta_t*np.sin(theta), 0],
                [0, 0, 0,  delta_t*np.cos(theta), 0],
                [0, 0, 0,  0,                     0],
                [0, 0, 0,  0,                     0],
        ])

        G_u = B
        '''

        sigma_u = np.array([
                [0.1, 0],
                [0,   0.1],
        ])

        #rospy.loginfo(u)
        #x_bar = np.dot(A, self.state) + np.dot(B, u)
        #self.state = x_bar
        #rospy.loginfo(x_bar)

        sigma_x_bar = (
                G_x @ self.sigma_x @ G_x.T +
                G_u @  sigma_u @ G_u.T
        )

       # rospy.loginfo(sigma_x_bar)

        C = np.array([
                [0, 0, 0, 1 / self.r_w,  self.R / self.r_w],
                [0, 0, 0, 1 / self.r_w,  self.R / self.r_w],
                [0, 0, 0, 1 / self.r_w, -self.R / self.r_w],
                [0, 0, 0, 1 / self.r_w, -self.R / self.r_w],
                [0, 0, 0, 0,             1],
        ])

        sigma_z = np.array([
                [0.10, 0.05, 0,    0,    0], # fr
                [0.05, 0.10, 0,    0,    0], # rr
                [0,    0,    0.10, 0.05, 0], # fl
                [0,    0,    0.05, 0.10, 0], # rl
                [0,    0,    0,    0,    0.1], # gyro
        ])

        K = np.dot(
                np.dot(sigma_x_bar, C.T),
                np.linalg.inv(
                        np.dot(np.dot(C, sigma_x_bar), C.T) +
                        sigma_z
                )
        )

        z = np.array([[w_fr],[w_rr],[w_fl],[w_rl],[w_g]])

        new_x = x_bar + np.dot(K, z - np.dot(C, x_bar))
        self.state = new_x

        #rospy.loginfo(self.state)

        new_sigma_x = np.dot(
                np.eye(5) - np.dot(K, C),
                sigma_x_bar
        )
        self.sigma_x = new_sigma_x
        #rospy.loginfo(self.sigma_x)

        self.publish(delta_t)

    def publish(self, delta_t):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.get_rostime()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        theta, x, y = self.state[0][0], self.state[1][0], self.state[2][0]
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.state[3][0]
        odom_msg.twist.twist.angular.z = self.state[4][0]

        odom_msg.pose.covariance = ([
                self.sigma_x[1][1], 0,                  0, 0, 0, 0,
                0,                  self.sigma_x[2][2], 0, 0, 0, 0,
                0,                  0,                  0, 0, 0, 0,
                0,                  0,                  0, 0, 0, 0,
                0,                  0,                  0, 0, 0, 0,
                0,                  0,                  0, 0, 0, self.sigma_x[0][0]
        ])

        odom_msg.twist.covariance = ([
                self.sigma_x[3][3], 0, 0, 0, 0, 0,
                0,                  0, 0, 0, 0, 0,
                0,                  0, 0, 0, 0, 0,
                0,                  0, 0, 0, 0, 0,
                0,                  0, 0, 0, 0, 0,
                0,                  0, 0, 0, 0, self.sigma_x[4][4]
        ])

        self.odom_pub.publish(odom_msg)

def main():
    rospy.init_node('odom_tracking')
    rospy.loginfo('starting odom_tracking')
    tracker = OdomTracker()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
