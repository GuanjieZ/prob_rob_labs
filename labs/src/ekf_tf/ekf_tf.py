#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped



class EKF_TF:
    def __init__(self):
        self.ekf_pose = rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.create_transform)

        self.tf_broadcaster = TransformBroadcaster()
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'map'
        self.transform.child_frame_id = 'odom'

        self.last_pose = None
        self.timer = rospy.Timer(rospy.Duration(1.0/30), self.publish_transform)

    def create_transform(self, data):
        current_pose = data.pose.pose

        position = data.pose.pose.position
        orientation = data.pose.pose.orientation

        if self.last_pose is None or self.last_pose != current_pose:
            self.transform.header.stamp = rospy.Time.now()
            self.transform.transform.translation.x = position.x
            self.transform.transform.translation.y = position.y
            self.transform.transform.translation.z = position.z
            self.transform.transform.rotation = orientation

            self.last_pose = current_pose

    def publish_transform(self, event):
        self.tf_broadcaster.sendTransform(self.transform)

def main():
    rospy.init_node('ekf_tf')
    rospy.loginfo('starting ekf_tf')
    EKF_TF()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
