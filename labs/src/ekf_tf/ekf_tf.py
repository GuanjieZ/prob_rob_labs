#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix

class EKF_TF:
    def __init__(self):
        # EKF Pose Subscriber
        self.ekf_pose_sub = rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.ekf_pose_callback)
        self.trigger_pub = rospy.Publisher('/covariance_trigger', Empty, queue_size = 10)
        
        # TF Listener for odom → base_link transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TF Broadcaster for publishing map → odom
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Variables to store EKF pose
        self.ekf_translation = None
        self.ekf_orientation = None

        # Timer to publish the transform periodically
        self.timer = rospy.Timer(rospy.Duration(1.0 / 30), self.publish_map_to_odom)

    def ekf_pose_callback(self, data):
        """
        Callback to store EKF pose (map → base_link).
        """
        self.ekf_translation = [
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z
        ]
        self.ekf_orientation = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]

    def publish_map_to_odom(self, event):
        """
        Compute and publish the transform map → odom.
        """
        if self.ekf_translation is None or self.ekf_orientation is None:
            rospy.logwarn_throttle(1.0, "EKF pose not received yet.")
            return

        try:
            # Get odom → base_link transform
            odom_to_base = self.tf_buffer.lookup_transform(
                "odom", "base_link", rospy.Time(0), rospy.Duration(0.1)
            )

            odom_translation = [
                odom_to_base.transform.translation.x,
                odom_to_base.transform.translation.y,
                odom_to_base.transform.translation.z,
            ]
            odom_orientation = [
                odom_to_base.transform.rotation.x,
                odom_to_base.transform.rotation.y,
                odom_to_base.transform.rotation.z,
                odom_to_base.transform.rotation.w,
            ]

            # Compute map → odom transform
            map_to_odom_translation, map_to_odom_orientation = self.compute_map_to_odom(
                self.ekf_translation, self.ekf_orientation,
                odom_translation, odom_orientation
            )

            # Publish the transform
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"
            transform.child_frame_id = "odom"
            transform.transform.translation.x = map_to_odom_translation[0]
            transform.transform.translation.y = map_to_odom_translation[1]
            transform.transform.translation.z = map_to_odom_translation[2]
            transform.transform.rotation.x = map_to_odom_orientation[0]
            transform.transform.rotation.y = map_to_odom_orientation[1]
            transform.transform.rotation.z = map_to_odom_orientation[2]
            transform.transform.rotation.w = map_to_odom_orientation[3]

            self.tf_broadcaster.sendTransform(transform)

            self.trigger_pub.publish(Empty())

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(1.0, "Waiting for odom → base_link transform.")

    def compute_map_to_odom(self, ekf_translation, ekf_orientation, odom_translation, odom_orientation):
        """
        Compute the map → odom transform using:
        T_map->odom = T_map->base_link * inv(T_odom->base_link)
        """
        # Invert the orientation of odom → base_link
        odom_orientation_inv = quaternion_inverse(odom_orientation)
        
        # Invert the translation of odom → base_link
        # Rotate the negative translation by the inverted quaternion
        inverted_odom_translation = [
            -odom_translation[0],
            -odom_translation[1],
            -odom_translation[2]
        ]
        odom_translation_rotated = self.rotate_vector_by_quaternion(inverted_odom_translation, odom_orientation_inv)
        
        # Compute the final translation (map → odom)
        map_to_odom_translation = [
            ekf_translation[0] + odom_translation_rotated[0],
            ekf_translation[1] + odom_translation_rotated[1],
            ekf_translation[2] + odom_translation_rotated[2]
        ]

        # Compute the final orientation (map → odom)
        map_to_odom_orientation = quaternion_multiply(ekf_orientation, odom_orientation_inv)

        return map_to_odom_translation, map_to_odom_orientation

    def rotate_vector_by_quaternion(self, vector, quaternion):
        """
        Rotate a vector by a quaternion.
        """
        from tf.transformations import quaternion_matrix
        rotation_matrix = quaternion_matrix(quaternion)[:3, :3]
        rotated_vector = rotation_matrix.dot(vector)
        return rotated_vector

def main():
    """
    Main function to initialize the node and run EKF_TF.
    """
    rospy.init_node('ekf_tf_broadcaster')
    ekf_tf = EKF_TF()
    rospy.spin()

if __name__ == "__main__":
    main()
