#!/usr/bin/env python
import rospy
from prob_rob_labs.msg import SatelliteState
import time
import threading
import hashlib
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


class SatelliteStateSynchronizer:
    """
    Collect as many messages as possible within a timeout window, then process them.
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("satellite_state_synchronizer", anonymous=True)

        # List of topic names for /Satellite_1_local to /Satellite_10_local
        self.topic_names = [f"/Satellite_{i}_local" for i in range(1, 11)]

        self.odom_sub = rospy.Subscriber('/ekf_odom', Odometry, self.odom_callback)
        self.ekf_pub = rospy.Publisher('/ekf_pose', PoseWithCovarianceStamped, queue_size = 1)

        self.x = 0
        self.y = 0
        self.z = 0
        self.theta = 0
        
        self.x_bar = np.zeros((4,1))
        self.sigma_bar = np.eye(4)

        # Message buffer and tracking for duplicates
        self.message_buffer = {}
        self.processed_messages = set()

        # Timeout settings
        self.timeout = 0.1
        self.lock = threading.Lock()

        # Start a timer thread to process messages periodically
        self.timer_thread = threading.Thread(target=self.timer_callback)
        self.timer_thread.daemon = True
        self.timer_thread.start()

        # Subscribe to each topic
        for topic in self.topic_names:
            rospy.Subscriber(topic, SatelliteState, self.message_callback, callback_args=topic, queue_size=1)

        rospy.loginfo("Satellite state synchronizer node started.")
        rospy.spin()

    def generate_message_hash(self, msg):
        """
        Generate a unique hash for the message based on its content.
        """
        msg_data = f"{msg.x},{msg.y},{msg.z},{msg.travel_time}"
        return hashlib.md5(msg_data.encode()).hexdigest()

    def message_callback(self, msg, topic):
        """
        Callback for each topic to store the latest message.
        """
        current_time = time.time()
        message_hash = self.generate_message_hash(msg)

        with self.lock:
            # Avoid duplicates by checking hash
            if message_hash in self.processed_messages:
                rospy.logdebug(f"Duplicate message detected from {topic}.")
                return

            # Add the message to the buffer
            self.message_buffer[topic] = (msg, current_time)
            self.processed_messages.add(message_hash)
            rospy.logdebug(f"Message added to buffer from {topic}.")

    def timer_callback(self):
        """
        Periodically process messages every self.timeout seconds.
        """
        while not rospy.is_shutdown():
            time.sleep(self.timeout)  # Wait for the timeout window
            with self.lock:
                if self.message_buffer:
                    self.process_messages()
                    #self.ekf()
                else:
                    rospy.logdebug("No messages to process in this window.")

    def odom_callback(self, msg):
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.z = msg.pose.pose.position.z
            _, _, self.theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])


            self.x_bar[0] = self.x
            self.x_bar[1] = self.y
            self.x_bar[2] = self.z
            self.x_bar[3] = self.theta

            process_noise = np.eye(4) * 0.1
            self.sigma_bar = self.sigma_bar + process_noise

    def process_messages(self):
        """
        Process all buffered messages and clear the buffer.
        """
        if len(self.message_buffer) < 4:
            return
        # rospy.loginfo("Processing synchronized messages:")
        predicted_distances = []
        act_distances = []
        satellite_positions = []
        for topic, (msg, timestamp) in self.message_buffer.items():
            msg_time = msg.header.stamp.to_sec() if hasattr(msg, 'header') else "No timestamp"
            #rospy.loginfo(
            #    f"Topic={topic}, x={msg.x}, y={msg.y}, z={msg.z}, travel_time={msg.travel_time}, timestamp={msg_time}"
            #)

            sat_x, sat_y, sat_z = msg.x, msg.y, msg.z
            satellite_positions.append((sat_x, sat_y, sat_z))
            travel_time = msg.travel_time

            predicted_dist =  np.sqrt((self.x_bar[0] - sat_x)**2 + (self.x_bar[1] - sat_y)**2 + (self.x_bar[2] - sat_z)**2)
            predicted_distances.append(predicted_dist)

            c = 3e8
            act_dist = c * travel_time
            act_distances.append(act_dist)

        error = np.array([x - y for x, y in zip(act_distances, predicted_distances)])
        rospy.loginfo(f"error={error}")
        # Clear the buffer and retain recent processed hashes
        self.message_buffer.clear()
        # rospy.loginfo("Buffer cleared, waiting for the next timeout window.")
        # rospy.loginfo(f"satellite_positions: {satellite_positions}")   
        H = self.compute_jacobian(satellite_positions)
        
        # Perform ekf here
        # Measurement covariance is manually setup in stellite_simu, which is in the form of noise in light speed
        measurement_covariance = np.zeros((error.shape[0],error.shape[0]))
        for i in range(measurement_covariance.shape[0]):
            measurement_covariance[i, i] = 2e-8
        
        self.ekf(error, H, measurement_covariance)
        
        
        
    def ekf(self, error, H, R):

        # EKF Update Equations
        H_transpose = H.T

        # Innovation covariance
        S = H @ self.sigma_bar @ H_transpose + R

        # Kalman gain
        K = self.sigma_bar @ H_transpose @ np.linalg.inv(S)

        # Update state estimate
        self.x_bar += K @ error.reshape(-1, 1)

        # Update covariance estimate
        I = np.eye(self.sigma_bar.shape[0])
        self.sigma_bar = (I - K @ H) @ self.sigma_bar
        
        self.publish_pose()

        rospy.loginfo(f"x_bar: {self.x_bar}")
        #rospy.loginfo(f"sigma_bar: {self.sigma_bar}")

    def compute_jacobian(self, satellite_positions):
        num_satellites = len(satellite_positions)

        J = np.zeros((num_satellites, 4))

        c = 3e8

        for i, (x_i, y_i, z_i) in enumerate(satellite_positions):
            dist = np.sqrt((self.x - x_i)**2 + (self.y - y_i)**2 + (self.z - z_i)**2)

            d_dist_dx_u = (self.x - x_i) / dist
            d_dist_dy_u = (self.y - y_i) / dist
            d_dist_dz_u = (self.z - z_i) / dist
            d_dist_dtheta_u = 0

            J[i, 0] = d_dist_dx_u
            J[i, 1] = d_dist_dy_u
            J[i, 2] = d_dist_dz_u
            J[i, 3] = d_dist_dtheta_u
        return J

    def publish_pose(self):
        cov_6x6 = np.zeros((6,6))
        cov_6x6[:3, :3] = self.sigma_bar[:3, :3]
        cov_6x6[3:, 3:] = self.sigma_bar[3:, 3:]
        cov_6x6[:3, 3:] = self.sigma_bar[:3, 3:]
        cov_6x6[3:, :3] = self.sigma_bar[3:, :3]
        covariance_list = np.array(cov_6x6).flatten().tolist()

        pose_msg = PoseWithCovariance()
        pose_msg.pose.position.x = float(self.x_bar[0])
        pose_msg.pose.position.y = float(self.x_bar[1])
        pose_msg.pose.position.z = float(self.x_bar[2])
        quaternion = quaternion_from_euler(0, 0, float(self.x_bar[3]))
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        pose_msg.covariance = covariance_list

        PoseWithCovarianceStamped_msg = PoseWithCovarianceStamped(
                            header = Header(
                                stamp = rospy.Time.now(),
                                frame_id = 'map'
                                            ),
                            pose = pose_msg
                        )

        self.ekf_pub.publish(PoseWithCovarianceStamped_msg)

def main():
    try:
        SatelliteStateSynchronizer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Satellite state synchronizer node terminated.")

if __name__ == "__main__":
    main()
