#!/usr/bin/env python
import rospy
from prob_rob_labs.msg import SatelliteState  # Import your custom message
import time
import threading
import hashlib
import numpy as np

class SatelliteStateSynchronizer:
    """
    Collect as many messages as possible within a timeout window, then process them.
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("satellite_state_synchronizer", anonymous=True)

        # List of topic names for /Satellite_1_local to /Satellite_10_local
        self.topic_names = [f"/Satellite_{i}_local" for i in range(1, 11)]

        self.x = 0
        self.y = 0
        self.z = 0

        # Message buffer and tracking for duplicates
        self.message_buffer = {}  # {topic: (msg, timestamp)}
        self.processed_messages = set()  # Hashes of processed messages

        # Timeout settings
        self.timeout = 0.1  # Time window in seconds
        self.lock = threading.Lock()  # To make buffer access thread-safe

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
                    #self.process_messages()
                    self.ekf()
                else:
                    rospy.logdebug("No messages to process in this window.")

    def process_messages(self):
        """
        Process all buffered messages and clear the buffer.
        """
        if len(self.message_buffer) < 4:
            return
        #rospy.loginfo("Processing synchronized messages:")
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

            predicted_dist =  np.sqrt((self.x - sat_x)**2 + (self.y - sat_y)**2 + (self.z - sat_z)**2)
            predicted_distances.append(predicted_dist)

            c = 3e8
            act_dist = c * travel_time
            act_distances.append(act_dist)

        error = np.array([x - y for x, y in zip(act_distances, predicted_distances)])
        #rospy.loginfo(f"error={error}")
        # Clear the buffer and retain recent processed hashes
        self.message_buffer.clear()
        rospy.loginfo("Buffer cleared, waiting for the next timeout window.")
        #rospy.loginfo(f"satellite_positions: {satellite_positions}")   
        H = self.compute_jacobian(satellite_positions)
        return error, H
   
    def prediction(self):
        rospy.loginfo("prediction function")

    def ekf(self):
        #rospy.loginfo("ekf function")
        error, H = self.process_messages()
        rospy.loginfo(f"{error}, {H}")

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

def main():
    try:
        SatelliteStateSynchronizer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Satellite state synchronizer node terminated.")

if __name__ == "__main__":
    main()
