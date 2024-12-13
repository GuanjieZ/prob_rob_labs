#!/usr/bin/env python
import rospy
from prob_rob_labs.msg import SatelliteState  # Import your custom message
import time
import threading
import hashlib

class SatelliteStateSynchronizer:
    """
    Collect as many messages as possible within a timeout window, then process them.
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("satellite_state_synchronizer", anonymous=True)

        # List of topic names for /Satellite_1_local to /Satellite_10_local
        self.topic_names = [f"/Satellite_{i}_local" for i in range(1, 11)]

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
                    self.process_messages()
                else:
                    rospy.logdebug("No messages to process in this window.")

    def process_messages(self):
        """
        Process all buffered messages and clear the buffer.
        """
        rospy.loginfo("Processing synchronized messages:")
        for topic, (msg, timestamp) in self.message_buffer.items():
            msg_time = msg.header.stamp.to_sec() if hasattr(msg, 'header') else "No timestamp"
            rospy.loginfo(
                f"Topic={topic}, x={msg.x}, y={msg.y}, z={msg.z}, travel_time={msg.travel_time}, timestamp={msg_time}"
            )

        # Clear the buffer and retain recent processed hashes
        self.message_buffer.clear()
        rospy.loginfo("Buffer cleared, waiting for the next timeout window.")

def main():
    try:
        SatelliteStateSynchronizer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Satellite state synchronizer node terminated.")

if __name__ == "__main__":
    main()
