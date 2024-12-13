#!/usr/bin/env python
import rospy
from message_filters import Subscriber
from prob_rob_labs.msg import SatelliteState  # Import your custom message
from collections import deque
import time

class SatelliteStateSynchronizer:
    """
    Collect messages from multiple topics and process them when 4 or more are available.
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("satellite_state_synchronizer", anonymous=True)

        # List of topic names for /Satellite_1_local to /Satellite_10_local
        self.topic_names = [f"/Satellite_{i}_local" for i in range(1, 11)]

        # Create subscribers for each topic
        self.subscribers = []
        for topic in self.topic_names:
            sub = Subscriber(topic, SatelliteState)
            sub.registerCallback(self.message_callback, topic)
            self.subscribers.append(sub)

        # Buffer to hold incoming messages
        self.message_buffer = {}
        self.timeout = 0.01  # Time window in seconds to collect messages

        rospy.loginfo("Satellite state synchronizer node started.")
        rospy.spin()

    def message_callback(self, msg, topic):
        """
        Callback for individual topic messages. Adds messages to the buffer.
        """
        current_time = time.time()

        # Update the buffer with the latest message and timestamp
        self.message_buffer[topic] = (msg, current_time)

        # Check if we have enough messages to process
        self.process_messages()

    def process_messages(self):
        """
        Process buffered messages if 4 or more are available within the timeout window.
        """
        current_time = time.time()

        # Filter messages within the timeout window
        valid_messages = [
            (topic, msg) for topic, (msg, timestamp) in self.message_buffer.items()
            if current_time - timestamp <= self.timeout
        ]

        # Process only if 4 or more messages are available
        if len(valid_messages) >= 4:
            rospy.loginfo("Processing synchronized messages:")
            for i, (topic, msg) in enumerate(valid_messages):
                timestamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') else "No timestamp"
                rospy.loginfo(
                    f"{i+1}: Topic={topic}, x={msg.x}, y={msg.y}, z={msg.z}, travel_time={msg.travel_time}, timestamp={timestamp}"
                )

            # Clear the buffer after processing
            self.message_buffer.clear()


def main():
    try:
        SatelliteStateSynchronizer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Satellite state synchronizer node terminated.")


if __name__ == "__main__":
    main()
