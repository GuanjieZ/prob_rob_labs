#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header

def find_frame_index(list, frame):
    for index, element in enumerate(list):
        if frame in element:
            return index

class state:
    def __init__(self):
        self.sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.pub)
        self.pose_pub = rospy.Publisher('/jackal/ground_truth/pose', PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher('/jackal/ground_truth/twist', TwistStamped, queue_size=1)


def main():
    rospy.init_node('measurement_predictor')
    rospy.loginfo('starting measurement_predictor')
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
