#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header

class state:
    def __init__(self):
        self.sub = rospy.Subscriber('/gazebo/link_states', LinkStates,self.pub)
        self.pose_pub = rospy.Publisher('/jackal/ground_truth/pose', PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher('/jackal/ground_truth/twist', TwistStamped, queue_size=1)

    def pub(self, data):
        now = rospy.get_rostime()
        pose = PoseStamped(
            header = Header(
	        stamp = now,
	        frame_id = 'odom'
	    ),
            pose = data.pose[1]
        )
        
        twist = TwistStamped()
        twist.twist = data.twist[1]
        twist.header = header
        
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)

def main():
    rospy.init_node('state')
    rospy.loginfo('starting state')
    pub_state = state()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
