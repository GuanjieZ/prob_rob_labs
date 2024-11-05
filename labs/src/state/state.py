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
<<<<<<< HEAD
        self.cmd_vel_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size = 1)
        self.move_bot = Twist()
        self.move_bot.linear.x = 0
        self.move_bot.angular.z = 0

    def pub(self, data):
        now = rospy.get_rostime()
=======
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.move_bot = Twist()
        self.move_bot.linear.x = 1
        self.move_bot.angular.z = 0.5


    def pub(self, data):
        now = rospy.get_rostime()
        for i in range(len(data.name)):
            if data.name[i] == 'jackal::base_link':
                link = i
                break
>>>>>>> Lab 4 Assignment 6
        pose = PoseStamped(
            header = Header(
	        stamp = now,
	        frame_id = 'odom'
	    ),
            pose = data.pose[find_frame_index(data.name, 'base_link')]
        )

        twist = TwistStamped(
            header = Header(
<<<<<<< HEAD
	        stamp = now,
	        frame_id = 'base_link'
	    ),
            twist = data.twist[find_frame_index(data.name, 'base_link')]
        )

        #self.cmd_vel_pub.publish(self.move_bot)
=======
                stamp = now,
                frame_id = 'base_link'
            ),
            twist = data.twist[link]
        )
>>>>>>> Lab 4 Assignment 6
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.cmd_vel_pub.publish(self.move_bot)

def main():
    rospy.init_node('state')
    rospy.loginfo('starting state')
    pub_state = state()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
