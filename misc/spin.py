#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def spin():
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rospy.init_node('spin', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		my_T = Twist()
		my_T.linear.x = 1.0
		my_T.angular.z = 0.5
		rospy.loginfo(my_T)
		pub.publish(my_T)
		rate.sleep()

if __name__ == '__main__':
	try:
		spin()
	except rospy.ROSInterruptException:
		pass
