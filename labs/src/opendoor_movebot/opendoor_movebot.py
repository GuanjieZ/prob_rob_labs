#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class opendoor_movebot:
    def __init__(self):
        self.door_pub = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=10)
        self.bot_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.door_feature = rospy.Subscriber('/feature_mean', Float64, self.check_feature)
        self.state = 'stop'
        self.rate = rospy.Rate(10)

    def check_feature(self, data):
        # rospy.loginfo(data.data)
        if data.data < 400:
            self.state = 'move'

    def publish(self,event):
        op_time = event.current_expected.to_sec()
        if op_time <= 6.0:
            self.door_pub.publish(Float64(3.0))
        elif op_time > 8.0:
            self.door_pub.publish(Float64(-3.0))
            self.state = 'stop'
        if self.state == 'move':
            move_bot = Twist()
            move_bot.linear.x = 3
            self.bot_pub.publish(move_bot)
            return
        if self.state == 'stop':
            stop_bot = Twist()
            self.bot_pub.publish(stop_bot)

def main():
    rospy.init_node('opendoor_movebot')
    bot = opendoor_movebot()
    rospy.loginfo('starting opendoor_movebot')
    rospy.Timer(rospy.Duration(0.05), bot.publish)
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
