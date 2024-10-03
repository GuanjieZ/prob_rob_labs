#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

class opendoor_movebot:
    def __init__(self):
        self.door_pub = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=10)
        self.bot_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.current_time = rospy.Time(0)
        self.is_start_time = True
        self.start_time = 0
        self.duration = 0
        self.op_time = 0



    def clock_callback(self, msg):
        if self.is_start_time:
            self.start_time = msg.clock
            self.duration = msg.clock - self.start_time
            self.is_start_time = False
        
        else:
            self.current_time = msg.clock
            self.duration = self.current_time - self.start_time

        self.publish()

    def publish(self):
        self.op_time = self.duration.to_sec()
        if self.op_time > 0.0 and self.op_time <= 2.0:
            self.door_pub.publish(Float64(5.0))
        elif self.op_time > 2 and self.op_time <= 6:
            move_bot = Twist()
            move_bot.linear.x = rospy.get_param('vel_x')
            self.bot_pub.publish(move_bot)
        elif self.op_time > 6 and self.op_time <= 10:
            move_bot = Twist()
            move_bot.linear.x = 0
            self.bot_pub.publish(move_bot)
        else:
            self.door_pub.publish(Float64(-5.0))


def main():
    rospy.init_node('opendoor_movebot')
    bot = opendoor_movebot()
    rospy.loginfo('starting opendoor_movebot')
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
