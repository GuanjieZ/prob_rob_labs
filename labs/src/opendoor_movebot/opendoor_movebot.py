#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np
from std_msgs.msg import Empty

class opendoor_movebot:
    def __init__(self):
        self.door_pub = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=1)
        self.bot_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.door_feature = rospy.Subscriber('/feature_mean', Float64, self.check_feature)
        self.door_open = rospy.Publisher('/door_open', Empty, queue_size=1)
        self.state = 'stop'
        self.threshold = 455.0
        self.rate = rospy.Rate(10)
        self.measurement_model = np.array([[0.973, 0.173],[0.027, 0.827]])
        self.bel = np.array([[0.1],[0.9]])
        self.measurement = 0
        self.no_action_model = np.array([[1,0],[0,1]])
        self.bel_list = []
        self.action_model = np.array([[1,0.18],[0,0.82]])

    def check_feature(self, data):
        # rospy.loginfo(data.data)
        if data.data < self.threshold:
            self.measurement = 0
            prediction = self.no_action_model @ self.bel
        else:
            self.measurement = 1
            open_door = Empty()
            self.door_open.publish(open_door)
            prediction = self.action_model @ self.bel
        
        unnormalized_posterior = (self.measurement_model * np.repeat(self.bel, 2, axis=1).transpose())[self.measurement]
        posterior = unnormalized_posterior / sum(unnormalized_posterior)
        self.bel_list.append(round(posterior[0], 5))
        self.bel = np.array([ posterior ]).transpose()
        rospy.loginfo("#Measurement: " + str(len(self.bel_list)) + " Bel: " + str(self.bel_list[-1])+" M: "+str(self.measurement))

    def publish(self,event):
        op_time = event.current_expected.to_sec()
        if self.bel[0] > 0.99:
            self.state = 'move'
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
