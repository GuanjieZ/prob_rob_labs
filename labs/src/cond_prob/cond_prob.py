#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from std_srvs.srv import Empty

class prob:
    def __init__(self):
        self.door_pub = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=10)
        self.door_feature = rospy.Subscriber('/feature_mean', Float64, self.record_data)
        self.state = 'stop'
        self.data = 0
        self.z = []
    
    def record_data(self, data):
        #rospy.loginfo(data.data)
        self.data = data.data
        if data.data < 440 and self.state == 'record':
            self.z.append(1)
        elif data.data >= 440 and self.state == 'record':
            self.z.append(0)

    def calc_prob(self, event):
        op_time = event.current_expected.to_sec()
        self.door_pub.publish(Float64(10))
        if op_time > 6:
            #rospy.loginfo("Start Recording")
            self.state = 'record'
        if op_time > 7:
            rospy.loginfo(self.data)
            samples = np.array(self.z)
            probability = samples.sum()/samples.shape[0]
            rospy.loginfo("Prob: " + str(probability))
    
    def reset_world(self, event):
        reset = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        reset()

def main():
    rospy.init_node('cond_prob')
    rospy.loginfo('starting cond_prob')
    calculation = prob()
    rospy.Timer(rospy.Duration(0.05), calculation.calc_prob)
    rospy.Timer(rospy.Duration(1), calculation.reset_world)
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
