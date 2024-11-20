#!/usr/bin/env python

import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float64MultiArray
from opencv_apps.msg import Point2DArrayStamped
import numpy as np

class CorrespMatcher:
    def __init__(self):
        self.color = rospy.get_param('~color', 'red')

        self.act_feature_sub = Subscriber('/goodfeature_'+self.color+'/corners', Point2DArrayStamped)
        self.pred_feature_sub = Subscriber('/'+self.color+'/pred_feature', Float64MultiArray)
        self.corresp_pub = rospy.Publisher('/matched_corresp', Float64MultiArray, queue_size = 1)

        self.sync = ApproximateTimeSynchronizer([self.act_feature_sub, self.pred_feature_sub], queue_size = 10, slop = 0.1, allow_headerless = True)
        self.sync.registerCallback(self.match_features)

    def match_features(self, act_feature_msg, pred_feature_msg):
        act_pixels = []
        for point in act_feature_msg.points:
            act_pixels.append((point.x, point.y))
        act_pixels = np.array(act_pixels)

        pred_pixels = np.array(pred_feature_msg.data).reshape(-1, 2)

        distances = []
        for point in pred_pixels:
            min_dist = float('inf')

            for feature in act_pixels:
                dist = np.linalg.norm(feature - point)
                if dist < min_dist:
                    min_dist = dist
          
            distances.append(min_dist)

        corresp_msg = Float64MultiArray()
        corresp_msg.data = distances
        self.corresp_pub.publish(corresp_msg)


def main():
    rospy.init_node('corresp_matcher')
    rospy.loginfo('starting corresp_matcher')
    CorrespMatcher()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
