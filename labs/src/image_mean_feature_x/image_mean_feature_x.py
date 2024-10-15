#!/usr/bin/env python

import rospy
import threading
import numpy

from std_msgs.msg import Float64
from opencv_apps.msg import Point2DArrayStamped

class ProcessFeatures():
    def __init__(self):
        self.feature_sub = rospy.Subscriber(
            '/goodfeature_track/corners', Point2DArrayStamped,
            self.handle_features, queue_size = 1)
        self.mean_pub = rospy.Publisher(
            'feature_mean', Float64, queue_size = 1)
        self.diff_pub = rospy.Publisher(
            'feature_diff', Float64, queue_size = 1)
        self.lock = threading.Lock()

    def handle_features(self, f):
        with self.lock:
            x_features = [ p.x for p in f.points ]
            mean_x = numpy.mean(x_features)
            self.mean_pub.publish(mean_x)
            diff_x = numpy.max(x_features) - numpy.min(x_features)
            self.diff_pub.publish(diff_x)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('image_mean_feature_x')
    rospy.loginfo('starting image_mean_feature_x')
    pf = ProcessFeatures()
    pf.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
