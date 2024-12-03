#!/usr/bin/env python

import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float64MultiArray
from opencv_apps.msg import Point2DArrayStamped
import numpy as np
from copy import copy
import rospkg

class CorrespMatcher:
    def __init__(self, color):
        self.color = color

        self.act_feature_sub = Subscriber('/goodfeature_'+self.color+'/corners', Point2DArrayStamped)
        self.pred_feature_sub = Subscriber('/pred_feature/'+self.color, Float64MultiArray)
        self.corresp_pub = rospy.Publisher('/matched_corresp/'+self.color, Float64MultiArray, queue_size = 1)
        self.mean_pub = rospy.Publisher('/error_mean/'+self.color, Float64MultiArray, queue_size = 1)
        self.var_pub = rospy.Publisher('/error_var/'+self.color, Float64MultiArray, queue_size = 1)

        self.sync = ApproximateTimeSynchronizer([self.act_feature_sub, self.pred_feature_sub], queue_size = 1, slop = 0.1, allow_headerless = True)
        self.sync.registerCallback(self.match_features)
        self.error_list = []
        
        # Find the path of prob_rob_labs package so that the script can be run on any computer
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('prob_rob_labs')
        error_file_path = package_path+"/datalog/"+self.color+".txt"
        self.error_file = open(error_file_path, 'w')
        #rospy.loginfo(file_path)
        self.data_count = 0

    def match_features(self, act_feature_msg, pred_feature_msg):
        
        # Exit if less than 4 points
        if len(act_feature_msg.points) < 4:
            #rospy.loginfo("exit")
            return
                
        act_pixels = []
        for point in act_feature_msg.points:
            act_pixels.append((point.x, point.y))
        act_pixels = np.array(act_pixels)

        pred_pixels = np.array(pred_feature_msg.data).reshape(-1, 2)
        
        act_pixels = act_pixels.tolist()
        pred_pixels = pred_pixels.tolist()
        
        distances = []
        ordered_error = copy(pred_pixels)
        for i in range(len(act_pixels)):
            min_dist = float('inf')
            
            for point in pred_pixels:
                dist = np.linalg.norm(np.array(act_pixels[i]) - np.array(point))
                if dist < min_dist:
                    min_dist = dist
                    ordered_error[i] = np.array(act_pixels[i]) - np.array(point)
                    
          
            distances.append(min_dist)

        ordered_error = np.array(ordered_error).flatten().tolist()
        
        error_list_max_len = 500
        if len(self.error_list) < error_list_max_len:
            #rospy.loginfo(f"{self.color}: {len(self.error_list)}")
            self.error_list.append(ordered_error)
        
        else:
            self.error_list.append(ordered_error)
            self.error_list = self.error_list[-error_list_max_len:]
            error_data = np.array(self.error_list)
            mean = np.mean(error_data, axis=0)
            var = np.var(error_data, axis=0)
            mean_pub_data = mean.flatten().tolist()
            var_pub_data = var.flatten().tolist()
            mean_msg = Float64MultiArray()
            mean_msg.data = mean_pub_data
            var_msg = Float64MultiArray()
            var_msg.data = var_pub_data
            self.mean_pub.publish(mean_msg)
            self.var_pub.publish(var_msg)
            
        
        corresp_msg = Float64MultiArray()
        corresp_msg.data = distances
        self.corresp_pub.publish(corresp_msg)
        
        
        self.error_file.write(",".join(map(str, ordered_error)) + "\n")
        self.data_count += 1
        min_num_data = 1000
        if self.data_count == min_num_data:
            rospy.loginfo(f"{min_num_data} data points recorded for {self.color}")

def main():
    rospy.init_node('corresp_matcher')
    rospy.loginfo('starting corresp_matcher')
    landmarks = rospy.get_param("landmark")
    Matchers = [CorrespMatcher(landmark["color"]) for landmark in landmarks]
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
