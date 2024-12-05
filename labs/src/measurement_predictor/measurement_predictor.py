#!/usr/bin/env python

import rospy
from sympy import symbols, Matrix, sin, cos, atan2, simplify, lambdify
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion
import tf2_ros
from opencv_apps.msg import Point2DArrayStamped
import numpy as np
from copy import copy
from message_filters import ApproximateTimeSynchronizer, Subscriber

def init_functions():
    x, y, theta, t_cx, t_cy, t_cz, x_l, y_l, r_l, h_l, f_x, f_y, c_x, c_y \
        = symbols("x y theta t_cx t_cy t_cz x_l y_l r_l h_l f_x f_y c_x c_y")
        
    x_c = cos(theta)*t_cx - sin(theta)*t_cy + x
    y_c = sin(theta)*t_cx + cos(theta)*t_cy + y
    phi = atan2(y_l - y_c, x_l - x_c)

    x1 = x_l - r_l*sin(phi)
    y1 = y_l + r_l*cos(phi)
    x2 = x_l + r_l*sin(phi)
    y2 = y_l - r_l*cos(phi)

    P_1g = Matrix([[x1, y1, 0, 1]]).T
    P_2g = Matrix([[x2, y2, 0, 1]]).T
    P_3g = Matrix([[x2, y2, h_l, 1]]).T
    P_4g = Matrix([[x1, y1, h_l, 1]]).T
    P_ig = [P_1g, P_2g, P_3g, P_4g]

    T_mr = Matrix([[cos(theta), -sin(theta), 0, x],
                   [sin(theta),  cos(theta), 0, y],
                   [0,           0,          1, 0],
                   [0,           0,          0, 1]    
                  ])

    T_ro = Matrix([[ 0,  0, 1, t_cx],
                   [-1,  0, 0, t_cy],
                   [ 0, -1, 0, t_cz+0.05],
                   [ 0,  0, 0, 1]    
                  ])

    T_mo = T_mr * T_ro
    # print(T_mo.inv())
    T_om = simplify(T_mo.inv())
    # print(T_om)

    P_io = []
    for element in P_ig:
        P_io.append(T_om * element)

    Proj_mat = Matrix([[f_x, 0,   c_x],
                       [0,   f_y, c_y],
                       [0,   0,   1]
                      ])

    auxiliary_vec = []
    for element in P_io:
        auxiliary_vec.append(Proj_mat * element[:3, :])

    P_ip_list = []
    for element in auxiliary_vec:
        P_ip_list.append(element[:2, :]/element[2,0])

    P_ip = Matrix([[P_ip_list[0][0,0]],
                [P_ip_list[0][1,0]],
                [P_ip_list[1][0,0]],
                [P_ip_list[1][1,0]],
                [P_ip_list[2][0,0]],
                [P_ip_list[2][1,0]],
                [P_ip_list[3][0,0]],
                [P_ip_list[3][1,0]],
    ])

    states = Matrix([x, y, theta])
    Hx = P_ip.jacobian(states)
    Hx_func = lambdify((x, y, theta, t_cx, t_cy, t_cz, x_l, y_l, r_l, h_l, f_x, f_y, c_x, c_y), Hx)
    P_ip_func = lambdify((x, y, theta, t_cx, t_cy, t_cz, x_l, y_l, r_l, h_l, f_x, f_y, c_x, c_y), P_ip)
    return Hx_func, P_ip_func

class MeasurementModel:
    def __init__(self, color, x, y, r, h):     
        # Initiate the parameters
        self.color = color
        self.x = 0; self.y = 0; self.theta = 0 # State vector
        self.f_x = 0; self.f_y = 0; self.c_x = 0; self.c_y = 0 # Camera projection matrix
        self.t_cx = 0; self.t_cy = 0; self.t_cz = 0; # Robot frame to camera frame translation
        self.x_l = x
        self.y_l = y
        self.r_l = r
        self.h_l = h
        self.CameraInfo_Retrieved = 0

        # Initiate functions
        self.Hx, self.P_ip = init_functions()

        # Declare the subscribers and publishers
        self.gt_pose = Subscriber('/jackal/ground_truth/pose', PoseStamped)
        self.CameraInfo_sub = rospy.Subscriber('/front/left/camera_info', CameraInfo, self.update_CameraInfo)
        #self.mmt_pred_pub = rospy.Publisher('/pred_feature/'+self.color, Float64MultiArray, queue_size = 1)
        self.act_feature_sub = Subscriber('/goodfeature_'+self.color+'/corners', Point2DArrayStamped)

        self.sync = ApproximateTimeSynchronizer([self.gt_pose, self.act_feature_sub], queue_size = 1, slop = 0.1, allow_headerless = True)
        self.sync.registerCallback(self.measurement)

        # Create tf listener and initiate transform parameters
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        trans = self.tfBuffer.lookup_transform('base_link', 'front_camera_optical', rospy.Time.now(), rospy.Duration(1.0))
        self.t_cx = trans.transform.translation.x
        self.t_cy = trans.transform.translation.y
        self.t_cz = trans.transform.translation.z

    def update_CameraInfo(self, data):
        self.f_x = data.P[0]
        self.c_x = data.P[2]
        self.f_y = data.P[5]
        self.c_y = data.P[6]
        
        self.image_width = data.width
        self.image_height = data.height
        self.CameraInfo_Retrieved = 1
    
    def jacobian(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        __, __, self.theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, 
                                                    data.pose.orientation.z, data.pose.orientation.w])

        Hx = self.Hx(self.x, self.y, self.theta,
                             self.t_cx, self.t_cy, self.t_cz, 
                             self.x_l, self.y_l, self.r_l, self.h_l,
                             self.f_x, self.f_y, self.c_x, self.c_y)
        return Hx
    
    def measurement(self, data, act_feature_msg):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        __, __, self.theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, 
                                                    data.pose.orientation.z, data.pose.orientation.w])
        
        # rospy.loginfo(self.y)
        Hx = self.jacobian(data)
        #rospy.loginfo(f"Hx: {Hx}")

        if self.CameraInfo_Retrieved == 1:
            P_ip = self.P_ip(self.x, self.y, self.theta,
                             self.t_cx, self.t_cy, self.t_cz, 
                             self.x_l, self.y_l, self.r_l, self.h_l,
                             self.f_x, self.f_y, self.c_x, self.c_y)
            #rospy.loginfo(f"P_ip: {P_ip}")
            valid_features = []
            valid_features_flag = 0
            for i in range(0, 8, 2):
                x_pixel = P_ip[i]
                y_pixel = P_ip[i+1]
                if x_pixel >=0  and x_pixel <= self.image_width and y_pixel >= 0 and y_pixel <= self.image_height: 
                    valid_features.append((x_pixel, y_pixel))
                    valid_features_flag += 1

            if valid_features_flag >= 4:
                #msg = Float64MultiArray()
                #msg.data = [point for feature in valid_features for point in feature]
                #self.mmt_pred_pub.publish(msg)
	        # Exit if less than 4 points
                if len(act_feature_msg.points) < 4:
                    #rospy.loginfo("exit")
                    return

                act_pixels = []
                for point in act_feature_msg.points:
                    act_pixels.append((point.x, point.y))
                act_pixels = np.array(act_pixels)
                #rospy.loginfo(f"Actual feature pixels: {act_pixels}")

                pred_pixels = np.array(valid_features).reshape(-1, 2)
                #rospy.loginfo(f"Predicted feature pixels: {pred_pixels}")

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

                #rospy.loginfo(f"Ordered error: {ordered_error}") 
                ordered_error = np.array(ordered_error).flatten().tolist()

        variances = [
	    24.3121, 0.2612, 22.4337, 0.2081,
	    25.3811, 2.4115, 25.4867, 17.9422
	]

        measurement_covariance = np.zeros((8,8))
        np.fill_diagonal(measurement_covariance, variances)
        #rospy.loginfo(f"Measurement covariance: {measurement_covariance}")

        return act_pixels, pred_pixels, ordered_error, measurement_covariance, Hx

def main():
    rospy.init_node('measurement_predictor')
    rospy.loginfo('starting measurement_predictor')
    landmarks = rospy.get_param("landmark")
    predictors = [MeasurementModel(**landmark) for landmark in landmarks]
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
