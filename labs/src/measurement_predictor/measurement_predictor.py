#!/usr/bin/env python

import rospy
from sympy import symbols, Matrix, sin, cos, atan2, simplify, lambdify, zeros, eye
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from opencv_apps.msg import Point2DArrayStamped
import numpy as np
from copy import deepcopy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
import threading
from collections import deque
from std_msgs.msg import Header

global_lock = threading.Lock()

# Define the global variables for prediction x_bar, sigma_bar and time_stamp
x_bar = 0; sigma_bar = 0; odom_queue = deque(maxlen=10); time_stamp = 0; odom_Retrieved = 0

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
                   [ 0, -1, 0, t_cz],
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
        global x_bar, sigma_bar, time_stamp, odom_queue, odom_Retrieved
        # Creating a lock
        self.lock = threading.Lock()
        
        # Initiate the parameters
        self.color = color
        rospy.loginfo(f"instance created for: {self.color}")
        self.x = 0; self.y = 0; self.theta = 0 # State vector
        self.f_x = 0; self.f_y = 0; self.c_x = 0; self.c_y = 0 # Camera projection matrix
        self.t_cx = 0; self.t_cy = 0; self.t_cz = 0; # Robot frame to camera frame translation
        self.x_l = x
        self.y_l = y
        self.r_l = r
        self.h_l = h
        self.CameraInfo_Retrieved = 0
        self.odom = Odometry()
        
        # Initiate functions
        self.Hx, self.P_ip = init_functions()

        # Declare the subscribers and publishers
        self.CameraInfo_sub = rospy.Subscriber('/front/left/camera_info', CameraInfo, self.update_CameraInfo)
        while(self.CameraInfo_Retrieved == 0): pass # Wait for CameraInfo
        self.ekf_pub = rospy.Publisher('/ekf_pose', PoseWithCovarianceStamped, queue_size = 1)
        self.act_feature_sub = rospy.Subscriber('/goodfeature_'+self.color+'/corners', Point2DArrayStamped, self.processor, queue_size=1)

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
    
    def prediction(self, odom, corner_msg):
        global x_bar, sigma_bar, time_stamp, odom_Retrieved
        if odom_Retrieved == 1:
            
            dt = corner_msg.header.stamp.to_sec() - time_stamp.to_sec()
            # rospy.loginfo(f"feature_{self.color}: {corner_msg.header.stamp.to_sec()}, time_stamp: {time_stamp.to_sec()} gap:{dt}")
            # Update predictions if dt is not 0
            if dt != 0:
                x = odom.pose.pose.position.x
                y = odom.pose.pose.position.y
                __, __, theta = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, 
                                                       odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
                omega = odom.twist.twist.angular.z
                vel = odom.twist.twist.linear.x
                # rospy.loginfo(f"t-1: {[x, y, theta]}")
                
                G_t = Matrix([[1, 0, -vel/omega*cos(theta)+vel/omega*cos(theta+dt*omega)], 
                              [0, 1, -vel/omega*sin(theta)+vel/omega*sin(theta+dt*omega)],
                              [0, 0, 1]
                            ])
                
                V_t_0_0 = (-sin(theta)+sin(theta+omega*dt))/omega
                V_t_0_1 = vel*(sin(theta)-sin(theta+omega*dt)/omega**2) + \
                          vel*cos(theta+omega*dt)*dt/omega
                V_t_1_0 = (cos(theta)-cos(theta+omega*dt))/omega
                V_t_1_1 = -vel*(cos(theta)-cos(theta+omega*dt)/omega**2) + \
                          vel*sin(theta+omega*dt)*dt/omega
                V_t = Matrix([[V_t_0_0, V_t_0_1],
                              [V_t_1_0, V_t_1_1],
                              [0,       dt]
                            ])
                
                M_t = Matrix([[0.01*vel**2+0.01*omega**2, 0                        ],
                              [0,                         0.01*vel**2+0.01*omega**2]
                            ])
                
                x_bar = Matrix([[x],[y],[theta]]) + \
                        Matrix([[-vel/omega*sin(theta) + vel/omega*sin(theta+omega*dt)],
                                [vel/omega*cos(theta) - vel/omega*cos(theta+omega*dt)],
                                [omega*dt]
                              ])
                        
                sigma_t_1_full = Matrix(6, 6, odom.pose.covariance)
                sigma_t_1 = sigma_t_1_full.extract([0, 1, 5], [0, 1, 5])
                
                sigma_bar = G_t*sigma_t_1*G_t.T + V_t*M_t*V_t.T
                # rospy.loginfo(f"sigma_bar: {sigma_bar}")
                # rospy.loginfo(f"t: {x_bar.tolist()}")
            time_stamp = corner_msg.header.stamp

    
    def measurement(self, act_feature_msg):
        self.x = float(x_bar[0,0])
        self.y = float(x_bar[1,0])
        self.theta = float(x_bar[2,0])
        #rospy.loginfo(f"t-1: {[self.x, self.y, self.theta]}")
        if self.CameraInfo_Retrieved == 1:
            P_ip = self.P_ip(self.x, self.y, self.theta,
                             self.t_cx, self.t_cy, self.t_cz, 
                             self.x_l, self.y_l, self.r_l, self.h_l,
                             self.f_x, self.f_y, self.c_x, self.c_y)
            
            # rospy.loginfo(f"{self.color}_pred: {P_ip}")
            # rospy.loginfo(f"{self.color}_actual: {act_feature_msg.points}")
            Hx = self.Hx(self.x, self.y, self.theta,
                         self.t_cx, self.t_cy, self.t_cz, 
                         self.x_l, self.y_l, self.r_l, self.h_l,
                         self.f_x, self.f_y, self.c_x, self.c_y)
            
            # Check if all features are within the image frame
            valid_features = []
            valid_features_flag = 0
            for i in range(0, 8, 2):
                x_pixel = P_ip[i]
                y_pixel = P_ip[i+1]
                valid_features.append([x_pixel, y_pixel])
                if x_pixel >=0  and x_pixel <= self.image_width and y_pixel >= 0 and y_pixel <= self.image_height: 
                    valid_features_flag += 1
            pred_pixels = np.array(valid_features).reshape(-1, 2)
            pred_pixels = pred_pixels.tolist()
            
            # Correspondance Matching
            act_pixels = []
            for point in act_feature_msg.points:
                act_pixels.append([point.x, point.y])
            if len(act_pixels) < 4 or valid_features_flag < 4:
                act_pixels = deepcopy(pred_pixels)  
            
            #rospy.loginfo(f"pred{pred_pixels}")
            #rospy.loginfo(f"act{act_pixels}")
            
            distances = []
            ordered_error = deepcopy(pred_pixels)
            variances = [[24.3121, 0.2612], 
                         [22.4337, 0.2081],
                         [25.3811, 2.4115], 
                         [25.4867, 17.9422]
                        ]
            ordered_variance = deepcopy(variances)
            ordered_pred_pixels = deepcopy(pred_pixels)
            Hx_list = Hx.tolist()
            ordered_Hx = deepcopy(Hx_list)
            for i in range(len(act_pixels)):
                min_dist = float('inf')

                for point in pred_pixels:
                    dist = np.linalg.norm(np.array(act_pixels[i]) - np.array(point))
                    if dist < min_dist:
                        min_dist = dist
                        ordered_error[i] = np.array(act_pixels[i]) - np.array(point)
                        ordered_pred_pixels[i] = pred_pixels[i]
                        ordered_variance[i] = variances[i]
                        ordered_Hx[i] = Hx_list[i]
                distances.append(min_dist)

            ordered_error = np.array(ordered_error).flatten().tolist()
            # rospy.loginfo(f"Ordered error {len(act_feature_msg.points)}|{self.color}|: {ordered_error}")
            

        measurement_covariance = zeros(8,8)
        for i, variance in enumerate(ordered_variance):
            measurement_covariance[i, i] = variance
        #rospy.loginfo(f"Measurement covariance: {measurement_covariance}")

        return act_pixels, ordered_pred_pixels, ordered_error, measurement_covariance, ordered_Hx

    def processor(self, act_feature_msg):
        global x_bar, sigma_bar, time_stamp, odom_queue, odom_Retrieved
        with global_lock:
            measurement_time = act_feature_msg.header.stamp.to_sec()
            odom = None
            
            # If the message is too old, ignore measurement
            if measurement_time < time_stamp.to_sec():
                return
            
            # Find the odom message just before the measurement signal
            for i in range(len(odom_queue)):
                if measurement_time > odom_queue[-i].header.stamp.to_sec():
                    odom = odom_queue[-i]
                    break
            
            # If all odom messages are older than the measurement, ignore measurement
            if not odom:
                return
            
            # Perform prediction
            self.prediction(odom, act_feature_msg)
            
            # Perform measurement prediction, ordered_error is a 1D 1x8 list, measurement_covariance is a 8x8 Matrix, ordered_Hx is a 2D 8x3 list
            # actual_pixels is a 2D 4x2 list, ordered_pred_pixels is a 2D 4x2 list
            act_pixels, ordered_pred_pixels, ordered_error, measurement_covariance, ordered_Hx = self.measurement(act_feature_msg)

            # Calculate Kalman Gain
            Hx = Matrix(ordered_Hx)
            partial_kalman = Hx*sigma_bar*Hx.T + measurement_covariance
            Kalman = sigma_bar*Hx.T*partial_kalman.inv()
            
            # Innovation
            error = Matrix(ordered_error)
            x_new = x_bar + Kalman*error
            Identity = eye(3)
            sigma_new = (Identity - Kalman*Hx)*sigma_bar
            
            x_bar = x_new; sigma_bar = sigma_new

            # Publish to /ekf_pose
            # Construct the 6x6 covariance message
            cov_6x6 = zeros(6)
            cov_6x6[0:2, 0:2] = sigma_new[0:2, 0:2]  # x, y terms
            cov_6x6[0:2, 5] = sigma_new[0:2, 2]      # Cov(x/y, yaw)
            cov_6x6[5, 0:2] = sigma_new[2, 0:2]      # Cov(yaw, x/y)
            cov_6x6[5, 5] = sigma_new[2, 2]          # yaw term
            covariance_list = np.array(cov_6x6).flatten().tolist()
            
            # Create and populate the PoseWithCovariance message
            pose_msg = PoseWithCovariance()
            pose_msg.pose.position.x = float(x_new[0])
            pose_msg.pose.position.y = float(x_new[1])
            quaternion = quaternion_from_euler(0, 0, float(x_new[2]))  # roll, pitch, yaw
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            pose_msg.covariance = covariance_list
            
            PoseWithCovarianceStamped_msg = PoseWithCovarianceStamped(
                                header = Header(
	                            stamp = time_stamp,
	                            frame_id = 'ekf'
                                                ),
                                pose = pose_msg
                            )
            
            self.ekf_pub.publish(PoseWithCovarianceStamped_msg)
            
        
def odom_callback(odom_msg):
    global x_bar, sigma_bar, time_stamp, odom_queue, odom_Retrieved
    with global_lock:
        odom_queue.append(odom_msg)
        if odom_Retrieved == 0:
            time_stamp = odom_msg.header.stamp
        # rospy.loginfo(odom_msg.header.stamp.to_sec())
        odom_Retrieved = 1


def main():
    global x_bar, sigma_bar, time_stamp, odom_queue, odom_Retrieved
    rospy.init_node('measurement_predictor')
    rospy.loginfo('starting measurement_predictor')
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    while(odom_Retrieved == 0): pass # Wait for the first odom message
    with global_lock:
        landmarks = rospy.get_param("landmark")
        predictors = [MeasurementModel(**landmark) for landmark in landmarks]
    rospy.loginfo(f"first odom msg time stamp: {time_stamp.to_sec()}")
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
