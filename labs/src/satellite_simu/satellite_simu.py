#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
import numpy as np
import threading
from prob_rob_labs.msg import SatelliteState

class Satellite:
    def __init__(self, num, c, vel_range=(300000, 500000), grid_size=5, cell_size=10):
        """Initialize a satellite centered around (0, 0)."""
        self.num = num
        self.c = c
        
        # Center grid around (0,0)
        range_min = -grid_size * cell_size // 2
        range_max = grid_size * cell_size // 2
        
        self.x = random.randint(range_min, range_max)
        self.y = random.randint(range_min, range_max)
        
        if num % 2 == 0:  # Even satellites move in the 'x' direction
            self.dir = 'x'
            self.z = random.randint(20200000, 20200100)
        else:  # Odd satellites move in the 'y' direction
            self.dir = 'y'
            self.z = random.randint(20200200, 20200300)
        
        # Random velocity within the range
        self.vel = random.randint(*vel_range)
        
        # global contains all state msgs, local only contains the msgs that the robot can receive
        self.satellite_state_pub_local = rospy.Publisher('/Satellite_'+str(self.num)+"_local", SatelliteState, queue_size=1)
        self.satellite_state_pub_global = rospy.Publisher('/Satellite_'+str(self.num)+"_global", SatelliteState, queue_size=1)
        
    def move(self, grid_size=5, cell_size=20000000):
        """Move the satellite in the specified direction with wrap-around behavior."""
        range_min = -grid_size * cell_size // 2
        range_max = grid_size * cell_size // 2
        
        if self.dir == 'x':
            self.x += self.vel
            if self.x > range_max:  # Wrap around to the left edge
                self.x = range_min
        elif self.dir == 'y':
            self.y += self.vel
            if self.y > range_max:  # Wrap around to the bottom edge
                self.y = range_min

    def broadcast(self, robot): # robot_coord is an np array [robot_x, robot_y, robot_z]
        planar_dist = np.linalg.norm(np.array([self.x, self.y]) - np.array([robot.x, robot.y]))
        spatial_dist = np.linalg.norm(np.array([self.x, self.y, self.z]) - np.array([robot.x, robot.y, robot.z]))
        dt = spatial_dist / self.c
        now = rospy.get_rostime()
        # rospy.loginfo(f"{self.num}: {now.to_sec()}, {travel_time}")
        sat_state = SatelliteState(
            header = Header(
                stamp = now,
                frame_id = 'map'
            ),
            x = self.x, y = self.y, z = self.z,
            travel_time = dt
        )
        
        # only publish to global topics
        if planar_dist > 30000000:
            self.satellite_state_pub_global.publish(sat_state)
        
        elif planar_dist <= 30000000:
            self.satellite_state_pub_global.publish(sat_state)
            self.satellite_state_pub_local.publish(sat_state)
            
        
        
class robot:
    def __init__(self):
        self.gt_sub = rospy.Subscriber('/jackal/ground_truth/pose', PoseStamped, self.update_pose, queue_size=1)
        self.x = 0
        self.y = 0
        self.z = 0
    
    def update_pose(self, gt_pose_msg):
        self.x = gt_pose_msg.pose.position.x
        self.y = gt_pose_msg.pose.position.y
        self.z = gt_pose_msg.pose.position.y
        
def broadcast_satellite_signal(instances, robot, lock):
    def timer_callback(event):
        with lock:
            for satellite in instances:
                satellite.move()
                satellite.broadcast(robot)
    return timer_callback
    

def main():
    # Initialize the ROS node
    rospy.init_node('satellite_simu')
    rospy.loginfo('starting satellite_simu')
    lock = threading.Lock()
    # Create 10 satellite instances
    c = 3e8 # c is the actual speed of light, remains constant for each trial of simulation
    satellite_count = rospy.get_param("~satellite_count", 10)
    satellite_instances = [Satellite(i, c) for i in range(1,satellite_count+1)]
    
    # use the robot() class to store and update the ground truth location of the robot
    rob = robot()
    
    # The Timer moves the satellites and thenbroadcasts the signals at 1Hz
    timer_period = 1.0
    rospy.Timer(rospy.Duration(timer_period), broadcast_satellite_signal(satellite_instances, rob, lock))

    rospy.spin()


if __name__ == "__main__":
    main()
