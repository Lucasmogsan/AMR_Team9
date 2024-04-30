#!/usr/bin/env python3

import rospy
from pid_regulator import PIDRegulator
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os
from obstacle_detector.msg import Obstacles, CircleObstacle
import matplotlib.pyplot as plt
import yaml
import numpy as np
import time
import tf.transformations
import time
import csv

class TrackControlNode:
    def __init__(self):
        rospy.init_node('track_control_node', anonymous=True)
        self.desired_distance = 1.0
        self.last_control_signal = 0
        self.last_time = 0.001

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"control_data_{timestamp}.npy"
        self.data_log = []
        self.base_path = "/overlay_ws/src/amr_prj/script/plots/data"
        self.data_file_path = os.path.join(self.base_path, filename)

        self.last_time = rospy.get_time()
        print('TrackControlNode: initializing node')
        
        self.yaw_ooi_gt = 0 # In radians
        self.yaw_bluerov_gt = 0
        self.load_pid_configs()
        # PID controller for surge motion control
        self.pid_surge = PIDRegulator(
            self.pos_pid_config['position_control/surge_p'],   # Proportional gain for surge
            self.pos_pid_config['position_control/surge_i'],   # Integral gain for surge
            self.pos_pid_config['position_control/surge_d'],   # Derivative gain for surge
            self.pos_pid_config['position_control/surge_sat']  # Saturation limit for surge control
        )

        # PID controller for yaw rotational control
        self.pid_yaw = PIDRegulator(
            self.pos_pid_config['position_control/yaw_p'],   # Proportional gain for yaw
            self.pos_pid_config['position_control/yaw_i'],   # Integral gain for yaw
            self.pos_pid_config['position_control/yaw_d'],   # Derivative gain for yaw
            self.pos_pid_config['position_control/yaw_sat']  # Saturation limit for yaw control
        )

        # PID controller for heave motion control
        self.pid_heave = PIDRegulator(
            self.pos_pid_config['position_control/heave_p'],   # Proportional gain for heave
            self.pos_pid_config['position_control/heave_i'],   # Integral gain for heave
            self.pos_pid_config['position_control/heave_d'],   # Derivative gain for heave
            self.pos_pid_config['position_control/heave_sat']  # Saturation limit for heave control
        )


        self.position_subscriber_ooi = rospy.Subscriber('/ooi/pose_gt', Odometry, self.ooi_position_callback_gt)
        self.subscriber_bluerov2_gt = rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.bluerov2_position_callback_gt)
        self.subscriber_bluerov2_measured_imu = rospy.Subscriber('/bluerov2/imu', Odometry, self.bluerov2_position_callback_kf)
        self.position_subscriber_ooi_kf = rospy.Subscriber('/target/kf', Obstacles, self.ooi_position_callback_kf)
        
        self.velocity_publisher = rospy.Publisher('bluerov2/cmd_vel', Twist, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.update_control)

        self.cameraFOV = 60;
        self.imageWidth = 640;

    def ooi_position_callback_gt(self, msg):
        self.current_ooi_position_gt = msg.pose.pose.position
        self.current_ooi_orientation_gt = msg.pose.pose.orientation

    def bluerov2_position_callback_gt(self, msg):
        self.gt_bluerov2_position = msg.pose.pose.position
        self.gt_bluerov2_orientation = msg.pose.pose.orientation

    def ooi_position_callback_kf(self, msg):
        self.current_ooi_position_kf = msg
        
    def bluerov2_position_callback_kf(self,msg):
        self.bluerov2_measured_orientation = msg.orientation
        

    def log_data(self, surge, surge_gt, surge_control, yaw, yaw_gt, yaw_control, heave, heave_gt, heave_control):
        time_stamp = rospy.Time.now()
        self.data_log.append((time_stamp, surge, surge_gt, surge_control, yaw, yaw_gt, yaw_control, heave, heave_gt, heave_control))

    def on_shutdown(self):
        # Save remaining data to numpy file before shutdown
        np.save(self.data_file_path, np.array(self.data_log))


    def get_rov_heading(self):
        # Assuming self.current_bluerov2_orientation is set to a geometry_msgs/Quaternion
        quaternion = (
            self.bluerov2_measured_orientation.x,
            self.bluerov2_measured_orientation.y,
            self.bluerov2_measured_orientation.z,
            self.bluerov2_measured_orientation.w
        )
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # euler is a tuple (roll, pitch, yaw), yaw is what we need
        yaw = euler[2]  # In radians
        return yaw

    def load_pid_configs(self):
        pos_pid_config_path = '/overlay_ws/src/amr_prj/config/pos_pid_control.yaml'
        if os.path.isfile(pos_pid_config_path):
            with open(pos_pid_config_path, 'r') as file:
                self.pos_pid_config = yaml.load(file, Loader=yaml.FullLoader)
        else:
            rospy.logerr('Position PID config file not found')

    def calculate_yaw_error(self):
        
        quaternion_bluerov2_gt = (
            self.gt_bluerov2_orientation.x,
            self.gt_bluerov2_orientation.y,
            self.gt_bluerov2_orientation.z,
            self.gt_bluerov2_orientation.w
        )
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = tf.transformations.euler_from_quaternion(quaternion_bluerov2_gt)
        # euler is a tuple (roll, pitch, yaw), yaw is what we need
        self.yaw_bluerov_gt = euler[2]  # In radians
        
        quaternion_ooi_gt = (
            self.current_ooi_orientation_gt.x,
            self.current_ooi_orientation_gt.y,
            self.current_ooi_orientation_gt.z,
            self.current_ooi_orientation_gt.w
        )
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = tf.transformations.euler_from_quaternion(quaternion_ooi_gt)
        # euler is a tuple (roll, pitch, yaw), yaw is what we need
        self.yaw_ooi_gt = euler[2]  # In radians

        dy = self.current_ooi_position_gt.y
        dx = self.current_ooi_position_gt.x
        self.yaw_ooi_gt = np.arctan2(dy, dx)

        
        
        # Kalman filter
        dy = self.current_ooi_position_kf.circles[0].center.y 
        dx = self.current_ooi_position_kf.circles[0].center.x
        yaw_kf = np.arctan2(dy, dx)
        return yaw_kf

    def update_control(self, event):
        current_time = rospy.get_time()  

        dt = current_time - self.last_time  # Calculate dt
        if dt < 1e-4: # Non zero division
            return
        
        

        if hasattr(self, 'current_ooi_position_kf'):
            # Surge control
            surge_error = self.current_ooi_position_kf.circles[0].center.x # Relative frame | For global frame: -self.current_bluerov2_position.x
            setpoint = self.desired_distance
            surge_control_signal = self.pid_surge.update(setpoint, surge_error, dt)

            # Yaw control
            yaw_error = self.calculate_yaw_error()
            yaw_control_signal = self.pid_yaw.update(0, yaw_error, dt)  # Setpoint is 0 for yaw, we want no yaw error

            # Heave Control
            heave_error = self.current_ooi_position_kf.circles[0].center.z-0.022020-0.007058
            heave_control_signal = self.pid_heave.update(0, heave_error, dt)  # Setpoint is 0 for yaw, we want no yaw error
            
            # Log the data | [surge, ]
            surge_gt = (self.current_ooi_position_gt.x-self.gt_bluerov2_position.x)
            yaw_gt = self.yaw_ooi_gt - self.yaw_bluerov_gt 
            heave_gt = (self.current_ooi_position_gt.z-self.gt_bluerov2_position.z -0.022020-0.007058)
            print(f"{yaw_error:1f}, {yaw_gt:1f}")
            self.log_data(surge_error, surge_gt, surge_control_signal,
                          yaw_error, yaw_gt, yaw_control_signal,
                          heave_error, heave_gt, heave_control_signal)
            
            # Create and publish the control message
            control_msg = Twist() # TODO: Understand why control signals are flipped

            # TODO: Implement logic for dependent (coupled) movement
            #control_msg.linear.x = -surge_control_signal
            control_msg.angular.z = -yaw_control_signal
            control_msg.linear.z =  -heave_control_signal  
            self.velocity_publisher.publish(control_msg)
            
            # Update last_time for the next cycle
            self.last_time = current_time
        else:
            rospy.logwarn("Waiting for initial position updates from OOI and BlueROV2.")

if __name__ == '__main__':
    try:
        node = TrackControlNode()
        rospy.on_shutdown(node.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
