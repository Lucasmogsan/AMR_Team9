#!/usr/bin/env python3

import rospy
from pid_regulator import PIDRegulator
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os
import matplotlib.pyplot as plt
import yaml
import numpy as np
import time
import tf.transformations


class TrackControlNode:
    def __init__(self):
        rospy.init_node('track_control_node', anonymous=True)
        self.desired_distance = 1.0
        self.data_log = []
        self.last_control_signal = 0
        self.last_time = 0.001

        print('TrackControlNode: initializing node')
        self.load_pid_configs()
        self.pid_forward = PIDRegulator(self.pos_pid_config['position_control/pos_p'],
                                        self.pos_pid_config['position_control/pos_i'],
                                        self.pos_pid_config['position_control/pos_d'],
                                        self.pos_pid_config['position_control/pos_sat'])
        self.pid_yaw = PIDRegulator(self.pos_pid_config['position_control/rot_p'],
                                   self.pos_pid_config['position_control/rot_i'],
                                   self.pos_pid_config['position_control/rot_d'],
                                   self.pos_pid_config['position_control/rot_sat'])

        self.position_subscriber_ooi = rospy.Subscriber('/ooi/pose_gt', Odometry, self.ooi_position_callback)
        self.subscriber_bluerov2_gt = rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.bluerov2_position_callback)
        self.subscriber_bluerov2_measured_imu = rospy.Subscriber('/bluerov2/imu', Odometry, self.bluerov2_measured_callback)

        self.velocity_publisher = rospy.Publisher('bluerov2/cmd_vel', Twist, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.update_control)

        self.cameraFOV = 60;
        self.imageWidth = 640;


    def ooi_position_callback(self, msg):
        self.current_ooi_position = msg.pose.pose.position

    def bluerov2_position_callback(self, msg):
        self.gt_bluerov2_position = msg.pose.pose.position
        self.gt_bluerov2_orientation = msg.pose.pose.orientation
    
    def bluerov2_measured_callback(self,msg):
        self.bluerov2_measured_orientation = msg.orientation

    def log_data(self, actual_distance, surge_control_signal, yaw_error, yaw_control_signal):
        error = self.desired_distance - actual_distance
        # Append distance, error, surge control, yaw error, and yaw control signal to the log
        self.data_log.append((actual_distance, error, surge_control_signal, yaw_error, yaw_control_signal))

    def plot_data(self):
        if not self.data_log:
            rospy.logwarn("No data to plot.")
            return

        relative_distances, errors, surge_control_signals, yaw_errors, yaw_control_signals = zip(*self.data_log)
        time_steps = range(len(self.data_log))

        plt.figure(figsize=(15, 10))  # Adjust size for better fit of 2x3 layout

        # Relative Distance Plot
        plt.subplot(2, 3, 1)
        plt.plot(time_steps, relative_distances, label='Relative Distance to OOI',color='blue')
        plt.axhline(y=self.desired_distance, color='r', linestyle='-', label='Desired Distance')
        plt.title('Relative Distance Over Time')
        plt.xlabel('Time Step')
        plt.ylabel('Distance (m)')
        plt.legend()

        # Error in Distance Plot
        plt.subplot(2, 3, 2)
        plt.plot(time_steps, errors, label='Error in Distance')
        plt.title('Error Over Time')
        plt.xlabel('Time Step')
        plt.ylabel('Error (m)')
        plt.legend()

        # Surge Control Signal Plot
        plt.subplot(2, 3, 3)
        plt.plot(time_steps, surge_control_signals, label='Surge Control Signal', color='magenta')
        plt.title('Surge Control Signal Over Time')
        plt.xlabel('Time Step')
        plt.ylabel('Control Signal')
        plt.legend()

        # Yaw Error Plot
        plt.subplot(2, 3, 4)
        plt.plot(time_steps, yaw_errors, label='Yaw Error', color='blue')
        plt.axhline(y=0, color='r', linestyle='-', label='Desired Yaw')
        plt.title('Yaw Error Over Time')
        plt.xlabel('Time Step')
        plt.ylabel('Error (rad)')
        plt.legend()

        # Yaw Control Signal Plot
        plt.subplot(2, 3, 5)
        plt.plot(time_steps, yaw_control_signals, label='Yaw Control Signal', color='magenta')
        plt.title('Yaw Control Signal Over Time')
        plt.xlabel('Time Step')
        plt.ylabel('Control Signal')
        plt.legend()

        plt.subplot(2, 3, 6)
        plt.axis('off') 

        plt.tight_layout()
        plt.savefig('/overlay_ws/src/amr_prj/script/plots/control_plot.png')  
        plt.close()


    def on_shutdown(self):
        self.plot_data()

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

    def calculate_actual_distance(self): 
        # Extract coordinates from ROS Pose messages 
        ooi_x = self.current_ooi_position.x
        ooi_y = self.current_ooi_position.y
        ooi_z = self.current_ooi_position.z
        rov_x = self.current_bluerov2_position.x
        rov_y = self.current_bluerov2_position.y
        rov_z = self.current_bluerov2_position.z
        
        # Calculate the Euclidean distance
        actual_distance = np.sqrt((ooi_x - rov_x)**2 + (ooi_y - rov_y)**2 + (ooi_z - rov_z)**2)
        return actual_distance

    def calculate_yaw_error(self):
        # Obtain the position differences
        dy = self.current_ooi_position.y - self.current_bluerov2_position.y
        dx = self.current_ooi_position.x - self.current_bluerov2_position.x
        
        # Bearing to the OOI from the ROV's current position
        heading_to_ooi = np.arctan2(dy, dx)

    
        yaw_detection = PIXEL_X_COORDINATE * (self.cameraFOV / self.imageWidth);
        
        # Get the ROV's current heading (yaw) from quaternion
        yaw_current = self.get_rov_heading()

        # Calculate yaw error
        yaw_error = (yaw_detection - yaw-current + np.pi) % (2 * np.pi) - np.pi

        return yaw_error


    def update_control(self, event):
        current_time = rospy.get_time()  

        dt = current_time - self.last_time  # Calculate dt
        if dt < 1e-4: # Non zero division
            return
        
        if hasattr(self, 'current_ooi_position') and hasattr(self, 'current_bluerov2_position'):

            # Surge control
            actual_distance = self.calculate_actual_distance()
            setpoint = self.desired_distance
            surge_control_signal = self.pid_forward.update(setpoint, actual_distance, dt)

             # Yaw control
            yaw_error = self.calculate_yaw_error()
            yaw_control_signal = self.pid_yaw.update(0, yaw_error, dt)  # Setpoint is 0 for yaw, we want no yaw error


            # Log the data
            self.log_data(actual_distance, surge_control_signal, yaw_error, yaw_control_signal)
            
            # Create and publish the control message
            control_msg = Twist() # TODO: Understand why control signals are flipped

            # TODO: Implement logic for dependent (coupled) movement
            #control_msg.linear.x = -surge_control_signal
            control_msg.angular.z = -yaw_control_signal  
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
