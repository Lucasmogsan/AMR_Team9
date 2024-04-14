#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import signal
import sys
import time

class OOI_Tracker:
    def __init__(self):
        self.position_ooi = None
        self.position_bluerov2 = None
        self.desired_distance = 1
        self.data_log = []
        self.control_signal_log = []  # Store control signals for plotting
        self.last_control_signal = 0.0  # Initialize the last control signal



        self.integral_error = 0.0
        self.last_time = None

        # PI controller gains
        self.Kp = 2.0  # Proportional gain
        self.Ki = 0  # Integral gain

        rospy.init_node('ooi_tracker', anonymous=True)
        self.position_subscriber_ooi = rospy.Subscriber('/ooi/pose_gt', Odometry, self.ooi_position_callback)
        self.position_subscriber_bluerov2 = rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.bluerov2_position_callback)
        self.velocity_publisher = rospy.Publisher('bluerov2/cmd_vel', Twist, queue_size=10)

        # Register a signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

    def ooi_position_callback(self, data):
        self.position_ooi = data.pose.pose.position.x

    def bluerov2_position_callback(self, data):
        self.position_bluerov2 = data.pose.pose.position.x
        if self.position_ooi is not None:
            self.log_data(self.last_control_signal)

    def log_data(self, last_control_signal):
        actual_distance = self.position_ooi - self.position_bluerov2
        error = self.desired_distance - actual_distance
        self.data_log.append((self.position_bluerov2, error, self.last_control_signal))

    def plot_data(self):
        # Unpack relative distances, errors, and control signal data
        relative_distances = [ self.position_ooi - data[0] for data in self.data_log]
        errors = [data[1] for data in self.data_log]
        control_signals = [data[2] for data in self.data_log]
        time_steps = range(len(self.data_log))

        plt.figure(figsize=(10, 5))

        # Plot relative distance over time
        plt.subplot(1, 2, 1)
        plt.plot(time_steps, relative_distances, label='Relative Distance to OOI')
        plt.axhline(y=self.desired_distance, color='r', linestyle='-', label='Desired Distance')
        plt.title('Relative Distance Over Time')
        plt.xlabel('Time Step')
        plt.ylabel('Distance (x-axis)')
        plt.legend()

        # Plot control signal over time
        plt.subplot(1, 2, 2)
        plt.plot(time_steps, control_signals, label='Control Signal', color='magenta')
        plt.title('Control Signal Over Time')
        plt.xlabel('Time Step')
        plt.ylabel('Control Signal Value')
        plt.legend()

        plt.tight_layout()
        plt.savefig('/overlay_ws/src/amr_prj/script/plots/combined_plot.png')  # Save the plot with a new name
        plt.close()




    def control_loop(self):
        rate = rospy.Rate(10)  # 10 Hz control loop
        while not rospy.is_shutdown():
            if self.position_ooi is not None and self.position_bluerov2 is not None:
                self.update_control()
            rate.sleep()
    
    def update_control(self):
        # Calculate the error between current position and desired position
        actual_distance = self.position_ooi - self.position_bluerov2
        error = self.desired_distance - actual_distance

        # Calculate the time elapsed
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return  # Skip the rest of the loop on the first pass
        dt = current_time - self.last_time

        # Update the integral of the error
        self.integral_error += error * dt

        # Calculate the control signal (PI controller)
        self.last_control_signal = -(self.Kp * error + self.Ki * self.integral_error)

        # Log data for plotting control signal
        self.log_data(self.last_control_signal)

        # Create and publish the Twist message to command the BlueROV2
        twist_msg = Twist()
        twist_msg.linear.x = self.last_control_signal
        self.velocity_publisher.publish(twist_msg)

        # Save the current time for the next loop iteration
        self.last_time = current_time

    def run(self):
        self.control_loop()  # Start the control loop
        self.plot_data()  # This line will only execute after the control loop is terminated


    def signal_handler(self, signum, frame):
        print('Signal handler called with signal', signum)
        self.plot_data()
        sys.exit(0)

if __name__ == '__main__':
    tracker = OOI_Tracker()
    tracker.run()
