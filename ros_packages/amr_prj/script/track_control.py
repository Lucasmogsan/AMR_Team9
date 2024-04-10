#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class OOI_Tracker:
    def __init__(self):
        self.current_position = None
        self.current_velocity = None
        self.integral_error = 0.0  # Initialize the integral error

        rospy.init_node('ooi_tracker', anonymous=True)
        self.last_time = rospy.Time.now()  # Keep track of the last time the control loop was executed

        
        self.position_subscriber = rospy.Subscriber('/ooi/pose_gt', Odometry, self.ooi_position_callback)
        self.velocity_publisher = rospy.Publisher('bluerov2/cmd_vel', Twist, queue_size=10)

    def ooi_position_callback(self, data):
        position = data.pose.pose.position
        # rospy.loginfo(f"OOI Position: x: {position.x:.4f}, y: {position.y:.4f}, z: {position.z:.4f}")
        
        # Store the position data for use in the control loop
        self.current_position = position

    def control_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.current_position is not None:
                # Implement your control logic here
                desired_position = 0.5 # Define your desired position
                error = desired_position - self.current_position.x  # Control error for x-axis
                time_step = (current_time - self.last_time).to_sec()  # Calculate time step

                # Integrate the error
                self.integral_error += error * time_step

                control_signal = self.calculate_control_signal(error, self.integral_error)
                
                print(control_signal, error)
                
                # Create and publish the Twist message
                twist_msg = Twist()
                twist_msg.linear.x = -control_signal
                self.velocity_publisher.publish(twist_msg)

                # Update the last time
                self.last_time = current_time
                
            rate.sleep()

    def calculate_control_signal(self, error, integral_error):
        # Controller gains
        Kp = 1.0
        Ki = 10  # Integral gain - this is just a placeholder value, adjust as necessary

        # PI controller
        control_signal = Kp * error + Ki * integral_error
        return control_signal

if __name__ == '__main__':
    try:
        ooi_tracker = OOI_Tracker()
        ooi_tracker.control_loop()
    except rospy.ROSInterruptException:
        pass
