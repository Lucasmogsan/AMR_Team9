#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np

class IMUSubscriber:
    def __init__(self):
        rospy.init_node('imu_analysis', anonymous=True)
        self.imu_sub = rospy.Subscriber('/bluerov2/imu', Imu, self.imu_callback)
        self.angular_velocity_std_dev = [0, 0, 0]
        self.linear_acceleration_std_dev = [0, 0, 0]

    def imu_callback(self, msg):
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        
        # Extract standard deviation of angular velocity
        self.angular_velocity_std_dev[0] = np.std([angular_velocity.x])
        self.angular_velocity_std_dev[1] = np.std([angular_velocity.y])
        self.angular_velocity_std_dev[2] = np.std([angular_velocity.z])
        
        # Extract standard deviation of linear acceleration
        self.linear_acceleration_std_dev[0] = np.std([linear_acceleration.x])
        self.linear_acceleration_std_dev[1] = np.std([linear_acceleration.y])
        self.linear_acceleration_std_dev[2] = np.std([linear_acceleration.z])
        
        # Print the standard deviations
        print("Angular Velocity Standard Deviation: ", self.angular_velocity_std_dev)
        print("Linear Acceleration Standard Deviation: ", self.linear_acceleration_std_dev)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    imu_subscriber = IMUSubscriber()
    imu_subscriber.run()
