#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
import random

def ooi_simple_move_publisher():
    """
    Moving target front and back with a given period and speed
    """
    print("----------------ooi movement initialized--------------")
    rospy.init_node('simple_movement', anonymous=True)

    # Get parameters from ROS parameter server
    period = rospy.get_param('~period', 20.0)
    speed = rospy.get_param('~speed', 0)

    pub = rospy.Publisher('ooi/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        twist_msg = Twist()
        twist_msg.linear.x = speed  # Initial linear velocity

        # Publish initial velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < period/2:
            pub.publish(twist_msg)
            rate.sleep()

        # Change linear velocity to -speed
        twist_msg.linear.x = -speed

        # Publish negative velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < period/2:
            pub.publish(twist_msg)
            rate.sleep()

def ooi_circle_move_publisher():
    """
    Moving target in a circle with a given period and speed
    """
    rospy.init_node('circle_movement', anonymous=True)

    # Get parameters from ROS parameter server
    period = rospy.get_param('~period', 20.0)
    speed = rospy.get_param('~speed', 0.15)
    angular_speed = rospy.get_param('~angular_speed', 0.1)

    pub = rospy.Publisher('ooi/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    twist_msg = Twist()

    while not rospy.is_shutdown():
        twist_msg.linear.y = speed  # Constant linear velocity
        twist_msg.angular.z = angular_speed  # Constant angular velocity

        # Publish velocity
        pub.publish(twist_msg)
        rate.sleep()


def ooi_random_move_publisher():
    """
    Moving target in random directions
    """
    rospy.init_node('random_movement', anonymous=True)

    # Get parameters from ROS parameter server
    max_speed = rospy.get_param('~max_speed', 0.15)
    max_angular_speed = rospy.get_param('~max_angular_speed', 0.1)

    pub = rospy.Publisher('ooi/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    twist_msg = Twist()
    counter = 0

    while not rospy.is_shutdown():
        if counter % 10 == 0:
            # Random linear velocities in x, y, z directions
            twist_msg.linear.x = random.uniform(-max_speed, max_speed)
            twist_msg.linear.y = random.uniform(-max_speed, max_speed)
            twist_msg.linear.z = random.uniform(-max_speed, max_speed)

            # Random angular velocities in x, y, z directions
            twist_msg.angular.x = random.uniform(-max_angular_speed, max_angular_speed)
            twist_msg.angular.y = random.uniform(-max_angular_speed, max_angular_speed)
            twist_msg.angular.z = random.uniform(-max_angular_speed, max_angular_speed)

        # Publish velocity
        pub.publish(twist_msg)
        rate.sleep()

        counter += 1


if __name__ == '__main__':
    try:
        # Get mode parameter from ROS parameter server
        #target_move_mode = rospy.get_param('/move_target/target_move_mode', '0')
        target_move_mode = '2'
        # Call the appropriate function based on the mode parameter
        if target_move_mode == '0':
            ooi_simple_move_publisher()
        elif target_move_mode == '1':
            ooi_circle_move_publisher()
        elif target_move_mode == '2':
            ooi_random_move_publisher()
        else:
            rospy.logerr('Invalid mode parameter. Valid options are "simple", "circle", or "random".')
    except rospy.ROSInterruptException:
        pass