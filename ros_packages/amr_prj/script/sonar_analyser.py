#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

import numpy as np

class SonarAnalyser:
    
    def __init__(self):
        # Define sensor name
        robot_name = '/bluerov2'
        sonar_name = '/sonar_front_front'
        sonar_topic = robot_name + sonar_name

        # SUBSCRIBER
        sub_topic_name1 = sonar_topic
        self.sub1 = rospy.Subscriber(sub_topic_name1, LaserScan, self.callback_1)

        # PUBLISHER
        pub_topic_name1 = sonar_topic + '/obj_distance'
        self.pub1 = rospy.Publisher(pub_topic_name1, Float32, queue_size=10)


    def object_detector(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I got laser scan with amount of data %s", len(msg.ranges))


        # LASERSCAN PROCESSING TO BE DONE HERE:

        # Find the closest object
        min_distance = np.min(msg.ranges)

        # return the distance
        obj_dist = min_distance

        return obj_dist

    # Callback function for the subscriber
    def callback_1(self, msg):
        obj_dist = self.object_detector(msg)
        self.pub1.publish(obj_dist)  # Publish the message


def main():
    rospy.init_node('sonar_analyser', anonymous=True)
    rospy.loginfo("sonar_analyser node started")

    SonarAnalyser()

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()