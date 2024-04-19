#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge
import cv2

import numpy as np

class ImageAnalyser:
    
    def __init__(self):
        self.bridge = CvBridge()

        # Define sensor name
        robot_name = '/bluerov2'
        camera_name = '/camera_front'
        cam_topic = robot_name + camera_name

        # SUBSCRIBER
        sub_topic_name1 = cam_topic + '/camera_image'
        self.sub1 = rospy.Subscriber(sub_topic_name1, Image, self.callback_1)

        # PUBLISHER
        pub_topic_name1 = cam_topic + '/camera_image_new'
        self.pub1 = rospy.Publisher(pub_topic_name1, Image, queue_size=10)


    def object_detector(self, msg):
        shape = (msg.height, msg.width)
        rospy.loginfo(rospy.get_caller_id() + "I got image with shape %s", shape)

        # ROS MSG -> CV2
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo(rospy.get_caller_id() + " I got image with shape %s", img.shape)


        # IMAGE PROCESSING TO BE DONE HERE:

        ...

        
        
        
        # Convert to grayscale
        new_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)







        # CV2 -> ROS MSG
        imgMsg = self.bridge.cv2_to_imgmsg(new_img, "mono8")
        imgMsg.header.frame_id = "camera_front_link_optical"

        angleMsg = 0


        return angleMsg, imgMsg

    # Callback function for the subscriber
    def callback_1(self, msg):
        angle_msg, img_msg = self.object_detector(msg)
        self.pub1.publish(img_msg)  # Publish the message


def main():
    rospy.init_node('image_analyser', anonymous=True)
    rospy.loginfo("image_analyser node started")

    ImageAnalyser()

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()