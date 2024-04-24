#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Image
from amr_prj.msg import Circle
from cv_bridge import CvBridge, CvBridgeError
import cv2
from object_detector import CircleDetector

class ImageAnalyser:

    def __init__(self):
        self.bridge = CvBridge()
        self.detector = CircleDetector()

        # Define sensor name
        robot_name = '/bluerov2'
        camera_name = '/camera_front'
        cam_topic = robot_name + camera_name

        # Subscriber
        sub_topic_name1 = cam_topic + '/camera_image'
        self.sub1 = rospy.Subscriber(sub_topic_name1, Image, self.callback_1, queue_size=1)

        # Publisher for processed images
        pub_topic_name1 = cam_topic + '/processed_image'
        self.pub1 = rospy.Publisher(pub_topic_name1, Image, queue_size=1)

        # Publisher for circle data
        pub_topic_name2 = cam_topic + '/ooi_center'
        self.pub2 = rospy.Publisher(pub_topic_name2, Circle, queue_size=1)

    def callback_1(self, msg):
        start_time = time.time()
        rospy.loginfo("Received an image!")

        # Convert ROS image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image,(int(cv_image.shape[1]//4), int(cv_image.shape[0]//4)))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Process the image with the CircleDetector
        detect_start = time.time()
        circle = self.detector.get_circle(cv_image)
        detect_end = time.time()
        rospy.loginfo(f"Circle detection time: {detect_end - detect_start}s")

        if circle is not None and len(circle) == 3:
            x, y, r = circle
            cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)  # Draw the circle boundary in green
            cv2.circle(cv_image, (x, y), 2, (0, 0, 255), 3)  # Draw the center of the circle in red
            circle_msg = Circle(x=x, y=y, radius=r)
            self.pub2.publish(circle_msg)  # Publish circle's data

        # Convert the processed image back to a ROS image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "camera_front_link_optical"

        # Publish the processed image
        self.pub1.publish(img_msg)
        #rospy.loginfo("Published processed image and circle data!")
        #ospy.loginfo(f"Total callback duration: {time.time() - start_time}s")

def main():
    rospy.init_node('image_analyser', anonymous=True)
    rospy.loginfo("Image analyser node started")
    ia = ImageAnalyser()
    rospy.spin()

if __name__ == '__main__':
    main()
