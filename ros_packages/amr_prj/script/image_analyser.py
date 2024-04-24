#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from amr_prj.msg import Circle  # Import your custom message
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
        self.sub1 = rospy.Subscriber(sub_topic_name1, Image, self.callback_1)

        # Publisher for processed images
        pub_topic_name1 = cam_topic + '/processed_image'
        self.pub1 = rospy.Publisher(pub_topic_name1, Image, queue_size=10)

        # Publisher for circle data
        pub_topic_name2 = cam_topic + '/ooi_center'
        self.pub2 = rospy.Publisher(pub_topic_name2, Circle, queue_size=10)

    def callback_1(self, msg):
        rospy.loginfo("Received an image!")
        
        # Convert ROS image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # # Process the image with the CircleDetector
        # max_circle = self.detector.recognize_OOI(cv_image)
        # if max_circle is not None:
        #     tracked_circle = detector.get_circle()
        #     detector.track_OOI(max_circle,frame)
        # rospy.loginfo(circles)
        # for (x, y, r) in circles:
        #     cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)
        #     cv2.circle(cv_image, (x, y), 2, (0, 0, 255), 3)
        #     circle_msg = Circle(x=x, y=y, radius=r)
        #     self.pub2.publish(circle_msg)  # Publish each circle's data

        # Process the image with the CircleDetector

        circle = self.detector.get_circle(cv_image)
        if circle is not None:
            rospy.loginfo(circle)
            x,y,r = circle
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
        rospy.loginfo("Published processed image and circle data!")

def main():
    rospy.init_node('image_analyser', anonymous=True)
    rospy.loginfo("Image analyser node started")
    ia = ImageAnalyser()
    rospy.spin()

if __name__ == '__main__':
    main()
