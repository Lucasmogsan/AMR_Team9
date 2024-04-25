#!/usr/bin/env python

import rospy
import time
import numpy as np
from obstacle_detector.msg import Obstacles, CircleObstacle
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image, CameraInfo
from amr_prj.msg import Circle
from cv_bridge import CvBridge, CvBridgeError
import cv2
from object_detector import CircleDetector

class ImageAnalyser:

    def __init__(self):
        self.bridge = CvBridge()
        self.detector = CircleDetector()

        self.cam_info_received = False
        self.cam_intrinsic_matrix = None
        self.cam_proj_matrix = None
        self.cam_focal_length = None
        self.cam_width = None
        self.cam_height = None
        self.cam_focal_length = None


        self.ooi_real_radius = 0.25          # Real radius of the object of interest in meters

        self.scaling = 0.25                 # Scaling tbd in processing

        # Define sensor name
        robot_name = '/bluerov2'
        camera_name = '/camera_front'
        cam_topic = robot_name + camera_name


        # Subscriber
        sub_topic_name1 = cam_topic + '/camera_image'
        self.sub1 = rospy.Subscriber(sub_topic_name1, Image, self.callback_1, queue_size=10)

        self.sub_cam_info = rospy.Subscriber(cam_topic + '/camera_info', CameraInfo, self.callback_cam_info)

        # Publisher for processed images
        pub_topic_name1 = cam_topic + '/processed_image'
        self.pub1 = rospy.Publisher(pub_topic_name1, Image, queue_size=10)

        # Publisher for target data
        pub_topic_name2 = 'target/cam'
        self.pub2 = rospy.Publisher(pub_topic_name2, Obstacles, queue_size=10)

        # Create a new Obstacle message
        self.obstacle_msg = Obstacles()
        self.circle_msg = CircleObstacle()
        self.circle_msg.center = Point()

    def callback_cam_info(self, msg):
        if self.cam_info_received:
            return
        else:
            self.cam_intrinsic_matrix = msg.K
            self.cam_intrinsic_matrix = np.array(self.cam_intrinsic_matrix).reshape(3, 3)

            self.cam_proj_matrix = msg.P
            self.cam_proj_matrix = np.array(self.cam_proj_matrix).reshape(3, 4)

            self.cam_height = msg.height
            self.cam_width = msg.width

            self.cam_focal_length = self.cam_intrinsic_matrix[0, 0]

            self.cam_info_received = True

    def callback_1(self, msg):
        start_time = time.time()
        #rospy.loginfo("Received an image!")

        # Convert ROS image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image,(int(cv_image.shape[1]*self.scaling), int(cv_image.shape[0]*self.scaling)))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Process the image with the CircleDetector
        detect_start = time.time()
        circle = self.detector.get_circle(cv_image)
        detect_end = time.time()
        rospy.loginfo(f"Circle detection time: {detect_end - detect_start}s")

        if circle is not None and len(circle) == 3:
            u, v, r = (np.array(circle) / self.scaling).astype(int)
            

            if self.cam_info_received:
                # Convert pixel coordinates to real-world coordinates
                img_coord = np.array([u, v, 1])

                # Invert the camera's intrinsic matrix
                intrinsic_matrix_inv = np.linalg.inv(self.cam_intrinsic_matrix)
            
                # Convert image coordinates to normalized camera coordinates
                cam_coord_norm = intrinsic_matrix_inv.dot(img_coord)

                # Estimate distance to the object of interest
                # F = (r x z) / R where F = focal length, R = real radius, z = distance, r = radius measured in pixels
                z = (self.ooi_real_radius * self.cam_focal_length) / r

                # Convert normalized camera coordinates to actual camera coordinates
                cam_coord = cam_coord_norm * z

                # Populate the center point
                self.circle_msg.center.x = z
                self.circle_msg.center.y = -cam_coord[0]
                self.circle_msg.center.z = -cam_coord[1]
                self.circle_msg.true_radius = r
                # Add the CircleObstacle to the Obstacle message (but remove old data first)
                self.obstacle_msg.circles = []
                self.obstacle_msg.circles.append(self.circle_msg)

                self.pub2.publish(self.obstacle_msg)  # Publish circle's data

            # Draw the circle on the image
            cv2.circle(cv_image, (u, v), r, (0, 255, 0), 2)  # Draw the circle boundary in green
            cv2.circle(cv_image, (u, v), 2, (0, 0, 255), 3)  # Draw the center of the circle in red


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
        #rospy.loginfo(f"Total callback duration: {time.time() - start_time}s")

def main():
    rospy.init_node('image_analyser', anonymous=True)
    rospy.loginfo("Image analyser node started")
    ia = ImageAnalyser()
    rospy.spin()

if __name__ == '__main__':
    main()
