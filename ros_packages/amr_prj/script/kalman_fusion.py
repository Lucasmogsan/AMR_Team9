#!/usr/bin/env python

import rospy
import numpy as np

from filterpy.kalman import KalmanFilter

from obstacle_detector.msg import Obstacles, CircleObstacle
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header

class KalmanFusion:
    
    def __init__(self):

        # Flags
        self.flag_sonar = False
        self.flag_cam = False

        self.sonar_msg = None
        self.cam_msg = None


        # SUBSCRIBERS
        sub_topic_name1 = 'target/sonar'
        self.sub1 = rospy.Subscriber(sub_topic_name1, Obstacles, self.callback_1)
        self.cov_sonar = [[0.1, 0, 0],
                          [0, 0.1, 0],
                          [0, 0, 100]]


        sub_topic_name2 = 'target/cam'
        self.sub1 = rospy.Subscriber(sub_topic_name2, Obstacles, self.callback_2)
        self.cov_cam = [[0.1, 0, 0],
                        [0, 0.1, 0],
                        [0, 0, 10]]

        # PUBLISHER
        pub_topic_name1 = 'target/kf'
        self.pub1 = rospy.Publisher(pub_topic_name1, Obstacles, queue_size=10)



        # Kalman Filter
        self.f = KalmanFilter(6, 3)
        self.f.x = np.zeros((6, 1))     # State vector: x, y, z, vx, vy, vz
        self.f.F = np.asarray(          # State transition matrix
            [
                [1., 0., 0., 1., 0., 0.],
                [0., 1., 0., 0., 1., 0.],
                [0., 0., 1., 0., 0., 1.],
                [0., 0., 0., 1., 0., 0.],
                [0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 1.]
            ]
        )
        self.f.H = np.array([           # Measurement function
            [1., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [0., 0., 1., 0., 0., 0.]
        ])

        self.f.P = np.eye(6) * 0.1      # Covariance matrix
        self.f.R = np.eye(3) * 0.1      # Measurement noise (to be updated in the callback functions)
        self.f.Q = np.eye(6) * 0.01     # Process noise


    # Callback function for the subscribers
    def callback_1(self, msg):
        if len(msg.circles) == 0:
            self.flag_sonar = True
        self.sonar_msg = msg

    
    def callback_2(self, msg):
        self.flag_cam = True
        self.cam_msg = msg



def main():
    rospy.init_node('image_analyser', anonymous=True)
    rospy.loginfo("image_analyser node started")

    kf = KalmanFusion()

    # Create a new Obstacle message
    obstacle_msg = Obstacles()
    # Create a new CircleObstacle
    circle = CircleObstacle()

    rate = rospy.Rate(50)  # 50 Hz
    while not rospy.is_shutdown():

        # if new msgs from sonar and/or camera update the filter
        if kf.flag_sonar:
            z_sonar = np.array([kf.sonar_msg.circles[0].center.x, kf.sonar_msg.circles[0].center.y, kf.sonar_msg.circles[0].center.z])
            kf.f.update(z_sonar, R=kf.cov_sonar)
            rospy.loginfo("Sonar update step")
            kf.flag_sonar = False


        if kf.flag_cam:
            z_cam = np.array([kf.cam_msg.circles[0].center.x, kf.cam_msg.circles[0].center.y, kf.cam_msg.circles[0].center.z])
            kf.f.update(z_cam, R=kf.cov_cam)
            rospy.loginfo("Camera update step")
            kf.flag_cam = False


        kf.f.predict()    # Prediction step
        rospy.loginfo("Prediction step")


        # Populate the center point
        circle.center = Point()
        circle.center.x = kf.f.x[0][0]  # Replace with your x coordinate
        circle.center.y = kf.f.x[1][0]  # Replace with your y coordinate
        circle.center.z = kf.f.x[2][0]  # Replace with your z coordinate

        # Add the CircleObstacle to the Obstacle message
        obstacle_msg.circles.append(circle)
        
        # Publish the message
        kf.pub1.publish(obstacle_msg)
        
        rate.sleep()


if __name__ == '__main__':
    main()