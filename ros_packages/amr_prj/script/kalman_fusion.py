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

        # For plotting
        self.sonar_pos = []
        self.cam_pos = []
        self.kf_pos = []
        self.kf_cov = []


        # SUBSCRIBERS
        sub_topic_name1 = 'target/sonar'
        self.sub1 = rospy.Subscriber(sub_topic_name1, Obstacles, self.callback_1)
        self.cov_sonar = [[0.1, 0, 0],
                          [0, 0.1, 0],
                          [0, 0, 1000000]]


        sub_topic_name2 = 'target/cam'
        self.sub2 = rospy.Subscriber(sub_topic_name2, Obstacles, self.callback_2)
        self.cov_cam = [[10, 0, 0],
                        [0, 0.5, 0],
                        [0, 0, 0.5]]

        # PUBLISHER
        pub_topic_name1 = 'target/kf'
        self.pub1 = rospy.Publisher(pub_topic_name1, Obstacles, queue_size=10)


        # Kalman Filter
        self.update_rate = 50
        self.f = KalmanFilter(6, 3)
        self.f.x = np.zeros((6, 1))     # State vector: x, y, z, vx, vy, vz
        dt = 1 / self.update_rate       # Time step
        self.f.F = np.asarray(          # State transition matrix
            [[1., 0., 0., dt, 0., 0.],
             [0., 1., 0., 0., dt, 0.],
             [0., 0., 1., 0., 0., dt],
             [0., 0., 0., 1., 0., 0.],
             [0., 0., 0., 0., 1., 0.],
             [0., 0., 0., 0., 0., 1.]]
        )

        self.f.P = np.eye(6) * 0.1      # Covariance matrix
        self.f.H = np.array([           # Measurement function (what is measured from the state vector)
            [1., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [0., 0., 1., 0., 0., 0.]
        ])
        self.f.R = np.eye(3) * 0.1      # Measurement uncertainty / noise (to be updated in the callback functions)
        self.f.Q = np.eye(6) * 0.01     # Process noise


    # Callback function for the subscribers
    def callback_1(self, msg):
        if len(msg.circles) != 0:
            self.flag_sonar = True
        self.sonar_msg = msg

    
    def callback_2(self, msg):
        self.flag_cam = True
        self.cam_msg = msg



def main():
    rospy.init_node('kalman_fusion', anonymous=True)
    rospy.loginfo("kalman_fusion node started")

    kf = KalmanFusion()

    # Create a new Obstacle message
    obstacle_msg = Obstacles()
    # Create a new CircleObstacle
    circle = CircleObstacle()

    counter = 0
    rate = rospy.Rate(kf.update_rate)
    while not rospy.is_shutdown():


        # if new msgs from sonar and/or camera update the filter
        if kf.flag_sonar:
            rospy.loginfo("Sonar update step")
            # Measurement (z and R to be updated in the callback function)
            z_sonar = np.array([kf.sonar_msg.circles[0].center.x, kf.sonar_msg.circles[0].center.y, kf.sonar_msg.circles[0].center.z])
            # Update step
            kf.f.update(z=z_sonar, R=kf.cov_sonar, H=kf.f.H)
            kf.flag_sonar = False

            # # Print the state vector
            # rospy.loginfo(f"Position X: {kf.f.x[0][0]}")
            # rospy.loginfo(f"Position Z: {kf.f.x[2][0]}")
            # # Print the covariance matrix
            # rospy.loginfo(f"Covariance X: {kf.f.P[0][0]}")
            # rospy.loginfo(f"Covariance Z: {kf.f.P[2][2]}")
            # For plotting
            kf.sonar_pos.append(np.array([kf.sonar_msg.header.stamp.to_sec(), kf.sonar_msg.circles[0].center.x, kf.sonar_msg.circles[0].center.y, kf.sonar_msg.circles[0].center.z]))
            kf.kf_pos.append(np.array([kf.sonar_msg.header.stamp.to_sec(), kf.f.x[0][0], kf.f.x[1][0], kf.f.x[2][0]]))
            kf.kf_cov.append(np.array([kf.sonar_msg.header.stamp.to_sec(), kf.f.P[0][0], kf.f.P[1][1], kf.f.P[2][2]]))


        if kf.flag_cam:
            rospy.loginfo("Camera update step")
            # Measurement (z and R to be updated in the callback function)
            z_cam = np.array([kf.cam_msg.circles[0].center.x, kf.cam_msg.circles[0].center.y, kf.cam_msg.circles[0].center.z])
            # Update step
            kf.f.update(z=z_cam, R=kf.cov_cam, H=kf.f.H)
            kf.flag_cam = False

            # # Print the state vector
            # rospy.loginfo(f"Position X: {kf.f.x[0][0]}")
            # rospy.loginfo(f"Position Z: {kf.f.x[2][0]}")
            # # Print the covariance matrix
            # rospy.loginfo(f"Covariance X: {kf.f.P[0][0]}")
            # rospy.loginfo(f"Covariance Z: {kf.f.P[2][2]}")
            # For plotting
            kf.cam_pos.append(np.array([kf.cam_msg.header.stamp.to_sec(), kf.cam_msg.circles[0].center.x, kf.cam_msg.circles[0].center.y, kf.cam_msg.circles[0].center.z]))
            kf.kf_pos.append(np.array([kf.cam_msg.header.stamp.to_sec(), kf.f.x[0][0], kf.f.x[1][0], kf.f.x[2][0]]))
            kf.kf_cov.append(np.array([kf.cam_msg.header.stamp.to_sec(), kf.f.P[0][0], kf.f.P[1][1], kf.f.P[2][2]]))

        # only predict if state vector is not zeroes
        if np.any(kf.f.x):
            # Prediction step
            kf.f.predict(F=kf.f.F, Q=kf.f.Q)    
            rospy.loginfo("Prediction step")


            # Remove old circle data
            obstacle_msg.circles = []

            # Add header
            obstacle_msg.header = Header()
            obstacle_msg.header.stamp = rospy.Time.now()
            obstacle_msg.header.frame_id = "camera_front_link_optical"

            # Populate the velocity
            circle.velocity = Vector3()
            circle.velocity.x = kf.f.x[3][0]
            circle.velocity.y = kf.f.x[4][0]
            circle.velocity.z = kf.f.x[5][0]

            # Populate the center point
            circle.center = Point()
            circle.center.x = kf.f.x[0][0]  # Replace with your x coordinate
            circle.center.y = kf.f.x[1][0]  # Replace with your y coordinate
            circle.center.z = kf.f.x[2][0]  # Replace with your z coordinate

            # Add the CircleObstacle to the Obstacle message
            obstacle_msg.circles.append(circle)
            
            # Publish the message
            kf.pub1.publish(obstacle_msg)

            # Print the state vector
            # rospy.loginfo(f"Position X: {kf.f.x[0][0]}")
            # rospy.loginfo(f"Position Z: {kf.f.x[2][0]}")
            # Print the covariance matrix
            # rospy.loginfo(f"Covariance X: {kf.f.P[0][0]}")
            # rospy.loginfo(f"Covariance Z: {kf.f.P[2][2]}")
            # For plotting
            kf.kf_pos.append(np.array([obstacle_msg.header.stamp.to_sec(), kf.f.x[0][0], kf.f.x[1][0], kf.f.x[2][0]]))
            kf.kf_cov.append(np.array([obstacle_msg.header.stamp.to_sec(), kf.f.P[0][0], kf.f.P[1][1], kf.f.P[2][2]]))

            # Save the data every 10 seconds
            counter += 1
            if counter % kf.update_rate * 10 == 0:
                path_save = '/overlay_ws/src/amr_prj/script/plots/'
                np.save(path_save + 'sonar_pos.npy', np.array(kf.sonar_pos))
                np.save(path_save + 'cam_pos.npy', np.array(kf.cam_pos))
                np.save(path_save + 'kf_pos.npy', np.array(kf.kf_pos))
                np.save(path_save + 'kf_cov.npy', np.array(kf.kf_cov))
            
        rate.sleep()


if __name__ == '__main__':
    main()