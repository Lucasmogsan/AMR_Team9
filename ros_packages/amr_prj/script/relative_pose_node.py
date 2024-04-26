#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf.transformations as tf_trans

import numpy as np

class SonarAnalyser:
    
    def __init__(self):

        self.position_subscriber_ooi = rospy.Subscriber('/ooi/pose_gt', Odometry, self.ooi_position_callback)
        self.subscriber_bluerov2_gt = rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.bluerov2_position_callback)

        self.rov_position = None
        self.rov_orientation = None
        self.cam_rel_pos_x = 0.2

        self.counter = 0

        self.pos_array = []


    def ooi_position_callback(self, msg):
        #rospy.loginfo(rospy.get_caller_id() + "I got ooi position %s", msg.pose.pose.position)

        self.counter += 1

        if self.counter % 10 != 0 and msg.pose.pose.position.x is not None and self.rov_position is not None:
            # Get the object's position and orientation
            object_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            object_orientation = msg.pose.pose.orientation
            object_orientation = np.array([object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w])

            # Get the ROV's position and orientation
            rov_position = np.array([self.rov_position.x, self.rov_position.y, self.rov_position.z])
            rov_orientation = self.rov_orientation
            rov_orientation = np.array([rov_orientation.x, rov_orientation.y, rov_orientation.z, rov_orientation.w])

            # Convert the ROV's orientation from quaternion to rotation matrix
            rov_rot_matrix = tf_trans.quaternion_matrix(rov_orientation)

            # Convert the object's orientation from quaternion to rotation matrix
            object_rot_matrix = tf_trans.quaternion_matrix(object_orientation)

            # Convert the object's position to the ROV's frame
            relative_position = np.dot(rov_rot_matrix[0:3, 0:3], object_position - rov_position)

            # Convert the object's orientation to the ROV's frame
            relative_orientation = np.dot(rov_rot_matrix[0:3, 0:3], object_rot_matrix[0:3, 0:3])

            #rospy.loginfo("Relative position: %s", relative_position)

            # Add position to array
            self.pos_array.append(np.array([msg.header.stamp.to_sec(), relative_position[0]-self.cam_rel_pos_x, relative_position[1], relative_position[2]]))

            # Save the data
            path_save = '/overlay_ws/src/amr_prj/script/plots/'
            np.save(path_save + 'true_pos.npy', np.array(self.pos_array))


    def bluerov2_position_callback(self, msg):
        #rospy.loginfo(rospy.get_caller_id() + "I got bluerov2 position %s", msg.pose.pose.position)

        self.rov_position = msg.pose.pose.position
        self.rov_orientation = msg.pose.pose.orientation



def main():
    rospy.init_node('pose_analyser', anonymous=True)
    rospy.loginfo("pose_analyser node started")

    SonarAnalyser()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()


