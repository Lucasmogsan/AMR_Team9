#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class PoseGetter:
    def __init__(self):
        rospy.init_node('get_pose', anonymous=True)
        self.model_states_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.blueROV2_pose = Pose()
        self.ooi_pose = Pose()

    def callback(self, data):
        try:
            blueROV2_index = data.name.index('bluerov2')  # Replace with your actual BlueROV2 model name
            self.blueROV2_pose = data.pose[blueROV2_index]
            rospy.loginfo("BlueROV2 Position: %s", self.blueROV2_pose.position)

            ooi_index = data.name.index('ooi')  # Replace with your actual OOI model name
            self.ooi_pose = data.pose[ooi_index]
            rospy.loginfo("OOI Position: %s", self.ooi_pose.position)

        except ValueError as e:
            rospy.logerr("Model not found: %s", e)

    def run(self):
        rate = rospy.Rate(0.1)  # 10 Hz control loop
        rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    pose_getter = PoseGetter()
    pose_getter.run()
