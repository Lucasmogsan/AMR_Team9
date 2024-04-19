#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

def reset_state(model_name, new_pose):
    rospy.init_node(f'{model_name}_reset_state', anonymous=True)
    
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = new_pose
        model_state.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))  # Reset velocities to zero
        model_state.reference_frame = 'world'  # Or 'map', depending on your reference frame

        resp = set_state(model_state)
        
        if resp.success:
            rospy.loginfo(f"{model_name} state reset successful: {resp.status_message}")
        else:
            rospy.logwarn(f"Failed to reset {model_name} state: {resp.status_message}")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        # Reset the BlueROV2's position and velocity
        reset_state('bluerov2', Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)))
        
        # Reset the OOI's position and velocity
        reset_state('ooi', Pose(Point(2.0, 0, -23.0), Quaternion(0, 0, 0, 1)))
    except rospy.ROSInterruptException:
        pass
