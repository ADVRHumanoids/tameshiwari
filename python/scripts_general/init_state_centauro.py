#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

def homing(q=None):
    rospy.init_node('posePublisher', anonymous=True)
    pub = rospy.Publisher('pose_state', JointState, queue_size=10)
    rate = rospy.Rate(1) # default is 10 Hz
    state_str = JointState()
    state_str.header = Header()
    now = rospy.get_rostime()
    state_str.header.stamp.secs = now.secs
    state_str.header.stamp.nsecs = now.nsecs
    state_str.name = ['torso_yaw', 'neck_pitch', 'j_arm1_1', 'j_arm1_2', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'j_arm1_7', 'j_arm2_1', 'j_arm2_2', 'j_arm2_3', 'j_arm2_4', 'j_arm2_5', 'j_arm2_6', 'j_arm2_7', 'hip_yaw_1', 'hip_pitch_1', 'knee_pitch_1', 'ankle_yaw_1', 'hip_yaw_2', 'hip_pitch_2', 'knee_pitch_2', 'ankle_yaw_2', 'hip_yaw_3', 'hip_pitch_3', 'knee_pitch_3', 'ankle_yaw_3', 'hip_yaw_4', 'hip_pitch_4', 'knee_pitch_4', 'ankle_yaw_4']
    state_str.name += ['j_wheel_1','j_wheel_2','j_wheel_3','j_wheel_4','ankle_pitch_1','ankle_pitch_2','ankle_pitch_3','ankle_pitch_4']
    if not q:
        state_str.position = [0.0, 0.45, 0.5, -0.3, -0.3, -2.2, 0.0, -0.8, 0.0, 0.5, -0.3, -0.3, -2.2, -0.0, -0.8, -0.0, 0.0, -1.0, -1.0, -0.78, 0.0, 1.0, 1.0, 0.78, 0.0, 1.0, 1.0, -0.78, 0.0, -1.0, -1.0, 0.78]
        state_str.position += [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    else:
        state_str.position = q
    ter_publish = False

    while not ter_publish:
        connections = pub.get_num_connections()
        # print "number of connections to this publisher: %s" %connections
        if connections > 0:
            rospy.loginfo(state_str)            # use for debugging
            pub.publish(state_str)
            ter_publish = True
            # print "Message Published. Closing Program..."
        else:
            rate.sleep()

if __name__ == '__main__':
    try:
        homing()
    except rospy.ROSInterruptException:
        pass