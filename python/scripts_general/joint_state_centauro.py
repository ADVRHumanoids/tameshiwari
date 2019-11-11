#!/usr/bin/env python

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia
#
#   This script can publish joint states to the robot_state_publisher running in Rviz.
#   With this script it is possible to playback generated data/trajectories from python directly to Rviz.
#

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

def posePublisher(pose):
    pub = rospy.Publisher('pose_state', JointState, queue_size=10)
    rospy.init_node('posePublisher', anonymous=True)
    rate = rospy.Rate(pose.rate) # default is 10 Hz
    state_str = JointState()
    state_str.header = Header()
    iteration = 0
    
    while not rospy.is_shutdown() and iteration < pose.q.shape[0]:
        now = rospy.get_rostime()
        state_str.header.stamp.secs = now.secs
        state_str.header.stamp.nsecs = now.nsecs
        state_str.name = pose.name + ['torso_yaw', 'neck_pitch', 'j_arm1_1', 'j_arm1_2', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'j_arm1_7', 'j_arm2_2', 'j_arm2_3', 'j_arm2_5', 'j_arm2_6', 'j_arm2_7', 'hip_yaw_1', 'hip_pitch_1', 'knee_pitch_1', 'ankle_yaw_1', 'hip_yaw_2', 'hip_pitch_2', 'knee_pitch_2', 'ankle_yaw_2', 'hip_yaw_3', 'hip_pitch_3', 'knee_pitch_3', 'ankle_yaw_3', 'hip_yaw_4', 'hip_pitch_4', 'knee_pitch_4', 'ankle_yaw_4','j_wheel_1','j_wheel_2','j_wheel_3','j_wheel_4','ankle_pitch_1','ankle_pitch_2','ankle_pitch_3','ankle_pitch_4']
        # print state_str.name
        state_str.position = pose.q[iteration,:]
        # print np.shape(state_str.position)
        rest = np.array([0.0, 0.45, 0.5, -0.3, -0.3, -2.2, 0.0, -0.8, 0.0, -0.3, -0.3, -0.0, -0.8, -0.0, 0.0, -1.0, -1.0, -0.78, 0.0, 1.0, 1.0, 0.78, 0.0, 1.0, 1.0, -0.78, 0.0, -1.0, -1.0, 0.78, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        state_str.position = np.concatenate((state_str.position,rest))
        zerovec = np.zeros(len(state_str.name)-2)
        if np.shape(pose.qdot)[0] > 1:
            state_str.velocity = pose.qdot[iteration,:]
            state_str.velocity = np.concatenate((state_str.velocity,zerovec))
        if np.shape(pose.tau)[0] > 1:
            state_str.effort = pose.tau[iteration,:]
            state_str.effort = np.concatenate((state_str.effort,zerovec))
        # rospy.loginfo(state_str)            # use for debugging
        pub.publish(state_str)
        iteration += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        posePublisher(pose)
    except rospy.ROSInterruptException:
        pass
