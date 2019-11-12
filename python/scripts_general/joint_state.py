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
    try:
        nj = np.shape(pose.q)[1]
        print "number of joints: %s" %nj
    except:
        print "error on q"
    
    while not rospy.is_shutdown() and iteration < pose.q.shape[0]:
        now = rospy.get_rostime()
        state_str.header.stamp.secs = now.secs
        state_str.header.stamp.nsecs = now.nsecs
        state_str.name = pose.name
        state_str.position = pose.q[iteration,:]
        if np.shape(pose.qdot)[0] > 1:
            state_str.velocity = pose.qdot[iteration,:]
        if np.shape(pose.tau)[0] > 1:
            state_str.effort = pose.tau[iteration,:]
        # rospy.loginfo(state_str)            # use for debugging
        pub.publish(state_str)
        iteration += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        posePublisher(pose)
    except rospy.ROSInterruptException:
        pass
