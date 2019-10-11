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

# def posePublisher(name=[],q=[],qdot=[],tau=[]):
#     pub = rospy.Publisher('pose_state', JointState, queue_size=10)
#     rospy.init_node('posePublisher', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
    
#     while not rospy.is_shutdown():
#         state_str = JointState()
#         state_str.header = Header()
#         now = rospy.get_rostime()
#         state_str.header.stamp.secs = now.secs
#         state_str.header.stamp.nsecs = now.nsecs
#         state_str.name = name
#         state_str.position = q
#         state_str.velocity = qdot
#         state_str.effort = tau
#         rospy.loginfo(state_str)
#         pub.publish(state_str)
#         rate.sleep()

def posePublisher(pose):
    pub = rospy.Publisher('pose_state', JointState, queue_size=10)
    rospy.init_node('posePublisher', anonymous=True)
    rate = rospy.Rate(pose.rate) # default is 10 Hz
    state_str = JointState()
    state_str.header = Header()
    iter = 0
    
    while not rospy.is_shutdown() and iter < pose.q.shape[0]:
        now = rospy.get_rostime()
        state_str.header.stamp.secs = now.secs
        state_str.header.stamp.nsecs = now.nsecs
        state_str.name = pose.name
        state_str.position = pose.q[iter,:]
        state_str.velocity = pose.qdot
        state_str.effort = pose.tau
        rospy.loginfo(state_str)
        pub.publish(state_str)
        iter += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        posePublisher(pose)
    except rospy.ROSInterruptException:
        pass
