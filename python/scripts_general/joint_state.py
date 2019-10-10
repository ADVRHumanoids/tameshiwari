#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def posePublisher():
    pub = rospy.Publisher('pose_state', JointState, queue_size=10)
    rospy.init_node('posePublisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        state_str = JointState()
        state_str.header = Header()
        now = rospy.get_rostime()
        state_str.header.stamp.secs = now.secs
        state_str.header.stamp.nsecs = now.nsecs
        state_str.name = ['J00', 'J01']
        state_str.position = [0.5, 0.2]
        state_str.velocity = []
        state_str.effort = []
        rospy.loginfo(state_str)
        pub.publish(state_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        posePublisher()
    except rospy.ROSInterruptException:
        pass
