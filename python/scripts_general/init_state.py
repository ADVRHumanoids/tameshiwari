#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

def homing(q=None,name=None):
    rospy.init_node('posePublisher', anonymous=True)
    pub = rospy.Publisher('pose_state', JointState, queue_size=10)
    rate = rospy.Rate(1) # default is 10 Hz
    state_str = JointState()
    state_str.header = Header()
    now = rospy.get_rostime()
    state_str.header.stamp.secs = now.secs
    state_str.header.stamp.nsecs = now.nsecs
    if not name:
        state_str.name = ["J01","J02"]
    else:
        state_str.name = name
    if not q:
        state_str.position = [0,0]
    else:
        state_str.position = q
    ter_publish = False

    while not ter_publish:
        connections = pub.get_num_connections()
        # print "number of connections to this publisher: %s" %connections
        if connections > 0:
            # rospy.loginfo(state_str)            # use for debugging
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