#!/usr/bin/env python

import numpy
import rospy

from visualization_msgs.msg import Marker, MarkerArray

def spawn_sphere(diameter=1):
    rospy.init_node('workspace_shapes', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        t = rospy.Time.now()        
        
        marker_array = MarkerArray()
        markerp = Marker()
        # markerp.pose.position.x = 0.2
        # markerp.pose.position.y = 0
        # markerp.pose.position.z = 1.19045

        markerp.pose.position.x = 0.
        markerp.pose.position.y = 0.
        markerp.pose.position.z = 0.

        markerp.color.r = 0.
        markerp.color.g = 0.
        markerp.color.b = 1.
        markerp.color.a = 0.5

        markerp.scale.x = diameter
        markerp.scale.y = diameter
        markerp.scale.z = diameter

        markerp.type = markerp.SPHERE

        markerp.header.frame_id = "torso_2"
        markerp.header.stamp = t

        markerp.id = 0
        marker_array.markers.append(markerp)

        pubm = rospy.Publisher("marker_array_viz", MarkerArray, queue_size=1)
        pubm.publish(marker_array) 
        
        rate.sleep()


if __name__ == '__main__':
    try:
        spawn_sphere(0.75)
    except rospy.ROSInterruptException:
        pass