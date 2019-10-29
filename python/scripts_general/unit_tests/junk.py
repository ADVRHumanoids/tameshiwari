import time, sys, os
from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

TOPIC = 'camera/image'

bag = rosbag.Bag('test12345.bag', 'w')
cap = cv2.VideoCapture('/home/user/catkin_ws/src/tameshiwari/python/OCP_pendulum_2dof_rr/camera_image.jpeg')
cb = CvBridge()
prop_fps = 1
ret = True
frame_id = 0
while(ret):
    ret, frame = cap.read()
    if not ret:
        break
    stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
    frame_id += 1
    image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
    image.header.stamp = stamp
    image.header.frame_id = "camera"
    bag.write(TOPIC, image, stamp)
cap.release()
bag.close()