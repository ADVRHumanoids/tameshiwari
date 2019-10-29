import time, sys, os
from ros import rosbag
import roslib
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('rosbag')
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError

bag_r = rosbag.Bag('test_image.bag','r')
bridge = CvBridge()
# topic, msg = bag_r.read_messages(topics='camera/image')
# print msg
print bag_r.get_message_count()
count = 0
# for topic, msg, t in bag_r.read_messages(topics='camera/image'):
for topic, msg, t in bag_r.read_messages(topics='camera1/image/compressed'):
    # print msg
    # print msg.height
    # print msg.width
    print msg.format
    # print count
    # count += 1
    # print msg.format.find("jpeg")
    
    # print msg.is_bigendian
    # print msg.step
    # try:
    #     # Convert your ROS Image message to OpenCV2
    #     # cv2_img = bridge.imgmsg_to_cv2(msg, "bgra8")
    #     cv2_img = bridge.imgmsg_to_cv2(msg, "bgra8")
    #     # print cv2_img.shape
    #     cv2.imshow('camera_image', cv2_img)
    #     framerate = 1000/30
    #     cv2.waitKey(15)
    # except CvBridgeError, e:
    #     print(e)
    # pass

# img_container = []
# for i in range(3):
#     img_container += [cv2_img]
# print len(img_container)

# cv2.imshow('camera_image', cv2_img)
# cv2.waitKey(1000)