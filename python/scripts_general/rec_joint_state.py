#!/usr/bin/env python

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia
#
#   This script can publish joint states to the robot_state_publisher running in Rviz.
#   With this script it is possible to playback generated data/trajectories from python directly to Rviz.
#

import rospy
import rosbag
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rosbag2video as r2v

# bridge = CvBridge()

# def Callback_test(data):
#     imgmsg = data
#     # print imgmsg.header
#     # print imgmsg.width
#     try:
#         cv_image = bridge.imgmsg_to_cv2(data, "bgra8")
#         # print "succes"
#     except CvBridgeError as e:
#         print(e)
#     # rate.sleep()
class SubImage:
        def __init__(self):
            # self.bridge = CvBridge()
            self.imgmsg = CompressedImage()
            # self.imgmsg = Image()
            # self.cv_image = 255 * np.ones(shape=[530, 640, 3], dtype=np.uint8)
            # self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
            # self.out = cv2.VideoWriter('output.avi',self.fourcc, 30.0, (480,480))
            print "new class instance has been created"
        
        def callback(self, data):
            self.imgmsg = data
            # print self.imgmsg.header
            # print "this message is printed at every time the callback is ran"
            # try:
            #     self.cv_image = bridge.imgmsg_to_cv2(data, "bgra8")
            #     print "succes"
            # except CvBridgeError as e:
            #     print(e)
            # self.out.write(self.cv_image)

        def finish(self):
            # self.out.release()
            print "video has been released to 'test_image.bag' in the same folder"



def posePublisher(pose,record=True):
    if record:
        # bag = rosbag.Bag('test.bag','w')
        # imgmsg = Image()
        # print imgmsg
        testclass = SubImage()
        filename = 'test_image.bag'
        bag = rosbag.Bag(filename,mode='w')
        # print testclass.imgmsg
        # fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # out = cv2.VideoWriter('output.avi',fourcc, 30.0, (480,480))
        print "start recording now:"
    
    pub = rospy.Publisher('pose_state', JointState, queue_size=10)
    # sub = rospy.Subscriber('/rviz1/camera1/image', Image, testclass.callback,tcp_nodelay=True)
    sub = rospy.Subscriber('/rviz1/camera1/image/compressed', CompressedImage, testclass.callback, tcp_nodelay=True)


    rospy.init_node('posePublisher', anonymous=True)
    rate = rospy.Rate(pose.rate) # default is 10 Hz
    # rate = rospy.Rate(1)
    state_str = JointState()
    state_str.header = Header()
    iteration = 0
    
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
        if record:
            # sub = rospy.Subscriber('/rviz1/camera1/image', Image, testclass.callback, queue_size=10)
            
            bag.write('camera1/image',testclass.imgmsg)
            # bag.write('camera1/cv_img',testclass.cv_image)

            # filename = 'frame_' + str(iteration) + '.jpeg'
            # print filename
            # cv2.imwrite(filename,testclass.cv_image)
            # print testclass.imgmsg.header
            # testclass.out.write(testclass.cv_image)
            # bag.write('camera/image', imgmsg)
            # try:
            #     cv_image = bridge.imgmsg_to_cv2(imgmsg, "bgra8")
            # except CvBridgeError as e:
            #     print(e)
            # cv2_img = bridge.imgmsg_to_cv2(imgmsg,"bgra8")
            # out.write(testclass.cv_image)
            # print imgmsg.header
            # print "this is frame number %s" %iteration
        iteration += 1
        rate.sleep()

    if record:
        # bag = rosbag.Bag('test.bag','w')
        print "stop recording now:"
        testclass.finish()
        bag.close()

        # out.release()
        # testclass.out.release()
        # print "video has been released to 'output.avi' in the same folder"
        # bag.close()

if __name__ == '__main__':
    try:
        posePublisher(pose)
    except rospy.ROSInterruptException:
        pass
