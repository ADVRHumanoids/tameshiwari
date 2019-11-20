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
import scipy.io as sio
import functions as fn
import centauro_config as config

def posePublisher(pose,pubrate=120,init_pose=None,full_model=False,rostopic='pose_state'):
    ####################################################
    # This part is to add extra values to the print statements
    if full_model:
        if init_pose is None:
            init_pose = 'home'
        centauro = config.HomePose(pose=init_pose)
        full_name = centauro.getName()
        # print len(full_name)
        nj_extra = len(full_name) - len(pose.name)
        index = [i for i, x in enumerate(full_name) if x in pose.name]
        name_extra = [i for j, i in enumerate(full_name) if j not in index]
        q_extra = np.delete(centauro.getValue(),index)
    else:
        nj_extra = 0

    #####################################################

    pub = rospy.Publisher(rostopic, JointState, queue_size=10)
    rospy.init_node('posePublisher', anonymous=True)
    rate = rospy.Rate(pose.rate) # default is 10 Hz
    # print pose.rate
    state_str = JointState()
    state_str.header = Header()
    iteration = 0
    # print np.shape(pose.q)

    while not rospy.is_shutdown() and iteration < pose.q.shape[0]:
        now = rospy.get_rostime()
        state_str.header.seq = iteration
        state_str.header.stamp.secs = now.secs
        state_str.header.stamp.nsecs = now.nsecs
        state_str.name = pose.name
        state_str.position = pose.q[iteration,:]
        if nj_extra > 0:
            state_str.name = pose.name + name_extra
            state_str.position = np.concatenate((state_str.position,q_extra))
            zerovec = np.zeros(nj_extra)
        if np.shape(pose.qdot)[0] > 1:
            state_str.velocity = pose.qdot[iteration,:]
            if nj_extra > 0:
                state_str.velocity = np.concatenate((state_str.velocity,zerovec))
        if np.shape(pose.tau)[0] > 1:
            state_str.effort = pose.tau[iteration,:]
            if nj_extra > 0:
                state_str.effort = np.concatenate((state_str.effort,zerovec))
        # rospy.loginfo(state_str)            # use for debugging
        pub.publish(state_str)
        iteration += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        path = '/home/user/catkin_ws/src/tameshiwari/python/OCP_centauro_7dof_arm/results'
        filename = 'RobotPose_centauro_max_momentum_6dof_v1_2019-11-20T14:07:20.mat'
        
        matfile = sio.loadmat(path+'/'+filename)
        matfile['name'] = [str(x) for x in matfile['name']]
        pose = fn.RobotPose(
            name=matfile['name'],
            q=matfile['q'],
            qdot=matfile['qdot'],
            # tau=matfile['tau'],
            rate=matfile['rate']
        )
        
        rostopic = 'pose_state'
        pubrate = 120.0
        slowrate = 1.0
        poserate = pubrate*slowrate
        pose.interpolate(poserate)

        create_init = True
        if create_init:
            T = 3
            name = matfile['name']
            q_init = matfile['q'][0,:]
            q_init = np.matlib.repmat(q_init,int(T*pubrate),1)
            pose_init = fn.RobotPose(name=name,q=q_init,rate=pubrate)
            posePublisher(pose=pose_init,pubrate=pubrate,rostopic=rostopic)

        raw_input('Press enter to continue, get ready to record: ')

        posePublisher(pose=pose,pubrate=pubrate,rostopic=rostopic)
    except rospy.ROSInterruptException:
        pass
