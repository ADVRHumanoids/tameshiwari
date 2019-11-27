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
import subprocess


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
    rate = rospy.Rate(pubrate) # default is 10 Hz
    # print pose.rate
    state_str = JointState()
    state_str.header = Header()
    iteration = 0
    # print np.shape(pose.q)
    send_velocity = False

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
        if send_velocity:
            if np.shape(pose.qdot)[0] > 1:
                state_str.velocity = pose.qdot[iteration,:]
                if nj_extra > 0:
                    state_str.velocity = np.concatenate((state_str.velocity,zerovec))
            if np.shape(pose.tau)[0] > 1:
                state_str.effort = pose.tau[iteration,:]
                if nj_extra > 0:
                    state_str.effort = np.concatenate((state_str.effort,zerovec))
        else:
            pass
        # rospy.loginfo(state_str)            # use for debugging
        pub.publish(state_str)
        iteration += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        # KARATE CHOP HORIZONTAL BOARD
        # TESTED: TRUE
        path = '/home/user/catkin_ws/src/tameshiwari/python/OCP_centauro_7dof_arm/results'
        filename = 'RobotPose_centauro_max_momentum_6dof_v1_2019-11-21T15:46:28.mat'
        # KARATE PUNCH VERTICAL BOARD:
        # TESTED: FALSE
        path = '/home/user/catkin_ws/src/tameshiwari/python/OCP_centauro_7dof_arm/results'
        filename = 'RobotPose_centauro_max_momentum_TRUE6dof_v3_2019-11-27T12:55:13.mat'
        #######################################
        # LOAD FILE
        #######################################
        matfile = sio.loadmat(path+'/'+filename)
        matfile['name'] = [str(x) for x in matfile['name']]
        pose = fn.RobotPose(
            name=matfile['name'],
            q=matfile['q'],
            qdot=matfile['qdot'],
            # tau=matfile['tau'],
            rate=matfile['rate']
        )
        
        rostopic = '/xbotcore/ros_command'
        # rostopic = 'pose_state'
        pubrate = 120.0
        slowrate = 1.0
        poserate = pubrate*slowrate
        pose.interpolate(poserate)
        if rostopic == 'pose_state':
            T = 0.5
        else:
            T = 3

        #######################################
        # HOMING & INITIATE COMMUNICATION PLUGIN
        #######################################
        if rostopic != 'pose_state':
            execute = raw_input('FIRST RUN? (y/n): ')
            if execute == 'y':
                # START HOMING
                raw_input('Press enter to call: /xbotcore/HomingExample_switch 1')
                subprocess.call(["rosservice", "call", "/xbotcore/HomingExample_switch", "1"])

                # FINISH HOMING
                raw_input('Press enter to call: /xbotcore/HomingExample_switch 0')
                subprocess.call(["rosservice", "call", "/xbotcore/HomingExample_switch", "0"])
                
                # SET COMMUNICATION PLUGIN
                raw_input('Press enter to call: /xbotcore/XBotCommunicationPlugin_switch 1')
                subprocess.call(["rosservice", "call", "/xbotcore/XBotCommunicationPlugin_switch", "1"])
            else:
                pass

        #######################################
        # FILTER SAFE MODE
        #######################################
        if rostopic != 'pose_state':
            raw_input('Press enter to call: /xbotcore/set_filter_profile_safe')
            subprocess.call(["rosservice", "call", "/xbotcore/set_filter_profile_safe"])

        #######################################
        # MOVE TO HITTING POSITION
        #######################################
        q_iota = [-0.589212, 0.47064095, -0.18100785, -1.0331983, -1.16513015, -0.00538894, 0.0]
        create_impact = True
        if create_impact:
            name = matfile['name']
            q_iota = np.matlib.repmat(q_iota,int(T*pubrate),1)
            # print np.shape(q_iota)
            pose_iota = fn.RobotPose(name=name,q=q_iota,rate=pubrate)
            raw_input('Press enter to move to IMPACT: ')
            posePublisher(pose=pose_iota,pubrate=pubrate,rostopic=rostopic)


        #######################################
        # MOVE TO INITIAL POSITION
        #######################################
        create_init = True
        if create_init:
            name = matfile['name']
            q_init = matfile['q'][0,:]
            q_init = np.matlib.repmat(q_init,int(T*pubrate),1)
            raw_input('Press enter to move to INITIAL POSE: ')
            pose_init = fn.RobotPose(name=name,q=q_init,rate=pubrate)
            posePublisher(pose=pose_init,pubrate=pubrate,rostopic=rostopic)

        #######################################
        # FILTER FAST MODE
        #######################################
        if rostopic != 'pose_state':
            raw_input('Press enter to call: /xbotcore/set_filter_profile_fast')
            subprocess.call(["rosservice", "call", "/xbotcore/set_filter_profile_fast"])

        #######################################
        # EXECUTE TRAJECTORY
        #######################################
        raw_input('Press enter to EXECUTE TRAJECTORY: ')
        posePublisher(pose=pose,pubrate=pubrate,rostopic=rostopic)


    except rospy.ROSInterruptException:
        pass
