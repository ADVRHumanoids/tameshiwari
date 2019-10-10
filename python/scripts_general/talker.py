#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def talker():
    # rospy.Publisher creates a publisher that is publishing to the topic 'chatter' with a imported msg type 'String' imported from std_msgs.msg.
    # it also has a queue size of 10 meaning that if it cannot publish the maximum backlog has the size 10
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # creates the node 'talker' with a unique name by calling anonymous=True
    rospy.init_node('talker', anonymous=True)
    
    # the rate  at which this script will publish string to the topic
    rate = rospy.Rate(10) # 10hz
    
    
    while not rospy.is_shutdown():
        # define the string hello_str including the cpu time by adding %s in the string and calling % rospy.get_time()
        hello_str = "hello world %s" % rospy.get_time()

        # the rospy.loginfo has a multiple use, here it used for debugging because it prints hello_str to the terminal, and logs it in the logfile
        rospy.loginfo(hello_str)

        # the pub.publish publishes the hello_str to the topic 'chatter' as described before.
        pub.publish(hello_str)
        
        # sleep until loop is called again
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
