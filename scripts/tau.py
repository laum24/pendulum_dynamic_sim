#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

if __name__ == __name__:
    pub = rospy.Publisher("tau_topic", Float32, queue_size=10)
    rospy.init_node("tau")
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        tau = 1
        pub.publish(tau)
        rate.sleep()