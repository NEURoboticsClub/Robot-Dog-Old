#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    # this is publisher code
    rospy.init_node('test_node')
    pub = rospy.Publisher('test_topic', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        # rospy.loginfo("Publishing a message...")
        pub.publish("Hello from Test Node!")
        rate.sleep()