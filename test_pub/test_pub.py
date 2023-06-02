#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from datetime import datetime

if __name__ == "__main__":
    rospy.init_node('test_node')
    pub = rospy.Publisher('test_topic', String, queue_size=10)
    rate = rospy.Rate(50) # check messages 10 times per second

    while not rospy.is_shutdown():
        # rospy.loginfo("Publishing a message...")
        current_time = datetime.now()
        formatted_time = current_time.strftime("%H:%M:%S")
        pub.publish("hello from test node!" + formatted_time)
        rate.sleep()