#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from datetime import datetime

TOPIC = 'cpu_topic'
if __name__ == "__main__":
    rospy.init_node('cpu_node')
    pub = rospy.Publisher(TOPIC, String, queue_size=10)
    rate = rospy.Rate(50) # check messages 50 times per second

    while not rospy.is_shutdown():
        current_time = datetime.now()
        formatted_time = current_time.strftime("%H:%M:%S")
        pub.publish("hello from cpu node!" + formatted_time)
        rate.sleep()