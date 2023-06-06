#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from datetime import datetime

# THIS IS CPU code if it was to run on raspberry pi in a single container

# Callback function when publisher publishe something on this topic
def callback(data):
    # rospy.loginfo("CPU_NODE: from MC= %s", data.data)
    print("CPU_NODE: from MC={}".format(data.data))


if __name__ == "__main__":
    rospy.init_node('cpu_node')
    pub = rospy.Publisher('cpu_topic', String, queue_size=10)
    sub = rospy.Subscriber('mc_topic', String, callback)
    rate = rospy.Rate(50) # publishing rate (50 per seconds)

    # publish
    while not rospy.is_shutdown():
        current_time = datetime.now()
        formatted_time = current_time.strftime("%H:%M:%S")
        pub.publish("hello from cpu node!" + formatted_time)
        rate.sleep()