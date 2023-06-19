#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from datetime import datetime

# Callback function when publisher publishe something on this topic
def callback(data):
    # rospy.loginfo("CPU_NODE: from MC= %s", data.data)
    print("CPU_NODE: from MC={}".format(data.data))


if __name__ == "__main__":
    rospy.init_node('cpu_node')
    pub = rospy.Publisher('cpu_topic', String, queue_size=100)
    sub = rospy.Subscriber('mc_topic', String, callback)
    rate = rospy.Rate(40) # publishing rate (40 msgs per seconds)
    print("cpu_node started...")

    # publish and increment id at each publish
    id = 1
    while not rospy.is_shutdown():
        # current_time = datetime.now()
        # formatted_time = current_time.strftime("%H:%M:%S")  
        msg = "hello from cpu node! id=" + str(id)
        pub.publish(msg)
        id+=1
        rate.sleep()