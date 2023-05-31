#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from datetime import datetime

# Get the current date and time



if __name__ == "__main__":
    # this is publisher code
    rospy.init_node('test_node')
    pub = rospy.Publisher('test_topic', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        # rospy.loginfo("Publishing a message...")
        current_time = datetime.now()
        formatted_time = current_time.strftime("%H:%M:%S")
        pub.publish("hello from test node!" + formatted_time)
        rate.sleep()