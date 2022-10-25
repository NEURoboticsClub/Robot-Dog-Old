#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from Jared.msg import simpleMoteus

i = 0

def talker():
    pub = rospy.Publisher('chatter', simpleMoteus, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(100)
    command = simpleMoteus()

    i = 0
    multiplier = 1
    while not rospy.is_shutdown():
        
        simpleMoteus.position = i
        simpleMoteus.velocity = -.05
        simpleMoteus.torque = .04
        rospy.loginfo("Published: " + str(i))
        #rospy.loginfo(command)
        pub.publish(command)
        rate.sleep()
        i += .01 * multiplier

        if(i <= 0 or i >= 3):
            multiplier *= -1

def on_press(key):
    print(key)

def on_release(key):
    print(key)

if __name__ == "__main__":

    try:
        talker()
    except rospy.ROSInterruptException:
        pass