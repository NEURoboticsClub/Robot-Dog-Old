#!/usr/bin/env python3

#This script isn't very good, so you shouldn't copy it
#rewrite it in OOP/without global variables for extra rep

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

rangeAhead = 0
maxRange = 0
minRange = 0
right = 0
left = 0

def rangeCallback(msg):
    global rangeAhead, maxRange, minRange, left, right
    rangeAhead = msg.ranges[int(len(msg.ranges) / 2)]
    maxRange = max(msg.ranges)
    minRange = min(msg.ranges)
    right = msg.ranges[0]
    left = msg.ranges[len(msg.ranges) - 1]

def botHandler():
    pub = rospy.Publisher('simple_skid_steer/cmd_vel', Twist, queue_size = 10 )
    vel = Twist()
    while not (rospy.is_shutdown()):
        if(rangeAhead < .65 or minRange < .35):
            vel.linear.x = 0
            if(left < right):
                vel.angular.z = 3
            else:
                vel.angular.z = -3
            if(left + .1 > right and left - .1 < right):
                vel.angular.z = 3
        else:
            vel.angular.z = 0
            vel.linear.x = 1
        pub.publish(vel)

if __name__ == '__main__':
    rospy.init_node('setVelocities')
    rangeSub = rospy.Subscriber('simple_skid_steer/laser/scan', LaserScan, rangeCallback)
    botHandler()

