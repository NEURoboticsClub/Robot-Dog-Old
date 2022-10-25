#!/usr/bin/env python

import rospy

#task 1. import the Pose type from the module turtlesim
from turtlesim.msg import Pose

def poseCallback(pose_message):

   #task 4. display the x, y, and theta received from the message
    print ("pose callback")
    print ('x = {}'.format(pose_message.x))
    print ('y = {}'.format(pose_message.y))
    print ('yaw = {}'.format(pose_message.theta)) 

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        position_topic = "/turtle1/pose"
       #task 2. subscribe to the topic of the pose of the Turtlesim
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)        
       #task 3. spin
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")