#!/usr/bin/env python2
import rospy
import json
import math
import Queue as queue
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose as Pose

from datetime import datetime

global mc_data
mc_data = queue.Queue() 

# Callback function on mc_topic
def get_mc_info(data):

    # 1. convert data to json
    json_dict = json.loads(data.data)
    msg_id = json_dict.get("id", "No ID found")

    # list of 12 lists of [mc_id, pos, vel, torque]
    mc12 = json_dict.get("mc12", "No mc12 data found")

    # 2. print data and msg_id
    # print("CPU_NODE: id={}, from MC={}".format(msg_id, mc12))
    # CPU_NODE: id=413, from MC=[[1,  float('nan'), -0.010213678702712059, 0.00903321709483862]]

    # 3. save it to global variable
    mc_data.put(mc12)

# Callback function on joint_states topic (published by teleop)
def get_joint_states(data):
#  CPU_NODE: joint_states=
#  header: 
#    seq: 3794
#    stamp: 
#      secs: 1690071772
#      nsecs: 377252500
#    frame_id: ''
#  name: [lf_hip_joint, lf_upper_leg_joint, lf_lower_leg_joint, rf_hip_joint, rf_upper_leg_joint,
#    rf_lower_leg_joint, lh_hip_joint, lh_upper_leg_joint, lh_lower_leg_joint, rh_hip_joint,
#    rh_upper_leg_joint, rh_lower_leg_joint]
#  position: [-7.351371067443324e-08, 0.7824053764343262, -1.5648106336593628, -1.3909067675399456e-08, 0.7824053764343262, -1.5648106336593628, -7.351371067443324e-08, 0.7824053764343262, -1.5648106336593628, -1.3909067675399456e-08, 0.7824053764343262, -1.5648106336593628]
#  velocity: []

    # 2. print data and msg_id
    print("CPU_NODE: joint_states={}".format(data.position))

# Callback function on cmd_vel topic (published by teleop)
def get_cmd_vel(data):
    # 2. print data and msg_id
    print("CPU_NODE: joint_states={}".format(data))

# Callback function on body_pose topic (published by teleop)
def get_body_pose(data):
    print("CPU_NODE: get_pose={}".format(data))


if __name__ == "__main__":
    rospy.init_node('cpu_node')
    pub = rospy.Publisher('cpu_topic', String, queue_size=100)
    sub_mc = rospy.Subscriber('mc_topic', String, get_mc_info)
    sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, get_cmd_vel)
    # sub_pose = rospy.Subscriber('body_pose', Pose, get_body_pose)
    sub_joint_states = rospy.Subscriber('joint_states', JointState, get_joint_states)
    rate = rospy.Rate(40) # publishing rate (40 per seconds)
    print("cpu_node started...")


    # publish and increment id at each publish
    msg_id = 1
    while not rospy.is_shutdown():

        mc12_publish = None

        # 1. if we have no previous mcs data
        if mc_data.empty():
            # create new command for the 12 mcs
            mc12 = [[mcid, float('nan'), 2.0, 1.0] for mcid in range(1, 13)]
        
        # 2. use the previous data
        else:
            
            # - pop the latest data
            mc12_publish = mc_data.get()

            # - hard code id 2 only
            # mc12_publish = [ [2, pos, vel, tor] for mc_id, pos,vel, tor in mc12_publish]
            mc12_publish= [[2, float('nan'), 3.0, 3.0]]
            
        # 3. jsonify and publish
        json_tosend = json.dumps({"id":msg_id, "mc12": mc12_publish})
        pub.publish(json_tosend)

        # 4. incr id
        msg_id+=1

        #  ensure a consistent loop frequency
        rate.sleep()