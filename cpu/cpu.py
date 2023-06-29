#!/usr/bin/env python3
import rospy
import json
import math
import queue
from std_msgs.msg import String
from datetime import datetime

global mc_data
mc_data = queue.Queue() 
# Callback function when publisher publishe something on this topic
def callback(data):

    # 1. convert data to json
    json_dict = json.loads(data.data)
    msg_id = json_dict.get("id", "No ID found")

    # list of 12 lists of [mc_id, pos, vel, torque]
    mc12 = json_dict.get("mc12", "No mc12 data found")

    # 2. print data and msg_id
    print("CPU_NODE: id={}, from MC={}".format(msg_id, mc12))

    # 3. save it to global variable
    mc_data.put(mc12)

if __name__ == "__main__":
    rospy.init_node('cpu_node')
    pub = rospy.Publisher('cpu_topic', String, queue_size=100)
    sub = rospy.Subscriber('mc_topic', String, callback)
    rate = rospy.Rate(40) # publishing rate (40 per seconds)
    print("cpu_node started...")


    # publish and increment id at each publish
    msg_id = 1
    while not rospy.is_shutdown():

        mc12_publish = None

        # 1. if we have no previous mcs data
        if mc_data.empty():
            # create new command for the 12 mcs
            mcs12 = [[mcid, math.nan, 2.0, 1.0] for mcid in range(1, 13)]
        
        # 2. use the previous data
        else:
            
            # - pop the latest data
            mc12_publish = mc_data.get()

            # - fake modify
            mc12_publish = [ [mc_id, pos, vel, tor] for mc_id, pos,vel, tor in mc12_publish]

        # 3. jsonify and publish
        json_tosend = json.dumps({"id":msg_id, "mc12": mc12_publish})
        pub.publish(json_tosend)

        # 4. incr id
        msg_id+=1

        #  ensure a consistent loop frequency
        rate.sleep()