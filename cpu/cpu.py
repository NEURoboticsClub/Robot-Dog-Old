#!/usr/bin/env python3
import rospy
import json
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
    rate = rospy.Rate(100) # publishing rate (40 per seconds)
    print("cpu_node started...")


    # publish and increment id at each publish
    msg_id = 1
    while not rospy.is_shutdown():
        # 1. test publish string
        # pub.publish("hello from cpu id=" + str(msg_id))
        # msg_id += 1

        # 2. publish actual data
        if not mc_data.empty():
            
            # 1. pop the latest data
            ms12_modified = mc_data.get()

            # 2. fake modify
            ms12_modified = [ [mc_id, pos+8, vel+8, tor+8] for mc_id, pos,vel, tor in ms12_modified]

            # 3. jsonify and publish
            json_tosend = json.dumps({"id":msg_id, "mc12": ms12_modified})

            pub.publish(json_tosend)

            # test publish string
            msg_id+=1

        #  ensure a consistent loop frequency
        rate.sleep()