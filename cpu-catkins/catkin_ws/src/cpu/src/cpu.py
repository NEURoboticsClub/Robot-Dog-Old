#!/usr/bin/env python2
import rospy
import json
import math
import Queue as queue
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class CPUNode:
    def __init__(self):
        rospy.init_node('cpu_node')
        # 1. each element is an array of 12 elements:
        # [[id1, pos1, vel1, torque1], ..., id12, pos12, vel12, torque12]]
        self.mc12_data = queue.Queue() 

        # 2. this will store positions for the 12 mc 
        # [ 0.22, 1.231 , .... ]
        self.joints_pos_data = queue.Queue()
        self.msg_id = 1
        self.prevErrors = [0] * 12
        self.prevTime = rospy.Time.now().to_sec()
        self.time = rospy.Time.now().to_sec()

        
        # 3. set publisher for the command from cpu
        self.pub = rospy.Publisher('cpu_topic', String, queue_size=100)

        # 4. subscribe to mc and joint_states
        self.sub_mc = rospy.Subscriber('mc_topic', String, self.get_mc_info)
        self.sub_joint_states = rospy.Subscriber('joint_states', JointState, self.get_joint_states)

        print("cpu_node started...")

    def get_mc_info(self, data):
        # 1. grab the data that's just been published
        json_dict = json.loads(data.data)
        msg_id = json_dict.get("id", "No ID found")
        mc12 = json_dict.get("mc12", "No mc12 data found")

        # 2. store it 
        self.mc12_data.put(mc12)

        print("CPU_NODE: id={}, from MC={}".format(msg_id, mc12))

    def get_joint_states(self, data):
        self.time = data.header.stamp.secs
        self.joints_pos_data.put(data.position)

    def arrayDiffFinder(self, mcs12_pos, joint_states_pos):
        """
        mcs12_pos is array of position of the 12 motors from mc
        joint_states_pos is array of posisition of the 12 motors from joint_states_pos
        time is the last time we get joint_states postion
        prevTime is the last time this function is called
        """

        kp = 0.5
        kd = 0.5
        errors = [(joint_states_pos[i] - mcs12_pos[i]) for i in range(0, len(mcs12_pos))]
        dError = [((errors[i] - self.prevErrors[i]) / (self.time - self.prevTime)) for i in range(0,len(errors))] 	
        vels = [kp * errors[i]+ kd * dError[i] for i in range(0,len(errors))]
        
        return vels, self.time, errors

    def publish_command(self):
        mc12_command = None

        # 1. if this is the first command, just hard code or use the first command from joint_states?
        if self.mc12_data.empty():
            mc12_command = [[mcid, float('nan'), 2.0, 1.0] for mcid in range(1, 13)]

        # 2. motor already moving, do some compute
        else:
            # don't publish any command if the last time we publish is the same second
            if self.prevTime == self.time:
                return
            # - grab the last mc states and the command from joints
            mc12_last = [ datas[1] for datas in self.mc12_data.get()]
            joints_last = self.joints_pos_data.get()

            # - do some compute
            # print("CPU_NODE: currT={}, prevT={}".format(self.time, self.prevTime))
            vels, self.prevTime, self.prevErrors = self.arrayDiffFinder(mc12_last, joints_last)
            
            # - assign to mc12 command
            mc12_command= [[id, float('nan'),vels[id-1], 3.0] for id in range(1,13)]

        # send the 12 of them
        json_tosend = json.dumps({"id": self.msg_id, "mc12": mc12_command})
        self.pub.publish(json_tosend)
        self.msg_id += 1


    def run(self):
        rate = rospy.Rate(40) # publishing rate (40 per second)
        while not rospy.is_shutdown():
            self.publish_command()
            rate.sleep()

if __name__ == "__main__":
    node = CPUNode()
    node.run()
