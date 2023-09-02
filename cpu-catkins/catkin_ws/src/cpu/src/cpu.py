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
        # 1. to store mc12 current states (from mc)
        # each element is an array of 12 elements:
        # [[id1, pos1, vel1, torque1], ..., id12, pos12, vel12, torque12]]
        self.mc12_data = queue.Queue() 
        self.motor_started = False

        # 2. to store mc12 expected states or command (from joint_states)
        # [ 0.22, 1.231 , .... ]
        self.joints_cmd_pos_data = queue.Queue()
        self.prevErrors = [0] * 12

        # 3. time and id tracker
        self.prevTime = rospy.Time.now().to_sec()
        self.time = rospy.Time.now().to_sec()
        self.msg_id = 1

        
        # 4. set publisher for the command from cpu
        self.pub = rospy.Publisher('cpu_topic', String, queue_size=100)

        # 5. subscribe to mc and joint_states
        self.sub_mc = rospy.Subscriber('mc_topic', String, self.get_mc_states)
        self.sub_joint_states = rospy.Subscriber('joint_states', JointState, self.get_joint_cmd)

        print("cpu_node started...")

    def get_mc_states(self, data):

        if not self.motor_started:
            self.motor_started = True

        # 1. grab the data that's just been published
        json_dict = json.loads(data.data)
        msg_id = json_dict.get("id", "No ID found")
        mc12 = json_dict.get("mc12", "No mc12 data found")

        # 2. store it 
        self.mc12_data.put(mc12)

        # print("CPU_NODE: id={}, from MC={}".format(msg_id, mc12))

    def get_joint_cmd(self, data):
        self.time = data.header.stamp.to_sec()

        # Directly call publish_command with the received position command
        self.publish_command(data.position)


    def publish_command(self, joints_pos_cmd):

        # Uncomment this to check if command from teleop is received
        epsilon = 1e-10
        if abs(joints_pos_cmd[0] - (-7.351371067443324e-08)) > epsilon:
            print("CPU_NODE: teleop cmd={}".format(joints_pos_cmd))


        mc12_command = None
        # case 1. if mc12 not yet started, keep sending the first hard coded command
        # it will only set to true once we receive the first mc parsed result
        if not self.motor_started:
            mc12_command = [[mcid, float('nan'), 2.0, 1.0] for mcid in range(1, 13)]
            print("CPU_NODE: id={}, 1st cmd2={}".format(self.msg_id, mc12_command[1]))
             

        # case 2. motor already moving
        else:
            # 1. don't publish any command if no data from mc 
            if self.mc12_data.empty():
                return
            
            # 2. grab the latest 12 mc states position
            mc12_last = [ datas[1] for datas in self.mc12_data.get()]

            # 3. do some compute to get next velocity
            vels, self.prevTime, self.prevErrors = self.arrayDiffFinder(mc12_last, joints_pos_cmd)
            
            # 4. assign the output velocity
            # OPTION 1: use command from teleop and output from the arrayDiffFinder computation.
            # WARNING: Be prepare to stop the machine as the computation from arrDiffFinder can be erroneous and make
            # the motor run too fast and breaks
            # mc12_command= [[id, float('nan'),vels[id-1], 1.0] for id in range(1,13)]
            # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            
            # OPTION2: hard code velocity to always be 2.0
            mc12_command= [[id, float('nan'), 2.0, 1.0] for id in range(1,13)]


            print("CPU_NODE: id={}, cont cmd2={}".format(self.msg_id, mc12_command[1]))
            
        
        # 5. send the 12 motor commands to mc
        json_tosend = json.dumps({"id": self.msg_id, "mc12": mc12_command})
        self.pub.publish(json_tosend)
        self.msg_id += 1


    # Helper function to compute next velocity
    def arrayDiffFinder(self, mcs12_pos, joint_cmd_pos):
        """
        mcs12_pos is array of position of the 12 motors from mc
        joint_cmd_pos is array of position of the 12 motors from champ command
        time is the last time we get joint_states postion
        prevTime is the last time this function is called
        """

        kp = 0.5
        kd = 0.5
        # print("CPU_NODE: currPos2={}, cmdPos2={}".format(mcs12_pos[1], joint_cmd_pos))
        errors = [(joint_cmd_pos[i] - mcs12_pos[i]) for i in range(0, len(mcs12_pos))]
        dError = [((errors[i] - self.prevErrors[i]) / (self.time - self.prevTime)) for i in range(0,len(errors))] 	
        vels = [kp * errors[i]+ kd * dError[i] for i in range(0,len(errors))]
        
        return vels, self.time, errors



    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = CPUNode()
    node.run()
