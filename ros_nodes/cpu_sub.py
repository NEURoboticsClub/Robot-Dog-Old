#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
import socket
import json


# global variable
client_socket = None
global cpu_data
cpu_data = ""

# Callback function when publisher publish something on this topic
def callback(data):
    global client_socket, cpu_data
    # 1. This function gets called every time a message is published on the 'test_topic' topic
    # rospy.loginfo("CPU_SUB: %s", data.data)

    # 2. save lastest data on this topic
    cpu_data = data.data



if __name__ == "__main__":
    print("CPU_SUB Node is running...")

    # 1. init node
    rospy.init_node('cpusub_node')
    rospy.Subscriber('cpu_topic', String, callback)

    # 2. init server socket to listen to client request
    SERVER_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    SERVER_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    HOST = socket.gethostname()
    PORT = 9999
    SERVER_SOCKET.bind((HOST, PORT))
    SERVER_SOCKET.listen(5)

    
    while not rospy.is_shutdown():
        # 3. connect with new client (MC)
        client_socket, addr = SERVER_SOCKET.accept()
        print("Got a connection from {}, curr_host={}".format(str(addr), HOST))

        # 4.keep sending new messages
        while True:  
            # convert dict to json string
            json_data = json.dumps({"data": cpu_data}) +"\n"
            client_socket.send(json_data)
            
    client_socket.close()
