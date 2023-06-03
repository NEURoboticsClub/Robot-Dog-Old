#!/usr/bin/env python2
import rospy
import socket
import json
from std_msgs.msg import String


# global variable
client_socket = None
data_buffer = {} # ROStopic: data

if __name__ == "__main__":
    print("Server Node is running...")

    # 1. init node
    rospy.init_node('cpusub_node')

    # 2. init server socket to listen to client req
    SERVER_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    HOST = socket.gethostname()
    PORT = 9999
    SERVER_SOCKET.bind((HOST, PORT))
    SERVER_SOCKET.listen(5)

    
    while not rospy.is_shutdown():
        # 3. connect with new client (MC)
        client_socket, addr = SERVER_SOCKET.accept()
        
            
    # Close the connection
    client_socket.close()
