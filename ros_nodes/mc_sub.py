#!/usr/bin/env python2
import rospy
import socket


# global variable
client_socket = None
data_buffer = {} # ROStopic: data

if __name__ == "__main__":
    print("MC_SUB node is running...")

    # 1. init node
    rospy.init_node('mcsub_node')

    # 2. init server socket to listen to client req
    HOST = socket.gethostname()
    MSG_SIZE = 1024
    MC_SUB_PORT = 9998

    MC_SUB_SERVER_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    MC_SUB_SERVER_SOCKET.bind((HOST, MC_SUB_PORT))
    MC_SUB_SERVER_SOCKET.listen(5)
    
    while not rospy.is_shutdown():
        # 3. connect with new client (MC)
        client_socket, addr = MC_SUB_SERVER_SOCKET.accept()

        # 4. receive message and log
        while True:
            msg= client_socket.recv(MSG_SIZE)

            if not msg: 
                break
            print("received from MC:", msg)
        
            
    # Close the connection
    client_socket.close()
