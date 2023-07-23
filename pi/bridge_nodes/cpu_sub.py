#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket
import json
import threading
import queue
# global variable
client_socket = None
global cpu_data
cpu_data = queue.Queue()
# Callback function when publisher publish something on this topic
def callback(data):
    global client_socket, cpu_data
    # rospy.loginfo("CPU_SUB_THREAD: %s", data.data)

    # save latest json data on this topic
    cpu_data.put(data.data)

def send_to_mc(sock):
    while True:
        if not cpu_data.empty():
            json_data = cpu_data.get()
            # print(f"sending to mc={json_data}")
            bytes_data = json_data.encode('utf-8')  # Convert the JSON string to bytes
            sock.send(bytes_data)


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

    # 3. connect with new client (MC)
    client_socket, addr = SERVER_SOCKET.accept()
    print("CPU_SUB=Got a connection from {}, curr_host={}".format(str(addr), HOST))
    
    client_thread = threading.Thread(target=send_to_mc, args=(client_socket,))
    client_thread.start()

    # 4.keep sending new messages
    while True: 
        pass
            
            
    # client_socket.close()
