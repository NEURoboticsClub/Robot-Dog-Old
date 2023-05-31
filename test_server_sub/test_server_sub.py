#!/usr/bin/env python3
import rospy
import socket
import json
from std_msgs.msg import String


# global variable
client_socket = None
data_buffer = {} # ROStopic: data

# Callback function when publisher publishe something on this topic
def callback(data):
    global client_socket
    # 1. This function gets called every time a message is published on the 'test_topic' topic
    rospy.loginfo("Received a message from a ROS test_node: %s", data.data)

    # 2. save lastest data on this topic
    data_buffer["test_topic"] = data.data



if __name__ == "__main__":
    print("Server Node is running...")

    # 1. init node
    rospy.init_node('server_node')
    rospy.Subscriber('test_topic', String, callback)

    # 2. init server socket to listen to client request
    SERVER_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    HOST = socket.gethostname()
    PORT = 9999
    SERVER_SOCKET.bind((HOST, PORT))
    SERVER_SOCKET.listen(5)

    # 3. connect with new client
    client_socket, addr = SERVER_SOCKET.accept()
    print(f"Got a connection from {str(addr)}, curr_host={HOST}")

    while not rospy.is_shutdown():
    
        # 4. receive msg from client
        msg = client_socket.recv(1024)
        client_data = json.loads(msg.decode("utf-8"))
        print(f"Received client data: {client_data}")

        # 5. Send data to client
        client_socket.send(json.dumps({"data": data_buffer["test_topic"], "id": {client_data.id}}).encode("utf-8"))

    # Close the connection
    client_socket.close()
