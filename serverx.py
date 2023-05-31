#!/usr/bin/env python3
import socket
import json

# to run:
# sherly@sherly-Inspiron-15-3510:~/Desktop/ROS/catkin_ws/src$ rosrun my_robot_controller my_first_node.py 

# Server code that runs in container without rosnode
# Create a socket object
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Get local machine name
host = socket.gethostname()
port = 9999

# Bind to the port
serversocket.bind((host, port))

# Start listening for client connections
serversocket.listen(5)

while True:
    print("server starts from docker...")
    # Establish connection with client
    clientsocket, addr = serversocket.accept()
    print(f"Got a connection from {str(addr)}, curr_host={host}")

    # Receive up to 1024 bytes
    msg = clientsocket.recv(1024)
    client_data = json.loads(msg.decode("utf-8"))
    print(f"Received client data: {client_data}")

    # Send some ROS data (as a JSON string)
    ros_data = {"from docker": "some_command", "parameter": 123}
    clientsocket.send(json.dumps(ros_data).encode("utf-8"))

    # Close the connection
    clientsocket.close()
