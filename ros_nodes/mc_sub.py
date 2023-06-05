import rospy
import socket
import threading


# global variable
# client_socket = None
data_buffer = {} # ROStopic: data


def get_mc_info():
    # init server socket to listen to client req
    HOST = socket.gethostname()
    MSG_SIZE = 1024
    MC_SUB_PORT = 9998

    MC_SUB_SERVER_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    MC_SUB_SERVER_SOCKET.bind((HOST, MC_SUB_PORT))
    MC_SUB_SERVER_SOCKET.listen(5)

    # 3. connect with new client (MC)
    client_socket, addr = MC_SUB_SERVER_SOCKET.accept()

    # 4. receive message and log
    while True:
        msg = client_socket.recv(MSG_SIZE)

        if not msg:
            break
        print("Received from MC:", msg)


# def publish_messages():
#     # init ROS publisher
#     pub = rospy.Publisher('topic', std_msgs.msg.String, queue_size=10)

#     # publish messages at 50 Hz
#     rate = rospy.Rate(50)
#     while not rospy.is_shutdown():
#         msg = std_msgs.msg.String()
#         msg.data = "Hello, world!"
#         pub.publish(msg)
#         rate.sleep()


if __name__ == "__main__":
    print("MC_SUB node is running...")

    # 1. init node
    rospy.init_node('mcsub_node')

    # start listening and logging in a separate thread
    listen_thread = threading.Thread(target=get_mc_info)
    listen_thread.start()

    # # start publishing messages in a separate thread
    # publish_thread = threading.Thread(target=publish_messages)
    # publish_thread.start()

    # main loop
    while not rospy.is_shutdown():
        rospy.spin()
