#!/usr/bin/env python3
import asyncio
import websockets
import rospy
from std_msgs.msg import String

# global variable
mc_data = ""

async def get_mc_info(websocket, path):
    global mc_data
    async for message in websocket:
        mc_data = message

# def publish_mc_topic():
#     # publish messages at 50 Hz
#     rate = rospy.Rate(50)
#     while not rospy.is_shutdown():
#         msg = String()
#         if mc_data:
#             msg.data = mc_data
#         else:
#             msg.data = "no data yet"
#         pub.publish(msg)
#         rate.sleep()

def ros_stuff():
    rospy.init_node('mcsub_node')
    pub = rospy.Publisher('mc_topic',String, queue_size=100)

    # start publishing messages in a separate thread
    #     # publish messages at 50 Hz
#     rate = rospy.Rate(50)
#     while not rospy.is_shutdown():
#         msg = String()
#         if mc_data:
#             msg.data = mc_data
#         else:
#             msg.data = "no data yet"
#         pub.publish(msg)
#         rate.sleep()

async def main():
    # Set the server settings
    HOST = 'localhost'  # Modify as per your requirement
    PORT = 9998  # Modify as per your requirement

    # Run ROS related tasks in a separate thread
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, ros_stuff)

    # Start the server
    async with websockets.serve(get_mc_info, HOST, PORT):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
