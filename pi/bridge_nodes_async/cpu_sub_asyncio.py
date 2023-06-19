import asyncio
import websockets
import json
import threading
import rospy
import queue
from std_msgs.msg import String

# global variable
cpu_data = queue.Queue()

# Callback function when publisher publish something on this topic
def callback(data):
    global cpu_data
    rospy.loginfo("CPU_SUB_asyncio: %s", data.data)

    # save latest data on this topic
    cpu_data.put(data.data)

async def send_to_mc(websocket, path):
    try:
        while True:
            if not cpu_data.empty():
                data = cpu_data.get()
                print("sending data")
                json_data = json.dumps({"data": data}) + "\n"
                await websocket.send(json_data)
            else:
                await asyncio.sleep(0.2)  # sleep only when there's no data to send
    except Exception as e:
        print(f"Error occurred: {e}")


async def main():
    HOST = 'localhost'  # pi ip
    PORT = 9999  # this cpu_sub port

    start_server = websockets.serve(send_to_mc, HOST, PORT)

    try:
        server = await start_server
        print("Server started")
        await asyncio.Future()  # This will never complete, keeping the event loop running indefinitely.
    except Exception as e:
        print(f"An error occurred while starting the server: {e}")


if __name__ == "__main__":
    # Initialize ROS node in the main thread
    rospy.init_node('cpusub_node')
    rospy.Subscriber('cpu_topic', String, callback)

    # Start ros_stuff in a separate thread
    ros_thread = threading.Thread(target=rospy.spin)
    ros_thread.start()

    asyncio.run(main())