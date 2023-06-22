import socket
import json
import errno
import threading
import time
import math

# Get local machine name
SERVER_HOST = socket.gethostname()
MSG_SIZE = 1024
CPU_SUB_SERVER_PORT = 9999
MC_SUB_SERVER_PORT = 9998

def get_cpu_info(sock):
    while True:
        try:
            # 0. get message and process
            raw_msg = sock.recv(MSG_SIZE)
            if not raw_msg:
                break # exit the loop if no more data to read

            # 1. test get message and print (string)
            # msg = raw_msg.decode()
            # print("MC: from CPU={}".format(msg))

            # 2. test get actual data
            msg = raw_msg.decode()
            json_msg = json.loads(msg) # first deserialization
            final_msg = json.loads(json_msg["data"]) # second deserialization
            msg_id = final_msg["id"]
            mc12 = final_msg["mc12"]
            
            print("MC: from CPU id={}, m_mc12={}".format(msg_id, mc12))


        except socket.timeout as e:
            print("Timeout occurred while waiting for response: {}".format(e))
        
        except IOError as e:  
            if e.errno == errno.EPIPE:  
                print("broken pipe: {}".format(e))
      

def send_mc_info(sock):
    id = 1
    mcs12 = [[2, math.nan, 1.0, 2] for mcid in range(12)]

    while True:
        # 1. init and send data
        data = {"mc12": mcs12, "id":id}
        sock.send((json.dumps(data)).encode())
        id+=1

        # sleep for 20ms so its sending at 50Hz
        time.sleep(0.02)

def main():
    # 1. init socket and time out to listen to cpu_sub node
    cpu_sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cpu_sub_socket.settimeout(2.0)
    # connect to server
    cpu_sub_socket.connect((SERVER_HOST, CPU_SUB_SERVER_PORT))
    cpu_sub_socket.setblocking(False)  # set socket to non-blocking

    # 2. inis socket and timeout to send msg to mc_sub node
    mc_sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mc_sub_socket.settimeout(10.0)
    mc_sub_socket.connect((SERVER_HOST, MC_SUB_SERVER_PORT))

    # 3. init thread
    get_cpu_info_thread = threading.Thread(target=get_cpu_info, args=(cpu_sub_socket,))
    send_mc_info_thread = threading.Thread(target=send_mc_info, args=(mc_sub_socket,))

    # 4. run
    get_cpu_info_thread.start()
    send_mc_info_thread.start()

    get_cpu_info_thread.join()
    send_mc_info_thread.join()


if __name__ == "__main__":
    main()