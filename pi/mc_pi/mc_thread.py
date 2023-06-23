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
            bytes_msg = sock.recv(MSG_SIZE)

            # 1. exit the loop if no more data to read
            if not bytes_msg:
                break 

            # 2. convert to json string, then to objt
            json_msg = json.loads(bytes_msg)
            msg_id = json_msg["id"]
            mc12 = json_msg["mc12"]

            # 3. get data for 2nd motor (1st index)
            mc2data = mc12[1]

            # pos, vel , tor
            print("MC: from CPU id={}, m_mc2={}".format(msg_id, mc2data))

            # 4. set attributes
            # m.setAttributes(mc2data[0], pos=mc2data[1], velocity = mc2data[2], torque=mc2data[3])
        except KeyError:
                print("Error: 'id' or 'mc12' key not found in JSON data")
        except json.JSONDecodeError:
            print("Error: Invalid JSON data received. Reconnecting...")
        
        except socket.timeout as e:
            print("Timeout occurred while waiting for response: {}".format(e))
        
        except IOError as e:  
            if e.errno == errno.EPIPE:  
                print("broken pipe: {}".format(e))
    
      

def send_mc_info(sock):
        id = 1

        # create new data for 12 MC
        mcs12 = [[mcid, math.nan, 2.0, 1.0] for mcid in range(1, 13)]

        while True:
            
            # 1. create a dict
            data = {"mc12": mcs12, "id":id}

            # 2. send as bytes encoded json
            sock.send((json.dumps(data)).encode())
            id+=1

            # 3. sleep for 20ms so its sending at 50Hz
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