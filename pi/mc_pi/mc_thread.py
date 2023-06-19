import socket
import json
import errno
import threading
import time

# Get local machine name
SERVER_HOST = socket.gethostname()
MSG_SIZE = 1024
CPU_SUB_SERVER_PORT = 9999
MC_SUB_SERVER_PORT = 9998

def get_cpu_info(sock):
    while True:
        try:
            # 1. get message and process
            raw_msg = sock.recv(MSG_SIZE)
            if not raw_msg:
                break # exit the loop if no more data to read

            # 2. split msg by newline
            messages = raw_msg.decode()
            print("MC: from CPU={}".format(messages, id))

        except BlockingIOError:
            # No data available right now, continue on with other work and try again later.
            continue
        except socket.timeout as e:
            print("Timeout occurred while waiting for response: {}".format(e))
        
        except IOError as e:  
            if e.errno == errno.EPIPE:  
                print("broken pipe: {}".format(e))
      

def send_mc_info(sock):
    id = 1
    while True:
        # 1. init and send data
        data = {"data": "hello from mc", "id":id}
        sock.send((json.dumps(data)).encode())
        id+=1

        # sleep for 20ms (50Hz)
        time.sleep(0.02)

def main():
    # 1. init socket and time out to listen to cpu_sub node
    cpu_sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cpu_sub_socket.settimeout(10.0)
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

    # while True:
    #     pass

if __name__ == "__main__":
    main()