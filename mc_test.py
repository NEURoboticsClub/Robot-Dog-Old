import socket
import json
import errno
import threading

# Get local machine name
SERVER_HOST = socket.gethostname()
MSG_SIZE = 1024
CPU_SUB_SERVER_PORT = 9999
MC_SUB_SERVER_PORT = 9998

def get_cpu_info(sock):
    # init random id
    id = 1
    single_json = []
    while True:

        try:
            # 1. get message and process
            raw_msg = sock.recv(MSG_SIZE)
            if raw_msg:

                # 2. split msg by newline
                messages = raw_msg.split("\n")
                for single_line in messages:
                    
                    # 3. keep storing to buffer
                    single_json.append(single_line)

                    # 4. if this is the end, of a single json
                    if single_line and single_line[-1] == "}":

                        # join as string
                        res = "".join(single_json)

                        # ready for next json
                        single_json = []

                        print("Received from cpu_sub={}, id={}".format(res, id))
                        id+=1
            else:
                print("no response")
        
        except socket.timeout as e:
            print("Timeout occurred while waiting for response: {}".format(e))
        
        except IOError as e:  
            if e.errno == errno.EPIPE:  
                print("broken pipe: {}".format(e))
      

def send_mc_info(sock):
    id = 1
    while True:
        # 1. init and send data
        data = {"data": "mc_test", "id":id}
        sock.send(json.dumps(data))

        # 2 log
        # print("Sent: {}, id={}".format(data,id))
        id+=1

def main():
    # 1. init socket and time out to listen to cpu_sub node
    cpu_sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cpu_sub_socket.settimeout(10.0)
    cpu_sub_socket.connect((SERVER_HOST, CPU_SUB_SERVER_PORT))

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

    while True:
        pass

if __name__ == "__main__":
    main()