import socket
import json
import errno
import threading

# Get local machine name
SERVER_HOST = socket.gethostname()
SERVER_PORT = 9999
MSG_SIZE = 1024

def get_cpu_info(cpu_sub):
    # init random id
    id = 1
    single_json = []
    while True:

        try:
            # 1. get message and process
            raw_msg = cpu_sub.recv(MSG_SIZE)
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

                        print("res={}, id={}".format(res, id))
                        id+=1
            else:
                print("no response")
        
        except socket.timeout as e:
            print("Timeout occurred while waiting for response: {}".format(e))
        
        except IOError as e:  
            if e.errno == errno.EPIPE:  
                print("broken pipe: {}".format(e))
      

def main():
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Set a timeout for socket operations
    s.settimeout(10.0)
    # Connection to server on local machine
    s.connect((SERVER_HOST, SERVER_PORT))

    get_cpu_info_thread = threading.Thread(target=get_cpu_info, args=(s,))
    get_cpu_info_thread.start()

    while True:
        pass

if __name__ == "__main__":
    main()