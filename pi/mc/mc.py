import errno
import json
import random
import socket
import threading
import time


# Get local machine name
SERVER_HOST = socket.gethostname()
MSG_SIZE = 1024
CPU_SUB_SERVER_PORT = 9999
MC_SUB_SERVER_PORT = 9998

def get_parsed_results():
        parsed = []
        totalConnectedMotors = 1
        for _ in range(totalConnectedMotors):
            parsed.append(
                {
                    "MODE" : 3,
                    "POSITION" : random.uniform(1.0, 3.0),
                    "VELOCITY" : 2.2,
                    "TORQUE" : 2.2,
                    "VOLTAGE": 2.2,
                    "TEMPERATURE" : 1.0,
                    "FAULT" : 1.0
                }
            )
        return parsed

def get_cpu_command(sock):
    while True:
        try:
            # 0. get message and process
            bytes_msg = sock.recv(MSG_SIZE)

            # 1. exit the loop if no more data to read
            if not bytes_msg:
                continue

            # 2. cconvert to json object and get the id and mc12 data
            json_msg = json.loads(bytes_msg)
            msg_id = json_msg["id"]
            mc12 = json_msg["mc12"]
            

            # 3. skip if not data received yet
            if not mc12:
                continue
            
            # OPTION 1: If there is only 1 motor
            # - get data for 2nd motor (1st index) we are getting 12 datas from cpu
            mc2data = mc12[1]

            # -set attributes for motor 2 only
            # m.setAttributes(mc2data[0], pos=mc2data[1], velocity = mc2data[2], torque=mc2data[3])

            # OPTION 2: there are 12 motors to set

            # 6. log
            print("MC: from CPU id={}, m_mc2={}".format(msg_id, mc2data))
        
        except KeyError:
                print("Error: 'id' or 'mc12' key not found in JSON data")
        except json.JSONDecodeError:
            print("Error: Invalid JSON data received. Reconnecting...")
        
        except socket.timeout as e:
            print("Timeout occurred while waiting for response: {}".format(e))
        
        except IOError as e:  
            if e.errno == errno.EPIPE:  
                print("broken pipe: {}".format(e))
    
      

def send_mc_states(sock):
        
    id = 1
    while True:
        # 0. get data from the 12 controllers
        # if connected to 1 motor, there's only 1 element
        parsedRes = get_parsed_results()    
                
        mcs12_current = None
        # 1. if there is only 1 motor connected
        if len(parsedRes) == 1:

            # set all to the same vel and torque of the only motor connected and add to parsedRes
            pos, vel, tor = parsedRes[0]["POSITION"], parsedRes[0]["VELOCITY"], parsedRes[0]["TORQUE"]
            for _ in range(11):
                parsedRes.append(
                {
                    "MODE" : 3,
                    "POSITION" : pos,
                    "VELOCITY" : vel,
                    "TORQUE" : tor,
                    "VOLTAGE": 1.0,
                    "TEMPERATURE" : 1.0,
                    "FAULT" : 1.0
                }
            )

        # 2. set all the 12, use motor id from 1-12 and send to cpu
        mcs12_current = [[id+1, parsedRes[0]["POSITION"], parsed["VELOCITY"], parsed["TORQUE"] ] for id, parsed in enumerate(parsedRes)]
        
        # 3. create a dict
        data = {"mc12": mcs12_current, "id":id}

        # 4. send as bytes encoded json
        sock.send((json.dumps(data)).encode())
        id+=1

        # 5. sleep for 20ms so its sending at 50Hz
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
    get_cpu_command_thread = threading.Thread(target=get_cpu_command, args=(cpu_sub_socket,))
    send_mc_states_thread = threading.Thread(target=send_mc_states, args=(mc_sub_socket,))
    
    # 4. run
    get_cpu_command_thread.start()
    send_mc_states_thread.start()

    get_cpu_command_thread.join()
    send_mc_states_thread.join()


if __name__ == "__main__":
    main()