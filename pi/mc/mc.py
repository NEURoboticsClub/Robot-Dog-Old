import asyncio
import errno
import json
import random
import socket
import signal

from sim_controller import SimController

# Get local machine name
SERVER_HOST = socket.gethostname()
MSG_SIZE = 1024
CPU_SUB_SERVER_PORT = 9999
MC_SUB_SERVER_PORT = 9998


def get_parsed_results():
	return [{
		"MODE": 3,
		"POSITION": random.uniform(1.0, 3.0),
		"VELOCITY": 2.2,
		"TORQUE": 2.2,
		"VOLTAGE": 2.2,
		"TEMPERATURE": 1.0,
		"FAULT": 1.0
	}]


async def get_cpu_command(sock):
	loop = asyncio.get_event_loop()
	while True:
		try:
			# 0. get message and process
			bytes_msg = await loop.sock_recv(sock, MSG_SIZE)

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


async def send_mc_states(sock):
	mc_id = 1
	loop = asyncio.get_event_loop()
	while True:
		# 0. get data from the 12 controllers
		# if connected to 1 motor, there's only 1 element
		parsed_res = get_parsed_results()

		# 1. if there is only 1 motor connected
		if len(parsed_res) == 1:
			# set all to the same vel and torque of the only motor connected and add to parsedRes
			pos, vel, tor = parsed_res[0]["POSITION"], parsed_res[0]["VELOCITY"], parsed_res[0]["TORQUE"]
			parsed_res += [{
				"MODE": 3,
				"POSITION": pos,
				"VELOCITY": vel,
				"TORQUE": tor,
				"VOLTAGE": 1.0,
				"TEMPERATURE": 1.0,
				"FAULT": 1.0
			}] * 11

		# 2. set all the 12, use motor id from 1-12 and send to cpu
		mcs12_current = [[motor_id + 1, parsed_res[0]["POSITION"], parsed["VELOCITY"], parsed["TORQUE"]] for
						 motor_id, parsed in
						 enumerate(parsed_res)]

		# 3. create a dict
		data = {"mc12": mcs12_current, "id": mc_id}

		# 4. send as bytes encoded json
		await loop.sock_sendall(sock, json.dumps(data).encode())
		mc_id += 1

		# 5. sleep for 20ms so its sending at 50Hz
		await asyncio.sleep(0.02)


async def close_key(m):
	await m.close_moteus()
	m.mprint("Moteus Closed Properly")


async def main(controller: SimController):
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
	get_cpu_command_task = asyncio.create_task(get_cpu_command(cpu_sub_socket))
	send_mc_states_task = asyncio.create_task(send_mc_states(mc_sub_socket))
	controller_task = asyncio.create_task(controller.run())

	# 4. run
	await asyncio.gather(controller_task, get_cpu_command_task, send_mc_states_task)


if __name__ == '__main__':
	loop = asyncio.get_event_loop()
	m = loop.run_until_complete(SimController.create())
	try:
		loop.run_until_complete(main(m))
	except KeyboardInterrupt:
		loop.run_until_complete(close_key(m))
