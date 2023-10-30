import asyncio
import json
import socket
import errno

from moteus_controller import MoteusController

MSG_SIZE = 1024


async def get_cpu_command(sock: socket.socket, _m: MoteusController):
	"""
	Receive command from bridge nodes using tcp socket
	and use the data to set attributes to the 12 motor controllers
	"""
	loop = asyncio.get_event_loop()
	while True:
		try:
			await asyncio.sleep(0.01)
			# 0. get message and process
			bytes_msg = await loop.sock_recv(sock, MSG_SIZE)

			# 1. exit the loop if no more data to read
			if not bytes_msg:
				continue

			# 2. convert to json object and get the id and mc12 data
			json_msg = json.loads(bytes_msg.decode())
			mc12 = json_msg["mc12"]

			# 3. skip if not data received yet
			if not mc12:
				continue

			# 4. get data for 2nd motor (index = 1, if there are 12 motors)
			motor_idx = 1
			mc2data = mc12[motor_idx]

			# 5. set attributes
			_m.set_attributes(mc2data[0], pos=mc2data[1], velocity=mc2data[2], torque=mc2data[3])

			# 6. log
			# print("MC: from CPU id={}, m_mc2={}".format(msg_id, mc2data))
		except KeyError:
			_m.mprint("Error: 'id' or 'mc12' key not found in JSON data")
		except json.JSONDecodeError:
			_m.mprint("Error: Invalid JSON data received. Reconnecting...")

		except socket.timeout as e:
			_m.mprint("Timeout occurred while waiting for response: {}".format(e))

		except IOError as e:
			if e.errno == errno.EPIPE:
				_m.mprint("broken pipe: {}".format(e))
