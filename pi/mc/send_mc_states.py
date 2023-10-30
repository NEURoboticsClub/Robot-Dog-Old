import json
import asyncio
import socket
import time

from moteus_controller import MoteusController


async def send_mc_states(mdata: MoteusController, sock: socket.socket):
	mc_id = 1
	loop = asyncio.get_event_loop()
	while True:
		start_time = time.time()
		# 0. get data from the 12 motors
		parsed_res = mdata.get_parsed_results()
		if not parsed_res:
			await asyncio.sleep(0.02)
			continue

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
		mcs12_current = [
			[motor_id + 1,
			 float("{:.4f}".format(parsed_res[0]["POSITION"])),
			 float("{:.4f}".format(parsed["VELOCITY"])),
			 float("{:.4f}".format(parsed["TORQUE"]))
			 ] for motor_id, parsed in enumerate(parsed_res)
		]

		# 3. create a dict
		data = {"mc12": mcs12_current, "id": mc_id}
		mdata.mprint(data)
		jsonify_data = json.dumps(data)
		mdata.mprint(f"MC: sending curr2={mcs12_current[1]}")

		# 4. send as bytes encoded json
		await loop.sock_sendall(sock, jsonify_data.encode())
		mc_id += 1

		# 5. sleep for 20ms so its sending at 50Hz
		sleep_time = time.time()-start_time
		await asyncio.sleep(max(0.02-sleep_time, 0))
