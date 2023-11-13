"""@package docstring
Documentation for this module

More details
"""

import asyncio

# for bridge nodes sockets
import sys
import socket

from get_cpu_command import get_cpu_command
from motor_controller import MotorController
from send_mc_states import send_mc_states

# Get local machine name
SERVER_HOST = socket.gethostname()
CPU_SUB_SERVER_PORT = 9999
MC_SUB_SERVER_PORT = 9998


async def close_key(m):
	await m.close_moteus()
	m.mprint("Moteus Closed Properly")


async def main(controller: MotorController):
	# to = 3                      #0.1 seems to be the lower limit for a standalone motor. This is max torque.
	# vel = 1

	# board can sense where position 0 is via absolute encoder within 1/10 rotation this offset changes where it's zero
	# is

	# sockets:
	# 1. init socket and time out to listen to cpu_sub node
	cpu_sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	cpu_sub_socket.settimeout(10.0)
	# connect to server
	cpu_sub_socket.connect((SERVER_HOST, CPU_SUB_SERVER_PORT))

	# 2. inis socket and timeout to send msg to mc_sub node
	mc_sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	mc_sub_socket.settimeout(10.0)
	mc_sub_socket.connect((SERVER_HOST, MC_SUB_SERVER_PORT))

	# 3. init thread
	controller_task = asyncio.create_task(controller.run())
	cpu_task = asyncio.create_task(get_cpu_command(cpu_sub_socket, controller))
	mc_task = asyncio.create_task(send_mc_states(controller, mc_sub_socket))

	# # 4. run

	try:
		await asyncio.gather(controller_task, cpu_task, mc_task)
	except KeyboardInterrupt:
		await close_key(controller)
		return

	controller.mprint(controller.get_parsed_results())


if __name__ == '__main__':
	loop = asyncio.get_event_loop()
	m = loop.run_until_complete(MotorController.create(ids=[[], [], [2], [], []]))
	try:
		loop.run_until_complete(main(m))
	except KeyboardInterrupt:
		loop.run_until_complete(close_key(m))

# to add:
# flux braking- moteus defaults to discharging voltage when braking to DC power bus
# servo.flux_brake_min_voltage and servo.flux_brake_resistance_ohm can change this
