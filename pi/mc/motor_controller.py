import asyncio

import moteus
import moteus_pi3hat
from moteus_controller import MoteusController
from MoteusException import MoteusPermissionsError, MoteusCanError


class MotorController(MoteusController):
	async def on_open(self, transport=None, servos=None):  # Starts on open
		if transport is not None and servos is not None:
			results = await transport.cycle([x.make_stop(query=True) for x in servos.values()])
			self.mprint([x.make_stop(query=True) for x in servos.values()])
			results = self.get_parsed_results_custom(results)

			try:
				await transport.cycle([
					x.make_position(
						position=results[i]["POSITION"],
						velocity=results[i]["VELOCITY"],
						maximum_torque=results[i]["TORQUE"]
					) for i, x in enumerate(servos.values())
				])
			except IndexError:
				self.raise_error(await MoteusCanError.create(self.raw_ids, self.ids))  # This error corresponds to

			await transport.cycle([x.make_rezero(query=True) for x in servos.values()])

	async def main(self):
		self.mprint("in main")
		servo_bus_map = {}  # Servo bus map is for the pi3hat router in order to know which motors are on which CAN bus
		for i in range(len(self.ids)):  # Go through all of CAN buses
			bus_ids = []
			for bus_id in self.ids[i]:  # Go through all the IDs in the particular bus
				bus_ids.append(bus_id)
			servo_bus_map[i + 1] = bus_ids  # Set the bus dictionary index to the ids

		try:
			transport = moteus_pi3hat.Pi3HatRouter(  # Create a router using the servo bus map
				servo_bus_map=servo_bus_map
			)
		except RuntimeError:
			self.raise_error(MoteusPermissionsError())  # Raise more descriptive error than what python can describe
			return

		servos = {}  # Go through all the motors and create the moteus.Controller objects associated with them
		for raw_id in self.raw_ids:
			servos[raw_id] = moteus.Controller(id=raw_id, transport=transport)

		await self.on_open(transport, servos)  # Call the onOpen function above.

		# Set the rawIDs so they can be used later for reference. Using local variables where possible saves a sliver of time
		raw_ids = self.raw_ids

		self.isReady.set()  # Set ready to true so the class can be implemented elsewhere

		while not self.exitFlag:  # Loop while the exit method was not called
			# print("looping")
			# Set all of the states to the class variables, and use the lock to avoid issues
			commands = [  # Create an array of moteus commands for each motor
				servos[raw_id].make_position(
					position=self.motor_states[raw_id]["position"],
					velocity=self.motor_states[raw_id]["velocity"],
					maximum_torque=self.motor_states[raw_id]["torque"],
					query=True) for raw_id in raw_ids
			]

			# Set the results and wait until they are free to write.
			self.results = await transport.cycle(commands)  # Cycle through the commands made earlier

			await asyncio.sleep(0.02)  # Minimum sleep time in order to make this method thread safe

		await self.on_close(transport, servos)  # Call onClose after the exitFlag is called

	async def run(self):
		if len(self.mainResults) == 0:
			self.moteus_task = asyncio.create_task(self.main())
			await self.isReady.wait()  # Wait until the motors are initialized, blocking

		if len(self.mainResults) != 0:
			# If it is not in simulation mode, it prints the first error
			# it's not
			raise self.mainResults[0]
