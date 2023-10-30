import asyncio


def __simulation_main(self):
	"""Creates an asynchronous environment and loop in order to host the simulation environment.
		This method contains all the asynchronous methods that make the simulation work properly and in a non-blocking way

		- createEventLoop() creates a new event loop in order to avoid startup requirements in certain file configurations

		- onOpen() Sets a default position on startup to 0

		- main() is the main loop in which the majority of the work happens
			- Connects with the motors, raises MoteusException if there is no connection
			- Returns 0 upon successful completion
			- Checks constantly the classes exitFlag on when to exit
	"""

	class SimResults:
		values = {}

		def __init__(self, dic) -> None:
			self.values = dic

	def position_to_result():
		unparsed = []
		for motor in self.motor_states:
			unparsed.append(
				SimResults(
					{
						0x0: "Simulation",
						0x1: self.motor_states[motor]["position"],
						0x2: self.motor_states[motor]["velocity"],
						0x3: self.motor_states[motor]["torque"],
						0x00d: 0,
						0x00e: 0,
						0x00f: []
					})
			)
		return unparsed

	async def on_open():  # Starts on open
		results = position_to_result()
		self.results = get_parsed_results_custom(results)

	async def _main():

		await on_open()  # Call the onOpen function above.

		self.isReady.set()  # Set ready to true so the class can be implemented elsewhere

		while not self.exitFlag:  # Loop while the exit method was not called

			# Set all of the states to the class variables, and use the lock to avoid issues
			unparsed = position_to_result()

			# Set the results and wait until they are free to write.
			self.results = unparsed

			await asyncio.sleep(0.02)  # Minimum sleep time in order to make this method thread safe

	asyncio.run(_main())  # Run the main method (blocks until complete)
