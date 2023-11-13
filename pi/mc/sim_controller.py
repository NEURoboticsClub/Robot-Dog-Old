import warnings

from moteus_controller import MoteusController
import asyncio

from MoteusException import MoteusWarning


class SimResults:
	values = {}

	def __init__(self, dic) -> None:
		self.values = dic


class SimController(MoteusController):
	"""Creates an asynchronous environment and loop in order to host the simulation environment.
		This method contains all the asynchronous methods that make the simulation work properly and in a non-blocking way

		- createEventLoop() creates a new event loop in order to avoid startup requirements in certain file configurations

		- onOpen() Sets a default position on startup to 0

		- main() is the main loop in which the majority of the work happens
			- Connects with the motors, raises MoteusException if there is no connection
			- Returns 0 upon successful completion
			- Checks constantly the classes exitFlag on when to exit
	"""

	def position_to_result(self):
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

	async def on_open(self):  # Starts on open
		results = self.position_to_result()
		self.results = self.get_parsed_results_custom(results)

	async def main(self):
		await self.on_open()  # Call the onOpen function above.

		self.isReady.set()  # Set ready to true so the class can be implemented elsewhere

		while not self.exitFlag:  # Loop while the exit method was not called

			# Set all the states to the class variables, and use the lock to avoid issues
			unparsed = self.position_to_result()

			# Set the results and wait until they are free to write.
			self.results = unparsed

			await asyncio.sleep(0.02)  # Minimum sleep time in order to make this method thread safe

	async def run(self):
		self.exitFlag = False  # Reset all the variables
		self.isReady.clear()
		self.mainResults = []

		# Create a warning and set the simulation prefix for all prints from now on, to make sure user is aware
		warnings.warn('', MoteusWarning)
		# global print
		self.mprint = MoteusWarning.get_simulation_print_function()

		await asyncio.sleep(2)  # Added sleep so the user is aware of the warning since it is easy to miss

		self.moteus_task = asyncio.create_task(self.main())  # Create a new thread for the moteus async environment
		# Start the thread

		await self.isReady.wait()  # Wait until the sim is initialized, blocking

		await self.moteus_task  # Run the main method (blocks until complete)
