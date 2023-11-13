import abc
import asyncio
import math
from copy import deepcopy
from MoteusException import MoteusCanError


class MoteusController(abc.ABC):
	"""Class used to manipulate the Moteus motor controllers via CAN using the Pi3Hat

		Implements functions in order to set the attributes of all the motors as well as get the current states

		Notes:
			- Make sure that the motors CAN properties are configured properly
			- Make sure to handle program the program being killed by using the closeMoteus() method,
			which is used to clean the threading properly

		@author Jared Cohen (cohen.jar@northeastern.edu)
		@date 10/27/2021
	"""

	def __init__(self, ids):
		"""Default constructor for the Moteus class.

			Before utilizing motors, make sure to wait using the waitUntilReady() function.

			The Moteus class will do a few things on startup.
				- Check all CAN ports for valid connections as described
					- If there is one error, a MoteusException will be raised.
					It will then enter simulation mode (described below) if the parameter is True
				- Initialize all motor positions to the current value on the motor.
				Therefore, if the class was restarted but the motors were not, the position remains the same.
				- Wait until all motors are ready. This will BLOCK code, make sure to accommodate accordingly.
				This is only done once on startup.
			@param ids These are the CAN ids of the motors attached to the Pi3Hat.
				Is a list of lists, with each internal list representing the IDs of the motors attached to that can
				drive
				 For example, [[],[],[4]] means that on CAN bus 3, there is 1 motor with ID 4 attached.
				   - All of these IDs must be unique, regardless if they are on the same CAN bus or not.
			simulation (NOT IMPLEMENTED YET) This parameter, default True,
				will bring the class into simulation mode if an error arises on startup
								Simulation mode will mimic all values set as if there were a motor present, therefore,
								code can be tested without being connected to hardware
		"""

		self.ids = ids
		self.exitFlag = False  # Exit flag will tell the inner loop when to leave, otherwise it does not know
		self.isReady = asyncio.Event()  # This will become true once all the motors are initialized

		self.raw_ids = []  # Go through the array and get all the ids in order to make it easier to deal with.
		for bus in self.ids:
			for raw_id in bus:
				self.raw_ids.append(raw_id)

		self.mainResults = []

		self.results = None  # Set the current results to None, will be updated after initialization
		self.motor_states = None
		self.mprint = print
		self.moteus_task = None

	@classmethod
	async def create(cls, ids=[[], [], [], [], []]):
		if cls is MoteusController:
			raise TypeError("create() must be called on a subclass")
		self = cls(ids)
		if MoteusCanError.has_duplicates(self.raw_ids):
			self.mainResults.append(await MoteusCanError.create(self.raw_ids, self.ids))
		elif len(self.ids) > 5:
			self.mainResults.append(await MoteusCanError.create(self.raw_ids, self.ids))

		# This will be the position, velocity, and torque to set to each individual motor.
		self.motor_states = {raw_id: {"position": math.nan, "velocity": 0, "torque": 0} for raw_id in self.raw_ids}

		self.mprint(self.mainResults)
		return self

	"""
		Creates an asynchronous environment and loop in order to host the Moteus package.
	    This method contains all of the asynchronous methods that make the moteus work properly and in a non-blocking way

	    - createEventLoop() creates a new event loop in order to avoid startup requirements in certain file configurations

	    - onClose() defined the closing behavior of the motor. 
	    In the current implementation, it just makes all of the motors stop where they are.

	    - onOpen() defines the opening behavior on startup. 
	    It first stops all of the motors and then sets all of the class variables to the current motor positions

	    - main() is the main loop in which the majority of the work happens
	        - Connects with the motors, raises MoteusException if there is no connection
	        - Returns 0 upon successful completion
	        - Checks constantly the classes exitFlag on when to exit
	"""

	def raise_error(self, error):
		"""
			Safely closes the Thread while appending the error to be raised at a later time
			@param  error   The error to be raised
		"""
		self.exitFlag = True
		self.mainResults.append(error)

	@staticmethod
	async def on_close(transport=None, servos=None):  # Called on close, after mdata.exitFlag is True

		if transport is not None and servos is not None:  # Go through all motors and stop them

			await transport.cycle(
				[x.make_position(position=math.nan, velocity=0.1, maximum_torque=0.23) for x in servos.values()])

			await asyncio.sleep(1)

			await transport.cycle([x.make_stop() for x in servos.values()])

	@abc.abstractmethod
	async def main(self):
		pass

	@abc.abstractmethod
	async def run(self):
		pass

	def set_attributes(self, can_id, pos=math.nan, velocity=math.nan, torque=math.nan):
		""" Set attributes function is used to set the position, velocity and torque of a specific motor.
			Note, if this method is called once, it will use the motor's built in configuration for max velocity and PID
			in order to reach that position. Calling this method continuously for each motor if a constant velocity is
			desired is recommended. Read the pos parameter's description for more information on how it reaches the
			position.

			@param  can_id  The ID specific to one motor (Not the bus). Note, if this value is incorrect, an error will
							be thrown to avoid wasting time checking if the canID is valid every single time
			@param pos      The position for the motor to move to. The motor will proceed to move to this position
							(using PID) with the maximum velocity defined in the motor's configuration and with a
							maximum torque specified by the torque parameter. Defaults to math.nan, do not pass "None".
			@param velocity The velocity the motor will move at after the position is reached, not while moving to the
							position. Defaults to math.nan, do not pass "None"
			@param  torque  The maximum torque the motor should have when moving to a position.
		"""
		# Make sure it is not being used, and then set the corresponding values

		self.motor_states[can_id]["position"] = pos
		self.motor_states[can_id]["velocity"] = int(velocity)
		self.motor_states[can_id]["torque"] = int(torque)

	# self.motor_states[canId]['velocity_limit']=velocity_limit

	async def turn_to(self, can_id, pos=0, speed=math.nan, torque=math.nan, tol=0.2):
		"""Function that uses setAttributes to turn to a position at a given speed. This function is asynchronous and
		must be awaited. Will return True once turn is complete.
			Params are same as setAttributes except:

			@param can_id   The can id
			@param pos      The position for the motor to turn to. The motor will turn to this position at the
							designated speed (with PID). Do not pass math.nan. Defaults to zero.
			@param speed    The speed at which the motor will turn to the given position. Defaults to math.nan, do not
							pass "None".
			@param torque   The motor's torque. Defaults to math.nan
			@param tol      The error tolerance of this function. AFTER the position is reached, error will be
							determined. If error exceeds tolerance, the function will recurse in the opposite direction
							with the same parameters but at half speed. Note that this means this function will only
							overshoot, then correct itself for one iteration.

			NOTES:          Do NOT call the function twice concurrently for the same motor. Also, do not call
							setAttributes while the function runs. It will not break things, but it will cause abnormal
							behavior. This function should be called and awaited before calling again for the same
							motor. It will, however, work for multiple motors concurrently.
		"""
		params = self.get_parsed_results()
		position = params[can_id - 1]["POSITION"]  # current position
		if pos - position >= 0:  # determines direction to turn
			direction = 1  # direction indicates direction to turn, positive is clockwise
		else:
			direction = -1
		self.set_attributes(can_id, pos=math.nan, velocity=speed * direction, torque=torque)
		while (pos - position) * direction > 0:  # works until position has increased beyond the called position
			params = self.get_parsed_results()
			position = params[can_id - 1]["POSITION"]
			await asyncio.sleep(
				0.01)  # asyncio.sleep pauses execution of the current task and allows the next task to be performed
		# print(position)

		# checks if error exceeds tolerance. Iterates in reverse at half speed if it does.
		if abs(pos - position) > tol:
			await self.turn_to(can_id, pos=pos, speed=speed / 2, torque=torque, tol=tol)
		else:
			self.set_attributes(can_id, pos=math.nan, velocity=0, torque=torque)
			self.mprint(position)
			self.mprint("Position Reached")
			return True

	async def close_moteus(self):
		""" This will safely stop the operation of all the motors. Will stop the motors regardless of position or
			current velocity.
			Make sure to account for this.
		"""
		self.exitFlag = True
		await self.moteus_task

	def get_raw_results(self):
		"""
		Returns the raw results straight from the Moteus motors, unparsed. May be faster, although it is more of a
		hassle.
		To be added: Format of the raw data type
		"""
		return self.results

	@staticmethod
	def get_parsed_results_custom(results):
		""" Returns a nicely parsed list containing all the motor's current attributes

			@param  results A specific set of results from the motors to be parsed

			@return A list containing dictionaries with each motor's attributes. -> [{motor1 data}, {motor2 data}, ...]
					Contained (also the names of each key inside each dictionary):
						- MODE
						- POSITION
						- VELOCITY
						- TORQUE
						- VOLTAGE
						- TEMPERATURE
						- FAULT

		"""

		return [{
			"MODE": result.values[0x0],
			"POSITION": result.values[0x1],
			"VELOCITY": result.values[0x2],
			"TORQUE": result.values[0x3],
			"VOLTAGE": result.values[0x00d],
			"TEMPERATURE": result.values[0x00e],
			"FAULT": result.values[0x00f]
		}
			for result in results]

	def get_parsed_results(self):
		""" Returns a nicely parsed list containing all the motor's current attributes

			@return A list containing dictionaries with each motor's attributes. -> [{motor1 data}, {motor2 data}, ...]
					Contained (also the names of each key inside each dictionary):
						- MODE
						- POSITION
						- VELOCITY
						- TORQUE
						- VOLTAGE
						- TEMPERATURE
						- FAULT

		"""
		# print(self.results)
		# fix this later, don't know how much it's locking
		if not self.results:
			return []
		results = deepcopy(self.results)

		parsed = []

		for result in results:
			parsed.append(
				{
					"MODE": result.values[0x0],
					"POSITION": result.values[0x1],
					"VELOCITY": result.values[0x2],
					"TORQUE": result.values[0x3],
					"VOLTAGE": result.values[0x00d],
					"TEMPERATURE": result.values[0x00e],
					"FAULT": result.values[0x00f]
				}
			)
		return parsed

	def is_ready(self):
		""" Gets the property to see if the motors are initialized or not

			@return True if the motors are ready and initialized, false if they are not
		"""
		return self.isReady.is_set()

	# Wait until it is ready or there is an error

	async def jump(self, factor=3, pos_offset_1=0, pos_offset_2=0):
		self.mprint('crouching')
		crouchtask1 = asyncio.create_task(
			self.turn_to(1, pos=int(-0.166 * 10 + pos_offset_1), speed=0.5, torque=1, tol=0.05))
		crouchtask2 = asyncio.create_task(
			self.turn_to(2, pos=int(2 * 0.166 * 10 + pos_offset_2), speed=1, torque=1, tol=0.05))
		await crouchtask1
		await crouchtask2
		params = self.get_parsed_results()
		pos1 = params[0]["POSITION"]
		pos2 = params[1]["POSITION"]
		self.mprint(pos1)
		self.mprint(pos2)
		while pos1 < -0.5 + pos_offset_1 and pos2 > 0:  # and pos2>1.1:
			self.mprint('jumping')
			params = self.get_parsed_results()
			pos1 = params[0]["POSITION"]
			pos2 = params[1]["POSITION"]
			self.set_attributes(1, pos=math.nan, velocity=factor * math.sin(-pos1 * math.pi / 5), torque=10)
			self.set_attributes(2, pos=math.nan, velocity=factor * math.sin(-pos2 * math.pi / 5), torque=10)
		jumpreturn1 = asyncio.create_task(
			self.turn_to(1, pos=int(-0.25 * 10 + pos_offset_1), speed=5, torque=0.5, tol=0.05))
		jumpreturn2 = asyncio.create_task(
			self.turn_to(2, pos=int(0.25 * 10 + pos_offset_2), speed=10, torque=0.5, tol=0.05))
		await jumpreturn1
		await jumpreturn2
		self.set_attributes(1, pos=math.nan, velocity=0, torque=2)
		self.set_attributes(2, pos=math.nan, velocity=0, torque=2)
