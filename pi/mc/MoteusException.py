import asyncio
import builtins
import warnings
from copy import deepcopy

import moteus
import moteus_pi3hat
from colorama import Fore, init

exec_print = print

init()


class MoteusException(Exception):
	"""This is the base class for all exceptions that have to do with Moteus motor controllers."""

	def __init__(self, message) -> None:
		"""The constructor for all MoteusExceptions and child classes. Has a standard output message.

		@param  message  Used to print out the error message. The final printed message is:
		"Error with the Moteus Controllers: " + message
		"""
		self.message = "Error with the Moteus Controllers: " + message
		super().__init__(self.message)


class MoteusPermissionsError(MoteusException):
	"""This class is used when the computer does not have correct permissions to use the pi3hat for CAN.
	Used because the error normally thrown does not offer solutions,
	whereas since we know the issue we can suggest solutions
	"""

	def __init__(self) -> None:
		self.message = (
			"The program does not have access to /dev/mem. "
			"Make sure you are you running this on the Raspberry Pi and have root permissions"
		)
		super().__init__(self.message)


class MoteusCanError(MoteusException):
	"""
	The MoteusCanError is a more specific MoteusError that alerts the user there is something wrong with the CAN
	configuration

	As of now, it detects for three errors: Incorrect IDs/Buses, duplicate IDs, and too many CAN buses.
	It will automatically detect them.
	"""

	def __init__(self, message):
		"""
		The default constructor for the MoteusCanError. It will automatically detect and output the correct issue,
		either duplicate IDs, too many CAN buses, or incorrect Can ID/Bus ID

			@param raw_ids These are the CAN ids of the motors attached to the Pi3Hat.
						Is a list of lists, with each internal list representing the IDs of the motors attached to that
						can drive
						For example, [[],[],[4]] means that on CAN bus 3, there is 1 motor with ID 4 attached.

			@param  ids This is a flattened down version of rawIds. For example, [[1,2],[3,4],[5,6]] -> [1,2,3,4,5,6]
		"""
		super().__init__(message)

	@classmethod
	async def create(cls, raw_ids, ids):
		message = ''
		# Check to make sure there is not more than 5 can buses
		if len(ids) > 5:
			message = (
					"You cannot have more than 5 CAN busses, since the Pi3hat only has 5."
					+ "\n\t\t\tHere were the ID's passed to the Moteus class: "
					+ ids.__str__()
			)
			return MoteusCanError(message)

		if len(raw_ids) != len(
				set(raw_ids)
		):  # Check to make sure there are no duplicate IDs
			message = (
					"You cannot have Moteus controllers with the same id, even on separate CAN busses."
					+ "\n\t\t\tHere were the ID's passed to the Moteus class: "
					+ ids.__str__()
			)
			return MoteusCanError(message)

		errors = deepcopy(
			ids
		)  # #Create a copy of the ids and then set the arrays blank in order to keep shape for the returned
		# errors
		for i in range(len(errors)):
			errors[i] = []
		cur_bus = 1
		cur_id = 1

		for raw_id in raw_ids:
			servo_bus_map = (
				{}
			)  # Servo bus map is for the pi3hat router in order to know which motors are on which
			# CAN bus

			for i in range(len(ids)):  # Go through all of CAN buses
				bus_ids = []
				for bus_id in ids[i]:  # Go through all the IDs in the particular bus
					if raw_id == bus_id:
						bus_ids.append(bus_id)
						cur_id = bus_id
						cur_bus = i
				servo_bus_map[
					i + 1
					] = bus_ids  # Set the bus dictionary index to the ids

			transport = (
				moteus_pi3hat.Pi3HatRouter(  # Create a router using the servo bus map
					servo_bus_map=servo_bus_map
				)
			)

			servos = (
				{}
			)  # Go through all the motors and create the moteus.Controller objects associated with them
			for bus_id in raw_ids:
				servos[bus_id] = moteus.Controller(id=bus_id, transport=transport)

			# function in order to test the single motor. If results aren't returned, the ID is incorrect.
			# If it is wrong, the id is appended to the errors list

			results = await transport.cycle(
				[x.make_stop(query=True) for x in servos.values()]
			)
			if len(results) == 0:
				errors[cur_bus].append(cur_id)

		# For each ID, test it to make sure it is error free

		# Create the message and call the parent MoteusException class
		message = (
				"The following Moteus Controllers could not be detected: "
				+ errors.__str__()
				+ "\n\t\tDouble check that the motors are connected to the correct CAN bus, "
				  "and that each motor's physical IDs match the ones passed in."
		)
		return MoteusCanError(message)

	@staticmethod
	def has_duplicates(arr):
		"""This checks a list to see if it has any duplicates. Can be used for any list, but particularly it is meant
		to make sure there are no duplicate IDs.
		"""
		return len(arr) != len(set(arr))


class MoteusWarning(UserWarning):
	"""MoteusWarning class is used as a warning instead of an error. Used for suggestions or if it is entering
	simulation mode
	"""

	def __init__(self, message=None):
		"""Default constructor. It will print the message given, or the default as described below

		@param message
		Will print the message, if it is left None it prints: - "The Moteus initialization
		encountered errors so it is in Simulation Mode. To see the errors, disable Simulation Mode in the Moteus
		class constructor by setting simulation = false"
		"""
		if message is None:
			message = (
				"The Moteus initialization encountered errors so it is in Simulation Mode. To see the errors, "
				"disable Simulation Mode in the Moteus class constructor by setting simulation = false"
			)
			MoteusWarning.set_simulation_printing()
		self.message = (
				Fore.LIGHTRED_EX + "Warning: " + Fore.YELLOW + message + Fore.RESET
		)
		super().__init__(self.message)
		self.originalPrint = print

	@staticmethod
	def set_simulation_printing():
		"""This function is useful for making all the prints have the prefix "[Simulation Mode]:",
		although it only works in this thread so it is not necessarily useful.
		"""
		global exec_print

		def new_print(*objs, **kwargs):
			my_prefix = Fore.YELLOW + "[Simulation Mode]: " + Fore.RESET
			builtins.print(my_prefix, *objs, **kwargs)

		exec_print = new_print

	@staticmethod
	def get_simulation_print_function():
		"""This function gets the modified print function with the new simulation prefix

		@return new print function with prefix
		"""

		def new_print(*objs, **kwargs):
			my_prefix = Fore.YELLOW + "[Simulation Mode]: " + Fore.RESET
			builtins.print(my_prefix, *objs, **kwargs)

		return new_print

	@staticmethod
	def get_original_print():
		"""Returns the original print function

		@return The print function that is normally used with python
		"""
		return builtins.print

	@staticmethod
	def reset_print_function():
		"""Resets the print function to non-simulation mode if its needed"""
		global exec_print
		exec_print = builtins.print


def set_highlighted_excepthook():
	"""
	This method simply makes the output of the terminal colorful and easier to look at,
	makes errors a lot clearer and easier to read

	The downside is if this file is not implemented, this method will also be missing and therefore no pretty colors :(
	"""
	import sys
	import traceback
	from pygments import highlight
	from pygments.lexers import get_lexer_by_name
	from pygments.formatters import TerminalFormatter

	lexer = get_lexer_by_name("pytb" if sys.version_info.major < 3 else "py3tb")
	formatter = TerminalFormatter()

	def myexcepthook(ex_type, value, tb):
		tbtext = "".join(traceback.format_exception(ex_type, value, tb))
		sys.stderr.write(highlight(tbtext, lexer, formatter))

	sys.excepthook = myexcepthook


set_highlighted_excepthook()

if __name__ == "__main__":
	warnings.warn("", MoteusWarning)
