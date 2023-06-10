
from copy import deepcopy
import moteus
import asyncio
import nest_asyncio
import moteus_pi3hat
import warnings
import builtins



from colorama import Fore, init
init()

class MoteusException(Exception):
    """This is the base class for all exceptions that have to do with Moteus motor controllers.
    """
    def __init__(self, message) -> None:
        """The constructor for all MoteusExceptions and child classes. Has a standard output message.

            @param  message  Used to print out the error message. The final printed message is: "Error with the Moteus Controllers: " + message
        """
        self.message = "Error with the Moteus Controllers: " + message
        super().__init__(self.message)

class MoteusPermissionsError(MoteusException):
    """This class is used when the computer does not have correct permissions to use the pi3hat for CAN. Used because the error normally thrown does not offer solutions, wheras since we know the issue we can suggest solutions
    """
    def __init__(self) -> None:
        self.message = "The program does not have access to /dev/mem. Make sure you are you running this on the Raspberry Pi and have root permissions"
        super().__init__(self.message)


class MoteusCanError(MoteusException):
    """The MoteusCanError is a more specific MoteusError that alerts the user there is something wrong with the CAN configuration

        As of now, it detects for three errors: Incorrect IDs/Busses, duplicate IDs, and too many CAN busses. It will automatically detect them.
    """
    def __init__(self, rawIds, ids) -> None:
        """The default constructor for the MoteusCanError. It will automatically detect and output the correct issue, either duplicate IDs, too many CAN busses, or incorrect Can ID/Bus ID

            @param rawIds These are the CAN ids of the motors attached to the Pi3Hat.
                        Is a list of lists, with each internal list representing the IDs of the motors attatched to that can drive
                        For example, [[],[],[4]] means that on CAN bus 3, there is 1 motor with ID 4 attatched.
        
            @param  ids This is a flattened down version of rawIds. For example, [[1,2],[3,4],[5,6]] -> [1,2,3,4,5,6]
        """

        if(len(ids) > 5): ##Check to make sure there is not more than 5 can busses
            self.message = "You cannot have more than 5 CAN busses, since the Pi3hat only has 5." + "\n\t\t\tHere were the ID's passed to the Moteus class: " + ids.__str__()
            super().__init__(self.message)
            return

        if(len(rawIds) != len(set(rawIds))): #Check to make sure there are no duplicate IDs
            self.message = "You cannot have Moteus controllers with the same id, even on separate CAN busses." + "\n\t\t\tHere were the ID's passed to the Moteus class: " + ids.__str__()
            super().__init__(self.message)
            return

        nest_asyncio.apply() #This is to allow to call transport async functions from in here instead of the main loop

        errors = deepcopy(ids) ##Create a copy of the ids and then set the arrays blank in order to keep shape for the returned errors
        for i in range(len(errors)):
            errors[i] = []
        cur_bus = 1
        cur_id = 1

        loop = asyncio.get_event_loop() #get the event loop for later
                

        for _id in rawIds:
            servo_bus_map = {} #Servo bus map is for the pi3hat router in order to know which motors are on which CAN bus

            for i in range(len(ids)): #Go through all of CAN busses
                    bus_ids = []
                    for id in ids[i]: #Go through all of the IDs in the particular bus
                        if(_id) == id:
                            bus_ids.append(id)
                            cur_id = id
                            cur_bus = i
                    servo_bus_map[i+1] = bus_ids #Set the bus dictionary index to the ids

                
            transport = moteus_pi3hat.Pi3HatRouter( #Create a router using the servo bus map
                servo_bus_map = servo_bus_map
            )

            servos = {} #Go through all of the motors and create the moteus.Controller objects associated with them
            for id in rawIds:
                servos[id] = moteus.Controller(id = id, transport=transport)
            

            async def testMotor(): #function in order to test the single motor. If results aren't returned, the ID is incorrect. If it is wrong, the id is appended to the errors list
                results = await transport.cycle([x.make_stop(query=True) for x in servos.values()])
                if(len(results) == 0):
                    errors[cur_bus].append(cur_id)
                
                
            
            loop.run_until_complete(testMotor()) #For each ID, test it to make sure it is error free

        #Create the messsage and call the parent MoteusException class
        self.message = "The following Moteus Controllers could not be detected: " + errors.__str__() + "\n\t\tDouble check that the motors are connected to the correct CAN bus, and that each motor's physical IDs match the ones passed in."
        super().__init__(self.message)

    def hasDuplicates(arr):
        """ This checks a list to see if it has any duplicates. Can be used for any list, but particularly it is meant to make sure there are no duplicate IDs.
        """
        if(len(arr) != len(set(arr))):
            return True
        else:
            return False
        

class MoteusWarning(UserWarning):
    """MoteusWarning class is used as a warning instead of an error. Used for suggestions or if it is entering simulation mode
    """
    def __init__(self, message = None):
        """Default constructor. It will print the message given, or the default as described below

            @param  message Will print the message, if it is left None it prints:
                - "The Moteus initialization encountered errors so it is in Simulation Mode. To see the errors, disable Simulation Mode in the Moteus class constructor by setting simulation = false"
        """
        if(message is None):
            message = "The Moteus initialization encountered errors so it is in Simulation Mode. To see the errors, disable Simulation Mode in the Moteus class constructor by setting simulation = false"
            MoteusWarning.setSimulationPrinting()
        self.message = Fore.LIGHTRED_EX + "Warning: " + Fore.YELLOW +  message + Fore.RESET
        super().__init__(self.message)
        self.originalPrint = print

    def setSimulationPrinting():
        """This function is useful for making all the prints have the prefix "[Simulation Mode]:", although it only works in this thread
            so it is not necessarily useful.
        """
        global print
        def print(*objs, **kwargs):
            my_prefix = Fore.YELLOW + "[Simulation Mode]: " + Fore.RESET
            builtins.print(my_prefix, *objs, **kwargs)

    def getSimulationPrintFunction():
        """This function gets the modified print function with the new simulation prefix

            @return new print function with prefix
        """
        def print(*objs, **kwargs):
            my_prefix = Fore.YELLOW + "[Simulation Mode]: " + Fore.RESET
            builtins.print(my_prefix, *objs, **kwargs)
        return print

    def getOriginalPrint():
        """Returns the original print function

            @return The print function that is normally used with python
        """
        return builtins.print

    def resetPrintFunction():
        """Resets the print function to non-simulation mode if its needed
        """
        global print
        print = builtins.print

def set_highlighted_excepthook():
    """This method simply makes the output of the terminal colorful and easier to look at, makes errors a lot clearer and easier to read

        The downside is if this file is not implemented, this method will also be missing and therefore no pretty colors :(
    """
    import sys, traceback
    from pygments import highlight
    from pygments.lexers import get_lexer_by_name
    from pygments.formatters import TerminalFormatter

    lexer = get_lexer_by_name("pytb" if sys.version_info.major < 3 else "py3tb")
    formatter = TerminalFormatter()

    def myexcepthook(type, value, tb):
        tbtext = ''.join(traceback.format_exception(type, value, tb))
        sys.stderr.write(highlight(tbtext, lexer, formatter))

    sys.excepthook = myexcepthook

set_highlighted_excepthook()

if __name__ == "__main__":
    warnings.warn(None, MoteusWarning)