"""@package docstring
Documentation for this module

More details
"""


from asyncio.tasks import sleep
import moteus
from moteus.moteus import Controller
import moteus_pi3hat
import asyncio
from threading import Thread
from threading import Lock
import math
from time import sleep
import traceback
from copy import deepcopy


class Moteus:
    """Class used to manipulate the Moteus motor controllers via CAN using the Pi3Hat
        @author Jared Cohen (cohen.jar@northeastern.edu)
        @date 10/27/2021
    """

    def __init__(self, ids=[[], [], [], [], []], simulation=True):
        """Default constructor for the Moteus class.
            
            Before utilizing motors, make sure to wait using the waitUntilReady() function.
            
            The Moteus class will do a few things on startup.
                - Check all CAN ports for valid connections as described
                    - If there is one error, a MoteusException will be raised. It will then enter simulation mode (described below) if the parameter is True
                - Initialize all motor positions to the current value on the motor. Therefore, if the class was restarted but the motors were not, the position remains the same.
                - Wait until all motors are ready. This will BLOCK code, make sure to accomodate accordingly. This is only done once on startup.
            @param ids These are the CAN ids of the motors attached to the Pi3Hat.
                        Is a list of lists, with each internal list representing the IDs of the motors attatched to that can drive
                        For example, [[],[],[4]] means that on CAN bus 3, there is 1 motor with ID 4 attatched.
                        - All of these IDs must be unique, regardless if they are on the same CAN bus or not.
            @param simulation (NOT IMPLEMENTED YET) This parameter, default True, will bring the class into simulation mode if an error arises on startup
                                Simulation mode will mimic all values set as if there were a motor present, therefore, code can be tested without being connected to hardware
        """
        self.ids = ids
        self.exitFlag = False #Exit flag will tell the inner loop when to leave, otherwise it does not know
        self.isReady = False #This will become true once all the motors are initialized

        self.rawIds = [] #Go through the array and get all the ids in order to make it easier to deal with.
        for bus in self.ids:
            for id in bus:
                self.rawIds.append(id)

        self.motor_states = {} #This will be the position, velocity, and torque to set to each individual motor.
        for id in self.rawIds:
            self.motor_states[id] = {"position" : math.nan, "velocity" : 0, "torque" : 0}

        self.results = None #Set the current results to None, will be updated after initialization

        self.lock = Lock() #Create locks in order to avoid threading issues. This one is for the results,
        self.statesLock = Lock() #This lock is for the states (what to set the motors to)
        self.mainResults = [None] * 1
        self.moteus_thread = Thread(target=self.__moteusMain, args=()) #Create a new thread for the moteus async environment
        self.moteus_thread.start() #Start the thread

        self.waitUntilReady() #Wait until the motors are initialized, blocking
        
        #Process self.mainResults here for errors, to be implemented



    def setAttributes(self, canId, pos=math.nan, velocity=math.nan, torque=math.nan):
        """ Set attributes function is used to set the position, velocity and torque of a specific motor. Note, if this method is called once,
            it will use the motor's built in configuration for max velocity and PID in order to reach that position. Calling this method continuously
            for each motor if a constant velocity is desired is recommended. Read the pos paramter's description for more information on how it reaches the position.

            @param  canID   The ID specific to one motor (Not the bus). Note, if this value is incorrect, an error will be thrown
                            to avoid wasting time checking if the canID is valid every single time
            @param pos      The position for the motor to move to. The motor will proceed to move to this position (using PID) with the maximum velocity defined 
                            in the motor's configuration and with a maximum torque specified by the torque parameter. Defaults to math.nan, do not pass "None".
            @param velocity The velocity the motor will move at after the position is reached, not while moving to the position. Defaults to math.nan, do not pass "None"
            @param  torque  The maximum torque the motor should have when moving to a position.
        """
        self.statesLock.acquire() #Make sure it is not being used, and then set the corresponding values
        self.motor_states[canId]["position"] = pos
        self.motor_states[canId]["velocity"] = velocity
        self.motor_states[canId]["torque"] = torque
        self.statesLock.release()

    def __moteusMain(self):
        """Creates an asyncronous environment and loop in order to host the Moteus package.
            This method contains all of the asyncronous methods that make the moteus work properly and in a non-blocking way

            - createEventLoop() creates a new event loop in order to avoid startup requirements in certain file configurations
            
            - onClose() defined the closing behavior of the motor. In the current implementation, it just makes all of the motors stop where they are.

            - onOpen() defines the opening behavior on startup. It first stops all of the motors and then sets all of the class variables to the current motor positions 
            
            - main() is the main loop in which the majority of the work happens
                - Connects with the motors, raises MoteusException if there is no connection
                - Returns 0 upon succesful completion
                - Checks constantly the classes exitFlag on when to exit
        """

        def createEventLoop(): #Function to create and get a new event loop
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            return asyncio.get_event_loop()

        loop = createEventLoop() #Create an event loop and set the current event loop to it.
        asyncio.set_event_loop(loop)

        async def onClose(transport=None, servos = None): #Called on close, after self.exitFlag is True

            if(transport != None and servos != None): #Go through all motors and stop them
                await transport.cycle([x.make_stop() for x in servos.values()])

        async def onOpen(transport=None, servos = None): #Starts on open
            if(transport != None and servos != None):
                results = await transport.cycle([x.make_stop(query=True) for x in servos.values()])

                results = self.getParsedResultsCustom(results)

                await transport.cycle([
                    x.make_position(
                    position = results[i]["POSITION"],
                    velocity = results[i]["VELOCITY"],
                    maximum_torque = results[i]["TORQUE"]
                    ) for i, x in enumerate(servos.values())
                ])
        
        async def main():

            servo_bus_map = {} #Servo bus map is for the pi3hat router in order to know which motors are on which CAN bus
            for i in range(len(self.ids)): #Go through all of CAN busses
                bus_ids = []
                for id in self.ids[i]: #Go through all of the IDs in the particular bus
                    bus_ids.append(id)
                servo_bus_map[i+1] = bus_ids #Set the bus dictionary index to the ids

            transport = moteus_pi3hat.Pi3HatRouter( #Create a router using the servo bus map
                servo_bus_map = servo_bus_map
            )

            servos = {} #Go through all of the motors and create the moteus.Controller objects associated with them
            for id in self.rawIds:
                servos[id] = moteus.Controller(id = id, transport=transport)


            await onOpen(transport, servos) #Call the onOpen function above.

            raw_ids = self.rawIds #Set the rawIDs so they can be used later for reference. Using local variables where possible saves a sliver of time

            self.isReady = True #Set ready to true so the class can be implemented elsewhere


            while not self.exitFlag: #Loop while the exit method was not called

                self.statesLock.acquire() #Set all of the states to the class variables, and use the lock to avoid issues
                commands = [ #Create an array of moteus commands for each motor
                    servos[id].make_position(
                        position = self.motor_states[id]["position"],
                        velocity = self.motor_states[id]["velocity"],
                        maximum_torque = self.motor_states[id]["torque"],
                        query = True) for id in raw_ids
                ]
                self.statesLock.release() #release the lock

                self.lock.acquire() #Set the results and wait until they are free to write.
                self.results = await transport.cycle(commands) #Cycle through the commands made earlier
                self.lock.release()

                await asyncio.sleep(0.02) #Minimum sleep time in order to make this method thread safe


            await onClose(transport, servos) #Call onClose after the exitFlag is called

        asyncio.run(main()) #Run the main method (blocks until complete)
        self.mainResults[0] = 1 #Set the results to [None], signifying correct exit procedure.

    def closeMoteus(self):
        """ This will safely stop the operation of all of the motors. Will stop the motors regardless of position or current velocity.
            Make sure to account for this.
        """
        self.exitFlag = True
        self.moteus_thread.join()

    def getRawResults(self): 
        """ Returns the raw results straight from the Moteus motors, unparsed. May be faster although it is more of a hastle.

            To be added: Format of the raw data type
        """
        return self.results

    def getParsedResultsCustom(self, results):
        """ Returns a nicely parsed list containing all of the motor's current attributes

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

        parsed = []

        for result in results:
            parsed.append(
                {
                    "MODE" : result.values[0x0],
                    "POSITION" : result.values[0x1],
                    "VELOCITY" : result.values[0x2],
                    "TORQUE" : result.values[0x3],
                    "VOLTAGE": result.values[0x00d],
                    "TEMPERATURE" : result.values[0x00e],
                    "FAULT" : result.values[0x00f]
                }
            )

        return parsed

    def getParsedResults(self):
        """ Returns a nicely parsed list containing all of the motor's current attributes

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
        self.lock.acquire()
        results = deepcopy(self.results) #Deep copy so the lock can be released as soon as possible during parsing as to not block
        self.lock.release()

        parsed = []

        for result in results:
            parsed.append(
                {
                    "MODE" : result.values[0x0],
                    "POSITION" : result.values[0x1],
                    "VELOCITY" : result.values[0x2],
                    "TORQUE" : result.values[0x3],
                    "VOLTAGE": result.values[0x00d],
                    "TEMPERATURE" : result.values[0x00e],
                    "FAULT" : result.values[0x00f]
                }
            )

        return parsed

    def isReady(self):
        """ Gets the property to see if the motors are initialized or not

            @return True if the motors are ready and initialized, false if they are not
        """
        return self.isReady

    def waitUntilReady(self):
        """ This is a blocking function that waits until the motors are fully initialized without errors. It will block the main Thread (aka, where this method was called)
            
            Returns if and only if the motors were deemed ready or there is an error. Eventually, a timeout needs to be implemented
        """
        while((not self.isReady) and (self.mainResults[0] is None)): #Wait until it is ready or there is an error
            sleep(0.1)
        
if __name__ == '__main__':
    m = Moteus(ids=[[],[],[2]])
    to = 0.03
    m.setAttributes(2, pos=100, velocity = 0.2, torque=to)
    torques = []
    while(True):
        value = m.getParsedResults()
        #print(value)
        if(value != None):
            torques.append(value[0]["TORQUE"])
            if(value[0]["POSITION"] >= 100):
                m.setAttributes(2, pos=0, velocity =0.2, torque=to)
            elif(value[0]["POSITION"] <= 0):
                m.setAttributes(2, pos=100, velocity =0.2, torque=to)
            print(m.getRawResults())
        #m.getParsedResults()