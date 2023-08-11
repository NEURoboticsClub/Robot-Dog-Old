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
import time
from time import sleep
import traceback
from copy import deepcopy

# for bridge nodes sockets
import sys
import socket
import json
import errno

# Get local machine name
SERVER_HOST = socket.gethostname()
MSG_SIZE = 1024
CPU_SUB_SERVER_PORT = 9999
MC_SUB_SERVER_PORT = 9998

from MoteusException import *

class Moteus:
    """Class used to manipulate the Moteus motor controllers via CAN using the Pi3Hat

        Implements functions in order to set the attributes of all the motors as well as get the current states

        Notes:
            - Make sure that the motors CAN properties are configured properly
            - Make sure to handle program the program being killed by using the closeMoteus() method, which is used to clean the threading properly

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
        global print
        self.ids = ids
        self.exitFlag = False #Exit flag will tell the inner loop when to leave, otherwise it does not know
        self.isReady = False #This will become true once all the motors are initialized

        self.rawIds = [] #Go through the array and get all the ids in order to make it easier to deal with.
        for bus in self.ids:
            for id in bus:
                self.rawIds.append(id)

        self.mainResults = []

        if(MoteusCanError.hasDuplicates(self.rawIds)):
            self.mainResults.append(MoteusCanError(self.rawIds, self.ids))
        elif(len(self.ids) > 5):
            self.mainResults.append(MoteusCanError(self.rawIds, self.ids))

        self.motor_states = {} #This will be the position, velocity, and torque to set to each individual motor.
        for id in self.rawIds:
            self.motor_states[id] = {"position" : math.nan, "velocity" : 0, "torque" : 0}

        self.results = None #Set the current results to None, will be updated after initialization

        self.lock = Lock() #Create locks in order to avoid threading issues. This one is for the results,
        self.statesLock = Lock() #This lock is for the states (what to set the motors to)
        print(self.mainResults)
        if(len(self.mainResults) == 0):
            self.moteus_thread = Thread(target=self.__moteusMain, args=()) #Create a new thread for the moteus async environment
            self.moteus_thread.start() #Start the thread

            self.waitUntilReady() #Wait until the motors are initialized, blocking
        
        if(len(self.mainResults) != 0):
            if(not simulation): #If it is not in simulation mode, it prints the first error
                raise self.mainResults[0]
            else: #This is to enter simulation mode
                self.exitFlag = False #Reset all of the variables
                self.isReady = False
                self.mainResults = []

                warnings.warn(None, MoteusWarning) #Create a warning and set the simulation prefix for all prints from now on, to make sure user is aware
                #global print
                print = MoteusWarning.getSimulationPrintFunction()
                
                sleep(2) #Added sleep so the user is aware of the warning since it is easy to miss
                
                self.sim_thread = Thread(target=self.__simulationMain, args=()) #Create a new thread for the moteus async environment
                self.sim_thread.start() #Start the thread

                self.waitUntilReady() #Wait until the sim is initialized, blocking



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
        #print(canId)
        #print(pos)
        self.statesLock.acquire() #Make sure it is not being used, and then set the corresponding values
        self.motor_states[canId]["position"] = pos
        self.motor_states[canId]["velocity"] = velocity
        self.motor_states[canId]["torque"] = torque
        #self.motor_states[canId]['velocity_limit']=velocity_limit
        self.statesLock.release()
    
    async def turnTo(self, canId, pos=0, speed=math.nan, torque=math.nan, tol=0.2):
        """Function that uses setAttributes to turn to a position at a given speed. This function is asynchronous and must be awaited. Will return True once turn is complete.
            Params are same as setAttributes except:
            
            @param pos      The position for the motor to turn to. The motor will turn to this position at the designated speed (with PID). Do not pass math.nan.
                            Defaults to zero.
            @param speed    The speed at which the motor will turn to the given position. Defaults to math.nan, do not pass "None".
            @param tol      The error tolerance of this function. AFTER the position is reached, error will be determined. If error exceeds tolerance, the function will recurse
                            in the opposite direction with the same parameters but at half speed. Note that this means this function will only overshoot, then correct itself for
                            one iteration.
                            
            NOTES:          Do NOT call the function twice concurrently for the same motor. Also, do not call setAttributes while the function runs. It will not break things, but
                            it will cause abnormal behavior. This function should be called and awaited before calling again for the same motor.
                            It will, however, work for multiple motors concurrently.
        """
        params=self.getParsedResults()
        position=params[canId-1]["POSITION"]   #current position
        if (pos-position>=0):                     #determines direction to turn
            direction=1                        #direction indicates direction to turn, positive is clockwise 
        else:
            direction=-1
        self.setAttributes(canId, pos=math.nan, velocity=speed*direction, torque=torque)
        while ((pos-position)*direction>0):       #works until position has increased beyond the called position
            params=self.getParsedResults()
            position=params[canId-1]["POSITION"]
            await asyncio.sleep(0.01)                 #asyncio.sleep pauses execution of the current task and allows the next task to be performed
            #print(position)
        if (abs(pos-position)>tol):               #checks if error exceeds tolerance. Iterates in reverse at half speed if it does.
            await self.turnTo(canId, pos=pos, speed=speed/2, torque=torque, tol=tol)
        else:
            self.setAttributes(canId, pos=math.nan, velocity=0, torque=torque)
            print(position)
            print("Position Reached")
            return True
    

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

        def raiseError(error):
            """
            Safely closes the Thread while appending the error to be raised at a later time

            @param  error   The error to be raised
            """
            self.exitFlag = True
            self.mainResults.append(error)
        
        

        async def onClose(transport=None, servos = None): #Called on close, after self.exitFlag is True

            if(transport != None and servos != None): #Go through all motors and stop them
                
                await transport.cycle([x.make_position(position = math.nan, velocity = 0.1, maximum_torque = 0.23) for x in servos.values()])
                
                sleep(1)
                
                await transport.cycle([x.make_stop() for x in servos.values()])

        async def onOpen(transport=None, servos = None): #Starts on open
            if(transport != None and servos != None):
                results = await transport.cycle([x.make_stop(query=True) for x in servos.values()])
                print([x.make_stop(query=True) for x in servos.values()])
                results = self.getParsedResultsCustom(results)

                try:
                    await transport.cycle([
                        x.make_position(
                        position = results[i]["POSITION"],
                        velocity = results[i]["VELOCITY"],
                        maximum_torque = results[i]["TORQUE"]
                        ) for i, x in enumerate(servos.values())
                    ])
                except IndexError:
                    raiseError(MoteusCanError(self.rawIds, self.ids)) #This error corresponds to 
                
                results = await transport.cycle([x.make_rezero(query=True) for x in servos.values()])

                #await transport.cycle([x.rezero() for x in servos.values()])#Remove Later
                
                
        async def main():

            print("in main")
            servo_bus_map = {} #Servo bus map is for the pi3hat router in order to know which motors are on which CAN bus
            for i in range(len(self.ids)): #Go through all of CAN busses
                bus_ids = []
                for id in self.ids[i]: #Go through all of the IDs in the particular bus
                    bus_ids.append(id)
                servo_bus_map[i+1] = bus_ids #Set the bus dictionary index to the ids

            try:
                transport = moteus_pi3hat.Pi3HatRouter( #Create a router using the servo bus map
                    servo_bus_map = servo_bus_map
                )
            except RuntimeError:
                raiseError(MoteusPermissionsError()) #Raise more descriptive error than what python can describe
                return

            servos = {} #Go through all of the motors and create the moteus.Controller objects associated with them
            for id in self.rawIds:
                servos[id] = moteus.Controller(id = id, transport=transport)

            await onOpen(transport, servos) #Call the onOpen function above.

            raw_ids = self.rawIds #Set the rawIDs so they can be used later for reference. Using local variables where possible saves a sliver of time


            self.isReady = True #Set ready to true so the class can be implemented elsewhere


            while not self.exitFlag: #Loop while the exit method was not called
                #print("looping")
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

    def __simulationMain(self):
        """Creates an asyncronous environment and loop in order to host the simulation environment.
            This method contains all of the asyncronous methods that make the simulation work properly and in a non-blocking way

            - createEventLoop() creates a new event loop in order to avoid startup requirements in certain file configurations

            - onOpen() Sets a default position on startup to 0
            
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

        class sim_results:
            values = {}

            def __init__(self, dic) -> None:
                self.values = dic

        def positionToResult():
            unparsed = []
            for motor in self.motor_states:
                unparsed.append(
                    sim_results(
                    {
                        0x0:"Simulation",
                        0x1:self.motor_states[motor]["position"],
                        0x2:self.motor_states[motor]["velocity"],
                        0x3:self.motor_states[motor]["torque"],
                        0x00d: 0,
                        0x00e: 0,
                        0x00f: []
                    })
                )
            return unparsed

        async def onOpen(): #Starts on open
            results = positionToResult()
            self.results = self.getParsedResultsCustom(results)

        async def main():

            await onOpen() #Call the onOpen function above.

            self.isReady = True #Set ready to true so the class can be implemented elsewhere

            while not self.exitFlag: #Loop while the exit method was not called

                self.statesLock.acquire() #Set all of the states to the class variables, and use the lock to avoid issues
                unparsed = positionToResult()
                self.statesLock.release() #release the lock

                self.lock.acquire() #Set the results and wait until they are free to write.
                self.results = unparsed
                self.lock.release()

                await asyncio.sleep(0.02) #Minimum sleep time in order to make this method thread safe

        asyncio.run(main()) #Run the main method (blocks until complete)

    def closeMoteus(self):
        """ This will safely stop the operation of all of the motors. Will stop the motors regardless of position or current velocity.
            Make sure to account for this.
        """
        self.exitFlag = True
        if(self.sim_thread is None):
            self.moteus_thread.join()
        else:
            self.sim_thread.join()

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
        #print(self.results)
        with Lock():   #fix this later, don't know how much it's locking
            results = deepcopy(self.results)
        #print(results)
        #lock.acquire()
        #print("1")
        #results = deepcopy(self.results) #Deep copy so the lock can be released as soon as possible during parsing as to not block
        #print("2")
        #lock.release()

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
        while((not self.isReady) and (len(self.mainResults) == 0)): #Wait until it is ready or there is an error
            #print(self.isReady)
            #print(self.mainResults)
            #print((not self.isReady) and (len(self.mainResults) == 0))
            sleep(0.1)
            
    async def asyncTest(self):
        print("sleep start")
        await asyncio.sleep(0)
        print('sleeping')
        
    async def jump(self, factor=3, posOffset1=0, posOffset2=0):
        print('crouching')
        crouchtask1=asyncio.create_task(self.turnTo(1, pos=-0.166*10+posOffset1, speed=0.5, torque=1, tol=0.05))
        crouchtask2=asyncio.create_task(self.turnTo(2, pos=2*0.166*10+posOffset2, speed=1, torque=1, tol=0.05))
        await crouchtask1
        await crouchtask2
        params=self.getParsedResults()
        pos1=params[0]["POSITION"]
        pos2=params[1]["POSITION"]
        print(pos1)
        print(pos2)
        #pos2=params(1)["POSITION"]
        while pos1<-0.5+posOffset1 and pos2>0: #and pos2>1.1:
            print('jumping')
            params=self.getParsedResults()
            pos1=params[0]["POSITION"]
            pos2=params[1]["POSITION"]
            self.setAttributes(1,pos=math.nan,velocity=factor*math.sin(-pos1*math.pi/5),torque=10)
            self.setAttributes(2,pos=math.nan,velocity=factor*math.sin(-pos2*math.pi/5),torque=10)
        jumpreturn1=asyncio.create_task(self.turnTo(1, pos=-0.25*10+posOffset1, speed=5, torque=0.5, tol=0.05))
        jumpreturn2=asyncio.create_task(self.turnTo(2, pos=0.25*10+posOffset2, speed=10, torque=0.5, tol=0.05))
        #self.turnTo(2, pos=0.25, speed=1, torque=1, tol=0.05)
        await jumpreturn1
        await jumpreturn2
        self.setAttributes(1,pos=math.nan,velocity=0, torque=2)
        self.setAttributes(2,pos=math.nan,velocity=0, torque=2)
    
    def get_cpu_command(self, sock, m):
        """
        Receive cpu message from bridge nodes using tcp socket
        and use the data to set attributes to the 12 motor controllers
        and save the data to cpu_info queue
        """
        while True:
            try:
                # 0. get message and process
                bytes_msg = sock.recv(MSG_SIZE)

                # 1. exit the loop if no more data to read
                if not bytes_msg:
                    continue

                # 2. convert to json object and get the id and mc12 data
                json_msg = json.loads(bytes_msg)
                msg_id = json_msg["id"]
                mc12 = json_msg["mc12"]

                # 3. skip if not data received yet
                if not mc12:
                    continue
                
                # 4. get data for 2nd motor (index = 1, if there are 12 motors)
                motorIdx = 1
                mc2data = mc12[motorIdx]


                # 5. set attributes
                m.setAttributes(mc2data[0], pos=mc2data[1], velocity = mc2data[2], torque=mc2data[3])
                
                # 6. log
                print("MC: from CPU id={}, m_mc2={}".format(msg_id, mc2data))           
            except KeyError:
                print("Error: 'id' or 'mc12' key not found in JSON data")
            except json.JSONDecodeError:
                print("Error: Invalid JSON data received. Reconnecting...")
            
            except socket.timeout as e:
                print("Timeout occurred while waiting for response: {}".format(e))
            
            except IOError as e:  
                if e.errno == errno.EPIPE:  
                    print("broken pipe: {}".format(e))
      
      
    def send_mc_states(self, sock):
        id = 1
        while True:
            # 0. get data from the 12 motors
            parsedRes = self.getParsedResults()    
                    
            mcs12_current = None
            # 1. if there is only 1 motor connected
            if len(parsedRes) == 1:

                # set all to the same vel and torque of the only motor connected and add to parsedRes
                pos, vel, tor = parsedRes[0]["POSITION"], parsedRes[0]["VELOCITY"], parsedRes[0]["TORQUE"]
                for _ in range(11):
                    parsedRes.append(
                    {
                        "MODE" : 3,
                        "POSITION" : pos,
                        "VELOCITY" : vel,
                        "TORQUE" : tor,
                        "VOLTAGE": 1.0,
                        "TEMPERATURE" : 1.0,
                        "FAULT" : 1.0
                    }
                )

            # 2. set all the 12, use motor id from 1-12 and send to cpu
            mcs12_current = [[id+1, parsedRes[0]["POSITION"], parsed["VELOCITY"], parsed["TORQUE"] ] for id, parsed in enumerate(parsedRes)]
            
            # 3. create a dict
            data = {"mc12": mcs12_current, "id":id}
            
            # 4. send as bytes encoded json
            sock.send((json.dumps(data)).encode())
            id+=1

            # 5. sleep for 20ms so its sending at 50Hz
            time.sleep(0.02)


async def main(m):
    
    #eventLoop=asyncio.get_event_loop()
    value = m.getParsedResults()
    # to = 3                      #0.1 seems to be the lower limit for a stanalone motor. This is max torque.
    # vel = 1
    pos_offset1 = value[0]["POSITION"]           #board can sense where position 0 is via absolute encoder within 1/10 rotation this offset changes where it's zero is
    pos_offset2 = value[0]["POSITION"]
    # print(pos_offset1)
    # print(pos_offset2)
    max_pos = 5 + pos_offset1
    min_pos = 0 + pos_offset1
    # init_pos1 = -10*(math.pi/4)
    # counter = 0
    # oper=1
    # flag=1
    # print(pos_offset1)
    # #jumptask=asyncio.create_task(m.jump(8, pos_offset1, pos_offset2))
    # #await jumptask
    # print(pos_offset2)
    # #transport.cycle([m.pid_position(ilimit=2)])
    # #m.setAttributes(2, pos=math.nan, velocity = vel, torque=to)
    
    # m.velocity_limit = 1
    # torques = []
    # #task0=asyncio.create_task(m.asyncTest())
    # #await m.turnTo(1, pos=0, speed=1, torque=1, tol=0.2)
    # #task2= asyncio.create_task(m.turnTo(1, pos=-5, speed=1, torque=1, tol=0.01))
    # #task1= asyncio.create_task(m.turnTo(1, pos=0.5*10+pos_offset1, speed=1, torque=1, tol=0.01))
    # #await task2
    # #await task1

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
    get_cpu_command_thread = Thread(target=m.get_cpu_command, args=(cpu_sub_socket, m,))
    send_mc_states_thread = Thread(target=m.send_mc_states, args=(mc_sub_socket,))

    # # 4. run
    get_cpu_command_thread.start()
    send_mc_states_thread.start()

    get_cpu_command_thread.join()
    send_mc_states_thread.join()


    # while True:
    #      down1=asyncio.create_task(m.turnTo(1,pos=1.5+pos_offset1,speed=1,torque=0.5,tol=0.1))
    #      down2=asyncio.create_task(m.turnTo(2,pos=-3+pos_offset2,speed=2,torque=0.5,tol=0.1))
    #      print("Going Down")
    #      await down1
    #      await down2
    #      sleep(2)
    #      print("Going Up")
    #      up1=asyncio.create_task(m.turnTo(1,pos=pos_offset1,speed=1,torque=0.5,tol=0.01))
    #      up2=asyncio.create_task(m.turnTo(2,pos=+pos_offset2,speed=2,torque=0.5,tol=0.01))
    #      await up1
    #      await up2
    #      sleep(2)
        
    #m.jump(factor=4,posOffset1=pos_offset1,posOffset2=pos_offset2)

#     down1=asyncio.create_task(m.turnTo(1,pos=1.5+pos_offset1,speed=.5,torque=1,tol=0.1))
#     down2=asyncio.create_task(m.turnTo(2,pos=-3+pos_offset2,speed=1,torque=1,tol=0.1))
#     print("Going Down")
#     await down1
#     await down2
#     sleep(1)
#     print("Going Up")
#     up1=asyncio.create_task(m.turnTo(1,pos=.5+pos_offset1,speed=7,torque=5,tol=0.25))
#     up2=asyncio.create_task(m.turnTo(2,pos=-1+pos_offset2,speed=16,torque=5,tol=0.25))
#     await up1
#     await up2
#     print("Retract Legs")
#     land1=asyncio.create_task(m.turnTo(1,pos=.75+pos_offset1,speed=7,torque=3,tol=0.01))
#     land2=asyncio.create_task(m.turnTo(2,pos=-1.5+pos_offset2,speed=16,torque=3,tol=0.01))
#     await land1
#     await land2
#     sleep(5)
#     stand1=asyncio.create_task(m.turnTo(1,pos=pos_offset1,speed=1,torque=3,tol=0.01))
#     stand2=asyncio.create_task(m.turnTo(2,pos=pos_offset2,speed=2,torque=3,tol=0.01))
#     await stand1
    #await stand2
    #await task1
    
    #await task0
    #await task1
    #await task2
    #done=await task1
    #await task2
    #m.setAttributes(2, pos=math.nan, velocity =-1, torque=to)   #motor 2 is knee, positive velocity is clockwise
    #m.setAttributes(1, pos=math.nan, velocity =0.5, torque=to)  #motor 1 is hip, positive velocity is clockwise
    print(m.getParsedResults())
    while(True):
        try:
            value = m.getParsedResults()
            #print(value)
            #print(flag)
            if(value != None):
                #torques.append(value[0]["TORQUE"])
              #  m.setAttributes(1, pos=None, velocity =oper*vel, torque=to)
                if(value[0]["POSITION"] >= max_pos):
                    flag=-1
                    #m.setAttributes(2, pos=math.nan, velocity =1, torque=to)
                    #m.setAttributes(1, pos=math.nan, velocity =-0.5, torque=to)
                elif(value[0]["POSITION"] <= min_pos):
                    flag=1
                    #m.setAttributes(2, pos=math.nan, velocity =-1, torque=to)
                    #m.setAttributes(1, pos=math.nan, velocity =-0.5, torque=to)
                #if (flag>0):
                    
                #if counter%20==0:
                    #print(m.getParsedResults())
                #counter+=1
        except Exception as e:
            print(e)
            m.closeMoteus()
            print("Moteus Closed Properly")
        #m.getParsedResults()

def closeKey(m):
    m.closeMoteus()
    print("Moteus Closed Properly")


if __name__ == '__main__':
    try:
        m = Moteus(ids=[[],[],[2], [], []], simulation=False)
        asyncio.run(main(m))
    except KeyboardInterrupt:
        closeKey(m)
        sys.exit(0)
    
    
#to add:
#flux braking- moteus defaults to discharging voltage when braking to DC power bus
#servo.flux_brake_min_voltage and servo.flux_brake_resistance_ohm can change this
