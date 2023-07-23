# ros-docker
# INTRODUCTION: 
This repo contain a communication framework that will a part of robot dog project in this repo:  
https://github.com/NEURoboticsClub/NU-Dog/tree/main  

Our motor controller (MC) program moteus_test.py is a program that will control the 12 motors that are running the robot's movement. We will be sending ROS commands to MC from our laptop using the script in cpu.py.  
So we need another ROS node inside our Raspberry pi to receive these commands and pass it to MC. However, there is an incompatibility issue with Raspberry Pi used in this project and the ROS version that we want to use which is ROS melodic.  

So the objective is to set up a communication framework so that a ROS node in our machine (cpu.py) can communicate with the motor controller program (moteus_test.py) in Raspberry Pi.  

This framework will solve the incompatibility issue by leveraging Docker containers.  

### INTRO: How does this work?  
We will run what we will call the “bridge” nodes which are ROS nodes that will be running in the Docker environment with ROS package and other dependencies installed. These nodes do not do anything other than passing messages back and forth between MC and CPU. They are multi-threaded so that they can listen and publish messages either using Python sockets or use ROS built-in publisher-subscriber framework concurrently. 

### INTRO: What actually happens when we start running these nodes?  

Take a look at the diagram below which illustrates the high level overview of the messages that are passed.  

<img width="1157" alt="Screenshot 2023-07-13 at 2 42 40 PM" src="https://github.com/freecode23/ros-docker/assets/67333705/8d88c263-55ef-4767-9753-e63fb5ea1e48"> 

1. The cpu and ROS master node will start and the cpu node will send the initial command to MC using the ROS publisher framework. This message will be received by cpu_sub.py which is one of the bridge nodes that subscribes to the topic published by cpu.  

2. Cpu_sub will then pass on to MC using socket. MC will receive these bytes messages, convert them to JSON and set the attributes of the 12 motors. The attributes are: motor id, velocity, position, and torques.  

3. MC will call getParsedResults() method, which will grab the current motor attributes at real time and send them to mc_sub bridge node via socket.  

4. MC_sub bridge node will publish messages on MC topic so that its subscriber (the cpu node) can listen and invoke its callback function and do whatever it wants to do with this new motor attributes info.  

# QUICK START 
## PREREQUSITES:  
1. Docker and docker compose installed in your machine and PI  
Follow the link below to install docker compose in linux:  
https://docs.docker.com/compose/install/linux/#install-using-the-repository  

2. Python3 installed in PI  
## Step 0: Prepare the workspace directory in both machine:  
git clone this repo into your machine (your laptop).  
git clone this repo into your Raspberry Pi ( if you also need to run pi)

## Step 1: Setting up the network:   
### Step 1.1 Setting up the network in CPU: 
#### Linux:
1. Enter the command below in your machine to get ROS_MASTER_URI which is the ip address of your machine:  
```ifconfig```  
2. If you are connected to PI using ethernet, find the ip address that starts with "enx"  
Example:  
```enx00e04c681fa8: flags=4163<UP,BROADCAST,RUNNING,MULTICAST> \n mtu 1500 inet 10.42.0.1  netmask 255.255.255.0  broadcast 10.42.0.255 ```  
3. The ip address we want is 10.42.0.1  
4. Then the environment variables in docker-compose file in cpu-catkins directory should have the following:  
```"ROS_MASTER_URI=http://10.42.0.1:11311"```  
```"ROS_IP=10.42.0.1"```  
The ROS_MASTER_URI is to identify which container ip the ROS master is in.  
The ROS_IP is to identify which ip address the container that run all of the ROS nodes are in. 

#### MacOS (will not be able to connect to pi)
The environment variables in docker-compose file in cpu-catkins directory should have the following:  
```"ROS_MASTER_URI=http://localhost:11311"```  
```"ROS_IP=localhost"```  
The ROS_MASTER_URI is to identify which container ip the ROS master is in.  
The ROS_IP is to identify which ip address the container that run all of the ROS nodes are in. 

### Step 1.2: Setting up network in Pi (linux only):  
Now we want to do the same for the nodes in our Pi. 
1. In your pi terminal enter: 
```hostname -I```.  
2. Get the ip address that has the same first few digits as the one in your CPU since they are connected with ethernet
3. You will see the output like below  :
```10.42.0.215 172.17.0.1 ```  
4. Notice that the first one has the same first few digits as the one in the previous step. That is the ip for PI container that we want to use.  
5. Now in your docker-compose file in PI directory, add the following to your environment variable:  
```"ROS_MASTER_URI=http://10.42.0.1:11311"```  
```"ROS_IP=10.42.0.215"```  
The ROS_IP is to identify which container ip the bridge nodes are in. 
This is needed so that ROS nodes can communicate with each other not just with the master  


## Step 2: Run nodes in CPU:  
Inside cpu-catkins directory where your docker-compose.yaml files is run:  
```docker compose up --build```  

## Step 3. Run nodes in Pi:  
Inside the pi/ directory where your docker-compose.yaml files is run:  
```docker compose up --build``` 

## Step 4: Inside pi dir, run motor controller:  
Inside the pi/ directory where mc_test.py is run:  
```python3 mc_test.py```  

## Step 5: To stop any running containers:  
```docker compose down```


## Step 7: Run teleop in a new terminal:  
1. open new terminal in your laptop 
2. run:  
```docker exec -it cpu-catkins-cpu-node-1 bash```  
```source /app/devel/setup.bash```    
```roslaunch champ_teleop teleop.launch``` 

Note that the first line is to allow you to go inside the terminal in docker container and run any command you wish. "cpu-catkins-cpu-node-1" is the docker container name that you can see on the terminal in your cpu where you run docker compose up in step 2.  

## EXTRA INFO: To run any ros command in a new terminal
1. open new terminal in your laptop 
2. run:  
```docker exec -it cpu-catkins-cpu-node-1 bash```    
```source /app/devel/setup.bash```
3. run any ros command such as:  
```rostopic echo /joint_states/position```  
```rostopic info /cmd_vel```    






