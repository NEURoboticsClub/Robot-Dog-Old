# ros-docker

## PREREQUSITES:  
Docker and docker compose installed  
https://docs.docker.com/compose/install/linux/#install-using-the-repository  
Python 2.7 installed  

## OPTION 1: Run ros master and cpu node in linux, bridge nodes in pi:  

### Step 0: Get ROS_MASTER_URI and ROS_IP from your CPU machine for the docker-compose in the cpu directory
Make sure Raspberry pi is connected to CPU  
Enter the command in your PC to get ROS_MASTER_URI and add this to all docker compose file :  
```ifconfig```  
if you are connected to pi with ethernet find the network that starts with enx  
then find the inet. example:  
```enx00e04c681fa8: flags=4163<UP,BROADCAST,RUNNING,MULTICAST> \n mtu 1500 inet 10.42.0.1  netmask 255.255.255.0  broadcast 10.42.0.255 ```
then the environment variables will be:
"ROS_MASTER_URI=http://10.42.0.1:11311"  
"ROS_IP=10.42.0.1"  

### Step 1: Get the ROS_IP for docker-compose inside pi directory:  
in you pi terminal enter: 
```hostname -I```
Get the ip address that has the same first few digits as the one in your CPU since they are connected with ethernet
It will be something like 
```10.42.0.215 172.17.0.1 ```  
Then in your docker-compose file in pi directory, the environment variable should have    
"ROS_MASTER_URI=http://10.42.0.1:11311"  
"ROS_IP=10.42.0.215"

The ROS_IP is needed so that ROS nodes can communicate with each other not just with the master  


### Step 2: run cpu and master node in cpu:  
cd into cpu folder in your linux machine  
```docker compose up --build```  

### Step 3: run bridge nodes in pi:  
```docker compose up --build``` 

## OPTION 2:  
### Step 1: Run rosmaster, bridge nodes and cpu node in pi:
```docker compose -f docker-compose-allpi.yaml up --build```

## run motor controller:  
```/usr/bin/python2.7 mc_test.py```  

## To stop containers:  
```docker compose down```
