# ros-docker

## PREREQUSITES:  
Docker and docker compose installed  
https://docs.docker.com/compose/install/linux/#install-using-the-repository  
Python 2.7 installed  

## OPTION 1: Run ros master and cpu node in linux, bridge nodes in pi:  

### Step 0: Get ROS_MASTER_URI and ROS_IP from your CPU machine for the docker-compose in the cpu directory
Make sure Raspberry pi is connected to CPU  
Enter the command below in your machine to get ROS_MASTER_URI which is the ip address of your machine:  
```ifconfig```  
If you are connected to pi using ethernet, find the ip address that starts with "enx"  
Example:  
```enx00e04c681fa8: flags=4163<UP,BROADCAST,RUNNING,MULTICAST> \n mtu 1500 inet 10.42.0.1  netmask 255.255.255.0  broadcast 10.42.0.255 ```
The ip address is 10.42.0.1  
Then then environment variables in docker-compose file in cpu directory should have the following:  
```"ROS_MASTER_URI=http://10.42.0.1:11311"```  
```"ROS_IP=10.42.0.1"```  
The ROS_MASTER_URI is to identify which container ip the ROS master is in.  
The ROS_IP is to identify which container ip the cpu_node is in. 

### Step 1: Get the ROS_IP for docker-compose inside pi directory:  
Now we want to do the same for the nodes in our pi. 
In your pi terminal enter: 
```hostname -I```
Get the ip address that has the same first few digits as the one in your CPU since they are connected with ethernet
You will see the output like below  :
```10.42.0.215 172.17.0.1 ```  
Notice that the first one has the same first few digits as the one in linux. This is the ip for pi container that we want to use.  
We can use this. In your docker-compose file in pi directory, add the following to your environment variable:  
```"ROS_MASTER_URI=http://10.42.0.1:11311"```  
```"ROS_IP=10.42.0.215"```  
The ROS_IP is to identify which container ip the bridge nodes are in. 
This is needed so that ROS nodes can communicate with each other not just with the master  


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
