# ros-docker

## PREREQUSITES:  
1. Docker and docker compose installed in your machine and PI  
Follow the link below to install docker compose in linux:  
https://docs.docker.com/compose/install/linux/#install-using-the-repository  

2. Python 2.7 installed in PI  
## Step 0: Prepare the workspace directory in both machine:  
Copy the cpu directory into your machine (non-PI).  
Copy the pi directory into your PI

## Step 1: Run ros-master, cpu-node, bridge-nodes:  
### OPTION 1: ros-master and cpu-node in cpu, bridge-nodes in PI:  
#### Step 1.1: Inside the cpu dir, get ROS_MASTER_URI and ROS_IP from your CPU machine for the docker-compose
Make sure Raspberry PI is connected to your machine.    
Enter the command below in your machine to get ROS_MASTER_URI which is the ip address of your machine:  
```ifconfig```  
If you are connected to PI using ethernet, find the ip address that starts with "enx"  
Example:  
```enx00e04c681fa8: flags=4163<UP,BROADCAST,RUNNING,MULTICAST> \n mtu 1500 inet 10.42.0.1  netmask 255.255.255.0  broadcast 10.42.0.255 ```
The ip address is 10.42.0.1  
Then then environment variables in docker-compose file in cpu directory should have the following:  
```"ROS_MASTER_URI=http://10.42.0.1:11311"```  
```"ROS_IP=10.42.0.1"```  
The ROS_MASTER_URI is to identify which container ip the ROS master is in.  
The ROS_IP is to identify which container ip the cpu_node is in. 

#### Step 1.2: Inside the pi dir, get the ROS_IP for docker-compose:  
Now we want to do the same for the nodes in our PI. 
In your pi terminal enter: 
```hostname -I```.  
Get the ip address that has the same first few digits as the one in your CPU since they are connected with ethernet
You will see the output like below  :
```10.42.0.215 172.17.0.1 ```  
Notice that the first one has the same first few digits as the one in the previous step. This is the ip for PI container that we want to use.  
We can use this. In your docker-compose file in PI directory, add the following to your environment variable:  
```"ROS_MASTER_URI=http://10.42.0.1:11311"```  
```"ROS_IP=10.42.0.215"```  
The ROS_IP is to identify which container ip the bridge nodes are in. 
This is needed so that ROS nodes can communicate with each other not just with the master  


#### Step 1.3: Inside cpu dir, run cpu and master node:  
```docker compose up --build```  

#### Step 1.4: Inside pi dir, run bridge nodes:  
```docker compose up --build``` 

### OPTION 2: Run all nodes in PI:  
cd into the pi directory and run:  
```docker compose -f docker-compose-allpi.yaml up --build```

## Step 2: Inside pi dir, run motor controller:  
```/usr/bin/python2.7 mc_test.py```  

## Step 3: To stop any running containers:  
```docker compose down```
