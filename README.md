# ros-docker

PREREQUSITES:  
Docker installed  


Build the server container:  
run docker build -t rosserver .  

Run the container:
docker run --name server_container -p 9999:9999 rosserver  

Run the client python3 client.py
