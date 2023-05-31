# Use the official ROS Noetic image as the base image
# combined master, sub, publisher container
FROM ros:noetic

# Set a directory for the app
WORKDIR /usr/src/app

# Copy all your scripts into the container
COPY . .

# Install any needed packages
# For example, if you need additional Python libraries, you can use pip to install them
RUN apt-get update && apt-get install -y python3-pip
# RUN pip3 install SomePackage  # Replace 'SomePackage' with your required packages

# Run server.py when the container launches
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && roscore & python3 server.py & python3 test_node.py"]

