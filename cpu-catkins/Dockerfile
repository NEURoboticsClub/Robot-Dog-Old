# Dockerfile_test_node
FROM ros:melodic

WORKDIR /app
ENV ROS_PYTHON_VERSION=3

# 2 RUN
# 2.1 updates
RUN apt-get update && apt-get install -y \
    && apt-get update 

# 2.2 Install python3-rospkg and to remove ROS packages (already installed).
# Select Yes for that prompt. This will remove ROS packages and we will have to re-install them.
RUN apt-get install -y python3-all-dev python3-rospkg python3-pip

# 2.3 omplete installation
RUN apt-get install -y ros-melodic-desktop-full --fix-missing
RUN apt-get install python-rosdep
RUN rosdep update
# 2.4 Install catkins
RUN pip3 install defusedxml
RUN pip3 install netifaces
RUN pip3 install catkin_pkg

# 2.5 Ensure that Python 3 is used
RUN ln -sf /usr/bin/python3 /usr/bin/python

# 3. Copy your src/ directory into the container
# should not copy the build/, devel/, or install/ directories from your host machine to your Docker container. These directories are generated when you build your Catkin workspace, and their contents are specific to the system where you did the build.
COPY ./catkin_ws/src ./src

# 4. Run catkin_make to build your workspace
# This command sources the setup script for the ROS environment.
# This script sets a number of environment variables that ROS uses, such as ROS_ROOT, ROS_PACKAGE_PATH, and ROS_MASTER_URI, among others.
# It also adds various ROS tools to your PATH.
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /app; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3'

# 5. This command sources the setup script for your catkin workspace, it will be overriden by 
# command in docker-compose file
# 5.1 "source /app/devel/setup.bash
# This sets up the environment variables for the packages in your catkin workspace.
# For example, it modifies ROS_PACKAGE_PATH to include the packages in your workspace,
# and it adds any executables in your workspace to your PATH.
CMD ["bash", "-c", "source /app/devel/setup.bash && roslaunch cpu cpu_launch.launch"]
